#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import interp

from common.op_params import opParams
from common.params import Params
import cereal.messaging as messaging
from cereal import log, car
from common.realtime import DT_MDL
from common.realtime import sec_since_boot
from selfdrive.modeld.constants import T_IDXS
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.fcw import FCWChecker
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.lead_mpc import LeadMpc
from selfdrive.controls.lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.limits_long_mpc import LimitsLongitudinalMpc
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from selfdrive.controls.lib.vision_turn_controller import VisionTurnController
from selfdrive.controls.lib.speed_limit_controller import SpeedLimitController, SpeedLimitResolver
from selfdrive.controls.lib.turn_speed_controller import TurnSpeedController
from selfdrive.controls.lib.events import Events
from selfdrive.swaglog import cloudlog

GearShifter = car.CarState.GearShifter

BRAKE_SOURCES = {'lead0',
                 'lead1',
                 'lead2',
                 'lead0p1'}
COAST_SOURCES = {'cruise',
                 'limit'}

LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V_SPORT = [-2.0, -2.2, -2.0, -1.5, -1.0]
_A_CRUISE_MIN_V_FOLLOWING = [-3.0, -2.5, -2.0, -1.5, -1.0]
_A_CRUISE_MIN_V = [-1.0, -1.2, -1.0, -0.7, -0.5]
_A_CRUISE_MIN_V_ECO = [-0.7, -0.8, -0.7, -0.6, -0.5]
_A_CRUISE_MIN_BP = [i * CV.MPH_TO_MS for i in [0., 15., 30., 55., 85.]]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MAX_V_ECO = [.75, .65, .55, .45, .35]
_A_CRUISE_MAX_V = [1.1, 1.25, 1.0, 0.7, 0.65]
_A_CRUISE_MAX_V_SPORT = [2.0, 2.3, 2.0, 1.1, 0.9]
_A_CRUISE_MAX_V_FOLLOWING = [1.5, 1.5, 1.2, 0.7, 0.65]
_A_CRUISE_MAX_BP = _A_CRUISE_MIN_BP

_A_CRUISE_MIN_V_MODE_LIST = [_A_CRUISE_MIN_V, _A_CRUISE_MIN_V_SPORT, _A_CRUISE_MIN_V_ECO]
_A_CRUISE_MAX_V_MODE_LIST = [_A_CRUISE_MAX_V, _A_CRUISE_MAX_V_SPORT, _A_CRUISE_MAX_V_ECO]
_A_CRUISE_HIGHER_ACCEL_INDEX = [1, 1, 0]

# Lookup table for turns - fast accel
_A_TOTAL_MAX_V = [3.5, 4.0, 5.0]
_A_TOTAL_MAX_BP = [0., 25., 55.]

def calc_cruise_accel_limits(v_ego, following, accelMode):
  if following and accelMode == 0:
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V_FOLLOWING)
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING)
  else:
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V_MODE_LIST[accelMode])
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_MODE_LIST[accelMode])
  return [a_cruise_min, a_cruise_max]

LEAD_ONE_PLUS_TR_BUFFER = -0.3 # [s] follow distance between lead and lead+1 to run the lead+1 mpc (negative means the lead+1 doesn't affect planning unless they're rapidly approaching)
LEAD_ONE_PLUS_STOPPING_DISTANCE_BUFFER = 1.0 # [m]
LEAD_ONE_PLUS_TO_LEAD_ONE_CUTOFF_TTC = 1.5 # if lead TTC to lead+1 is less than this, then consider the lead+1

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max**2 - a_y**2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner():
  def __init__(self, CP):
    self.CP = CP
    self.mpcs = {}
    self.mpcs['lead0'] = LeadMpc(0)
    self.mpcs['lead1'] = LeadMpc(1)
    self.mpcs['lead0p1'] = LeadMpc(2)
    self.mpcs['cruise'] = LongitudinalMpc()
    self.mpcs['custom'] = LimitsLongitudinalMpc()
    
    self.mpcs['lead0p1'].stopping_distance_offset = LEAD_ONE_PLUS_STOPPING_DISTANCE_BUFFER

    self.fcw = False
    self.fcw_checker = FCWChecker()

    self.v_desired = 0.0
    self.a_desired = 0.0
    self.longitudinalPlanSource = 'cruise'
    self.alpha = np.exp(-CP.radarTimeStep/2.0)
    self.lead_0 = log.RadarState.LeadData.new_message()
    self.lead_1 = log.RadarState.LeadData.new_message()
    self.lead_0_plus = log.RadarState.LeadData.new_message()

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    self.vision_turn_controller = VisionTurnController(CP)
    self.speed_limit_controller = SpeedLimitController()
    self.events = Events()
    self.turn_speed_controller = TurnSpeedController()

    self._params = Params()
    self._op_params = opParams(calling_function="longitudinal planner")
    self.params_check_last_t = 0.
    self.params_check_freq = 0.1 # check params at 10Hz
    
    self.MADS_enabled = Params().get_bool("MADSEnabled")
    self.MADS_lead_braking_enabled = self.MADS_enabled and self._params.get_bool("MADSLeadBraking")
    self.accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco
    self.coasting_lead_d = -1. # [m] lead distance. -1. if no lead
    self.coasting_lead_v = -10. # lead "absolute"" velocity
    self.tr = 1.8
    self.seconds_stopped = 0.0
    self.gear_shifter_last = GearShifter.park
    self.update_op_params()


  def update_op_params(self):
    self.accel_profile_factors = [
      self._op_params.get('AP_stock_accel_factor'),
      self._op_params.get('AP_sport_accel_factor'),
      self._op_params.get('AP_eco_accel_factor')
    ]
    self.accel_profile_following_factor = self._op_params.get('AP_following_accel_factor')
  
  def update(self, sm, CP):
    cur_time = sec_since_boot()
    t = cur_time
    
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo
    
    if sm['carState'].standstill and sm['carState'].gearShifter in ['drive','low']:
      self.seconds_stopped += DT_MDL
    else:
      self.seconds_stopped = 0

    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_state = sm['controlsState'].longControlState
    force_slow_decel = sm['controlsState'].forceDecel

    self.lead_0 = sm['radarState'].leadOne
    self.lead_1 = sm['radarState'].leadTwo
    self.lead_0_plus = sm['radarState'].leadOnePlus
    
    self.mpcs['lead0'].df.lateralPlan = sm['lateralPlan']
    self.mpcs['lead1'].df.lateralPlan = sm['lateralPlan']

    enabled = (long_control_state == LongCtrlState.pid) or (long_control_state == LongCtrlState.stopping)
    following = self.lead_0.status and self.lead_0.dRel < 45.0 and self.lead_0.vLeadK > v_ego and self.lead_0.aLeadK > 0.0
    if self.lead_0.status:
      self.coasting_lead_d = self.lead_0.dRel
      self.coasting_lead_v = self.lead_0.vLeadK
      self.coasting_lead_a = self.lead_0.aLeadK
    elif self.lead_1.status:
      self.coasting_lead_d = self.lead_1.dRel
      self.coasting_lead_v = self.lead_1.vLeadK
      self.coasting_lead_a = self.lead_1.aLeadK
    else:
      self.coasting_lead_d = -1.
      self.coasting_lead_v = -10.
      self.coasting_lead_a = 10.
    self.tr = self.mpcs['lead0'].tr
    
    if sm['carState'].gearShifter == GearShifter.drive and self.gear_shifter_last != GearShifter.drive:
      self.mpcs['lead0'].df.reset()
      self.mpcs['lead1'].df.reset()
    self.gear_shifter_last = sm['carState'].gearShifter
    
    if long_control_state == LongCtrlState.off or sm['carState'].gasPressed:
      self.v_desired = v_ego
      self.a_desired = a_ego

    # Prevent divergence, smooth in current v_ego
    self.v_desired = self.alpha * self.v_desired + (1 - self.alpha) * v_ego
    self.v_desired = max(0.0, self.v_desired)

    # Get acceleration and active solutions for custom long mpc.
    a_mpc, active_mpc, c_source = self.mpc_solutions(enabled, self.v_desired, self.a_desired, v_cruise, sm)
    
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      self.update_op_params()
      self.MADS_lead_braking_enabled = self.MADS_enabled and self._params.get_bool("MADSLeadBraking")
      accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco
      if accel_mode != self.accel_mode:
          cloudlog.info(f"Acceleration mode changed, new value: {accel_mode} = {['normal','sport','eco','creep'][accel_mode]}")
          self.accel_mode = accel_mode

    accel_limits = calc_cruise_accel_limits(v_ego, following, self.accel_mode)
    accel_limits[1] *= self.accel_profile_following_factor if following else self.accel_profile_factors[self.accel_mode]
    if not sm['controlsState'].active:
      accel_limits[1] = min(0.0, accel_limits[1])
    if sm['carState'].gas > 1e-5:
      accel_limits[0] = 0.0
      
    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    if force_slow_decel:
      # if required so, force a smooth deceleration
      accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
      accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])

    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired)
    self.mpcs['cruise'].set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])

    # ensure lower accel limit (for braking) is lower than target acc for custom controllers.
    accel_limits = [min(accel_limits_turns[0], a_mpc['custom']), accel_limits_turns[1]]
    self.mpcs['custom'].set_accel_limits(accel_limits[0], accel_limits[1])

    if not sm['controlsState'].active and not self.MADS_lead_braking_enabled:
      self.v_desired_trajectory = np.ones(CONTROL_N) * v_ego
      self.a_desired_trajectory = np.ones(CONTROL_N) * a_ego
      self.j_desired_trajectory = np.zeros(CONTROL_N)
      self.longitudinalPlanSource = 'cruise'
      for key in self.mpcs:
        self.mpcs[key].reset_mpc()
    else:
      next_a = np.inf
      for key in self.mpcs:
        if 'lead' in key:
          self.mpcs[key].long_control_active = enabled
          self.mpcs[key].MADS_lead_braking_enabled = self.MADS_lead_braking_enabled
        if key == 'lead0p1':
          ttc = LEAD_ONE_PLUS_TO_LEAD_ONE_CUTOFF_TTC + 1
          if self.lead_0_plus.status and self.lead_0.status:
            v_rel = self.lead_0_plus.vLeadK - self.lead_0.vLeadK
            ttc = (self.lead_0_plus.dRel - self.lead_0.dRel) / max(v_rel, 0.01)
          if ttc > LEAD_ONE_PLUS_TO_LEAD_ONE_CUTOFF_TTC:
            self.mpcs['lead0p1'].reset_mpc()
            continue
          else:
            tr = self.mpcs['lead0'].tr + LEAD_ONE_PLUS_TR_BUFFER
            self.mpcs['lead0p1'].tr_override = True
            self.mpcs['lead0p1'].tr = tr
        if not sm['controlsState'].active and self.MADS_lead_braking_enabled \
            and (key not in BRAKE_SOURCES or (key == 'custom' and c_source not in BRAKE_SOURCES)):
          self.mpcs[key].reset_mpc()
          continue

        self.mpcs[key].set_cur_state(self.v_desired, self.a_desired)
        self.mpcs[key].update(sm['carState'], sm['radarState'], v_cruise, a_mpc[key], active_mpc[key])
        # picks slowest solution from accel in ~0.2 seconds
        if self.mpcs[key].status and active_mpc[key] and self.mpcs[key].a_solution[5] < next_a:
          self.longitudinalPlanSource = c_source if key == 'custom' else key
          self.v_desired_trajectory = self.mpcs[key].v_solution[:CONTROL_N]
          self.a_desired_trajectory = self.mpcs[key].a_solution[:CONTROL_N]
          self.j_desired_trajectory = self.mpcs[key].j_solution[:CONTROL_N]
          next_a = self.mpcs[key].a_solution[5]
        
    

    # determine fcw
    if self.mpcs['lead0'].new_lead:
      self.fcw_checker.reset_lead(cur_time)
    blinkers = sm['carState'].leftBlinker or sm['carState'].rightBlinker
    self.fcw = self.fcw_checker.update(self.mpcs['lead0'].mpc_solution, cur_time,
                                       sm['controlsState'].active,
                                       v_ego, sm['carState'].aEgo,
                                       self.lead_1.dRel, self.lead_1.vLead, self.lead_1.aLeadK,
                                       self.lead_1.yRel, self.lead_1.vLat,
                                       self.lead_1.fcw, blinkers) and not sm['carState'].brakePressed
    if self.fcw:
      cloudlog.info("FCW triggered %s", self.fcw_checker.counters)

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(CP.radarTimeStep, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired = self.v_desired + CP.radarTimeStep * (self.a_desired + a_prev)/2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = [float(x) for x in self.v_desired_trajectory]
    longitudinalPlan.accels = [float(x) for x in self.a_desired_trajectory]
    longitudinalPlan.jerks = [float(x) for x in self.j_desired_trajectory]

    longitudinalPlan.hasLead = self.mpcs['lead0'].status
    longitudinalPlan.leadDist = self.coasting_lead_d
    longitudinalPlan.leadV = self.coasting_lead_v
    longitudinalPlan.desiredFollowDistance = self.mpcs['lead0'].tr
    longitudinalPlan.leadDistCost = self.mpcs['lead0'].dist_cost
    longitudinalPlan.leadAccelCost = self.mpcs['lead0'].accel_cost
    longitudinalPlan.stoppingDistance = self.mpcs['lead0'].stopping_distance
    longitudinalPlan.longitudinalPlanSource = self.longitudinalPlanSource
    longitudinalPlan.fcw = self.fcw
    longitudinalPlan.dynamicFollowLevel = self.mpcs['lead0'].follow_level_df
    longitudinalPlan.secondsStopped = int(self.seconds_stopped)

    longitudinalPlan.visionTurnControllerState = self.vision_turn_controller.state
    longitudinalPlan.visionCurrentLateralAcceleration = float(self.vision_turn_controller._current_lat_acc)
    longitudinalPlan.visionMaxVForCurrentCurvature = float(self.vision_turn_controller._max_v_for_current_curvature)
    longitudinalPlan.visionMaxPredictedLateralAcceleration = float(self.vision_turn_controller._max_pred_lat_acc)
    longitudinalPlan.visionMaxPredictedCurvature = float(self.vision_turn_controller._max_pred_curvature)
    longitudinalPlan.visionCurrentLateralAccelerationNoRoll = float(self.vision_turn_controller._current_lat_acc_no_roll)
    longitudinalPlan.visionMaxPredictedRollCompensation = float(self.vision_turn_controller._max_pred_roll_compensation)
    longitudinalPlan.visionMaxPredictedLateralAccelerationDistance = float(self.vision_turn_controller._max_pred_lat_acc_dist)
    longitudinalPlan.visionTurnSpeed = float(self.vision_turn_controller.v_turn)
    longitudinalPlan.visionPredictedPathSource = self.vision_turn_controller._predicted_path_source
    longitudinalPlan.visionVf = float(self.vision_turn_controller._vf) if self.vision_turn_controller._vf is not None else -1.0
    
    longitudinalPlan.dynamicFollowState0.pointsCurrent = self.mpcs['lead0'].df.points_cur
    longitudinalPlan.dynamicFollowState0.newLead = self.mpcs['lead0'].df.new_lead
    longitudinalPlan.dynamicFollowState0.leadGone = self.mpcs['lead0'].df.lead_gone
    longitudinalPlan.dynamicFollowState0.penaltyDist = self.mpcs['lead0'].df.penalty_dist
    longitudinalPlan.dynamicFollowState0.penaltyVel = self.mpcs['lead0'].df.penalty_vel
    longitudinalPlan.dynamicFollowState0.penaltyTime = self.mpcs['lead0'].df.penalty_time
    longitudinalPlan.dynamicFollowState0.penalty = self.mpcs['lead0'].df.penalty
    longitudinalPlan.dynamicFollowState0.lastCutinFactor = self.mpcs['lead0'].df.last_cutin_factor
    longitudinalPlan.dynamicFollowState0.rescindedPenalty = self.mpcs['lead0'].df.rescinded_penalty
    longitudinalPlan.dynamicFollowState0.penaltyTraffic = self.mpcs['lead0'].df.traffic_penalty

    longitudinalPlan.dynamicFollowState1.pointsCurrent = self.mpcs['lead1'].df.points_cur
    longitudinalPlan.dynamicFollowState1.newLead = self.mpcs['lead1'].df.new_lead
    longitudinalPlan.dynamicFollowState1.leadGone = self.mpcs['lead1'].df.lead_gone
    longitudinalPlan.dynamicFollowState1.penaltyDist = self.mpcs['lead1'].df.penalty_dist
    longitudinalPlan.dynamicFollowState1.penaltyVel = self.mpcs['lead1'].df.penalty_vel
    longitudinalPlan.dynamicFollowState1.penaltyTime = self.mpcs['lead1'].df.penalty_time
    longitudinalPlan.dynamicFollowState1.penalty = self.mpcs['lead1'].df.penalty
    longitudinalPlan.dynamicFollowState1.lastCutinFactor = self.mpcs['lead1'].df.last_cutin_factor
    longitudinalPlan.dynamicFollowState1.rescindedPenalty = self.mpcs['lead1'].df.rescinded_penalty
    longitudinalPlan.dynamicFollowState1.penaltyTraffic = self.mpcs['lead1'].df.traffic_penalty

    longitudinalPlan.speedLimitControlState = self.speed_limit_controller.state
    longitudinalPlan.speedLimit = float(self.speed_limit_controller.speed_limit)
    longitudinalPlan.speedLimitOffset = float(self.speed_limit_controller.speed_limit_offset)
    longitudinalPlan.distToSpeedLimit = float(self.speed_limit_controller.distance)
    longitudinalPlan.isMapSpeedLimit = bool(self.speed_limit_controller.source == SpeedLimitResolver.Source.map_data)
    longitudinalPlan.eventsDEPRECATED = self.events.to_msg()

    longitudinalPlan.turnSpeedControlState = self.turn_speed_controller.state
    longitudinalPlan.turnSpeed = float(self.turn_speed_controller.speed_limit)
    longitudinalPlan.distToTurn = float(self.turn_speed_controller.distance)
    longitudinalPlan.turnSign = int(self.turn_speed_controller.turn_sign)

    pm.send('longitudinalPlan', plan_send)

  def mpc_solutions(self, enabled, v_ego, a_ego, v_cruise, sm):
    # Update controllers
    self.vision_turn_controller.update(enabled, v_ego, a_ego, v_cruise, sm)
    self.events = Events()
    self.speed_limit_controller.update(enabled, v_ego, a_ego, sm, v_cruise, self.events)
    self.turn_speed_controller.update(enabled, v_ego, a_ego, sm)

    # Pick solution with lowest acceleration target.
    a_solutions = {None: float("inf")}

    if self.vision_turn_controller.is_active:
      a_solutions['turn'] = self.vision_turn_controller.a_target

    if self.speed_limit_controller.is_active:
      a_solutions['limit'] = self.speed_limit_controller.a_target

    if self.turn_speed_controller.is_active:
      a_solutions['turnlimit'] = self.turn_speed_controller.a_target

    source = min(a_solutions, key=a_solutions.get)

    a_sol = {
      'cruise': a_ego,  # Irrelevant
      'lead0': a_ego,   # Irrelevant
      'lead1': a_ego,   # Irrelevant
      'lead0p1': a_ego,   # Irrelevant
      'custom': 0. if source is None else a_solutions[source],
    }

    active_sol = {
      'cruise': True,  # Irrelevant
      'lead0': True,   # Irrelevant
      'lead1': True,   # Irrelevant
      'lead0p1': True,   # Irrelevant
      'custom': source is not None,
    }

    return a_sol, active_sol, source
