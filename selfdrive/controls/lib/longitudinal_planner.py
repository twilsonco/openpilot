#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import interp

from common.params import Params
import cereal.messaging as messaging
from cereal import log
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

LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V_SPORT = [-2.0, -2.2, -2.0, -1.5, -1.0]
_A_CRUISE_MIN_V_FOLLOWING = [-3.0, -2.5, -2.0, -1.5, -1.0]
_A_CRUISE_MIN_V = [-1.0, -1.2, -1.0, -0.7, -0.5]
_A_CRUISE_MIN_V_ECO = [-0.7, -0.8, -0.7, -0.6, -0.5]
_A_CRUISE_MIN_BP = [0., 5., 10., 20., 55.]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MAX_V_CREEP = [0.22, .22, .2, .2, .2]
_A_CRUISE_MAX_V_ECO = [0.8, 0.7, 0.5, 0.4, 0.3]
_A_CRUISE_MAX_V = [1.2, 1.4, 1.2, 0.9, 0.7]
_A_CRUISE_MAX_V_SPORT = [2.2, 2.4, 2.2, 1.1, 0.9]
_A_CRUISE_MAX_V_FOLLOWING = [1.6, 1.8, 1.6, .9, .7]
_A_CRUISE_MAX_BP = [0., 5., 10., 20., 55.]

_A_CRUISE_MIN_V_MODE_LIST = [_A_CRUISE_MIN_V, _A_CRUISE_MIN_V_SPORT, _A_CRUISE_MIN_V_ECO, _A_CRUISE_MIN_V_ECO]
_A_CRUISE_MAX_V_MODE_LIST = [_A_CRUISE_MAX_V, _A_CRUISE_MAX_V_SPORT, _A_CRUISE_MAX_V_ECO, _A_CRUISE_MAX_V_CREEP]

# Lookup table for turns - fast accel
_A_TOTAL_MAX_V = [3.5, 4.0, 5.0]
_A_TOTAL_MAX_BP = [0., 25., 55.]

def calc_cruise_accel_limits(v_ego, following, accelMode):
  if following and accelMode < 2:
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V_FOLLOWING)
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING)
  else:
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V_MODE_LIST[accelMode])
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_MODE_LIST[accelMode])
  return [a_cruise_min, a_cruise_max]



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
    self.mpcs['cruise'] = LongitudinalMpc()
    self.mpcs['custom'] = LimitsLongitudinalMpc()

    self.fcw = False
    self.fcw_checker = FCWChecker()

    self.v_desired = 0.0
    self.a_desired = 0.0
    self.longitudinalPlanSource = 'cruise'
    self.alpha = np.exp(-CP.radarTimeStep/2.0)
    self.lead_0 = log.ModelDataV2.LeadDataV3.new_message()
    self.lead_1 = log.ModelDataV2.LeadDataV3.new_message()

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)

    self.vision_turn_controller = VisionTurnController(CP)
    self.speed_limit_controller = SpeedLimitController()
    self.events = Events()
    self.turn_speed_controller = TurnSpeedController()

    self._params = Params()
    self.params_check_last_t = 0.
    self.params_check_freq = 0.1 # check params at 10Hz
    
    self.accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco; 3 = creep
    self.coasting_lead_d = -1. # [m] lead distance. -1. if no lead
    self.coasting_lead_v = -10. # lead "absolute"" velocity
    self.tr = 1.8

    self.sessionInitTime = sec_since_boot()
    self.debug_logging = False
    self.debug_log_time_step = 0.333
    self.last_debug_log_t = 0.
    self.debug_log_path = "/data/openpilot/long_debug.csv"
    if self.debug_logging:
      with open(self.debug_log_path,"w") as f:
        f.write(",".join([
          "t",
          "vEgo", 
          "vEgo (mph)",
          "a lim low",
          "a lim high",
          "out a",
          "out long plan"]) + "\n")

  def update(self, sm, CP):
    cur_time = sec_since_boot()
    t = cur_time
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo

    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_state = sm['controlsState'].longControlState
    force_slow_decel = sm['controlsState'].forceDecel

    self.lead_0 = sm['radarState'].leadOne
    self.lead_1 = sm['radarState'].leadTwo

    enabled = (long_control_state == LongCtrlState.pid) or (long_control_state == LongCtrlState.stopping)
    following = self.lead_1.status and self.lead_1.dRel < 45.0 and self.lead_1.vLeadK > v_ego and self.lead_1.aLeadK > 0.0
    if self.lead_1.status:
      self.coasting_lead_d = self.lead_1.dRel
      self.coasting_lead_v = self.lead_1.vLead
    else:
      self.coasting_lead_d = -1.
      self.coasting_lead_v = -10.
    self.tr = self.mpcs['lead0'].tr
    
    
    if not enabled or sm['carState'].gasPressed:
      self.v_desired = v_ego
      self.a_desired = a_ego

    # Prevent divergence, smooth in current v_ego
    self.v_desired = self.alpha * self.v_desired + (1 - self.alpha) * v_ego
    self.v_desired = max(0.0, self.v_desired)

    # Get acceleration and active solutions for custom long mpc.
    a_mpc, active_mpc, c_source = self.mpc_solutions(enabled, self.v_desired, self.a_desired, v_cruise, sm)
    
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco
      if accel_mode != self.accel_mode:
          cloudlog.info(f"Acceleration mode changed, new value: {accel_mode} = {['normal','sport','eco','creep'][accel_mode]}")
          self.accel_mode = accel_mode

    accel_limits = calc_cruise_accel_limits(v_ego, following, self.accel_mode)
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

    next_a = np.inf
    for key in self.mpcs:
      self.mpcs[key].set_cur_state(self.v_desired, self.a_desired)
      self.mpcs[key].update(sm['carState'], sm['radarState'], v_cruise, a_mpc[key], active_mpc[key])
      # picks slowest solution from accel in ~0.2 seconds
      if self.mpcs[key].status and active_mpc[key] and self.mpcs[key].a_solution[5] < next_a:
        self.longitudinalPlanSource = c_source if key == 'custom' else key
        self.v_desired_trajectory = self.mpcs[key].v_solution[:CONTROL_N]
        self.a_desired_trajectory = self.mpcs[key].a_solution[:CONTROL_N]
        self.j_desired_trajectory = self.mpcs[key].j_solution[:CONTROL_N]
        next_a = self.mpcs[key].a_solution[5]
    
    # debug logging
    do_log = self.debug_logging and (t - self.last_debug_log_t > self.debug_log_time_step)
    if do_log:
      self.last_debug_log_t = t
      f = open(self.debug_log_path,"a")
      f.write(",".join([f"{i:.1f}" if i == float else str(i) for i in [
        t - self.sessionInitTime,
        v_ego, 
        v_ego * CV.MS_TO_MPH, 
        accel_limits[0],
        accel_limits[1],
        next_a,
        self.longitudinalPlanSource]]) + "\n")
      f.close()
        
    

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

    longitudinalPlan.visionTurnControllerState = self.vision_turn_controller.state
    longitudinalPlan.visionTurnSpeed = float(self.vision_turn_controller.v_turn)

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
      'custom': 0. if source is None else a_solutions[source],
    }

    active_sol = {
      'cruise': True,  # Irrelevant
      'lead0': True,   # Irrelevant
      'lead1': True,   # Irrelevant
      'custom': source is not None,
    }

    return a_sol, active_sol, source
