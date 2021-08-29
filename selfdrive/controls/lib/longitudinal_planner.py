#!/usr/bin/env python3
import math
import numpy as np
from common.params import Params
from common.numpy_fast import interp

import cereal.messaging as messaging
from cereal import log
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.fcw import FCWChecker
from selfdrive.controls.lib.long_mpc import LongitudinalMpc
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from selfdrive.kegman_conf import kegman_conf

kegman = kegman_conf()

LON_MPC_STEP = 0.2  # first step is 0.2s
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted
COAST_SPEED = 10.0 * CV.MPH_TO_MS # brake at COAST_SPEED above set point

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V_ECO = [-1.0, -0.7, -0.6, -0.5, -0.3]
_A_CRUISE_MIN_V_SPORT = [-3.0, -2.6, -2.3, -2.0, -1.0]
_A_CRUISE_MIN_V_FOLLOWING = [-3.0, -2.5, -2.0, -1.5, -1.0]
_A_CRUISE_MIN_V = [-2.0, -1.5, -1.0, -0.7, -0.5]
_A_CRUISE_MIN_BP = [0.0, 5.0, 10.0, 20.0, 55.0]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MAX_V = [2.0, 2.0, 1.5, .5, .3]
_A_CRUISE_MAX_V_ECO = [0.8, 0.9, 1.0, 0.4, 0.2]
_A_CRUISE_MAX_V_SPORT = [3.0, 3.5, 3.0, 2.0, 2.0]
_A_CRUISE_MAX_V_FOLLOWING = [1.6, 1.4, 1.4, .7, .3]
_A_CRUISE_MAX_BP = [0., 5., 10., 20., 55.]

# Lookup table for turns - fast accel
_A_TOTAL_MAX_V = [3.5, 4.0, 5.0]
_A_TOTAL_MAX_BP = [0., 25., 55.]

# Lookup table for turns original - used when slowOnCurves = 1 from kegman.json
_A_TOTAL_MAX_V_SOC = [1.7, 3.2]
_A_TOTAL_MAX_BP_SOC = [20., 40.]

Source = log.LongitudinalPlan.LongitudinalPlanSource


def calc_cruise_accel_limits(v_ego, following, accelMode):
  a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V)

  print("accelMode = ",accelMode) 

  if following:
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING)
  else:
    _A_CRUISE_MAX_V_MODE_LIST = [_A_CRUISE_MAX_V_ECO, _A_CRUISE_MAX_V, _A_CRUISE_MAX_V_SPORT]
    _A_CRUISE_MAX_V_MODE_LIST_INDEX = min(max(int(accelMode), 0), (len(_A_CRUISE_MAX_V_MODE_LIST) - 1))
    a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_MODE_LIST[_A_CRUISE_MAX_V_MODE_LIST_INDEX])
  return np.vstack([a_cruise_min, a_cruise_max])


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  
  if int(kegman.conf['slowOnCurves']):
    a_total_max = interp(v_ego, _A_TOTAL_MAX_BP_SOC, _A_TOTAL_MAX_V_SOC)
  else:
    a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  
  a_y = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max**2 - a_y**2, 0.))

  

  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner():
  def __init__(self, CP):
    self.CP = CP

    self.mpc1 = LongitudinalMpc(1)
    self.mpc2 = LongitudinalMpc(2)

    self.v_acc_start = 0.0
    self.a_acc_start = 0.0
    self.v_acc_next = 0.0
    self.a_acc_next = 0.0

    self.v_acc = 0.0
    self.v_acc_future = 0.0
    self.a_acc = 0.0
    self.v_cruise = 0.0
    self.a_cruise = 0.0

    self.source = Source.cruiseCoast
    self.cruise_source = Source.cruiseCoast

    self.fcw_checker = FCWChecker()
    self.path_x = np.arange(192)

    self.fcw = False

    self.params = Params()
    self.kegman = kegman_conf()
    self.mpc_frame = 0
    self.first_loop = True

    self.coast_enabled = True

  def choose_solution(self, v_cruise_setpoint, enabled):
    if enabled:
      solutions = {self.cruise_source: self.v_cruise}
      if self.mpc1.prev_lead_status:
        solutions[Source.mpc1] = self.mpc1.v_mpc
      if self.mpc2.prev_lead_status:
        solutions[Source.mpc2] = self.mpc2.v_mpc

      slowest = min(solutions, key=solutions.get)

      self.source = slowest
      # Choose lowest of MPC and cruise
      if slowest == Source.mpc1:
        self.v_acc = self.mpc1.v_mpc
        self.a_acc = self.mpc1.a_mpc
      elif slowest == Source.mpc2:
        self.v_acc = self.mpc2.v_mpc
        self.a_acc = self.mpc2.a_mpc
      elif slowest == self.cruise_source:
        self.v_acc = self.v_cruise
        self.a_acc = self.a_cruise

    self.v_acc_future = min([self.mpc1.v_mpc_future, self.mpc2.v_mpc_future, v_cruise_setpoint])

  def choose_cruise(self, v_ego, a_ego, v_cruise_setpoint, accel_limits_turns, jerk_limits, gasbrake):
    # WARNING: Logic is carefully verified. On change, review test_longitudinal.py output!

    # Standard cruise
    if not self.coast_enabled:
      self.cruise_source = Source.cruiseGas
      return speed_smoother(self.v_acc_start, self.a_acc_start,
                            v_cruise_setpoint,
                            accel_limits_turns[1], accel_limits_turns[0],
                            jerk_limits[1], jerk_limits[0],
                            LON_MPC_STEP)

    # If coasting, reset starting state for gas and brake plans
    if self.source == Source.cruiseCoast:
      self.v_acc_start = v_ego
      self.a_acc_start = a_ego
    elif self.source in [Source.mpc1, Source.mpc2]:
      self.cruise_source = Source.cruiseGas if gasbrake >= 0 else Source.cruiseBrake

    # Coast to (current state)
    v_coast, a_coast = v_ego, a_ego
    # Gas to (v_cruise_setpoint)
    v_gas,   a_gas   = speed_smoother(self.v_acc_start, self.a_acc_start,
                                      v_cruise_setpoint,
                                      accel_limits_turns[1], accel_limits_turns[0],
                                      jerk_limits[1], jerk_limits[0],
                                      LON_MPC_STEP)
    # Brake to (v_cruise_setpoint + COAST_SPEED)
    v_brake, a_brake = speed_smoother(self.v_acc_start, self.a_acc_start,
                                      v_cruise_setpoint + COAST_SPEED,
                                      accel_limits_turns[1], accel_limits_turns[0],
                                      jerk_limits[1], jerk_limits[0],
                                      LON_MPC_STEP)

    cruise = {
      Source.cruiseCoast: (v_coast, a_coast),
      Source.cruiseGas: (v_gas, a_gas),
      Source.cruiseBrake: (v_brake, a_brake),
    }

    # Entry conditions
    if gasbrake == 0:
      if a_brake <= a_coast:
        self.cruise_source = Source.cruiseBrake
      elif a_gas >= a_coast:
        self.cruise_source = Source.cruiseGas
      elif (a_brake >= a_coast >= a_gas):
        self.cruise_source = Source.cruiseCoast

    return cruise[self.cruise_source]

  def update(self, sm, CP, VM, PP):
    """Gets called when new radarState is available"""
    cur_time = sec_since_boot()
    v_ego = sm['carState'].vEgo
    a_ego = sm['carState'].aEgo
    gasbrake = sm['carControl'].actuators.gas - sm['carControl'].actuators.brake

    long_control_state = sm['controlsState'].longControlState
    v_cruise_kph = sm['controlsState'].vCruise
    force_slow_decel = sm['controlsState'].forceDecel

    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise_setpoint = v_cruise_kph * CV.KPH_TO_MS

    lead_1 = sm['radarState'].leadOne
    lead_2 = sm['radarState'].leadTwo

    enabled = long_control_state in [LongCtrlState.pid, LongCtrlState.stopping]
    following = lead_1.status and lead_1.dRel < 45.0 and lead_1.vLeadK > v_ego and lead_1.aLeadK > 0.0

    if self.mpc_frame % 1000 == 0:
      self.kegman = kegman_conf()
      self.mpc_frame = 0
      
    self.mpc_frame += 1
    
    self.v_acc_start = self.v_acc_next
    self.a_acc_start = self.a_acc_next

    # Calculate speed for normal cruise control
    if enabled and not self.first_loop and not sm['carState'].gasPressed:
      accel_limits = [float(x) for x in calc_cruise_accel_limits(v_ego, following, self.kegman.conf['accelerationMode'])]
      jerk_limits = [min(-0.1, accel_limits[0]), max(0.1, accel_limits[1])]  # TODO: make a separate lookup for jerk tuning
      accel_limits_turns = accel_limits # limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)

      if force_slow_decel:
        # if required so, force a smooth deceleration
        accel_limits_turns[1] = min(accel_limits_turns[1], AWARENESS_DECEL)
        accel_limits_turns[0] = min(accel_limits_turns[0], accel_limits_turns[1])

      self.v_cruise, self.a_cruise = self.choose_cruise(v_ego,
                                                        a_ego,
                                                        v_cruise_setpoint,
                                                        accel_limits_turns,
                                                        jerk_limits,
                                                        gasbrake)

      # cruise speed can't be negative even is user is distracted
      self.v_cruise = max(self.v_cruise, 0.)
    else:
      starting = long_control_state == LongCtrlState.starting
      a_ego = min(a_ego, 0.0)
      reset_speed = self.CP.minSpeedCan if starting else v_ego
      reset_accel = self.CP.startAccel if starting else a_ego
      self.v_acc = reset_speed
      self.a_acc = reset_accel
      self.v_acc_start = reset_speed
      self.a_acc_start = reset_accel
      self.v_cruise = reset_speed
      self.a_cruise = reset_accel
      self.cruise_source = Source.cruiseCoast

    self.mpc1.set_cur_state(self.v_acc_start, self.a_acc_start)
    self.mpc2.set_cur_state(self.v_acc_start, self.a_acc_start)

    self.mpc1.update(sm['carState'], lead_1)
    self.mpc2.update(sm['carState'], lead_2)

    self.choose_solution(v_cruise_setpoint, enabled)

    # determine fcw
    if self.mpc1.new_lead:
      self.fcw_checker.reset_lead(cur_time)

    blinkers = sm['carState'].leftBlinker or sm['carState'].rightBlinker
    self.fcw = self.fcw_checker.update(self.mpc1.mpc_solution, cur_time,
                                       sm['controlsState'].active,
                                       v_ego, sm['carState'].aEgo,
                                       lead_1.dRel, lead_1.vLead, lead_1.aLeadK,
                                       lead_1.yRel, lead_1.vLat,
                                       lead_1.fcw, blinkers) and not sm['carState'].brakePressed
    if self.fcw:
      cloudlog.info("FCW triggered %s", self.fcw_checker.counters)

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_acc_sol = self.a_acc_start + (CP.radarTimeStep / LON_MPC_STEP) * (self.a_acc - self.a_acc_start)
    v_acc_sol = self.v_acc_start + CP.radarTimeStep * (a_acc_sol + self.a_acc_start) / 2.0
    self.v_acc_next = v_acc_sol
    self.a_acc_next = a_acc_sol

    self.first_loop = False

  def publish(self, sm, pm):
    self.mpc1.publish(pm)
    self.mpc2.publish(pm)

    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.mdMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.radarStateMonoTime = sm.logMonoTime['radarState']

    longitudinalPlan.vCruise = float(self.v_cruise)
    longitudinalPlan.aCruise = float(self.a_cruise)
    longitudinalPlan.vStart = float(self.v_acc_start)
    longitudinalPlan.aStart = float(self.a_acc_start)
    longitudinalPlan.vTarget = float(self.v_acc)
    longitudinalPlan.aTarget = float(self.a_acc)
    longitudinalPlan.vTargetFuture = float(self.v_acc_future)
    longitudinalPlan.hasLead = self.mpc1.prev_lead_status
    longitudinalPlan.longitudinalPlanSource = self.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.rcv_time['radarState']

    pm.send('longitudinalPlan', plan_send)
