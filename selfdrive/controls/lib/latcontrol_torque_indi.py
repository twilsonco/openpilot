import math
import numpy as np

from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import clip, interp
from common.op_params import opParams
from common.realtime import DT_CTRL
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.toyota.values import CarControllerParams
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.controls.lib.latcontrol_torque import FRICTION_THRESHOLD
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY


class LatControlTorqueINDI():
  def __init__(self, CP, CI):
    self.angle_steers_des = 0.

    A = np.array([[1.0, DT_CTRL, 0.0],
                  [0.0, 1.0, DT_CTRL],
                  [0.0, 0.0, 1.0]])
    C = np.array([[1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0]])

    # Q = np.matrix([[1e-2, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 10.0]])
    # R = np.matrix([[1e-2, 0.0], [0.0, 1e3]])

    # (x, l, K) = control.dare(np.transpose(A), np.transpose(C), Q, R)
    # K = np.transpose(K)
    K = np.array([[7.30262179e-01, 2.07003658e-04],
                  [7.29394177e+00, 1.39159419e-02],
                  [1.71022442e+01, 3.38495381e-02]])

    self.speed = 0.

    self.K = K
    self.A_K = A - np.dot(K, C)
    self.x = np.array([[0.], [0.], [0.]])

    self.enforce_rate_limit = CP.carName == "toyota"

    self._RC = (CP.lateralTuning.torqueIndi.timeConstantBP, CP.lateralTuning.torqueIndi.timeConstantV)
    self._G = (CP.lateralTuning.torqueIndi.actuatorEffectivenessBP, CP.lateralTuning.torqueIndi.actuatorEffectivenessV)
    self._outer_loop_gain = (CP.lateralTuning.torqueIndi.outerLoopGainBP, CP.lateralTuning.torqueIndi.outerLoopGainV)
    self._inner_loop_gain = (CP.lateralTuning.torqueIndi.innerLoopGainBP, CP.lateralTuning.torqueIndi.innerLoopGainV)
    self.sat_count_rate = 1.0 * DT_CTRL
    self.sat_limit = CP.steerLimitTimer
    self.steer_filter = FirstOrderFilter(0., self.RC, DT_CTRL)
    self.get_steer_feedforward = CI.get_steer_feedforward_function_torque()
    self.friction = CP.lateralTuning.torqueIndi.friction
    self._k_f = CP.lateralTuning.torqueIndi.kf
    
    self._op_params = opParams(calling_function="latcontrol_torque_indi.py")
    self.roll_k = 0.0

    self.reset()
  
  def update_op_params(self):
    bp = [i * CV.MPH_TO_MS for i in self._op_params.get("TUNE_LAT_TRXINDI_bp_mph")]
    self._RC = (bp, self._op_params.get("TUNE_LAT_TRXINDI_time_constant"))
    self._G = (bp, self._op_params.get("TUNE_LAT_TRXINDI_actuator_effectiveness"))
    self._outer_loop_gain = (bp, self._op_params.get("TUNE_LAT_TRXINDI_outer_gain"))
    self._inner_loop_gain = (bp, self._op_params.get("TUNE_LAT_TRXINDI_inner_gain"))
    self.roll_k = self._op_params.get('TUNE_LAT_TRXINDI_roll_compensation')
    self.friction = self._op_params.get('TUNE_LAT_TRXINDI_friction')
    self._k_f = self._op_params.get('TUNE_LAT_TRXINDI_kf')

  @property
  def RC(self):
    return interp(self.speed, self._RC[0], self._RC[1])

  @property
  def G(self):
    return interp(self.speed, self._G[0], self._G[1])

  @property
  def outer_loop_gain(self):
    return interp(self.speed, self._outer_loop_gain[0], self._outer_loop_gain[1])

  @property
  def inner_loop_gain(self):
    return interp(self.speed, self._inner_loop_gain[0], self._inner_loop_gain[1])

  def reset(self):
    self.steer_filter.x = 0.
    self.output_steer = 0.
    self.sat_count = 0.
    self.speed = 0.

  def _check_saturation(self, control, check_saturation, limit):
    saturated = abs(control) == limit

    if saturated and check_saturation:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, use_roll=True, lat_plan=None):
    self.speed = CS.vEgo
    
    actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll if use_roll else 0.0)
    actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0)
    
    desired_lateral_jerk = desired_curvature_rate * CS.vEgo**2
    desired_lateral_accel = desired_curvature * CS.vEgo**2
    actual_lateral_accel = actual_curvature * CS.vEgo**2
    actual_lateral_jerk = actual_curvature_rate * CS.vEgo**2
    
    # Update Kalman filter
    y = np.array([[actual_lateral_accel], [actual_lateral_jerk]])
    self.x = np.dot(self.A_K, self.x) + np.dot(self.K, y)

    indi_log = log.ControlsState.LateralTorqueINDIState.new_message()
    indi_log.actualLateralAccel = float(self.x[0][0])
    indi_log.actualLateralJerk = float(self.x[1][0])
    indi_log.actualLateralJounce = float(self.x[2][0])

    steers_des = desired_lateral_accel
    indi_log.lateralAccelerationDesired = float(steers_des)

    rate_des = desired_lateral_jerk
    indi_log.lateralJerkDesired = (rate_des)
    
    if CS.vEgo < 0.3 or not active:
      indi_log.active = False
      self.output_steer = 0.0
      self.steer_filter.x = 0.0
    else:
      # Expected actuator value
      self.steer_filter.update_alpha(self.RC)
      self.steer_filter.update(self.output_steer)

      # Compute acceleration error
      low_speed_factor = interp(CS.vEgo, [10., 25.], [80., 50.])
      sp_off = low_speed_factor * desired_curvature
      measure_off = low_speed_factor * actual_curvature
      rate_sp = self.outer_loop_gain * ((steers_des + sp_off) - (self.x[0] + measure_off)) + rate_des
      accel_sp = self.inner_loop_gain * (rate_sp - self.x[1])
      accel_error = accel_sp - self.x[2]

      # Compute change in actuator
      g_inv = 1. / self.G
      delta_u = g_inv * accel_error

      # If steering pressed, only allow wind down
      if CS.steeringPressed and (delta_u * self.output_steer > 0):
        delta_u = 0

      # Enforce rate limit
      if self.enforce_rate_limit:
        steer_max = float(CarControllerParams.STEER_MAX)
        new_output_steer_cmd = steer_max * (self.steer_filter.x + delta_u)
        prev_output_steer_cmd = steer_max * self.output_steer
        new_output_steer_cmd = apply_toyota_steer_torque_limits(new_output_steer_cmd, prev_output_steer_cmd, prev_output_steer_cmd, CarControllerParams)
        self.output_steer = new_output_steer_cmd / steer_max
      else:
        self.output_steer = self.steer_filter.x + delta_u
      
      ff_roll = -math.sin(params.roll) * ACCELERATION_DUE_TO_GRAVITY * (self.roll_k if use_roll else 0.0)
      ff = self.get_steer_feedforward(desired_lateral_accel, CS.vEgo)
      friction_compensation = interp(desired_lateral_jerk, 
                                     [-FRICTION_THRESHOLD, FRICTION_THRESHOLD], 
                                     [-self.friction, self.friction])
      ff *= self._k_f
      indi_log.f = ff
      indi_log.friction = friction_compensation
      indi_log.rollCompensation = ff_roll
      ff += friction_compensation
      ff += ff_roll
      
      self.output_steer += ff

      steers_max = get_steer_max(CP, CS.vEgo)
      self.output_steer = clip(self.output_steer, -steers_max, steers_max)

      indi_log.active = True
      indi_log.rateSetPoint = float(rate_sp)
      indi_log.accelSetPoint = float(accel_sp)
      indi_log.accelError = float(accel_error)
      indi_log.delayedOutput = float(self.steer_filter.x)
      indi_log.delta = float(delta_u)
      indi_log.output = float(self.output_steer)

      check_saturation = (CS.vEgo > 10.) and not CS.steeringRateLimited and not CS.steeringPressed
      indi_log.saturated = self._check_saturation(self.output_steer, check_saturation, steers_max)

    return float(-self.output_steer), float(steers_des), indi_log
