import math
from selfdrive.controls.lib.pid import PIDController
from common.differentiator import Differentiator
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, sign
from common.op_params import opParams
from common.params import Params
from common.realtime import DT_MDL, DT_CTRL
from selfdrive.car.gm.values import CAR
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import apply_deadzone
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.modeld.constants import T_IDXS
from cereal import log

ROLL_FF_CARS = [CAR.VOLT, CAR.VOLT18]
NN_FF_CARS = [CAR.VOLT, CAR.VOLT18]

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.


FRICTION_THRESHOLD = 2.0

LAT_PLAN_MIN_IDX = 5

def get_lookahead_value(future_vals, current_val):
  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]
  
  # if any future val has opposite sign of current val, return 0
  if len(same_sign_vals) < len(future_vals):
    return 0.0
  
  # otherwise return the value with minimum absolute value
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val

def get_steer_feedforward(desired_lateral_accel, speed):
  return desired_lateral_accel
class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self._op_params = opParams(calling_function="latcontrol_torque.py")
    self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki,
                            k_d=CP.lateralTuning.torque.kd,
                            k_11 = 1.0, k_12 = 1.0, k_13 = 12.0, k_period=0.1,
                            k_f=CP.lateralTuning.torque.kf,
                            derivative_period=0.1,
                            pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
    self.friction = CP.lateralTuning.torque.friction
    self.CI = CI
    self.use_nn_ff = Params().get_bool("EnableTorqueNNFF")
    if CP.carFingerprint in NN_FF_CARS:
      self.CI.initialize_feedforward_function_torque_nn()
    self.get_steer_feedforward = CI.get_steer_feedforward_function_torque()
    self.get_friction = CI.get_steer_feedforward_function_torque_lat_jerk()
    self.get_roll_ff = CI.get_steer_feedforward_function_torque_roll()
    self.roll_k = 0.55 if CP.carFingerprint not in ROLL_FF_CARS else 1.0
    self.v_ego = 0.0
    self.friction_look_ahead_v = [0.3, 1.2]
    self.friction_look_ahead_bp = [9.0, 35.0]
    self.friction_curve_exit_ramp_bp = [0.6, 1.8] # lateral acceleration
    self.friction_curve_exit_ramp_v = [1.0, 0.7]
    self.low_speed_factor_look_ahead = 0.3
    self.low_speed_factor_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > self.low_speed_factor_look_ahead), 16)
    self.tune_override = self._op_params.get('TUNE_LAT_do_override', force_update=True)
    self.low_speed_factor_bp = [0.0, 30.0]
    self.low_speed_factor_v = [15.0, 5.0]
    self.steer_pressed_frames_since = 0
    
      
    # for actual lateral jerk calculation
    self.actual_lateral_jerk = Differentiator(0.1, 1/DT_MDL)
    self.lat_plan_last = None
  
  def update_op_params(self):
    if not self.tune_override:
      return
    self.use_steering_angle = self._op_params.get('TUNE_LAT_TRX_use_steering_angle')
    self.pid._k_p = [[0], [self._op_params.get('TUNE_LAT_TRX_kp')]]
    self.pid._k_i = [[0], [self._op_params.get('TUNE_LAT_TRX_ki')]]
    self.pid._k_d = [[0], [self._op_params.get('TUNE_LAT_TRX_kd')]]
    self.pid._k_11 = [[0], [self._op_params.get('TUNE_LAT_TRX_kp_e')]]
    self.pid._k_12 = [[0], [self._op_params.get('TUNE_LAT_TRX_ki_e')]]
    self.pid._k_13 = [[0], [self._op_params.get('TUNE_LAT_TRX_kd_e')]]
    self.pid.k_f = self._op_params.get('TUNE_LAT_TRX_kf')
    self.friction = self._op_params.get('TUNE_LAT_TRX_friction')
    self.roll_k = self._op_params.get('TUNE_LAT_TRX_roll_compensation')
    self.low_speed_factor_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_low_speed_factor_bp')]
    self.low_speed_factor_v = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_v')
    look_ahead = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_lookahead_s')
    if look_ahead != self.low_speed_factor_look_ahead:
      self.low_speed_factor_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > self.low_speed_factor_look_ahead), None)
    look_ahead = self._op_params.get('TUNE_LAT_TRX_friction_lookahead_v')
    
    self.friction_look_ahead_v = self._op_params.get('TUNE_LAT_TRX_friction_lookahead_v')
    self.friction_look_ahead_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_friction_lookahead_bp')]
    self.friction_curve_exit_ramp_bp = self._op_params.get('TUNE_LAT_TRX_friction_curve_exit_ramp_bp')
    self.friction_curve_exit_ramp_v = [1.0, self._op_params.get('TUNE_LAT_TRX_friction_curve_exit_ramp_v')]
  
  def reset(self):
    super().reset()
    self.pid.reset()
    self.actual_lateral_jerk.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, use_roll=True, lat_plan=None):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    self.v_ego = CS.vEgo

    if self.use_steering_angle:
      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll if use_roll else 0.0)
      actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.)
      self.actual_lateral_jerk._D.x = actual_curvature_rate * CS.vEgo**2
    else:
      actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
    actual_lateral_accel = actual_curvature * CS.vEgo**2
    if self.use_steering_angle:
      actual_lateral_accel += CS.aEgo * actual_curvature
    elif lat_plan != self.lat_plan_last:
      self.actual_lateral_jerk.update(actual_lateral_accel)
    self.lat_plan_last = lat_plan
    
    if CS.steeringPressed:
      self.steer_pressed_frames_since = 0
    else:
      self.steer_pressed_frames_since += 1

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      lookahead = interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
      friction_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > lookahead), 16)
      lookahead_curvature_rate = get_lookahead_value(list(lat_plan.curvatureRates)[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_curvature_rate)
      desired_lateral_jerk = desired_curvature_rate * CS.vEgo**2
      lookahead_lateral_jerk = lookahead_curvature_rate * CS.vEgo**2
      desired_lateral_accel = desired_curvature * CS.vEgo**2
      desired_lateral_accel += CS.aEgo * desired_curvature
      max_future_lateral_accel = max([i * CS.vEgo**2 for i in list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:16]] + [desired_lateral_accel], key=lambda x: abs(x))
      error_scale_lat_accel = max_future_lateral_accel
      
      low_speed_factor = interp(CS.vEgo, self.low_speed_factor_bp, self.low_speed_factor_v)**2
      lookahead_desired_curvature = get_lookahead_value(list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:self.low_speed_factor_upper_idx], desired_curvature)
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement
      error_scale_factor = 2
      if self.steer_pressed_frames_since > 150:
        error_scale_factor = 1.0 / (1.0 + min(apply_deadzone(abs(error_scale_lat_accel), 0.4) * error_scale_factor, error_scale_factor - 1))
      else:
        error_scale_factor = 1.0 / error_scale_factor
      error *= error_scale_factor
      pid_log.error = error

      lateral_accel_g = math.sin(params.roll) * ACCELERATION_DUE_TO_GRAVITY
      ff_roll = self.get_roll_ff(lateral_accel_g, self.v_ego) * (self.roll_k if use_roll else 0.0)
      
      # lateral jerk feedforward
      friction_compensation = self.get_friction(lookahead_lateral_jerk, self.v_ego, desired_lateral_accel, self.friction, FRICTION_THRESHOLD, ff_roll)
      friction_lat_accel_downscale_factor = interp(max_future_lateral_accel, [0.0, 1.5], [0.5, 1.0])
      friction_compensation *= friction_lat_accel_downscale_factor
      if sign(lookahead_lateral_jerk) != sign(desired_lateral_accel):
        # at higher lateral acceleration, it takes less jerk to initiate the return to center
        friction_compensation *= interp(abs(desired_lateral_accel), self.friction_curve_exit_ramp_bp, self.friction_curve_exit_ramp_v)
      
      lateral_jerk_error = desired_lateral_jerk - self.actual_lateral_jerk.x
      # lateral_jerk_error *= friction_lat_accel_downscale_factor
      
      # lateral acceleration feedforward
      ff = self.get_steer_feedforward(desired_lateral_accel, CS.vEgo) - ff_roll
      ff += friction_compensation
      
      if self.CI.ff_nn_model is not None:
        ff_nn = self.CI.get_ff_nn(CS.vEgo, desired_lateral_accel, lookahead_lateral_jerk, params.roll)
      else:
        ff_nn = None
      
      output_torque = self.pid.update(setpoint, measurement,
                                      override=CS.steeringPressed, 
                                      feedforward=ff if ff_nn is None or not self.use_nn_ff else ff_nn,
                                      speed=CS.vEgo,
                                      freeze_integrator=CS.steeringRateLimited or abs(CS.steeringTorque) > 0.3 or CS.vEgo < 5,
                                      D=lateral_jerk_error,
                                      error=error)

      # record steering angle error to the unused pid_log.error_rate
      angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
      pid_log.steerAngleError = angle_steers_des - CS.steeringAngleDeg
      pid_log.errorRate = self.pid.error_rate

      pid_log.active = True
      pid_log.desiredLateralAcceleration = desired_lateral_accel
      pid_log.desiredLateralJerk = desired_lateral_jerk
      pid_log.friction = friction_compensation
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = ff * self.pid.k_f
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)
      pid_log.kp = self.pid.kp
      pid_log.ki = self.pid.ki
      pid_log.kd = self.pid.kd
      pid_log.gainUpdateFactor = self.pid._gain_update_factor
      pid_log.lookaheadCurvature = lookahead_desired_curvature
      pid_log.lookaheadCurvatureRate = lookahead_curvature_rate
      pid_log.f2 = ff_nn * self.pid.k_f
      pid_log.maxFutureLatAccel = error_scale_lat_accel
      pid_log.errorScaleFactor = error_scale_factor
    pid_log.currentLateralAcceleration = actual_lateral_accel
    pid_log.currentLateralJerk = self.actual_lateral_jerk.x
      

    #TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
    
