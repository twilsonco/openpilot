import math
from selfdrive.controls.lib.pid import PIDController
from collections import deque
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
ERR_FRICTION_THRESHOLD = 0.3

LAT_PLAN_MIN_IDX = 5

def get_lookahead_value(future_vals, current_val):
  if len(future_vals) == 0:
    return current_val
  
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
    self.use_nn_ff = Params().get_bool("EnableNNFF")
    if CP.carFingerprint in NN_FF_CARS:
      self.CI.initialize_feedforward_function_torque_nn()
    self.use_nn_ff = self.use_nn_ff and self.CI.ff_nn_model is not None
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
    
    self.error_scale_recip = 2.0
    self.error_scale_factor = FirstOrderFilter(1.0, 0.5, DT_CTRL)
    
    if self.use_nn_ff:
      # NNFF model takes current v_ego, a_ego, lat_accel, lat_jerk, roll, and past/future data
      # of lat accel, lat jerk, and roll
      # Past/future data is relative to current values (i.e. 0.5 means current value + 0.5)
      # Times are relative to current time at (-0.5, -0.3, 0.3, 0.5, 0.9, 1.7) seconds
      # Past value is computed using observed car lat accel, jerk, and roll
      # actual current values are passed as the -0.3s value, the desired values are passed as the actual lat accel etc values, and the future values are interpolated from predicted planner/model data
      self.nnff_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.8]
      self.nnff_future_times = [i + self.nnff_time_offset for i in future_times]
      history_check_frames = [30] # 0.3 seconds ago
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lat_accel_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
    
      
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
    self.error_scale_recip = self._op_params.get('TUNE_LAT_TRX_error_downscale_in_curves')
    self.error_scale_factor.update_alpha(self._op_params.get('TUNE_LAT_TRX_error_downscale_smoothing'))
    
    self.friction_look_ahead_v = self._op_params.get('TUNE_LAT_TRX_friction_lookahead_v')
    self.friction_look_ahead_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_friction_lookahead_bp')]
    self.friction_curve_exit_ramp_bp = self._op_params.get('TUNE_LAT_TRX_friction_curve_exit_ramp_bp')
    self.friction_curve_exit_ramp_v = [1.0, self._op_params.get('TUNE_LAT_TRX_friction_curve_exit_ramp_v')]
  
  def reset(self):
    super().reset()
    self.pid.reset()
    self.actual_lateral_jerk.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, use_roll=True, lat_plan=None, model_data=None):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    self.v_ego = CS.vEgo

    if self.use_steering_angle:
      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll if use_roll else 0.0)
      actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.)
      self.actual_lateral_jerk._D.x = actual_curvature_rate * CS.vEgo**2
    else:
      actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
    actual_lateral_accel = actual_curvature * CS.vEgo**2
    self.lat_plan_last = lat_plan

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
      max_future_lateral_accel = max([i * CS.vEgo**2 for i in list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:16]] + [desired_curvature], key=lambda x: abs(x))
      error_scale_factor = 1.0 / (1.0 + min(apply_deadzone(abs(max_future_lateral_accel), 0.3) * self.error_scale_recip, self.error_scale_recip - 1))
      if error_scale_factor < self.error_scale_factor.x:
        self.error_scale_factor.x = error_scale_factor
      else:
        self.error_scale_factor.update(error_scale_factor)
      
      if self.use_nn_ff:
        low_speed_factor = interp(CS.vEgo, [0, 10, 20], [13, 5, 0])**2
      else:
        low_speed_factor = interp(CS.vEgo, self.low_speed_factor_bp, self.low_speed_factor_v)**2
      lookahead_desired_curvature = get_lookahead_value(list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:self.low_speed_factor_upper_idx], desired_curvature)
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement
      error *= self.error_scale_factor.x
      setpoint = measurement + error
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
      
      lateral_jerk_error = 0 if lookahead_lateral_jerk != 0 else lookahead_lateral_jerk - self.actual_lateral_jerk.x
      # lateral_jerk_error *= friction_lat_accel_downscale_factor
      
      # error-based friction term
      error_friction = interp(error, [-ERR_FRICTION_THRESHOLD, ERR_FRICTION_THRESHOLD], [-0.1, 0.1])
      error_friction *= interp(CS.vEgo, [20.0, 30.0], [1.0, 0.3])
      
      # lateral acceleration feedforward
      ff = self.get_steer_feedforward(desired_lateral_accel, CS.vEgo) - ff_roll
      ff += friction_compensation
      ff += error_friction
      
      if self.use_nn_ff:
        # prepare input data for NNFF model
        future_speeds = [math.sqrt(interp(t, T_IDXS, model_data.velocity.x)**2 \
                                            + interp(t, T_IDXS, model_data.velocity.y)**2) \
                                              for t in self.nnff_future_times]
        future_curvatures = [interp(t, T_IDXS, lat_plan.curvatures) for t in self.nnff_future_times]
        
        roll = params.roll
        
        self.lat_accel_deque.append(desired_lateral_accel)
        self.roll_deque.append(roll)
        
        past_lateral_accels = [self.lat_accel_deque[min(len(self.lat_accel_deque)-1, i)] for i in self.history_frame_offsets]
        future_lateral_accels = [k * v**2 for k, v in zip(future_curvatures, future_speeds)]
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [interp(t, T_IDXS, model_data.orientation.x) + roll for t in self.nnff_future_times]
        
        nnff_input = [CS.vEgo, desired_lateral_accel, lookahead_lateral_jerk, roll] \
                    + past_lateral_accels + future_lateral_accels \
                    + past_rolls + future_rolls
        ff_nn = self.CI.get_ff_nn(nnff_input)
        ff_nn += error_friction
      else:
        ff_nn = 0.0
      
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
      pid_log.maxFutureLatAccel = max_future_lateral_accel
      pid_log.errorScaleFactor = self.error_scale_factor.x
      if self.use_nn_ff:
        pid_log.nnffInputVector = nnff_input
    pid_log.currentLateralAcceleration = actual_lateral_accel
    pid_log.currentLateralJerk = self.actual_lateral_jerk.x
      

    #TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
    
