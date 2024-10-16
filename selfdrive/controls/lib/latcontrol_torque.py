import math
import numpy as np
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
from selfdrive.controls.lib.drive_helpers import apply_deadzone, CONTROL_N
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.modeld.constants import T_IDXS
from cereal import log

ROLL_FF_CARS = [CAR.VOLT, CAR.VOLT18]

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [14, 10, 1, 0]

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

# At a given roll, if pitch magnitude increases, the
# gravitational acceleration component starts pointing
# in the longitudinal direction, decreasing the lateral
# acceleration component. Here we do the same thing
# to the roll value itself, then passed to nnff.
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)
  
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
    self.kp = CP.lateralTuning.torque.kp
    self.CI = CI
    self.use_nn_ff = Params().get_bool("EnableNNFF")
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
    
<<<<<<< HEAD
    self.kp_scale_bp = [0.0]
    self.kp_scale_v = [1.0]
=======
    self.kp_scale_bp = self._op_params.get('TUNE_LAT_TRX_kp_scale_bp', force_update=True)
    self.kp_scale_v = self._op_params.get('TUNE_LAT_TRX_kp_scale_v', force_update=True)
    self.a_ego = FirstOrderFilter(0.0, 0.1, DT_CTRL)
    self.pitch = FirstOrderFilter(0.0, 0.5, DT_CTRL)
>>>>>>> 570662e34 (scaled lat torque kp uses pitch-adjusted aego, and is filtered so that higher kp persists longer.)
    
    self.max_lat_accel = 3.5 # m/s^2
    self.error_downscale = 1.0
    self.error_downscale_LJ_factor = 0.7
    self.error_downscale_LA_factor = 1.0 - self.error_downscale_LJ_factor
    self.error_downscale_LJ_deadzone = 0.0
    self.error_downscale_denom = FirstOrderFilter(0.0, 0.5, DT_CTRL)
    self.error_downscale_bp = [7.0, 15.0] # m/s
    self.error_filtered = FirstOrderFilter(0.0, 1.0, DT_CTRL)
    self.error_downscale_error_factor = 1.0
    
    if self.use_nn_ff:
      # NNFF model takes current v_ego, lateral_accel, lat accel/jerk error, roll, and past/future/planned data
      # of lat accel and roll
      # Past value is computed using previous desired lat accel and observed roll
      self.torque_from_nn = CI.get_ff_nn
      
      # setup future time offsets
      self.nnff_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # seconds in the future
      self.nnff_future_times = [i + self.nnff_time_offset for i in future_times]
      self.nnff_future_times_np = np.array(self.nnff_future_times)
      
      # setup past time offsets
      self.past_times = [-0.3, -0.2, -0.1]
      history_check_frames = [int(abs(i)*100) for i in self.past_times]
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
      self.past_future_len = len(self.past_times) + len(self.nnff_future_times)

      # Setup adjustable parameters
      # Additionally, we use a deadzone to make sure that we only put additional torque
      # when the jerk is large enough to be significant.
      self.lat_jerk_deadzone = 0.0 # m/s^3 in [0, ∞] in 0.05 increments
      # Finally, lateral jerk error is downscaled so it doesn't dominate the friction error
      # term.
      self.lat_jerk_friction_factor = 0.4 # in [0, 1] in 0.01 increments

      # Scaling the lateral acceleration "friction response" could be helpful for some.
      # Increase for a stronger response, decrease for a weaker response.
      self.lat_accel_friction_factor = 0.7 # in [0, 5], in 0.05 increments. 5 is arbitrary safety limit
    
      
    # for actual lateral jerk calculation
    self.actual_lateral_jerk = Differentiator(0.1, 1/DT_MDL)
    self.lat_plan_last = None
  
  def update_op_params(self):
    if not self.tune_override:
      return
    self.use_steering_angle = self._op_params.get('TUNE_LAT_TRX_use_steering_angle')
    self.kp = self._op_params.get('TUNE_LAT_TRX_kp')
    self.pid._k_p = [[0], [self.kp]]
    self.pid._k_i = [[0], [self._op_params.get('TUNE_LAT_TRX_ki')]]
    self.pid._k_d = [[0], [self._op_params.get('TUNE_LAT_TRX_kd')]]
    self.pid._k_11 = [[0], [self._op_params.get('TUNE_LAT_TRX_kp_e')]]
    self.pid._k_12 = [[0], [self._op_params.get('TUNE_LAT_TRX_ki_e')]]
    self.pid._k_13 = [[0], [self._op_params.get('TUNE_LAT_TRX_kd_e')]]
    self.kp_scale_bp = self._op_params.get('TUNE_LAT_TRX_kp_scale_bp')
    self.kp_scale_v = self._op_params.get('TUNE_LAT_TRX_kp_scale_v')
    self.pid.k_f = self._op_params.get('TUNE_LAT_TRX_kf')
    self.friction = self._op_params.get('TUNE_LAT_TRX_friction')
    self.roll_k = self._op_params.get('TUNE_LAT_TRX_roll_compensation')
    self.low_speed_factor_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_low_speed_factor_bp')]
    self.low_speed_factor_v = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_v')
    look_ahead = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_lookahead_s')
    if look_ahead != self.low_speed_factor_look_ahead:
      self.low_speed_factor_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > self.low_speed_factor_look_ahead), None)
    look_ahead = self._op_params.get('TUNE_LAT_TRX_friction_lookahead_v')
    self.error_downscale = self._op_params.get('TUNE_LAT_TRX_error_downscale_in_curves')
    self.error_downscale_LJ_factor = self._op_params.get('TUNE_LAT_TRX_error_downscale_LJ_factor')
    self.error_downscale_LA_factor = 1.0 - self.error_downscale_LJ_factor
    self.error_downscale_LJ_deadzone = self._op_params.get('TUNE_LAT_TRX_error_downscale_LJ_deadzone')
    self.error_downscale_denom.update_alpha(self._op_params.get('TUNE_LAT_TRX_error_downscale_smoothing'))
    self.error_filtered.update_alpha(self._op_params.get('TUNE_LAT_TRX_error_downscale_error_smoothing'))
    self.error_downscale_error_factor = self._op_params.get('TUNE_LAT_TRX_error_downscale_error_factor')
    
    self.friction_look_ahead_v = self._op_params.get('TUNE_LAT_TRX_friction_lookahead_v')
    self.friction_look_ahead_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_friction_lookahead_bp')]
    self.friction_curve_exit_ramp_bp = self._op_params.get('TUNE_LAT_TRX_friction_curve_exit_ramp_bp')
    self.friction_curve_exit_ramp_v = [1.0, self._op_params.get('TUNE_LAT_TRX_friction_curve_exit_ramp_v')]
    
    self.lat_jerk_deadzone = self._op_params.get('TUNE_LAT_TRX_NNFF_lat_jerk_deadzone')
    self.lat_jerk_friction_factor = self._op_params.get('TUNE_LAT_TRX_NNFF_lat_jerk_friction_factor')
    self.lat_accel_friction_factor = self._op_params.get('TUNE_LAT_TRX_NNFF_lat_accel_friction_factor')
    
  
  def reset(self):
    super().reset()
    self.pid.reset()
    self.actual_lateral_jerk.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, use_roll=True, lat_plan=None, model_data=None):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    self.v_ego = CS.vEgo
    nnff_log = None

    if self.use_steering_angle:
      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll if use_roll else 0.0)
      actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.)
      self.actual_lateral_jerk._D.x = actual_curvature_rate * CS.vEgo**2
    else:
      actual_curvature = (llk.angularVelocityCalibrated.value[2] / max(0.1, CS.vEgo)) if len(llk.angularVelocityCalibrated.value) > 2 else 0.0
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

      if self.use_nn_ff:
        low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      else:
        low_speed_factor = interp(CS.vEgo, self.low_speed_factor_bp, self.low_speed_factor_v)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement
      
      # error downscaling
      self.error_filtered.update_alpha(interp(CS.vEgo, self.error_downscale_bp, [self.error_filtered.alpha, 0.1]))
      error_downscale_LA_component = self.error_downscale_LA_factor * abs(desired_lateral_accel)
      error_downscale_LJ_component = self.error_downscale_LJ_factor * abs(apply_deadzone(desired_lateral_jerk, self.error_downscale_LJ_deadzone))
      error_downscale_denom = interp(error_downscale_LA_component + error_downscale_LJ_component, [0.0, self.max_lat_accel], [1.0, self.error_downscale])
      # error_downscale_denom = interp(CS.vEgo, self.error_downscale_bp, [error_downscale_denom, 1.0])
      if error_downscale_denom > self.error_downscale_denom.x:
        self.error_downscale_denom.x = error_downscale_denom
      else:
        self.error_downscale_denom.update(error_downscale_denom)
      self.error_downscale_denom.x = interp(abs(self.error_filtered.update(error)), [0.0, self.error_downscale_error_factor * self.max_lat_accel], [self.error_downscale_denom.x, 1.0])
      error_scale_factor = 1.0 / self.error_downscale_denom.x

      lookahead_desired_curvature = get_lookahead_value(list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:self.low_speed_factor_upper_idx], desired_curvature)
      error *= error_scale_factor
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
      friction_v = 0.05
      error_friction = 0.0 if sign(error) != sign(desired_lateral_accel) and abs(desired_lateral_accel) > 1.5 else interp(error, [-ERR_FRICTION_THRESHOLD, ERR_FRICTION_THRESHOLD], [-friction_v, friction_v])
      
      # lateral acceleration feedforward
      ff = self.get_steer_feedforward(desired_lateral_accel, CS.vEgo) - ff_roll
      ff += friction_compensation
      ff += error_friction
      
      pitch = self.pitch.update((llk.calibratedOrientationNED.value[1]) if len(llk.calibratedOrientationNED.value) > 1 else 0.0)
      
      a_ego = (ACCELERATION_DUE_TO_GRAVITY * math.sin(self.pitch)) + CS.aEgo
      if abs(a_ego) > abs(self.a_ego):
        self.a_ego.x = a_ego
      else:
        self.a_ego.update(a_ego)
      self.pid._k_p = [[0], [self.kp * interp(self.a_ego, self.kp_scale_bp, self.kp_scale_v)]]
      
      model_planner_good = None not in [lat_plan, model_data] and all([len(i) >= CONTROL_N for i in [model_data.orientation.x, lat_plan.curvatures]])
      if self.use_nn_ff and model_planner_good:
        # update measurements with controls
        roll = params.roll
        roll = roll_pitch_adjust(roll, pitch)
        self.roll_deque.append(roll)
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        self.lateral_accel_desired_deque.append(desired_lateral_accel)
        
        adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nnff_future_times]
        future_rolls = [interp(t, T_IDXS, model_data.orientation.x) + roll for t in adjusted_future_times]

        # prepare past and future values
        # adjust future times to account for longitudinal acceleration
        adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nnff_future_times]
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [roll_pitch_adjust(interp(t, T_IDXS, model_data.orientation.x) + roll, interp(t, T_IDXS, model_data.orientation.y) + pitch) for t in adjusted_future_times]
        past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque)-1, i)] for i in self.history_frame_offsets]
        future_planned_lateral_accels = [interp(t, T_IDXS[:CONTROL_N], lat_plan.curvatures) * CS.vEgo ** 2 for t in adjusted_future_times]

        # compute NN error response.
        lookahead_lateral_jerk = apply_deadzone(lookahead_lateral_jerk, self.lat_jerk_deadzone)
        lat_accel_friction_factor = self.lat_accel_friction_factor
        if self.use_steering_angle or lookahead_lateral_jerk == 0.0:
          lookahead_lateral_jerk = 0.0
          self.actual_lateral_jerk._D.x = 0.0
          lat_accel_friction_factor = 1.0
        lateral_jerk_setpoint = self.lat_jerk_friction_factor * lookahead_lateral_jerk
        lateral_jerk_measurement = self.lat_jerk_friction_factor * self.actual_lateral_jerk.x

        # compute NNFF error response
        nnff_setpoint_input = [CS.vEgo, setpoint, lateral_jerk_setpoint, roll] \
                              + [setpoint] * self.past_future_len \
                              + past_rolls + future_rolls
        # past lateral accel error shouldn't count, so use past desired like the setpoint input
        nnff_measurement_input = [CS.vEgo, measurement, lateral_jerk_measurement, roll] \
                              + [measurement] * self.past_future_len \
                              + past_rolls + future_rolls
        nnff_error_input = [CS.vEgo, setpoint - measurement, lateral_jerk_setpoint - lateral_jerk_measurement, 0.0]
        torque_from_setpoint = self.torque_from_nn(nnff_setpoint_input)
        torque_from_measurement = self.torque_from_nn(nnff_measurement_input)
        
        # compute feedforward (same as nnff setpoint output)
        error = setpoint - measurement
        friction_input = lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        nnff_input = [CS.vEgo, desired_lateral_accel, friction_input, roll] \
                              + past_lateral_accels_desired + future_planned_lateral_accels \
                              + past_rolls + future_rolls
        ff_nn = self.torque_from_nn(nnff_input)
        error = torque_from_setpoint - torque_from_measurement
        error_blend_factor = interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0])
        if error_blend_factor > 0.0:
          torque_from_error = self.torque_from_nn(nnff_error_input)
          if sign(error) == sign(torque_from_error) and abs(error) < abs(torque_from_error):
            error = error * (1.0 - error_blend_factor) + torque_from_error * error_blend_factor
        error *= error_scale_factor
        nnff_log = nnff_input + nnff_setpoint_input + nnff_measurement_input
      else:
        ff_nn = ff
      
      
      
      output_torque = self.pid.update(setpoint, measurement,
                                      override=CS.steeringPressed, 
                                      feedforward=ff if ff_nn is None or not self.use_nn_ff else ff_nn,
                                      speed=CS.vEgo,
                                      freeze_integrator=CS.steeringRateLimited or abs(CS.steeringTorque) > 0.3 or CS.vEgo < 5,
                                      D=0.0 if self.use_nn_ff else lateral_jerk_error,
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
      pid_log.errorScaleFactor = error_scale_factor
      if nnff_log is not None:
        pid_log.nnffInputVector = nnff_log
    pid_log.currentLateralAcceleration = actual_lateral_accel
    pid_log.currentLateralJerk = self.actual_lateral_jerk.x
      

    #TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
    
