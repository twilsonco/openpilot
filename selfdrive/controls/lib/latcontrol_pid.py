import math

from collections import deque
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, sign
from common.op_params import opParams
from common.realtime import DT_CTRL
from selfdrive.car.gm.values import CAR
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import get_steer_max, apply_deadzone
from selfdrive.config import Conversions as CV
from selfdrive.modeld.constants import T_IDXS
from common.params import Params
from cereal import log

LAT_PLAN_MIN_IDX = 5

ERR_FRICTION_THRESHOLD = 0.3

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

class LatControlPID():
  def __init__(self, CP, CI):
    self._op_params = opParams(calling_function="latcontrol_pid.py")
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             (CP.lateralTuning.pid.kdBP, CP.lateralTuning.pid.kdV),
                             k_11 = 0.5, k_12 = 1., k_13 = 2., k_period=0.1,
                             k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, neg_limit=-1.0,
                             sat_limit=CP.steerLimitTimer,
                             rate=20, derivative_period=0.1)
    self.CI = CI
    self.use_nn_ff = Params().get_bool("EnableNNFF")
    self.CI.initialize_feedforward_function_nn()
    self.use_nn_ff = self.use_nn_ff and self.CI.ff_nn_model is not None
    self.look_ahead_v = [0.6, 0.9]
    self.look_ahead_bp = [9.0, 35.0]
    self.error_scale_recip = 2.0
    self.error_scale_factor = FirstOrderFilter(1.0, 0.5, DT_CTRL)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.roll_k = 1.0
    self.tune_override = self._op_params.get('TUNE_LAT_do_override', force_update=True)
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
      history_check_frames = [6] # 0.3 seconds ago @ 20Hz
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.steer_angle_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
      self.past_future_len = len(history_check_frames) + len(self.nnff_future_times)
      
      # Finally, steer rate is downscaled so it doesn't dominate the friction error
      # term.
      self.steer_rate_friction_factor = 0.4 # in [0, 1] in 0.01 increments

      # Scaling the lateral acceleration "friction response" could be helpful for some.
      # Increase for a stronger response, decrease for a weaker response.
      self.steer_angle_error_friction_factor = 0.7 # in [0, 5], in 0.05 increments. 5 is arbitrary safety limit

  def update_op_params(self):
    if not self.tune_override:
      return
    bp = [i * CV.MPH_TO_MS for i in self._op_params.get(f"TUNE_LAT_PID_bp_mph")]
    self.pid._k_p = [bp, self._op_params.get("TUNE_LAT_PID_kp")]
    self.pid._k_i = [bp, self._op_params.get("TUNE_LAT_PID_ki")]
    self.pid._k_d = [bp, self._op_params.get("TUNE_LAT_PID_kd")]
    self.pid._k_11 = [[0], [self._op_params.get('TUNE_LAT_PID_kp_e')]]
    self.pid._k_12 = [[0], [self._op_params.get('TUNE_LAT_PID_ki_e')]]
    self.pid._k_13 = [[0], [self._op_params.get('TUNE_LAT_PID_kd_e')]]
    self.pid.k_f = self._op_params.get('TUNE_LAT_PID_kf')
    self.roll_k = self._op_params.get('TUNE_LAT_PID_roll_compensation')

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, use_roll=True, lat_plan=None, model_data=None):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll * self.roll_k if use_roll else 0.0))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

    pid_log.angleError = angle_steers_des - CS.steeringAngleDeg
    if CS.vEgo < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      steers_max = get_steer_max(CP, CS.vEgo)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max

      # torque for steer angle
      steer_feedforward = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)

      # torque for steer rate. ~0 angle, steer rate ~= steer command.
      steer_rate_actual = CS.steeringRateDeg
      steer_rate_desired = math.degrees(VM.get_steer_from_curvature(-desired_curvature_rate, CS.vEgo, 0))
      speed_mph =  CS.vEgo * CV.MS_TO_MPH
      steer_rate_max = 0.0389837 * speed_mph**2 - 5.34858 * speed_mph + 223.831

      steer_feedforward += ((steer_rate_desired - steer_rate_actual) / steer_rate_max)
      
      error = None
      
      if self.use_nn_ff:
        # prepare input data for NNFF model
        if len(lat_plan.curvatureRates) > 0 and len(model_data.velocity.x) > 0:
          lookahead = interp(CS.vEgo, self.look_ahead_bp, self.look_ahead_v)
          friction_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > lookahead), 16)
          lookahead_curvature_rate = get_lookahead_value(list(lat_plan.curvatureRates)[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_curvature_rate)
          future_speeds = [math.sqrt(interp(t, T_IDXS, model_data.velocity.x)**2 \
                                              + interp(t, T_IDXS, model_data.velocity.y)**2) \
                                                for t in self.nnff_future_times]
          future_curvatures = [interp(t, T_IDXS, lat_plan.curvatures) for t in self.nnff_future_times]
          # max_future_lateral_accel = max([i * CS.vEgo**2 for i in list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:16]] + [desired_curvature], key=lambda x: abs(x))
          # lookahead_lateral_jerk = lookahead_curvature_rate * CS.vEgo**2
          # error_scale_factor = 1.0 / (1.0 + min(apply_deadzone(abs(lookahead_lateral_jerk), 0.3) * self.error_scale_recip, self.error_scale_recip - 1))
          # if error_scale_factor < self.error_scale_factor.x:
          #   self.error_scale_factor.x = error_scale_factor
          # else:
          #   self.error_scale_factor.update(error_scale_factor)
          # pid_log.angleError *= self.error_scale_factor.x
          # angle_steers_des = CS.steeringAngleDeg + pid_log.angleError
          
          # set to same as torque controller, since using NN output torque for error rather than steer angle
          self.pid._k_p = [[0], [1]]
          self.pid._k_i = [[0], [0.15]]
          self.pid._k_d = [[0], [0.04]]
        else:
          lookahead_curvature_rate = 0.0
          future_speeds = [CS.vEgo] * len(self.nnff_future_times)
          future_curvatures = [desired_curvature] * len(self.nnff_future_times)
        
        steer_rate_desired_lookahead = math.degrees(VM.get_steer_from_curvature(-lookahead_curvature_rate, CS.vEgo, 0))
        
        roll = params.roll
        
        self.steer_angle_deque.append(angle_steers_des)
        self.roll_deque.append(roll)
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [interp(t, T_IDXS, model_data.orientation.x) + roll for t in self.nnff_future_times]
        
        past_steer_angles = [self.steer_angle_deque[min(len(self.steer_angle_deque)-1, i)] for i in self.history_frame_offsets]
        future_steer_angles = [math.degrees(VM.get_steer_from_curvature(-k, v, r * self.roll_k if use_roll else 0.0)) for k, v, r in zip(future_curvatures, future_speeds, future_rolls)]
        
        steer_angle_error_friction_factor = self.steer_angle_error_friction_factor
        if steer_rate_desired_lookahead == 0.0:
          steer_rate_actual = 0.0
          steer_angle_error_friction_factor = 1.0
        
        # compute NNFF error response
        nnff_setpoint_input = [CS.vEgo, angle_steers_des, steer_rate_desired_lookahead, roll] \
                              + [angle_steers_des] * self.past_future_len \
                              + past_rolls + future_rolls
        # past lateral accel error shouldn't count, so use past desired like the setpoint input
        nnff_measurement_input = [CS.vEgo, CS.steeringAngleDeg, steer_rate_actual, roll] \
                              + [CS.steeringAngleDeg] * self.past_future_len \
                              + past_rolls + future_rolls
        nnff_error_input = [CS.vEgo, angle_steers_des - CS.steeringAngleDeg, steer_rate_desired_lookahead - steer_rate_actual, 0.0]
        torque_from_setpoint = self.CI.get_ff_nn(nnff_setpoint_input)
        torque_from_measurement = self.CI.get_ff_nn(nnff_measurement_input)
        
        error = torque_from_setpoint - torque_from_measurement
        
        desired_lateral_accel = desired_curvature * CS.vEgo**2
        error_blend_factor = interp(abs(desired_lateral_accel), [1.0, 2.0], [0.0, 1.0])
        if error_blend_factor > 0.0:
          torque_from_error = self.CI.get_ff_nn(nnff_error_input)
          if sign(error) == sign(torque_from_error) and abs(error) < abs(torque_from_error):
            error = error * (1.0 - error_blend_factor) + torque_from_error * error_blend_factor
        
        friction_input = pid_log.angleError * steer_angle_error_friction_factor + steer_rate_desired_lookahead * self.steer_rate_friction_factor
        nnff_input = [CS.vEgo, angle_steers_des_no_offset, friction_input, roll] \
                    + past_steer_angles + future_steer_angles \
                    + past_rolls + future_rolls
        ff_nn = self.CI.get_ff_nn(nnff_input)
        
      else:
        ff_nn = 0.0

      check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
      output_steer = self.pid.update(angle_steers_des, CS.steeringAngleDeg, 
                                     check_saturation=check_saturation, 
                                     override=CS.steeringPressed,
                                     feedforward=steer_feedforward if (ff_nn is None or not self.use_nn_ff) else ff_nn,
                                     D=steer_rate_desired - steer_rate_actual if (ff_nn is None or not self.use_nn_ff) else None,
                                     speed=CS.vEgo,
                                     error=error)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = steer_feedforward * self.pid.k_f
      if self.use_nn_ff:
        pid_log.f2 = ff_nn * self.pid.k_f
        pid_log.nnffInput = nnff_input
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)
      pid_log.kp = self.pid.kp
      pid_log.ki = self.pid.ki
      pid_log.kd = self.pid.kd

    return output_steer, angle_steers_des, pid_log
