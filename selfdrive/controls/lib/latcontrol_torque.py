import math
from selfdrive.controls.lib.pid import PIDController
from common.differentiator import Differentiator
from common.numpy_fast import interp, sign
from common.op_params import opParams
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.modeld.constants import T_IDXS
from cereal import log

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


def get_steer_feedforward(desired_lateral_accel, speed):
  return desired_lateral_accel
class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self._op_params = opParams(calling_function="latcontrol_torque.py")
    self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki,
                            k_d=CP.lateralTuning.torque.kd,
                            k_11 = 1.0, k_12 = 2.0, k_13 = 1.0, k_period=0.1,
                            k_f=CP.lateralTuning.torque.kf,
                            integral_period=self._op_params.get('TUNE_LAT_TRX_ki_period_s', force_update=True),
                            derivative_period=self._op_params.get('TUNE_LAT_TRX_kd_period_s', force_update=True),
                            pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
    self.friction = CP.lateralTuning.torque.friction
    self.get_steer_feedforward = CI.get_steer_feedforward_function_torque()
    self.get_friction = CI.get_steer_feedforward_function_torque_lat_jerk()
    self.roll_k = 0.55
    self.v_ego = 0.0
    self.lat_plan_look_ahead = 1.5
    self.lat_plan_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > self.lat_plan_look_ahead), None)
    self.tune_override = self._op_params.get('TUNE_LAT_do_override', force_update=True)
    self.low_speed_factor_bp = [10.0, 25.0]
    self.low_speed_factor_v = [225.0, 50.0]
      
    # for actual lateral jerk calculation
    self.actual_lateral_jerk = Differentiator(self.pid.error_rate._d_period_s, 100.0)
  
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
    self.pid.update_i_period(self._op_params.get('TUNE_LAT_TRX_ki_period_s'))
    self.pid.update_d_period(self._op_params.get('TUNE_LAT_TRX_kd_period_s'))
    self.actual_lateral_jerk.update_period(self.pid.error_rate._d_period_s)
    self.pid.k_f = self._op_params.get('TUNE_LAT_TRX_kf')
    self.friction = self._op_params.get('TUNE_LAT_TRX_friction')
    self.roll_k = self._op_params.get('TUNE_LAT_TRX_roll_compensation')
    self.low_speed_factor_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_low_speed_factor_bp')]
    self.low_speed_factor_v = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_v')
    self.lat_plan_look_ahead = self._op_params.get('TUNE_LAT_TRX_friction_lookahead_s')
    self.lat_plan_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > self.lat_plan_look_ahead), None)
  
  def reset(self):
    super().reset()
    self.pid.reset()
    self.actual_lateral_jerk.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, mean_curvature=0.0, use_roll=True, lat_plan=None):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    self.v_ego = CS.vEgo

    if self.use_steering_angle:
      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll if use_roll else 0.0)
    else:
      actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
    actual_lateral_accel = actual_curvature * CS.vEgo**2
    self.actual_lateral_jerk.update(actual_lateral_accel)

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      min_planned_curvature_rate = min(list(lat_plan.curvatureRates)[LAT_PLAN_MIN_IDX:self.lat_plan_upper_idx] + [desired_curvature_rate], key=lambda x: abs(x))
      if sign(min_planned_curvature_rate) != sign(desired_curvature_rate):
        min_planned_curvature_rate = 0.0
      desired_lateral_jerk = min_planned_curvature_rate * CS.vEgo**2
      desired_lateral_accel = desired_curvature * CS.vEgo**2
      
      low_speed_factor = interp(CS.vEgo, self.low_speed_factor_bp, self.low_speed_factor_v)
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement
      pid_log.error = error
      
      # lateral jerk feedforward
      friction_compensation = self.get_friction(desired_lateral_jerk, self.v_ego, desired_lateral_accel, self.friction, FRICTION_THRESHOLD)
      
      # lateral acceleration feedforward
      ff_roll = math.sin(params.roll) * ACCELERATION_DUE_TO_GRAVITY
      ff = self.get_steer_feedforward(desired_lateral_accel, CS.vEgo) - ff_roll * (self.roll_k if use_roll else 0.0)
      ff += friction_compensation
      output_torque = self.pid.update(setpoint, measurement,
                                      override=CS.steeringPressed, feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=CS.steeringRateLimited)

      # record steering angle error to the unused pid_log.error_rate
      angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
      pid_log.steerAngleError = angle_steers_des - CS.steeringAngleDeg
      pid_log.errorRate = self.pid.error_rate.x

      pid_log.active = True
      pid_log.desiredLateralAcceleration = desired_lateral_accel
      pid_log.desiredLateralJerk = desired_lateral_jerk
      pid_log.friction = friction_compensation
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)
      pid_log.kp = self.pid.kp
      pid_log.ki = self.pid.ki
      pid_log.kd = self.pid.kd
      pid_log.gainUpdateFactor = self.pid._gain_update_factor
    pid_log.currentLateralAcceleration = actual_lateral_accel
    pid_log.currentLateralJerk = self.actual_lateral_jerk.x
      

    #TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
    
