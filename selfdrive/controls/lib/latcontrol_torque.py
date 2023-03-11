import math
from selfdrive.controls.lib.pid import PIDController
from common.differentiator import Differentiator
from common.filter_simple import FirstOrderFilter
from common.integrator import Integrator
from common.numpy_fast import interp, sign
from common.op_params import opParams
from common.realtime import DT_CTRL
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
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


def get_steer_feedforward(desired_lateral_accel, speed):
  return desired_lateral_accel
class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self._op_params = opParams(calling_function="latcontrol_torque.py")
    self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki,
                            k_d=CP.lateralTuning.torque.kd,
                            k_11 = 0.5, k_12 = 0.5, k_13 = 0.5, k_period=0.1,
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
    self.tune_override = self._op_params.get('TUNE_LAT_do_override', force_update=True)
    if self.tune_override:
      self.low_speed_factor_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_low_speed_factor_bp', force_update=True)]
      self.low_speed_factor_v = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_v', force_update=True)
    else:
      self.low_speed_factor_bp = [10.0, 25.0]
      self.low_speed_factor_v = [225.0, 50.0]
    self.friction_compensation = FirstOrderFilter(0., 0.0, DT_CTRL, rate_up=0.6 * DT_CTRL, rate_down=0.8*DT_CTRL)
    self.friction_integral = Integrator(1.0, 100)
    self.friction_factor = 1.0
      
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
    self.friction_compensation.rate_up = self._op_params.get('TUNE_LAT_TRX_friction_rate_up')
    self.friction_compensation.rate_down = self._op_params.get('TUNE_LAT_TRX_friction_rate_down')
    self.friction_integral.update_period(self._op_params.get('TUNE_LAT_TRX_friction_integral_period_s'))
    self.friction_max_time = self._op_params.get('TUNE_LAT_TRX_friction_max_time_s')
    self.friction_decay_factor = self._op_params.get('TUNE_LAT_TRX_friction_ramp_factor')
    self.friction_growth_factor = 1.0 / self.friction_decay_factor
    self.pid.k_f = self._op_params.get('TUNE_LAT_TRX_kf')
    self.friction = self._op_params.get('TUNE_LAT_TRX_friction')
    self.roll_k = self._op_params.get('TUNE_LAT_TRX_roll_compensation')
    self.low_speed_factor_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_low_speed_factor_bp')]
    self.low_speed_factor_v = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_v')
    self.friction_alpha = [self._op_params.get('TUNE_LAT_TRX_friction_smoothing_factor_bp'), self._op_params.get('TUNE_LAT_TRX_friction_smoothing_factor_v')]
  
  def reset(self):
    super().reset()
    self.pid.reset()
    self.actual_lateral_jerk.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, mean_curvature=0.0, use_roll=True):
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
      desired_lateral_jerk = desired_curvature_rate * CS.vEgo**2
      desired_lateral_accel = desired_curvature * CS.vEgo**2
      low_speed_factor = interp(CS.vEgo, self.low_speed_factor_bp, self.low_speed_factor_v)
      setpoint = desired_lateral_accel + low_speed_factor * min(abs(desired_curvature), abs(mean_curvature)) * sign(desired_curvature)
      measurement = actual_lateral_accel + low_speed_factor * min(abs(actual_curvature), abs(mean_curvature)) * sign(actual_curvature)
      error = setpoint - measurement
      pid_log.error = error
      
      # lateral jerk feedforward
      self.friction_compensation.update_alpha(interp(abs(desired_lateral_accel), self.friction_alpha[0], self.friction_alpha[1]))
      self.friction_compensation.update(self.get_friction(desired_lateral_jerk, self.v_ego, desired_lateral_accel, self.friction, FRICTION_THRESHOLD))
      self.friction_integral.update(self.friction_compensation.x)
      friction_integral_cap = self.friction_max_time * abs(self.get_friction(max(0.15, abs(desired_lateral_jerk)), self.v_ego, desired_lateral_accel, self.friction, FRICTION_THRESHOLD))
      if abs(self.friction_integral.x) >= friction_integral_cap:
        self.friction_factor *= self.friction_decay_factor
      elif self.friction_factor < 1.0:
        self.friction_factor = min(1.0, self.friction_factor * self.friction_growth_factor)
      friction_compensation = self.friction_compensation.x * self.friction_factor

      
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
      pid_log.frictionFactor = self.friction_factor
      pid_log.frictionIntegral = self.friction_integral.x
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
    
