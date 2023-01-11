import math
from numpy import sign
from selfdrive.controls.lib.pid import PIDController
from common.numpy_fast import interp
from common.op_params import opParams
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
    self.pid = PIDController(CP.lateralTuning.torque.kp, CP.lateralTuning.torque.ki,
                            k_d=CP.lateralTuning.torque.kd, derivative_period=0.1,
                            k_11 = 0.5, k_12 = 0.5, k_13 = 0.5, k_period=0.1,
                            k_f=CP.lateralTuning.torque.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.use_steering_angle = CP.lateralTuning.torque.useSteeringAngle
    self.friction = CP.lateralTuning.torque.friction
    self.get_steer_feedforward = CI.get_steer_feedforward_function_torque()
    self._op_params = opParams(calling_function="latcontrol_torque.py")
    self.roll_k = 0.5
    self.tune_override = self._op_params.get('TUNE_LAT_do_override', force_update=True)
    if self.tune_override:
      self.low_speed_factor_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_low_speed_factor_bp', force_update=True)]
      self.low_speed_factor_v = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_v', force_update=True)
    else:
      self.low_speed_factor_bp = [10.0, 25.0]
      self.low_speed_factor_v = [180.0, 50.0]
  
  def update_op_params(self):
    if not self.tune_override:
      return
    self.use_steering_angle = self._op_params.get('TUNE_LAT_TRX_use_steering_angle')
    self.pid._k_p = [[0], [self._op_params.get('TUNE_LAT_TRX_kp')]]
    self.pid._k_i = [[0], [self._op_params.get('TUNE_LAT_TRX_ki')]]
    self.pid._k_d = [[0], [self._op_params.get('TUNE_LAT_TRX_kd')]]
    self.pid.k_f = self._op_params.get('TUNE_LAT_TRX_kf')
    self.friction = self._op_params.get('TUNE_LAT_TRX_friction')
    self.roll_k = self._op_params.get('TUNE_LAT_TRX_roll_compensation')
    self.low_speed_factor_bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LAT_TRX_low_speed_factor_bp')]
    self.low_speed_factor_v = self._op_params.get('TUNE_LAT_TRX_low_speed_factor_v')

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, mean_curvature=0.0):
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_torque = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      else:
        actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
      desired_lateral_jerk = desired_curvature_rate * CS.vEgo**2
      desired_lateral_accel = desired_curvature * CS.vEgo**2
      actual_lateral_accel = actual_curvature * CS.vEgo**2

      low_speed_factor = interp(CS.vEgo, self.low_speed_factor_bp, self.low_speed_factor_v)
      setpoint = float(desired_lateral_accel + low_speed_factor * min(abs(desired_curvature), abs(mean_curvature)) * sign(desired_curvature))
      measurement = float(actual_lateral_accel + low_speed_factor * min(abs(actual_curvature), abs(mean_curvature)) * sign(actual_curvature))
      error = setpoint - measurement
      pid_log.error = error
      
      ff_roll = math.sin(params.roll) * ACCELERATION_DUE_TO_GRAVITY
      ff = self.get_steer_feedforward(desired_lateral_accel, CS.vEgo) - ff_roll * self.roll_k
      friction_compensation = interp(desired_lateral_jerk, 
                                     [-FRICTION_THRESHOLD, FRICTION_THRESHOLD], 
                                     [-self.friction, self.friction])
      ff += friction_compensation
      output_torque = self.pid.update(setpoint, measurement,
                                      override=CS.steeringPressed, feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=CS.steeringRateLimited)

      # record steering angle error to the unused pid_log.error_rate
      angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
      pid_log.errorRate = angle_steers_des - CS.steeringAngleDeg

      pid_log.active = True
      pid_log.currentLateralAcceleration = actual_lateral_accel
      pid_log.desiredLateralAcceleration = desired_lateral_accel
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS)
      pid_log.kp = self.pid.kp
      pid_log.ki = self.pid.ki
      pid_log.kd = self.pid.kd

    #TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
    
