import math

from cereal import log
from common.numpy_fast import interp, sign
from common.params import Params
from selfdrive.controls.lib.drive_helpers import apply_deadzone
from selfdrive.controls.lib.latcontrol import LatControl
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.modeld.constants import T_IDXS

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]

LAT_PLAN_MIN_IDX = 5
def get_lookahead_value(future_vals, current_val):
  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]
  
  # if any future val has opposite sign of current val, return 0
  if len(same_sign_vals) < len(future_vals):
    return 0.0
  
  # otherwise return the value with minimum absolute value
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val

class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki, k_d=0.03,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.use_nn = CI.initialize_ff_nn(CP.carFingerprint)
    if self.use_nn:
      self.torque_from_nn = CI.get_ff_nn
    self.friction_look_ahead_v = [1.0, 2.0]
    self.friction_look_ahead_bp = [9.0, 35.0]
    self.error_scale_lat_accel = 0.0
    self.error_scale_lat_accel_decay_factor = 0.99
    self.error_scale_lat_accel_counter = 0
    self.error_scale_lat_accel_decay_delay = 50 # wait 50 frames (1s) before beginning to scale error back up for better centering

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update_live_tune(self):
    if not self.custom_torque:
      return
    self._frame += 1
    if self._frame % 300 == 0:
      self.torque_params.latAccelFactor = float(self.param_s.get("TorqueMaxLatAccel", encoding="utf8")) * 0.01
      self.torque_params.friction = float(self.param_s.get("TorqueFriction", encoding="utf8")) * 0.01
      self._frame = 0

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk, lat_plan=None):
    self.update_live_tune()
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
        actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      if lat_plan is not None:
        lookahead = interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
        friction_upper_idx = next((i for i, val in enumerate(T_IDXS) if val > lookahead), 16)
        lookahead_curvature_rate = get_lookahead_value(list(lat_plan.curvatureRates)[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_curvature_rate)
        desired_lateral_jerk = lookahead_curvature_rate * CS.vEgo ** 2
      else:
        desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      
      
      max_future_lateral_accel = max([i * CS.vEgo**2 for i in list(lat_plan.curvatures)[LAT_PLAN_MIN_IDX:16]] + [desired_lateral_accel], key=lambda x: abs(x))
      if abs(max_future_lateral_accel) > abs(self.error_scale_lat_accel):
        self.error_scale_lat_accel = max_future_lateral_accel
        self.error_scale_lat_accel_counter = 0
      else:
        self.error_scale_lat_accel_counter += 1
        if self.error_scale_lat_accel_counter > self.error_scale_lat_accel_decay_delay:
          self.error_scale_lat_accel *= self.error_scale_lat_accel_decay_factor

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      error = setpoint - measurement
      error_scale_factor = 3.0
      error_scale_factor = 1.0 / (1.0 + min(apply_deadzone(abs(self.error_scale_lat_accel), 0.4) * error_scale_factor, error_scale_factor - 1))
      error *= error_scale_factor
      gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params, setpoint,
                                                     lateral_accel_deadzone, friction_compensation=False)
      torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params, measurement,
                                                     lateral_accel_deadzone, friction_compensation=False)
      pid_log.error = torque_from_setpoint - torque_from_measurement
      ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
                                          desired_lateral_accel - actual_lateral_accel,
                                          lateral_accel_deadzone, friction_compensation=True) if not self.use_nn \
          else self.torque_from_nn(CS.vEgo, desired_lateral_accel, desired_lateral_jerk, params.roll)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      error_rate=(desired_lateral_jerk - actual_lateral_jerk) if self.use_steering_angle else 0.0,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
