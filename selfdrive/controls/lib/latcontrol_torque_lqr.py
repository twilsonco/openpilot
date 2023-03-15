import math
import numpy as np

from common.numpy_fast import clip, interp
from common.op_params import opParams
from common.realtime import DT_CTRL
from cereal import log
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.controls.lib.latcontrol_torque import FRICTION_THRESHOLD
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY

class LatControlTorqueLQR():
  def __init__(self, CP, CI):
    self.scale = CP.lateralTuning.torqueLqr.scale
    self.ki = CP.lateralTuning.torqueLqr.ki

    self.A = np.array(CP.lateralTuning.torqueLqr.a).reshape((2, 2))
    self.B = np.array(CP.lateralTuning.torqueLqr.b).reshape((2, 1))
    self.C = np.array(CP.lateralTuning.torqueLqr.c).reshape((1, 2))
    self.K = np.array(CP.lateralTuning.torqueLqr.k).reshape((1, 2))
    self.L = np.array(CP.lateralTuning.torqueLqr.l).reshape((2, 1))
    self.dc_gain = CP.lateralTuning.torqueLqr.dcGain
    self.get_steer_feedforward = CI.get_steer_feedforward_function_torque()
    self.friction = CP.lateralTuning.torqueLqr.friction
    self.use_steering_angle = CP.lateralTuning.torqueLqr.useSteeringAngle
    self._k_f = CP.lateralTuning.torqueLqr.kf

    self.x_hat = np.array([[0], [0]])
    self.i_unwind_rate = 0.3 * DT_CTRL
    self.i_rate = 1.0 * DT_CTRL

    self.sat_count_rate = 1.0 * DT_CTRL
    self.sat_limit = CP.steerLimitTimer
    
    self._op_params = opParams(calling_function="latcontrol_torque_lqr.py")
    self.roll_k = 0.0

    self.reset()
  
  def update_op_params(self):
    self.scale = self._op_params.get('TUNE_LAT_TRXLQR_scale')
    self.ki = self._op_params.get('TUNE_LAT_TRXLQR_ki')
    self.dc_gain = self._op_params.get('TUNE_LAT_TRXLQR_dc_gain')
    a = self._op_params.get('TUNE_LAT_TRXLQR_a')
    b = self._op_params.get('TUNE_LAT_TRXLQR_b')
    c = self._op_params.get('TUNE_LAT_TRXLQR_c')
    k = self._op_params.get('TUNE_LAT_TRXLQR_k')
    l = self._op_params.get('TUNE_LAT_TRXLQR_l')
    self.A = np.array(a).reshape((2, 2))
    self.B = np.array(b).reshape((2, 1))
    self.C = np.array(c).reshape((1, 2))
    self.K = np.array(k).reshape((1, 2))
    self.L = np.array(l).reshape((2, 1))
    self.roll_k = self._op_params.get('TUNE_LAT_TRXLQR_roll_compensation')
    self.friction = self._op_params.get('TUNE_LAT_TRXLQR_friction')
    self._k_f = self._op_params.get('TUNE_LAT_TRXLQR_kf')
    self.use_steering_angle = self._op_params.get('TUNE_LAT_TRXLQR_use_steering_angle')

  def reset(self):
    self.i_lqr = 0.0
    self.sat_count = 0.0

  def _check_saturation(self, control, check_saturation, limit):
    saturated = abs(control) == limit

    if saturated and check_saturation:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, mean_curvature=0.0, use_roll=True, lat_plan=None):
    lqr_log = log.ControlsState.LateralTorqueLQRState.new_message()

    if self.use_steering_angle:
      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll if use_roll else 0.0)
      actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0)
    else:
      actual_curvature = llk.angularVelocityCalibrated.value[2] / CS.vEgo
    
    desired_lateral_jerk = desired_curvature_rate * CS.vEgo**2
    desired_lateral_accel = desired_curvature * CS.vEgo**2
    actual_lateral_accel = actual_curvature * CS.vEgo**2
    actual_lateral_jerk = actual_curvature_rate * CS.vEgo**2
    
    low_speed_factor = interp(CS.vEgo, [10., 25.], [80., 50.])
    sp_off = low_speed_factor * desired_curvature
    measure_off = low_speed_factor * actual_curvature
    
    steers_max = get_steer_max(CP, CS.vEgo)
    torque_scale = (0.45 + CS.vEgo / 60.0)**2  # Scale actuator model with speed

    # Update Kalman filter
    lat_accel_k = float(self.C.dot(self.x_hat))
    e = actual_lateral_accel - lat_accel_k
    self.x_hat = self.A.dot(self.x_hat) + self.B.dot(CS.steeringTorqueEps / torque_scale) + self.L.dot(e)

    if CS.vEgo < 0.3 or not active:
      lqr_log.active = False
      lqr_output = 0.
      output_steer = 0.
      self.reset()
    else:
      lqr_log.active = True

      # LQR
      u_lqr = float(desired_lateral_accel / self.dc_gain - self.K.dot(self.x_hat))
      lqr_output = torque_scale * u_lqr / self.scale

      # Integrator
      if CS.steeringPressed:
        self.i_lqr -= self.i_unwind_rate * float(np.sign(self.i_lqr))
      else:
        error = (desired_lateral_accel + sp_off) - (lat_accel_k + measure_off)
        i = self.i_lqr + self.ki * self.i_rate * error
        control = lqr_output + i

        if (error >= 0 and (control <= steers_max or i < 0.0)) or \
           (error <= 0 and (control >= -steers_max or i > 0.0)):
          self.i_lqr = i

      ff_roll = -math.sin(params.roll) * ACCELERATION_DUE_TO_GRAVITY * self.roll_k if use_roll else 0.0
      ff = self.get_steer_feedforward(desired_lateral_accel, CS.vEgo)
      friction_compensation = interp(desired_lateral_jerk, 
                                     [-FRICTION_THRESHOLD, FRICTION_THRESHOLD], 
                                     [-self.friction, self.friction])
      ff *= self._k_f
      lqr_log.f = ff
      lqr_log.friction = friction_compensation
      lqr_log.rollCompensation = ff_roll
      ff += friction_compensation
      ff += ff_roll
      
      output_steer = lqr_output + self.i_lqr + ff
      output_steer = clip(output_steer, -steers_max, steers_max)

    check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
    saturated = self._check_saturation(output_steer, check_saturation, steers_max)

    offset_curvature = -VM.calc_curvature(math.radians(params.angleOffsetAverageDeg), CS.vEgo, 0)
    offset_lat_accel = offset_curvature * CS.vEgo**2
    lqr_log.lateralAcceleration = lat_accel_k + offset_lat_accel
    lqr_log.i = self.i_lqr
    lqr_log.output = output_steer
    lqr_log.lqrOutput = lqr_output
    lqr_log.saturated = saturated
    return -output_steer, desired_lateral_accel, lqr_log
