import math

from common.op_params import opParams
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.config import Conversions as CV
from cereal import log


class LatControlPID():
  def __init__(self, CP, CI):
    self._op_params = opParams(calling_function="latcontrol_pid.py")
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             (CP.lateralTuning.pid.kdBP, CP.lateralTuning.pid.kdV),
                             k_11 = 0.5, k_12 = 1., k_13 = 2., k_period=0.1,
                             k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, neg_limit=-1.0,
                             sat_limit=CP.steerLimitTimer,
                             derivative_period=0.1)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.roll_k = 1.0
    self.tune_override = self._op_params.get('TUNE_LAT_do_override', force_update=True)

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

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate, llk = None, use_roll=True, lat_plan=None):
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

      deadzone = 0.0

      check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
      output_steer = self.pid.update(angle_steers_des, CS.steeringAngleDeg, check_saturation=check_saturation, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)
      pid_log.kp = self.pid.kp
      pid_log.ki = self.pid.ki
      pid_log.kd = self.pid.kd

    return output_steer, angle_steers_des, pid_log
