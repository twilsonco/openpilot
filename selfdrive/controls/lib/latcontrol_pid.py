import math

from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.controls.lib.latcontrol import LatControl, MIN_STEER_SPEED
from selfdrive.config import Conversions as CV
from cereal import log


class LatControlPID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             (CP.lateralTuning.pid.kdBP, CP.lateralTuning.pid.kdV),
                             k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, neg_limit=-1.0,
                             sat_limit=CP.steerLimitTimer, derivative_period=0.1)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, CP, VM, params, desired_curvature, desired_curvature_rate):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg

    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = angle_steers_des - CS.steeringAngleDeg
    if CS.vEgo < MIN_STEER_SPEED or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      steers_max = get_steer_max(CP, CS.vEgo)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max

      # offset does not contribute to resistive torque
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
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = self._check_saturation(steers_max - abs(output_steer) < 1e-3, CS)

    return output_steer, angle_steers_des, pid_log
