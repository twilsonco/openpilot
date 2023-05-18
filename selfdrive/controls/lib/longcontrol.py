from cereal import car
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import clip, interp
from common.op_params import opParams
from common.realtime import sec_since_boot, DT_CTRL
from selfdrive.car.gm.carstate import GAS_PRESSED_THRESHOLD
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.modeld.constants import T_IDXS

BRAKE_SOURCES = {'lead0',
                 'lead1',
                 'lead2',
                 'lead0p1'}

LongCtrlState = car.CarControl.Actuators.LongControlState

STOPPING_EGO_SPEED = 0.02
STOPPING_TARGET_SPEED_OFFSET = 0.01
STARTING_TARGET_SPEED = 0.5
DECEL_THRESHOLD_TO_PID = 0.8

DECEL_STOPPING_TARGET = 0.25  # apply at least this amount of brake to maintain the vehicle stationary

RATE = 100.0

# As NOT per ISO 15622:2018 for all speeds
ACCEL_MIN_ISO = -3.5 # m/s^2
ACCEL_MAX_ISO = 3.5 # m/s^2


# TODO this logic isn't really car independent, does not belong here
def long_control_state_trans(active, long_control_state, v_ego, v_target, v_target_future, v_pid,
                             output_accel, brake_pressed, cruise_standstill, min_speed_can, 
                             MADS_lead_braking_enabled=False, gas=0.0, gear_shifter='',
                             long_plan_source='', time_since_brake_press=100.0, brake_release_vego=0.0):
  """Update longitudinal control state machine"""
  accelerating = v_target_future > v_target
  stopping_condition = (v_ego < 2.0 and cruise_standstill) or \
                       (v_ego < STOPPING_EGO_SPEED and
                        ((v_target_future < STOPPING_EGO_SPEED and not accelerating) or
                         brake_pressed))

  starting_condition = v_target_future > STARTING_TARGET_SPEED and accelerating and not cruise_standstill

  if not active:
    if (time_since_brake_press > 3.0 or brake_release_vego > 2.0) \
        and gear_shifter in ['drive','low'] \
        and MADS_lead_braking_enabled \
        and gas < 1e-5 \
        and long_plan_source in BRAKE_SOURCES:
      long_control_state = LongCtrlState.pid
    else:
      long_control_state = LongCtrlState.off
  else:
    if long_control_state == LongCtrlState.off:
      if active:
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.pid:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition:
        long_control_state = LongCtrlState.starting

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif output_accel >= -DECEL_THRESHOLD_TO_PID:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl():
  def __init__(self, CP):
    self.long_control_state = LongCtrlState.off  # initialized to off
    self._op_params = opParams(calling_function="longcontrol.py")
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                            (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                            (CP.longitudinalTuning.kdBP, CP.longitudinalTuning.kdV),
                            derivative_period=0.1,
                            k_11 = 0.2, k_12 = 0.4, k_13 = 0.4, k_period=0.1,
                            rate=RATE,
                            sat_limit=0.8)
    self.v_pid = 0.0
    self.lead_present_last = False
    self.lead_gone_t = 0.
    self.last_gas_t = 0.
    self.active_last = False
    self.a_target = 0.0
    self.brake_pressed_last_t = sec_since_boot()
    self.brake_pressed_time_since = 0.0
    self.pos_accel_gas_smooth_k = 0.5
    self.pos_accel_lead_smooth_k = 0.5
    self.pos_accel_smooth_k = 0.0
    self.lead_gone_smooth_accel_time = 4.0
    self.gas_smooth_accel_time = 3.0
    smooth_max_speed = 9.0
    self.pos_accel_gas_smooth_speed_bp = [smooth_max_speed * 0.7, smooth_max_speed]
    self.pos_accel_smooth_min_speed = 1.0
    self.brake_release_vego = 0.0
    self.output_accel = FirstOrderFilter(0.0, 0.0, DT_CTRL)
    self.last_output_accel = 0.0
    self.tune_override = self._op_params.get('TUNE_LAT_do_override', force_update=True)
    self.deadzone_bp, self.deadzone_v = CP.longitudinalTuning.deadzoneBP, CP.longitudinalTuning.deadzoneV
  
  def update_op_params(self):
    self.pos_accel_gas_smooth_k = self._op_params.get('AP_positive_accel_post_resume_smoothing_factor')
    smooth_max_speed = self._op_params.get('AP_positive_accel_post_resume_smoothing_max_speed_mph') * CV.MPH_TO_MS
    self.pos_accel_gas_smooth_speed_bp = [smooth_max_speed * 0.7, smooth_max_speed]
    self.pos_accel_lead_smooth_k = self._op_params.get('AP_positive_accel_post_lead_smoothing_factor')
    self.pos_accel_smooth_k = self._op_params.get('AP_positive_accel_smoothing_factor')
    self.pos_accel_smooth_min_speed = self._op_params.get('AP_positive_accel_smoothing_min_speed_mph') * CV.MPH_TO_MS
    self.lead_gone_smooth_accel_time = self._op_params.get('AP_positive_accel_post_lead_smoothing_time_s')
    self.gas_smooth_accel_time = self._op_params.get('AP_positive_accel_post_resume_smoothing_time_s')
    if not self.tune_override:
      return
    bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LONG_speed_mph')]
    self.pid._k_p = [bp, self._op_params.get('TUNE_LONG_kp')]
    self.pid._k_i = [bp, self._op_params.get('TUNE_LONG_ki')]
    self.pid._k_d = [bp, self._op_params.get('TUNE_LONG_kd')]
    self.deadzone_bp, self.deadzone_v = bp, self._op_params.get('TUNE_LONG_deadzone_ms2')

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, CP, long_plan, accel_limits, t_since_plan, MADS_lead_braking_enabled=False):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target = interp(t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target = interp(t_since_plan, T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(CP.longitudinalActuatorDelayLowerBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target) / CP.longitudinalActuatorDelayLowerBound - a_target

      v_target_upper = interp(CP.longitudinalActuatorDelayUpperBound + t_since_plan, T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target) / CP.longitudinalActuatorDelayUpperBound - a_target
      a_target = min(a_target_lower, a_target_upper)

      v_target_future = speeds[-1]
    else:
      v_target = 0.0
      v_target_future = 0.0
      a_target = 0.0

    # TODO: This check is not complete and needs to be enforced by MPC
    a_target = clip(a_target, ACCEL_MIN_ISO, ACCEL_MAX_ISO)

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    # Update state machine
    output_accel = self.last_output_accel
    self.long_control_state = long_control_state_trans(active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_future, self.v_pid, output_accel,
                                                       CS.brakePressed, CS.cruiseState.standstill, CP.minSpeedCan, MADS_lead_braking_enabled=MADS_lead_braking_enabled,gas=CS.gas,
                                                       gear_shifter=CS.gearShifter,
                                                       long_plan_source=long_plan.longitudinalPlanSource,
                                                       time_since_brake_press=self.brake_pressed_time_since,
                                                       brake_release_vego=self.brake_release_vego)

    v_ego_pid = max(CS.vEgo, CP.minSpeedCan)  # Without this we get jumps, CAN bus reports 0 when speed < 0.3

    if self.long_control_state == LongCtrlState.off \
        or CS.gasPressed \
        or CS.brakePressed \
        or CS.gearShifter not in ['drive','low']:
      self.reset(v_ego_pid)
      output_accel = 0.
      
    # tracking objects and driving
    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target
      
      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      prevent_overshoot = not CP.stoppingControl and CS.vEgo < 1.5 and v_target_future < 0.7 and v_target_future < v_target
      deadzone = interp(CS.vEgo, self.deadzone_bp, self.deadzone_v)
      freeze_integrator = prevent_overshoot

      output_accel = self.pid.update(self.v_pid, v_ego_pid, speed=v_ego_pid, deadzone=deadzone, feedforward=a_target, freeze_integrator=freeze_integrator, override=CS.gas > 1e-5 or CS.brakePressed)

      if prevent_overshoot:
        output_accel = min(output_accel, 0.0)

    # Intention is to stop, switch to a different brake control until we stop
    elif self.long_control_state == LongCtrlState.stopping:
      # Keep applying brakes until the car is stopped
      if not CS.standstill or output_accel > -DECEL_STOPPING_TARGET:
        output_accel -= CP.stoppingDecelRate / RATE
      output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

      self.reset(CS.vEgo)

    # Intention is to move again, release brake fast before handing control to PID
    elif self.long_control_state == LongCtrlState.starting:
      if output_accel < -DECEL_THRESHOLD_TO_PID:
        output_accel += CP.startingAccelRate / RATE
      self.reset(CS.vEgo)
      
    t = sec_since_boot()
    if CS.brakePressed:
      self.brake_pressed_last_t = t
    elif CS.gas > 1e-5: # no delay after gas press
      self.brake_pressed_last_t = t - 10.0
    elif CS.gas > GAS_PRESSED_THRESHOLD or active and not self.active_last:
      self.last_gas_t = t
    self.active_last = active
    self.brake_pressed_time_since = t - self.brake_pressed_last_t
    if self.brake_pressed_time_since < 0.02:
      self.brake_release_vego = CS.vEgo
    lead_present = long_plan.leadDist > 0.
    if not lead_present and self.lead_present_last:
      self.lead_gone_t = t
    self.lead_present_last = lead_present
    
    time_since_lead = t - self.lead_gone_t
    time_since_gas = t - self.last_gas_t
    if CS.vEgo > self.pos_accel_smooth_min_speed and output_accel > self.last_output_accel:
      if not lead_present and time_since_lead < self.lead_gone_smooth_accel_time:
        self.output_accel.x = max(self.output_accel.x, CS.aEgo)
        smooth_factor = interp(time_since_lead, [self.lead_gone_smooth_accel_time * 0.5, self.lead_gone_smooth_accel_time], [self.pos_accel_lead_smooth_k, 0.0])
        self.output_accel.update_alpha(smooth_factor)
        self.output_accel.update(output_accel)
      elif time_since_gas < self.gas_smooth_accel_time:
        self.output_accel.x = max(self.output_accel.x, CS.aEgo)
        smooth_factor = interp(time_since_gas, [self.gas_smooth_accel_time * 0.5, self.gas_smooth_accel_time], [self.pos_accel_gas_smooth_k, 0.0])
        smooth_factor = interp(CS.vEgo, self.pos_accel_gas_smooth_speed_bp, [smooth_factor, 0.0])
        self.output_accel.update_alpha(smooth_factor)
        self.output_accel.update(output_accel)
      else:
        self.output_accel.update_alpha(self.pos_accel_smooth_k)
        self.output_accel.update(output_accel)
    else:
      self.output_accel.x = output_accel
    
    self.last_output_accel = self.output_accel.x
    final_accel = clip(self.output_accel.x, accel_limits[0], accel_limits[1])
    self.a_target = a_target

    return final_accel
