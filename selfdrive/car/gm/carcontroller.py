from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, AccState, CanBus, CarControllerParams
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.start_time = 0.
    self.apply_steer_last = 0
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False
    self.fcw_count = 0

    self.params = CarControllerParams()

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

    self.debug_logging = False
    self.debug_log_time_step = 0.333
    self.last_debug_log_t = 0.
    if self.debug_logging:
      with open("/data/openpilot/coast_debug.csv","w") as f:
        f.write(",".join([
          "t",
          "long plan",
          "d (m/s)", 
          "v", 
          "vEgo", 
          "v_cruise",
          "v (mph)", 
          "vEgo (mph)", 
          "v_cruise (mph)",
          "ttc", 
          "coast gas lockout", 
          "coast brake lockout", 
          "gas in", 
          "brake in", 
          "one-pedal",
          "coasting enabled",
          "no f brakes",
          "gas out",
          "brake out"]) + "\n")

  def update(self, enabled, CS, frame, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # Send CAN commands.
    can_sends = []

    # STEER
    if (frame % P.STEER_STEP) == 0:
      lkas_enabled = (enabled or CS.pause_long_on_gas_press) and CS.lkMode and not (CS.out.steerWarning or CS.out.steerError) and CS.out.vEgo > P.MIN_STEER_SPEED and CS.lane_change_steer_factor > 0.
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX * CS.lane_change_steer_factor))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      self.apply_steer_last = apply_steer
      idx = (frame // P.STEER_STEP) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    # Gas/regen prep
    if not enabled or CS.pause_long_on_gas_press:
      # Stock ECU sends max regen when not enabled.
      apply_gas = P.MAX_ACC_REGEN
      apply_brake = 0
    else:
      apply_gas = interp(actuators.accel, P.GAS_LOOKUP_BP, P.GAS_LOOKUP_V)
      apply_brake = interp(actuators.accel, P.BRAKE_LOOKUP_BP, P.BRAKE_LOOKUP_V)
      t = sec_since_boot()
      
      v_rel = CS.coasting_lead_v - CS.vEgo
      ttc = min(-CS.coasting_lead_d / v_rel if (CS.coasting_lead_d > 0. and v_rel < 0.) else 100.,100.)
      d_time = CS.coasting_lead_d / CS.vEgo if (CS.coasting_lead_d > 0. and CS.vEgo > 0. and CS.tr > 0.) else 10.
      
      if CS.coasting_lead_d > 0. and (ttc < CS.lead_ttc_long_gas_lockout_bp[-1] \
        or v_rel < CS.lead_v_rel_long_gas_lockout_bp[-1] \
        or CS.coasting_lead_v < CS.lead_v_long_gas_lockout_bp[-1] \
        or d_time < CS.tr * CS.lead_tr_long_gas_lockout_bp[-1]\
        or CS.coasting_lead_d < CS.lead_d_long_gas_lockout_bp[-1]):
        lead_long_gas_lockout_factor = max([
          interp(v_rel, CS.lead_v_rel_long_gas_lockout_bp, CS.lead_v_rel_long_gas_lockout_v), 
          interp(CS.coasting_lead_v, CS.lead_v_long_gas_lockout_bp, CS.lead_v_long_gas_lockout_v),
          interp(ttc, CS.lead_ttc_long_gas_lockout_bp, CS.lead_ttc_long_gas_lockout_v),
          interp(d_time / CS.tr, CS.lead_tr_long_gas_lockout_bp, CS.lead_tr_long_gas_lockout_v),
          interp(CS.coasting_lead_d, CS.lead_d_long_gas_lockout_bp, CS.lead_d_long_gas_lockout_v)])
          
        if CS.coasting_lead_d > 0. and (ttc < CS.lead_ttc_long_brake_lockout_bp[-1] \
          or v_rel < CS.lead_v_rel_long_brake_lockout_bp[-1] \
          or CS.coasting_lead_v < CS.lead_v_long_brake_lockout_bp[-1] \
          or d_time < CS.tr * CS.lead_tr_long_brake_lockout_bp[-1]\
          or CS.coasting_lead_d < CS.lead_d_long_brake_lockout_bp[-1]):
          lead_long_brake_lockout_factor = max([
            interp(v_rel, CS.lead_v_rel_long_brake_lockout_bp, CS.lead_v_rel_long_brake_lockout_v), 
            interp(CS.coasting_lead_v, CS.lead_v_long_brake_lockout_bp, CS.lead_v_long_brake_lockout_v),
            interp(ttc, CS.lead_ttc_long_brake_lockout_bp, CS.lead_ttc_long_brake_lockout_v),
            interp(d_time / CS.tr, CS.lead_tr_long_brake_lockout_bp, CS.lead_tr_long_brake_lockout_v),
            interp(CS.coasting_lead_d, CS.lead_d_long_brake_lockout_bp, CS.lead_d_long_brake_lockout_v)])
        else:
          lead_long_brake_lockout_factor =  0. # 1.0 means regular braking logic is completely unaltered, 0.0 means no cruise braking
      else:
        lead_long_gas_lockout_factor =  0. # 1.0 means regular braking logic is completely unaltered, 0.0 means no cruise braking
        lead_long_brake_lockout_factor =  0. # 1.0 means regular braking logic is completely unaltered, 0.0 means no cruise braking
        
      # debug logging
      do_log = self.debug_logging and (t - self.last_debug_log_t > self.debug_log_time_step)
      if do_log:
        self.last_debug_log_t = t
        f = open("/data/openpilot/coast_debug.csv","a")
        f.write(",".join([f"{i:.1f}" if i == float else str(i) for i in [
          t - CS.sessionInitTime,
          CS.coasting_long_plan,
          CS.coasting_lead_d, 
          CS.coasting_lead_v, 
          CS.vEgo, 
          CS.v_cruise_kph * CV.KPH_TO_MS,
          CS.coasting_lead_v * CV.MS_TO_MPH, 
          CS.vEgo * CV.MS_TO_MPH, 
          CS.v_cruise_kph * CV.KPH_TO_MPH,
          ttc, 
          lead_long_gas_lockout_factor, 
          lead_long_brake_lockout_factor,
          int(apply_gas), 
          int(apply_brake), 
          (CS.one_pedal_mode_active or CS.coast_one_pedal_mode_active),
          CS.coasting_enabled,
          CS.no_friction_braking]]) + ",")
      
      if (CS.one_pedal_mode_active or CS.coast_one_pedal_mode_active):
        apply_gas = apply_gas * lead_long_gas_lockout_factor + float(P.MAX_ACC_REGEN) * (1. - lead_long_gas_lockout_factor)
        if CS.one_pedal_mode_active:
          if abs(CS.angle_steers) > CS.one_pedal_angle_steers_cutoff_bp[0]:
            one_pedal_apply_brake = interp(CS.vEgo, CS.one_pedal_mode_stop_apply_brake_bp[CS.one_pedal_brake_mode], CS.one_pedal_mode_stop_apply_brake_v[CS.one_pedal_brake_mode])
            one_pedal_apply_brake_minus1 = interp(CS.vEgo, CS.one_pedal_mode_stop_apply_brake_bp[max(0,CS.one_pedal_brake_mode-1)], CS.one_pedal_mode_stop_apply_brake_v[max(0,CS.one_pedal_brake_mode-1)])
            one_pedal_apply_brake = interp(abs(CS.angle_steers), CS.one_pedal_angle_steers_cutoff_bp, [one_pedal_apply_brake, one_pedal_apply_brake_minus1])
          else:
            one_pedal_apply_brake = interp(CS.vEgo, CS.one_pedal_mode_stop_apply_brake_bp[CS.one_pedal_brake_mode], CS.one_pedal_mode_stop_apply_brake_v[CS.one_pedal_brake_mode])
          one_pedal_apply_brake *= interp(CS.pitch, CS.one_pedal_pitch_brake_adjust_bp, CS.one_pedal_pitch_brake_adjust_v)
          one_pedal_apply_brake = min(one_pedal_apply_brake, float(P.BRAKE_LOOKUP_V[0]))
          time_since_brake = t - CS.one_pedal_mode_last_gas_press_t
          one_pedal_apply_brake *= interp(time_since_brake, CS.one_pedal_mode_ramp_time_bp, CS.one_pedal_mode_ramp_time_v) if CS.one_pedal_brake_mode < 2 else 1.
        else:
          one_pedal_apply_brake = 0.
        if CS.one_pedal_mode_op_braking_allowed and CS.coasting_long_plan not in ['cruise', 'limit']:
          apply_brake = max(one_pedal_apply_brake, apply_brake * lead_long_brake_lockout_factor)
        else:
          apply_brake = one_pedal_apply_brake
        
      elif CS.coasting_enabled and lead_long_brake_lockout_factor < 1.:
        if CS.coasting_long_plan in ['cruise', 'limit'] and apply_gas < P.ZERO_GAS or apply_brake > 0.:
          check_speed_ms = (CS.speed_limit if CS.speed_limit_active and CS.speed_limit < CS.v_cruise_kph else CS.v_cruise_kph) * CV.KPH_TO_MS
          if apply_brake > 0.:
            coasting_over_speed_vEgo_BP = [
              interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_vEgo_BP[0]),
              interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_vEgo_BP[1])
            ]
            over_speed_factor = interp(CS.vEgo / check_speed_ms, coasting_over_speed_vEgo_BP, [0., 1.]) if (check_speed_ms > 0. and CS.coasting_brake_over_speed_enabled) else 0.
            over_speed_brake = apply_brake * over_speed_factor
            apply_brake = max([apply_brake * lead_long_brake_lockout_factor, over_speed_brake])
          if apply_gas < P.ZERO_GAS and lead_long_gas_lockout_factor < 1.:
            coasting_over_speed_vEgo_BP = [
              interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_regen_vEgo_BP[0]),
              interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_regen_vEgo_BP[1])
            ]
            over_speed_factor = interp(CS.vEgo / check_speed_ms, coasting_over_speed_vEgo_BP, [0., 1.]) if (check_speed_ms > 0 and CS.coasting_brake_over_speed_enabled) else 0.
            coast_apply_gas = int(round(float(P.ZERO_GAS) - over_speed_factor * (P.ZERO_GAS - apply_gas)))
            apply_gas = apply_gas * lead_long_gas_lockout_factor + coast_apply_gas * (1. - lead_long_gas_lockout_factor)
      elif CS.no_friction_braking and lead_long_brake_lockout_factor < 1.:
        if CS.coasting_long_plan in ['cruise', 'limit'] and apply_brake > 0.:
          apply_brake *= lead_long_brake_lockout_factor
      apply_gas = int(round(apply_gas))
      apply_brake = int(round(apply_brake))
      

      if do_log:
        f.write(",".join([str(i) for i in [
          apply_gas, 
          apply_brake]]) + "\n")
        f.close()
    
    if CS.showBrakeIndicator:
      CS.apply_brake_percent = int(interp(apply_brake, [float(P.BRAKE_LOOKUP_V[-1]), float(P.BRAKE_LOOKUP_V[0])], [0., 100.])) if CS.vEgo > 0.1 else 0
    
    # Gas/regen and brakes - all at 25Hz
    if (frame % 4) == 0:
      idx = (frame // 4) % 4

      if CS.cruiseMain and not enabled and CS.autoHold and CS.autoHoldActive and not CS.out.gasPressed and CS.out.gearShifter == 'drive' and CS.out.vEgo < 0.01 and not CS.regenPaddlePressed:
        # Auto Hold State
        car_stopping = apply_gas < P.ZERO_GAS
        standstill = CS.pcm_acc_status == AccState.STANDSTILL

        at_full_stop = standstill and car_stopping
        near_stop = (CS.out.vEgo < P.NEAR_STOP_BRAKE_PHASE) and car_stopping
        can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, apply_brake, idx, near_stop, at_full_stop))
        CS.autoHoldActivated = True

      else:
        if CS.pause_long_on_gas_press:
          at_full_stop = False
          near_stop = False
          car_stopping = False
          standstill = False
        else:
          car_stopping = apply_gas < P.ZERO_GAS
          standstill = CS.pcm_acc_status == AccState.STANDSTILL
          at_full_stop = enabled and standstill and car_stopping
          near_stop = enabled and (CS.out.vEgo < P.NEAR_STOP_BRAKE_PHASE) and car_stopping

        can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, apply_brake, idx, near_stop, at_full_stop))
        CS.autoHoldActivated = False

        # Auto-resume from full stop by resetting ACC control
        acc_enabled = enabled
      
        if standstill and not car_stopping:
          acc_enabled = False
      
        can_sends.append(gmcan.create_gas_regen_command(self.packer_pt, CanBus.POWERTRAIN, apply_gas, idx, acc_enabled, at_full_stop))


    # Send dashboard UI commands (ACC status), 25hz
    if (frame % 4) == 0:
      send_fcw = hud_alert == VisualAlert.fcw
      follow_level = CS.get_follow_level()
      can_sends.append(gmcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, enabled, 
                                                                 hud_v_cruise * CV.MS_TO_KPH, hud_show_car, follow_level, send_fcw))

    # Radar needs to know current speed and yaw rate (50hz),
    # and that ADAS is alive (10hz)
    time_and_headlights_step = 10
    tt = frame * DT_CTRL

    if frame % time_and_headlights_step == 0:
      idx = (frame // time_and_headlights_step) % 4
      can_sends.append(gmcan.create_adas_time_status(CanBus.OBSTACLE, int((tt - self.start_time) * 60), idx))
      can_sends.append(gmcan.create_adas_headlights_status(self.packer_obj, CanBus.OBSTACLE))

    speed_and_accelerometer_step = 2
    if frame % speed_and_accelerometer_step == 0:
      idx = (frame // speed_and_accelerometer_step) % 4
      can_sends.append(gmcan.create_adas_steering_status(CanBus.OBSTACLE, idx))
      can_sends.append(gmcan.create_adas_accelerometer_speed_status(CanBus.OBSTACLE, CS.out.vEgo, idx))

    if frame % P.ADAS_KEEPALIVE_STEP == 0:
      can_sends += gmcan.create_adas_keepalive(CanBus.POWERTRAIN)

    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = CS.lkas_status == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert in [VisualAlert.steerRequired, VisualAlert.ldw]
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    return can_sends
