from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from common.op_params import opParams
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, AccState, CanBus, CarControllerParams
from selfdrive.car.gm.carstate import GAS_PRESSED_THRESHOLD, GEAR_SHIFTER2
from selfdrive.controls.lib.longitudinal_planner import BRAKE_SOURCES, COAST_SOURCES
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

# only use pitch-compensated acceleration at 10m/s+
ACCEL_PITCH_FACTOR_BP = [5., 10.] # [m/s]
ACCEL_PITCH_FACTOR_V = [0., 1.] # [unitless in [0-1]]

ONE_PEDAL_ACCEL_PITCH_FACTOR_BP = [4., 8.] # [m/s]

ONE_PEDAL_MODE_DECEL_BP = [i * CV.MPH_TO_MS for i in [0.5, 6.]] # [mph to meters]
ONE_PEDAL_MIN_SPEED = 2.1
ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_BP = [i * CV.MPH_TO_MS for i in [0.0, 10.]] # [mph to meters]
ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_V = [0.2, 1.0] # factor of rate limit

ONE_PEDAL_DECEL_RATE_LIMIT_STEER_FACTOR_BP = [20.0, 120.0] # [deg] abs steering wheel angle
ONE_PEDAL_DECEL_RATE_LIMIT_STEER_FACTOR_V = [1.0, 0.2] # factor of rate limit

ONE_PEDAL_SPEED_ERROR_FACTOR_BP = [1.5, 20.] # [m/s] 
ONE_PEDAL_SPEED_ERROR_FACTOR_V = [0.4, 0.2] # factor of error for non-lead braking decel

ONE_PEDAL_LEAD_ACCEL_RATE_LOCKOUT_T = 0.6 # [s]

ONE_PEDAL_MODE_DECEL_V = [-1.0, -1.1] # m/s^2
ONE_PEDAL_MODE_REGEN_PADDLE_DECEL_V = [-1.3, -1.6] # m/s^2
ONE_PEDAL_MAX_DECEL = min(ONE_PEDAL_MODE_DECEL_V + ONE_PEDAL_MODE_REGEN_PADDLE_DECEL_V) - 0.5 # don't allow much more than the lowest requested amount
ONE_PEDAL_DECEL_RATE_LIMIT_UP = 0.8 * DT_CTRL * 4 # m/s^2 per second for increasing braking force
ONE_PEDAL_DECEL_RATE_LIMIT_DOWN = 0.8 * DT_CTRL * 4 # m/s^2 per second for decreasing
ONE_PEDAL_ACCEL_PITCH_FACTOR_V = [0.4, 1.] # [unitless in [0-1]]
ONE_PEDAL_ACCEL_PITCH_FACTOR_INCLINE_V = [0.2, 1.] # [unitless in [0-1]]

ONE_PEDAL_ALLOWED_GEARS = [GEAR_SHIFTER2.LOW, GEAR_SHIFTER2.REGEN_PADDLE_LOW, GEAR_SHIFTER2.REGEN_PADDLE_DRIVE]

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.start_time = 0.
    self.apply_steer_last = 0
    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False
    self.fcw_count = 0
    
    self.params = CarControllerParams()
    self._op_params = opParams("gm CarController")
    self.override_long_tune = self._op_params.get('TUNE_LONG_do_override', force_update=True)
    self.override_lat_tune = self._op_params.get('TUNE_LAT_do_override', force_update=True)
    self.min_steer_speed = self._op_params.get('TUNE_LAT_min_steer_speed_mph', force_update=True) * CV.MPH_TO_MS

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])
    
    # pid runs at 25Hz
    self.one_pedal_pid = PIDController(k_p=(CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV), 
                                      k_i=(CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV), 
                                      k_d=(CP.longitudinalTuning.kdBP, CP.longitudinalTuning.kdV),
                                      derivative_period=0.1,
                                      k_11 = 0.5, k_12 = 0.5, k_13 = 0.5, k_period=0.1,
                                      rate=1/(DT_CTRL * 4),
                                      sat_limit=0.8)
    self.one_pedal_decel = 0.0
    self.one_pedal_decel_in = 0.
    self.one_pedal_pid.neg_limit = -3.5
    self.one_pedal_pid.pos_limit = 0.0
    self.lead_accel_last_t = 0.
    self.one_pedal_apply_brake = 0.0
    
    self.apply_gas = 0
    self.apply_brake_out = 0
    self.apply_brake_in = 0
    self.apply_steer = 0
    self.brakes_allowed = False
    self.threshold_accel = 0.0
  
  def update_op_params(self):
    global ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_V, ONE_PEDAL_DECEL_RATE_LIMIT_STEER_FACTOR_V, ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_BP, ONE_PEDAL_DECEL_RATE_LIMIT_STEER_FACTOR_BP, ONE_PEDAL_MODE_DECEL_V, ONE_PEDAL_MAX_DECEL, ONE_PEDAL_DECEL_RATE_LIMIT_UP, ONE_PEDAL_DECEL_RATE_LIMIT_DOWN, ONE_PEDAL_SPEED_ERROR_FACTOR_V, ONE_PEDAL_ACCEL_PITCH_FACTOR_V, ONE_PEDAL_ACCEL_PITCH_FACTOR_INCLINE_V, ONE_PEDAL_MODE_REGEN_PADDLE_DECEL_V
    
    ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_V[0] = self._op_params.get('MADS_OP_rate_low_speed_factor')
    ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_BP = sorted(self._op_params.get('MADS_OP_rate_low_speed_factor_bp'))
    ONE_PEDAL_DECEL_RATE_LIMIT_STEER_FACTOR_V[1] = self._op_params.get('MADS_OP_rate_high_steer_factor')
    ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_BP = sorted(self._op_params.get('MADS_OP_rate_high_steer_factor_bp'))
    ONE_PEDAL_MODE_DECEL_V = self._op_params.get('MADS_OP_decel_ms2')
    ONE_PEDAL_MODE_REGEN_PADDLE_DECEL_V = self._op_params.get('MADS_OP_regen_paddle_decel_ms2')
    ONE_PEDAL_MAX_DECEL = min(ONE_PEDAL_MODE_DECEL_V + ONE_PEDAL_MODE_REGEN_PADDLE_DECEL_V) - 0.5 # don't allow much more than the lowest requested amount
    ONE_PEDAL_DECEL_RATE_LIMIT_UP = self._op_params.get('MADS_OP_rate_ramp_up') * DT_CTRL * 4 # m/s^2 per second for increasing braking force
    ONE_PEDAL_DECEL_RATE_LIMIT_DOWN = self._op_params.get('MADS_OP_rate_ramp_down') * DT_CTRL * 4 # m/s^2 per second for decreasing
    ONE_PEDAL_SPEED_ERROR_FACTOR_V = self._op_params.get('MADS_OP_speed_error_factor') # factor of error for non-lead braking decel
    ONE_PEDAL_ACCEL_PITCH_FACTOR_V[0] = self._op_params.get('MADS_OP_low_speed_pitch_factor_decline') # [unitless in [0-1]]
    ONE_PEDAL_ACCEL_PITCH_FACTOR_INCLINE_V[0] = self._op_params.get('MADS_OP_low_speed_pitch_factor_incline') # [unitless in [0-1]]
    if self.override_long_tune:
      bp = [i * CV.MPH_TO_MS for i in self._op_params.get('TUNE_LONG_speed_mph')]
      self.one_pedal_pid._k_p = [bp, self._op_params.get('TUNE_LONG_kp')]
      self.one_pedal_pid._k_i = [bp, self._op_params.get('TUNE_LONG_ki')]
      self.one_pedal_pid._k_d = [bp, self._op_params.get('TUNE_LONG_kd')]
    
    
  def update(self, enabled, CS, frame, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):
    P = self.params

    # Send CAN commands.
    can_sends = []
    
    no_pitch_apply_gas = 0

    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % P.STEER_STEP) == 0:
      lkas_enabled = (enabled or CS.pause_long_on_gas_press or (CS.MADS_enabled and CS.cruiseMain)) and CS.lkaEnabled and not (CS.out.steerWarning or CS.out.steerError) and CS.out.vEgo > self.min_steer_speed and CS.lane_change_steer_factor > 0.
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX * CS.lane_change_steer_factor))
        P.v_ego = CS.out.vEgo
        self.apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != self.apply_steer
      else:
        self.apply_steer = 0

      self.apply_steer_last = self.apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
      idx = (CS.lka_steering_cmd_counter + 1) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, self.apply_steer, idx, lkas_enabled))

    # Gas/regen prep
    if (frame % 4) == 0:
      if (frame % 48) == 0:
        self.update_op_params()
      if CS.out.gas >= 1e-5 or (not CS.out.onePedalModeActive and not CS.MADS_lead_braking_enabled) or CS.out.brakePressed:
        self.one_pedal_pid.reset()
        self.one_pedal_decel = CS.out.aEgo
        self.one_pedal_decel_in = CS.out.aEgo
      if (enabled or (CS.out.onePedalModeActive or CS.MADS_lead_braking_enabled)) or (CS.pause_long_on_gas_press and CS.out.gas > GAS_PRESSED_THRESHOLD):
        t = sec_since_boot()
        k = interp(CS.out.vEgo, ACCEL_PITCH_FACTOR_BP, ACCEL_PITCH_FACTOR_V)
        brake_accel = k * actuators.accelPitchCompensated + (1. - k) * actuators.accel
        if CS.out.onePedalModeActive and (not CS.MADS_lead_braking_enabled or t - self.lead_accel_last_t > ONE_PEDAL_LEAD_ACCEL_RATE_LOCKOUT_T):
          one_pedal_speed = max(CS.vEgo, ONE_PEDAL_MIN_SPEED)
          self.threshold_accel = self.params.update_gas_brake_threshold(one_pedal_speed, CS.engineRPM > 0)
        else:
          self.threshold_accel = self.params.update_gas_brake_threshold(CS.out.vEgo, CS.engineRPM > 0)
        self.apply_gas = interp(actuators.accelPitchCompensated, P.GAS_LOOKUP_BP, P.GAS_LOOKUP_V)
        no_pitch_apply_gas = interp(actuators.accel, P.GAS_LOOKUP_BP, P.GAS_LOOKUP_V)
        self.apply_brake_out = interp(brake_accel, P.BRAKE_LOOKUP_BP, P.BRAKE_LOOKUP_V)
        self.apply_brake_in = round(int(self.apply_brake_out))
        
        
        CS.MADS_lead_braking_active = CS.MADS_lead_braking_enabled and not enabled and CS.coasting_lead_d > 0.0 and actuators.accel < -0.1 and CS.coasting_long_plan in BRAKE_SOURCES
        
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
        
        if enabled or not CS.out.onePedalModeActive or CS.out.gas >= 1e-5 or CS.out.brakePressed:
          self.one_pedal_pid.reset()
          self.one_pedal_decel = CS.out.aEgo
          self.one_pedal_decel_in = CS.out.aEgo
          self.one_pedal_apply_brake = 0.0
        else:
          self.apply_gas = P.MAX_ACC_REGEN
          pitch_accel = CS.pitch * ACCELERATION_DUE_TO_GRAVITY
          pitch_accel *= interp(CS.vEgo, ONE_PEDAL_ACCEL_PITCH_FACTOR_BP, ONE_PEDAL_ACCEL_PITCH_FACTOR_V if pitch_accel <= 0 else ONE_PEDAL_ACCEL_PITCH_FACTOR_INCLINE_V)
          
          if CS.gear_shifter_ev in ONE_PEDAL_ALLOWED_GEARS:
            self.one_pedal_decel_in = interp(CS.vEgo, 
                                             ONE_PEDAL_MODE_DECEL_BP, 
                                             ONE_PEDAL_MODE_DECEL_V if CS.gear_shifter_ev == GEAR_SHIFTER2.LOW else ONE_PEDAL_MODE_REGEN_PADDLE_DECEL_V)
            
            error_factor = interp(CS.vEgo, ONE_PEDAL_SPEED_ERROR_FACTOR_BP, ONE_PEDAL_SPEED_ERROR_FACTOR_V)
            error = self.one_pedal_decel_in - min(0.0, CS.out.aEgo + pitch_accel)
            error *= error_factor
            one_pedal_decel = self.one_pedal_pid.update(self.one_pedal_decel_in, self.one_pedal_decel_in - error, speed=CS.out.vEgo, feedforward=self.one_pedal_decel_in)
            
            rate_limit_factor = interp(CS.vEgo, ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_BP, ONE_PEDAL_DECEL_RATE_LIMIT_SPEED_FACTOR_V)
            rate_limit_factor = min(rate_limit_factor,
                                    interp(abs(CS.out.steeringAngleDeg), ONE_PEDAL_DECEL_RATE_LIMIT_STEER_FACTOR_BP, ONE_PEDAL_DECEL_RATE_LIMIT_STEER_FACTOR_V))
            
            self.one_pedal_decel = clip(one_pedal_decel, min(self.one_pedal_decel, CS.out.aEgo + pitch_accel) - ONE_PEDAL_DECEL_RATE_LIMIT_UP * rate_limit_factor, max(self.one_pedal_decel, CS.out.aEgo + pitch_accel) + ONE_PEDAL_DECEL_RATE_LIMIT_DOWN + rate_limit_factor)
            self.one_pedal_decel = max(self.one_pedal_decel, ONE_PEDAL_MAX_DECEL)
            self.one_pedal_decel = min(self.one_pedal_decel, CS.out.aEgo)
            self.one_pedal_apply_brake = interp(self.one_pedal_decel, P.BRAKE_LOOKUP_BP, P.BRAKE_LOOKUP_V)
          else:
            self.one_pedal_decel_in = clip(0.0 if CS.gear_shifter_ev == GEAR_SHIFTER2.DRIVE and CS.one_pedal_dl_coasting_enabled and CS.vEgo > 0.05 else min(CS.out.aEgo,self.threshold_accel), self.one_pedal_decel_in - ONE_PEDAL_DECEL_RATE_LIMIT_UP, self.one_pedal_decel_in + ONE_PEDAL_DECEL_RATE_LIMIT_DOWN)
            self.one_pedal_apply_brake = 0.0
          
          
          if self.one_pedal_apply_brake > 0.0 \
              and (not CS.MADS_lead_braking_enabled \
              or self.one_pedal_apply_brake > self.apply_brake_out \
              or CS.coasting_lead_d < 0.0):
            self.apply_brake_out = self.one_pedal_apply_brake
            CS.MADS_lead_braking_active = False
          if CS.MADS_lead_braking_active:
            self.lead_accel_last_t = t
        
        if enabled:
          if CS.coasting_enabled and lead_long_brake_lockout_factor < 1.0 \
              and not CS.slippery_roads_active and not CS.low_visibility_active:
            if CS.coasting_long_plan in COAST_SOURCES and (self.apply_gas < P.ZERO_GAS or self.apply_brake_out > 0.0):
              check_speed_ms = (CS.speed_limit if CS.speed_limit_active and CS.speed_limit < CS.v_cruise_kph else CS.v_cruise_kph) * CV.KPH_TO_MS
              if self.apply_brake_out > 0.0:
                coasting_over_speed_vEgo_BP = [
                  interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_vEgo_BP[0]),
                  interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_vEgo_BP[1])
                ]
                over_speed_factor = interp(CS.vEgo / check_speed_ms, coasting_over_speed_vEgo_BP, [0., 1.]) if (check_speed_ms > 0. and CS.coasting_brake_over_speed_enabled) else 0.
                over_speed_brake = self.apply_brake_out * over_speed_factor
                self.apply_brake_out = max([self.apply_brake_out * lead_long_brake_lockout_factor, over_speed_brake])
              if self.apply_gas < P.ZERO_GAS and lead_long_gas_lockout_factor < 1.0:
                coasting_over_speed_vEgo_BP = [
                  interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_regen_vEgo_BP[0]),
                  interp(CS.vEgo, CS.coasting_over_speed_vEgo_BP_BP, CS.coasting_over_speed_regen_vEgo_BP[1])
                ]
                over_speed_factor = interp(CS.vEgo / check_speed_ms, coasting_over_speed_vEgo_BP, [0.0, 1.0]) if (check_speed_ms > 0 and CS.coasting_brake_over_speed_enabled) else 0.
                coast_apply_gas = int(round(float(P.ZERO_GAS) - over_speed_factor * (P.ZERO_GAS - self.apply_gas)))
                self.apply_gas = self.apply_gas * lead_long_gas_lockout_factor + coast_apply_gas * (1.0 - lead_long_gas_lockout_factor)
          elif CS.no_friction_braking and lead_long_brake_lockout_factor < 1.0:
            if CS.coasting_long_plan in COAST_SOURCES and self.apply_brake_out > 0.0:
              self.apply_brake_out *= lead_long_brake_lockout_factor
        self.apply_gas = int(round(self.apply_gas))
        self.apply_brake_out = int(round(self.apply_brake_out))
    
    
      self.brakes_allowed = any([CS.long_active,
                            enabled, 
                            CS.out.onePedalModeActive, 
                            CS.MADS_lead_braking_active]
                          ) and \
                        all([CS.out.gas < 1e-5,
                              CS.out.cruiseMain,
                              CS.out.gearShifter in ['drive','low'],
                              not CS.out.brakePressed])
      
      if any([not CS.cruiseMain,
              CS.out.brakePressed,
              CS.out.gearShifter not in ['drive','low'],
              not enabled,
              CS.out.gas >= GAS_PRESSED_THRESHOLD]):
        self.apply_gas = P.MAX_ACC_REGEN
      if not self.brakes_allowed:
        self.apply_brake_out = 0

    if CS.showBrakeIndicator:
      CS.apply_brake_percent = 0.
      if CS.vEgo > 0.1:
        if CS.out.cruiseState.enabled or CS.out.onePedalModeActive or CS.MADS_lead_braking_active:
          if CS.out.gas < 1e-5:
            if self.apply_brake_out > 1:
              CS.apply_brake_percent = interp(self.apply_brake_out, [float(P.BRAKE_LOOKUP_V[-1]), float(P.BRAKE_LOOKUP_V[0])], [51., 100.])
            elif CS.out.onePedalModeActive:
              CS.apply_brake_percent = interp(CS.hvb_wattage.x, CS.hvb_wattage_bp, [0., 49.])
            elif self.apply_gas < P.ZERO_GAS:
              CS.apply_brake_percent = interp(self.apply_gas, [float(P.GAS_LOOKUP_V[0]), float(P.GAS_LOOKUP_V[1])], [49., 0.])
          else:
            CS.apply_brake_percent = interp(CS.hvb_wattage.x, CS.hvb_wattage_bp, [0., 49.])
        elif CS.is_ev and CS.out.brake == 0.:
          CS.apply_brake_percent = interp(CS.hvb_wattage.x, CS.hvb_wattage_bp, [0., 49.])
        elif CS.out.brake > 0.:
          CS.apply_brake_percent = interp(CS.out.brake, [0., 0.5], [51., 100.])
      elif CS.out.brake > 0.:
        CS.apply_brake_percent = interp(CS.out.brake, [0., 0.5], [51., 100.])
    
    # Gas/regen and brakes - all at 25Hz
    if (frame % 4) == 0:
      idx = (frame // 4) % 4
      
      if enabled and self.brakes_allowed:
        self.apply_brake_out = self.apply_brake_in

      if CS.cruiseMain and not enabled and ((CS.autoHold and not CS.regen_paddle_pressed and CS.time_in_drive_autohold >= CS.MADS_long_min_time_in_drive) or (CS.one_pedal_mode_active and CS.time_in_drive_one_pedal >= CS.MADS_long_min_time_in_drive)) and CS.autoHoldActive and not CS.out.gas > 1e-5 and CS.out.vEgo < 0.02:
        # Auto Hold State
        standstill = CS.pcm_acc_status == AccState.STANDSTILL

        at_full_stop = standstill
        near_stop = (CS.out.vEgo < P.NEAR_STOP_BRAKE_PHASE)
        can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, self.apply_brake_out, idx, near_stop, at_full_stop))
        CS.autoHoldActivated = True

      else:
        if not self.brakes_allowed:
          at_full_stop = False
          near_stop = False
          car_stopping = False
          standstill = False
        else:
          car_stopping = no_pitch_apply_gas < P.ZERO_GAS
          standstill = CS.pcm_acc_status == AccState.STANDSTILL
          if standstill:
            self.apply_gas = P.MAX_ACC_REGEN
          at_full_stop = (enabled or (CS.out.onePedalModeActive or CS.MADS_lead_braking_enabled)) and standstill and car_stopping
          near_stop = (enabled or (CS.out.onePedalModeActive or CS.MADS_lead_braking_enabled)) and (CS.out.vEgo < P.NEAR_STOP_BRAKE_PHASE) and car_stopping

        can_sends.append(gmcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, self.apply_brake_out, idx, near_stop, at_full_stop))
        CS.autoHoldActivated = False

        # Auto-resume from full stop by resetting ACC control
        acc_enabled = enabled
        
        if standstill and not car_stopping:
          if CS.do_sng:
            acc_enabled = False
            CS.resume_button_pressed = True
          elif CS.out.vEgo < 1.5:
            CS.resume_required = True
      
        can_sends.append(gmcan.create_gas_regen_command(self.packer_pt, CanBus.POWERTRAIN, self.apply_gas, idx, acc_enabled, at_full_stop))


    CS.brake_cmd = self.apply_brake_out

    # Send dashboard UI commands (ACC status), 25hz
    if (frame % 4) == 0:
      send_fcw = hud_alert == VisualAlert.fcw
      follow_level = CS.get_follow_level()

      can_sends.append(gmcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, enabled, 
                                                                 hud_v_cruise * CV.MS_TO_KPH, hud_show_car, follow_level, send_fcw, CS.resume_button_pressed))
      CS.resume_button_pressed = False

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
