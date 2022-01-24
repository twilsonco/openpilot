from cereal import car
from common.params import Params
from common.numpy_fast import mean, interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, \
                                    CruiseButtons, STEER_THRESHOLD
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.drive_helpers import set_v_cruise_offset

def get_chassis_can_parser(CP, canbus):
  # this function generates lists for signal, messages and initial values
  signals = [
      # sig_name, sig_address, default
      ("FrictionBrakePressure", "EBCMFrictionBrakeStatus", 0),
  ]

  return CANParser(DBC[CP.carFingerprint]['chassis'], signals, [], canbus.chassis)
  

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["PRNDL"]
    self._params = Params()
    
    self.prev_distance_button = 0
    self.prev_lka_button = 0
    self.lka_button = 0
    self.distance_button = 0
    self.distance_button_last_press_t = 0.
    self.follow_level = int(self._params.get("FollowLevel", encoding="utf8"))
    self.lkMode = True
    set_v_cruise_offset(self._params.get_bool("CruiseSpeedOffset"))
    self.autoHold = self._params.get_bool("GMAutoHold")
    self.disengage_on_gas = not self._params.get_bool("DisableDisengageOnGas")
    self.autoHoldActive = False
    self.autoHoldActivated = False
    self.regenPaddlePressed = False
    self.cruiseMain = False
    self.engineRPM = 0
    self.lastAutoHoldTime = 0.0
    self.sessionInitTime = sec_since_boot()
    self.params_check_last_t = 0.
    self.params_check_freq = 0.1 # check params at 10Hz
    
    self.accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco; 3 = creep
    
    self.coasting_enabled = self._params.get_bool("Coasting")
    self.coasting_enabled_last = self.coasting_enabled
    self.no_friction_braking = self._params.get_bool("RegenBraking")
    self.coasting_brake_over_speed_enabled = self._params.get_bool("CoastingBrakeOverSpeed")
    self.coasting_over_speed_vEgo_BP = [[1.3, 1.2], [1.35, 1.25]]
    self.coasting_over_speed_regen_vEgo_BP = [[1.20, 1.10], [1.25, 1.15]]
    self.coasting_over_speed_vEgo_BP_BP = [i * CV.MPH_TO_MS for i in [20., 80.]]
    self.coasting_long_plan = ""
    self.coasting_lead_d = -1. # [m] lead distance. -1. if no lead
    self.coasting_lead_v = -1.
    self.tr = 1.8
    self.coast_one_pedal_mode_active = False
    self.pause_long_on_gas_press = False
    self.last_pause_long_on_gas_press_t = 0.
    self.gasPressed = False
    
    self.one_pedal_mode_enabled = self._params.get_bool("OnePedalMode") and not self.disengage_on_gas
    self.one_pedal_mode_op_braking_allowed = not self._params.get_bool("OnePedalModeSimple")
    self.one_pedal_mode_engage_on_gas_enabled = self._params.get_bool("OnePedalModeEngageOnGas") and (self.one_pedal_mode_enabled or not self.disengage_on_gas)
    self.one_pedal_mode_engage_on_gas = False
    self.one_pedal_mode_engage_on_gas_min_speed = 1. * CV.MPH_TO_MS # gas press at or above this speed with engage on gas enabled and one-pedal mode will activate
    self.one_pedal_mode_max_set_speed = 3 * CV.MPH_TO_MS #  one pedal mode activates if cruise set at or below this speed
    self.one_pedal_mode_stop_apply_brake_bp = [[i * CV.MPH_TO_MS for i in [1., 4., 45., 85.]], [i * CV.MPH_TO_MS for i in [1., 4., 45., 85.]], [1.]]
    self.one_pedal_mode_stop_apply_brake_v = [[80., 95., 115., 90.], [110., 165., 185., 140.], [280.]] # three levels. 1-2 are cycled using follow distance press, and 3 by holding
    self.one_pedal_mode_apply_brake = 0.
    self.one_pedal_mode_ramp_duration = 0.6
    self.one_pedal_mode_ramp_time_step = 60. / self.one_pedal_mode_ramp_duration
    self.one_pedal_mode_ramp_t_last = 0.
    self.one_pedal_mode_active_last = False
    self.one_pedal_mode_ramp_mode_last = 0 # stores brake mode for calculating the step when mode is changed
    self.one_pedal_mode_last_gas_press_t = 0.
    self.one_pedal_mode_engaged_with_button = False
    self.one_pedal_mode_ramp_time_bp = [0., 0.5]
    self.one_pedal_mode_ramp_time_v = [0.1, 1.0]
    self.one_pedal_mode_active = False
    self.one_pedal_brake_mode = int(self._params.get("OnePedalBrakeMode", encoding="utf8")) # 0, 1, or 2 selecting the brake profiles above. 2 is activated by pressing and holding the follow distance button for > 0.3s
    self.one_pedal_last_brake_mode = 0 # for saving brake mode when not in one-pedal-mode
    self.one_pedal_last_follow_level = 0 # for saving follow distance when in one-pedal mode
    self.one_pedal_v_cruise_kph_last = 0
    self.one_pedal_last_switch_to_friction_braking_t = 0.
    self.one_pedal_pause_steering_enabled = self._params.get_bool("OnePedalPauseBlinkerSteering")
    self.one_pedal_pitch_brake_adjust_bp = [-0.08, -0.005, 0.005, 0.10] # [radians] 0.12 radians of pitch ≈ 12% grade. No change within ±0.02
    self.one_pedal_pitch_brake_adjust_v = [[.6, 1., 1., 1.5], [.75, 1., 1., 1.5], [.9, 1., 1., 1.5]] # used to scale the value of apply_brake
    self.one_pedal_angle_steers_cutoff_bp = [60., 270.] # [degrees] one pedal braking goes down one "level" as steering wheel is turned more than this angle
    
    self.drive_mode_button = False
    self.drive_mode_button_last = False
    self.gear_shifter_raw = None
          
    self.pitch_rolling_iter = 0
    self.pitch_rolling_period = 2. # 2-second moving average
    self.pitch_check_freq = 0.1 # checked at 10Hz
    self.pitch_num_vals = int(self.pitch_rolling_period / self.pitch_check_freq)
    self.pitch_num_vals_recip = 1. / float(self.pitch_num_vals)
    self.pitch_check_last = 0.
    self.pitch_vals = [0. for i in range(self.pitch_num_vals)]
    self.pitch = 0.
    self.pitch_raw = 0.
    
    # similar to over-speed coast braking, lockout coast/one-pedal logic first for engine/regen braking, and then for actual brakes.
    # gas lockout lookup tables:
    self.lead_v_rel_long_gas_lockout_bp, self.lead_v_rel_long_gas_lockout_v = [[-12 * CV.MPH_TO_MS, -8 * CV.MPH_TO_MS], [1., 0.]] # pass-through all engine/regen braking for v_rel < -15mph
    self.lead_v_long_gas_lockout_bp, self.lead_v_long_gas_lockout_v = [[6. * CV.MPH_TO_MS, 12. * CV.MPH_TO_MS], [1., 0.]] # pass-through all engine/regen braking for v_lead < 4mph
    self.lead_ttc_long_gas_lockout_bp, self.lead_ttc_long_gas_lockout_v = [[4., 8.], [1., 0.]] # pass through all cruise engine/regen braking for time-to-collision < 4s
    self.lead_tr_long_gas_lockout_bp, self.lead_tr_long_gas_lockout_v = [[1.8, 2.5], [1., 0.]] # pass through all cruise engine/regen braking if follow distance < tr * 0.8
    self.lead_d_long_gas_lockout_bp, self.lead_d_long_gas_lockout_v = [[12, 20], [1., 0.]] # pass through all cruise engine/regen braking if follow distance < 6m
    
    # brake lockout lookup tables:
    self.lead_v_rel_long_brake_lockout_bp, self.lead_v_rel_long_brake_lockout_v = [[-20 * CV.MPH_TO_MS, -15 * CV.MPH_TO_MS], [1., 0.]] # pass-through all braking for v_rel < -15mph
    self.lead_v_long_brake_lockout_bp, self.lead_v_long_brake_lockout_v = [[2. * CV.MPH_TO_MS, 5. * CV.MPH_TO_MS], [1., 0.]] # pass-through all braking for v_lead < 4mph
    self.lead_ttc_long_brake_lockout_bp, self.lead_ttc_long_brake_lockout_v = [[2., 3.], [1., 0.]] # pass through all cruise braking for time-to-collision < 4s
    self.lead_tr_long_brake_lockout_bp, self.lead_tr_long_brake_lockout_v = [[1.2, 1.6], [1., 0.]] # pass through all cruise braking if follow distance < tr * 0.8
    self.lead_d_long_brake_lockout_bp, self.lead_d_long_brake_lockout_v = [[6, 10], [1., 0.]] # pass through all cruise braking if follow distance < 6m
    
    self.showBrakeIndicator = self._params.get_bool("BrakeIndicator")
    self.apply_brake_percent = 0 if self.showBrakeIndicator else -1 # for brake percent on ui
    self.vEgo = 0.
    self.v_cruise_kph = 1
    self.min_lane_change_speed = 20. * CV.MPH_TO_MS
    self.blinker = False
    self.prev_blinker = self.blinker
    self.lane_change_steer_factor = 1.
    self.lang_change_ramp_up_steer_start_t = 0.
    self.lang_change_ramp_down_steer_start_t = 0.
    self.lang_change_ramp_steer_dur = .5 # [s]

  def update(self, pt_cp):
    ret = car.CarState.new_message()
    
    t = sec_since_boot()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    self.prev_lka_button = self.lka_button
    self.lka_button = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]
    self.prev_distance_button = self.distance_button
    self.distance_button = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"]
    
    self.drive_mode_button_last = self.drive_mode_button
    self.drive_mode_button = pt_cp.vl["ASCMSteeringButton"]["DriveModeButton"]
    
    if (self.drive_mode_button != self.drive_mode_button_last):
        cloudlog.info(f"{t} Drive mode button event: new value = {self.drive_mode_button}")

    ret.wheelSpeeds.fl = pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    self.vEgo = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.01

    self.coasting_enabled_last = self.coasting_enabled
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      self.coasting_enabled = self._params.get_bool("Coasting")
      self.accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco; 3 = creep
      if not self.disengage_on_gas:
        self.one_pedal_pause_steering_enabled = self._params.get_bool("OnePedalPauseBlinkerSteering")
        self.one_pedal_mode_enabled = self._params.get_bool("OnePedalMode")
        self.one_pedal_mode_engage_on_gas_enabled = self._params.get_bool("OnePedalModeEngageOnGas") and (self.one_pedal_mode_enabled or not self.disengage_on_gas)
      
    if self.coasting_enabled != self.coasting_enabled_last:
      if not self.coasting_enabled and self.vEgo > self.v_cruise_kph * CV.KPH_TO_MS and not self.no_friction_braking:
        self._params.set_bool("Coasting", True)
        self.coasting_enabled = True
    ret.coastingActive = self.coasting_enabled

    self.angle_steers = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
    
    gear_shifter_raw = pt_cp.vl["ECMPRDNL"]['PRNDL']
    if (self.gear_shifter_raw != gear_shifter_raw):
      cloudlog.info(f"{t} Gear shifted to: {gear_shifter_raw = }, {self.shifter_values.get(gear_shifter_raw, None) = }, {self.parse_gear_shifter(self.shifter_values.get(gear_shifter_raw, None)) = }")
    self.gear_shifter_raw = gear_shifter_raw
      
    self.gear_shifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    self.user_brake = pt_cp.vl["EBCMBrakePedalPosition"]['BrakePedalPosition']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]["PRNDL"], None))
    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0
    # Brake pedal's potentiometer returns near-zero reading even when pedal is not pressed.
    if ret.brake < 10/0xd0:
      ret.brake = 0.
    
    if self.showBrakeIndicator:
      if t - self.sessionInitTime < 15.:
        self.apply_brake_percent = int(round(interp(t - self.sessionInitTime - 5., [0.,2.,4.,6.,8.,10.], ([100,0]*3))) % 100)
    ret.frictionBrakePercent = self.apply_brake_percent
    

    ret.gas = pt_cp.vl["AcceleratorPedal"]["AcceleratorPedal"] / 254.
    ret.gasPressed = ret.gas > 1e-5
    self.gasPressed = ret.gasPressed

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]["LKATorqueDeliveredStatus"]
    ret.steerWarning = self.lkas_status == 2
    ret.steerError = self.lkas_status == 3
    
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]['LKATorqueDelivered']
    engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']
    if self.engineRPM - engineRPM > 3000:
      self.engineRPM = engineRPM + 4096 # values above 4096 roll over to zero, so shift them
    else:
      self.engineRPM = engineRPM
    ret.engineRPM = self.engineRPM
    
    ret.engineCoolantTemp = pt_cp.vl["ECMEngineCoolantTemp"]['EngineCoolantTemp']

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    self.blinker = (ret.leftBlinker or ret.rightBlinker)
    if not self.disengage_on_gas and (self.pause_long_on_gas_press or self.v_cruise_kph * CV.KPH_TO_MPH <= 10.) and self.one_pedal_pause_steering_enabled:
      cur_time = sec_since_boot()
      if self.blinker and not self.prev_blinker:
        self.lang_change_ramp_down_steer_start_t = cur_time
      elif not self.blinker and self.prev_blinker:
        self.lang_change_ramp_up_steer_start_t = cur_time

      self.lane_change_steer_factor = interp(self.vEgo, [self.min_lane_change_speed * 0.9, self.min_lane_change_speed], [0., 1.])

      if self.blinker:
        self.lane_change_steer_factor = interp(cur_time - self.lang_change_ramp_down_steer_start_t, [0., self.lang_change_ramp_steer_dur * 4.], [1., self.lane_change_steer_factor])
      else:
        self.lane_change_steer_factor = interp(cur_time - self.lang_change_ramp_up_steer_start_t, [0., self.lang_change_ramp_steer_dur], [self.lane_change_steer_factor, 1.])
    else:
      self.lane_change_steer_factor = 1.0
    self.prev_blinker = self.blinker
    

    self.park_brake = pt_cp.vl["EPBStatus"]["EPBClosed"]
    ret.cruiseState.available = bool(pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"])
    self.cruiseMain = ret.cruiseState.available
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]

    ret.brakePressed = ret.brake > 1e-5
    # Regen braking is braking
    if self.car_fingerprint == CAR.VOLT:
      self.regenPaddlePressed = bool(pt_cp.vl["EBCMRegenPaddle"]['RegenPaddle'])
      ret.brakePressed = ret.brakePressed or self.regenPaddlePressed

    ret.cruiseState.enabled = self.pcm_acc_status != AccState.OFF
    ret.cruiseState.standstill = False
    
    one_pedal_mode_active = (self.one_pedal_mode_enabled and ret.cruiseState.enabled and self.v_cruise_kph * CV.KPH_TO_MS <= self.one_pedal_mode_max_set_speed)
    coast_one_pedal_mode_active = (ret.cruiseState.enabled and self.v_cruise_kph * CV.KPH_TO_MS <= self.one_pedal_mode_max_set_speed)
    if one_pedal_mode_active != self.one_pedal_mode_active or coast_one_pedal_mode_active != self.coast_one_pedal_mode_active:
      if one_pedal_mode_active or coast_one_pedal_mode_active:
        self.one_pedal_last_follow_level = self.follow_level
        self.one_pedal_brake_mode = min(1, self.one_pedal_last_brake_mode)
        self.follow_level = self.one_pedal_brake_mode + 1
      else:
        self.one_pedal_last_brake_mode = min(self.one_pedal_brake_mode, 1)
        self.follow_level = self.one_pedal_last_follow_level
        
    self.coast_one_pedal_mode_active = coast_one_pedal_mode_active
    ret.coastOnePedalModeActive = self.coast_one_pedal_mode_active
    self.one_pedal_mode_active = one_pedal_mode_active
    ret.onePedalModeActive = self.one_pedal_mode_active
    ret.onePedalBrakeMode = self.one_pedal_brake_mode
    
    if t - self.pitch_check_last > self.pitch_check_freq:
      self.pitch_check_last = t
      self.pitch_rolling_iter += 1
      if (self.pitch_rolling_iter >= self.pitch_num_vals):
        self.pitch_rolling_iter = 0
      self.pitch -= self.pitch_vals[self.pitch_rolling_iter] * self.pitch_num_vals_recip
      self.pitch += self.pitch_raw * self.pitch_num_vals_recip
      self.pitch_vals[self.pitch_rolling_iter] = self.pitch_raw
    ret.pitch = self.pitch

    ret.autoHoldActivated = self.autoHoldActivated
    
    ret.lkMode = self.lkMode
    

    return ret

  def get_follow_level(self):
    return self.follow_level

  @staticmethod
  def get_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("BrakePedalPosition", "EBCMBrakePedalPosition", 0),
      ("FrontLeftDoor", "BCMDoorBeltStatus", 0),
      ("FrontRightDoor", "BCMDoorBeltStatus", 0),
      ("RearLeftDoor", "BCMDoorBeltStatus", 0),
      ("RearRightDoor", "BCMDoorBeltStatus", 0),
      ("LeftSeatBelt", "BCMDoorBeltStatus", 0),
      ("RightSeatBelt", "BCMDoorBeltStatus", 0),
      ("TurnSignals", "BCMTurnSignals", 0),
      ("AcceleratorPedal", "AcceleratorPedal", 0),
      ("CruiseState", "AcceleratorPedal2", 0),
      ("ACCButtons", "ASCMSteeringButton", CruiseButtons.UNPRESS),
      ("DriveModeButton", "ASCMSteeringButton", 0),
      ("LKAButton", "ASCMSteeringButton", 0),
      ("SteeringWheelAngle", "PSCMSteeringAngle", 0),
      ("SteeringWheelRate", "PSCMSteeringAngle", 0),
      ("FLWheelSpd", "EBCMWheelSpdFront", 0),
      ("FRWheelSpd", "EBCMWheelSpdFront", 0),
      ("RLWheelSpd", "EBCMWheelSpdRear", 0),
      ("RRWheelSpd", "EBCMWheelSpdRear", 0),
      ("PRNDL", "ECMPRDNL", 0),
      ("LKADriverAppldTrq", "PSCMStatus", 0),
      ("LKATorqueDelivered", "PSCMStatus", 0),
      ("LKATorqueDeliveredStatus", "PSCMStatus", 0),
      ("DistanceButton", "ASCMSteeringButton", 0),
      ("LKATorqueDelivered", "PSCMStatus", 0),
      ("EngineRPM", "ECMEngineStatus", 0),
      ("EngineCoolantTemp", "ECMEngineCoolantTemp", 0),
      ("TractionControlOn", "ESPStatus", 0),
      ("EPBClosed", "EPBStatus", 0),
      ("CruiseMainOn", "ECMEngineStatus", 0),
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("ECMPRDNL", 10),
      ("PSCMStatus", 10),
      ("ESPStatus", 10),
      ("BCMDoorBeltStatus", 10),
      ("EPBStatus", 20),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("AcceleratorPedal", 33),
      ("AcceleratorPedal2", 33),
      ("ASCMSteeringButton", 33),
      ("ECMEngineStatus", 100),
      ("PSCMSteeringAngle", 100),
      ("EBCMBrakePedalPosition", 100),
      ("ECMEngineCoolantTemp", 1),
    ]

    if CP.carFingerprint == CAR.VOLT:
      signals += [
        ("RegenPaddle", "EBCMRegenPaddle", 0),
      ]
      checks += [
        ("EBCMRegenPaddle", 50),
      ]
      
    signals += [
      ("TractionControlOn", "ESPStatus", 0),
      ("EPBClosed", "EPBStatus", 0),
      ("CruiseMainOn", "ECMEngineStatus", 0),
      ("CruiseState", "AcceleratorPedal2", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)
