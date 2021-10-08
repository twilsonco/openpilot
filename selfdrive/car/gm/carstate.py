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
    self.follow_level = 2
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
    
    self.coasting_enabled = self._params.get_bool("Coasting")
    self.coasting_brake_over_speed_enabled = self._params.get_bool("CoastingBrakeOverSpeed")
    self.coasting_over_speed_vEgo_BP = [i * CV.MPH_TO_MS for i in [10., 15.]]
    self.coasting_long_plan = ""
    self.coasting_lead_d = -1. # [m] lead distance. -1. if no lead
    self.coasting_lead_v = -1.
    self.coasting_lead_min_v = 5. * CV.MPH_TO_MS
    self.coasting_lead_min_rel_dist_s = 1.8 # [s] coasting logic isn't used at less than this follow distance
    self.coasting_lead_min_abs_dist = 40. # [m] coasting logic isn't used at less than this absolute follow distance
    self.coasting_lead_abs_dist_max_check_speed = 40. * CV.MPH_TO_MS
    self.pause_long_on_gas_press = False
    self.last_pause_long_on_gas_press_t = 0.
    self.gasPressed = False
    
    self.one_pedal_mode_enabled = self._params.get_bool("OnePedalMode")
    self.one_pedal_mode_max_set_speed = 5 * CV.MPH_TO_MS #  one pedal mode activates if cruise set at or below this speed
    self.one_pedal_mode_max_apply_brake = 200 # based on the values of CS.BRAKE_LOOKUP_V
    self.one_pedal_mode_stop_apply_brake_bp = [i * CV.MPH_TO_MS for i in [0., 1., 3., 15., 45., 55., 85.]]
    self.one_pedal_mode_stop_apply_brake_v = [80., 80., 110., 160., 160., 90., 90.]
    self.one_pedal_mode_last_gas_press_t = 0.
    self.one_pedal_mode_max_low_speed = 8. * CV.MPH_TO_MS
    self.one_pedal_mode_low_speed_time_offset = 0.5
    self.one_pedal_mode_ramp_time_bp = [0., 0.5, 1.0, 2.5, 3.0, 4.0, 4.5]
    self.one_pedal_mode_ramp_time_v = [0., 0., 0.3, 0.3, 0.7, 0.7, 1.0]
    
    self.showBrakeIndicator = self._params.get_bool("BrakeIndicator")
    self.apply_brake_percent = 0 if self.showBrakeIndicator else -1 # for brake percent on ui
    self.brake_alpha = 0.
    self.brake_fade_duration = 0.3 # [s] time it takes for the brake indicator to fade in/out
    self.brake_alpha_step = 1. / self.brake_fade_duration # will step in the transparent or opaque direction
    self.brake_alpha_last_t = 0. # to know how much to step each iteration
    self.vEgo = 0.
    self.v_cruise_kph = 0
    self.min_lane_change_speed = 30. * CV.MPH_TO_MS
    self.blinker = False
    self.prev_blinker = self.blinker
    self.lane_change_steer_factor = 1.
    self.lang_change_ramp_up_steer_start_t = 0.
    self.lang_change_ramp_down_steer_start_t = 0.
    self.lang_change_ramp_steer_dur = .5 # [s]

  def update(self, pt_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    self.prev_lka_button = self.lka_button
    self.lka_button = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]
    self.prev_distance_button = self.distance_button
    self.distance_button = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"]

    ret.wheelSpeeds.fl = pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    self.vEgo = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.01

    self.angle_steers = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
    self.gear_shifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    self.user_brake = pt_cp.vl["EBCMBrakePedalPosition"]['BrakePedalPosition']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]["PRNDL"], None))
    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0
    # Brake pedal's potentiometer returns near-zero reading even when pedal is not pressed.
    if ret.brake < 10/0xd0:
      ret.brake = 0.

    ret.frictionBrakePercent = self.apply_brake_percent

    t = sec_since_boot()
    if self.apply_brake_percent > 0:
      self.brake_alpha += self.brake_alpha_step * (t - self.brake_alpha_last_t)
      self.brake_alpha = min(1., self.brake_alpha)
      self.brake_alpha_last_t = t
    elif self.brake_alpha > 0.:
      self.brake_alpha -= self.brake_alpha_step * (t - self.brake_alpha_last_t)
      self.brake_alpha = max(0., self.brake_alpha)
    self.brake_alpha_last_t = t
    ret.frictionBrakeAlpha = self.brake_alpha

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
    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']

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
    if (self.coasting_enabled or not self.disengage_on_gas) and (self.pause_long_on_gas_press or self.v_cruise_kph * CV.KPH_TO_MPH <= 10.):
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

    ret.autoHoldActivated = self.autoHoldActivated

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
