from cereal import car
from common.filter_simple import FirstOrderFilter
from common.params import Params, put_nonblocking
from common.numpy_fast import mean, interp
from common.realtime import sec_since_boot
from math import sin, cos
from selfdrive.config import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, \
                                    CruiseButtons, STEER_THRESHOLD, CarControllerParams
from selfdrive.controls.lib.drive_helpers import set_v_cruise_offset
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.swaglog import cloudlog

DRAG_FROM_MASS_BP = [1607., 3493.] # [kg; volt to suburban]
DRAG_Cd_FROM_MASS_V = [0.28, 0.35] # [drag coeffient for volt and suburban from https://www.tesla.com/sites/default/files/blog_attachments/the-slipperiest-car-on-the-road.pdf and https://www.automobile-catalog.com/car/2022/2970545/chevrolet_suburban_6_2l_v8_4wd.html]
DRAG_FRONTAL_AREA_FROM_MASS_V = [2.2, 3.96] # [m^2 for same cars from same source]
AIR_DENS_FROM_ELEV_BP = [-1000., 0., 1000., 2000., 3000., 4000., 5000., 6000.] # [m] highest road in the world is ~5800m
AIR_DENS_FROM_ELEV_V = [1.347, 1.225, 1.112, 1.007, 0.9093, 0.8194, 0.7364, 0.6601] # [kg/m^3] from https://www.engineeringtoolbox.com/standard-atmosphere-d_604.html
ROLLING_RESISTANCE_FROM_VEGO_V = [0.01, 0.012, 0.015, 0.019] # [unitless] I used the 4-7% figure in this article and chose to interpret that as Volt = 4%, Suburban = 7%  https://www.tirebuyer.com/education/rolling-resistance-and-fuel-economy
ROLLING_RESISTANCE_FROM_VEGO_BP = [3., 16., 28., 39.] # [m/s] using plot at https://www.engineeringtoolbox.com/docs/documents/1303/car_tire_pressure_rolling_resistance.png
# following efficiencies taken from https://sciendo.com/pdf/10.2478/rtuect-2020-0041 page 5 (673)
# EV_ICE_INPUT_EFFICIENCY = 1/0.88
# EV_DRIVE_EFFICIENCY = 1/0.82

GearShifter = car.CarState.GearShifter
class GEAR_SHIFTER2:
  DRIVE = 4
  LOW = 6
  
def ev_regen_accel(v_ego, ice_on):
  gas_brake_threshold = interp(v_ego, CarControllerParams.EV_GAS_BRAKE_THRESHOLD_BP, CarControllerParams.EV_GAS_BRAKE_THRESHOLD_ICE_V if ice_on else CarControllerParams.EV_GAS_BRAKE_THRESHOLD_V)
  return gas_brake_threshold

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
    
    self.iter = 0
    self.uiframe = 5
    
    self.cp_mass = CP.mass
    self.drag_cd = interp(self.cp_mass, DRAG_FROM_MASS_BP, DRAG_Cd_FROM_MASS_V)
    self.drag_csa = interp(self.cp_mass, DRAG_FROM_MASS_BP, DRAG_FRONTAL_AREA_FROM_MASS_V)
    self.altitude = 194. # [m] starts at the median human altitude https://www.pnas.org/doi/10.1073/pnas.95.24.14009
    
    self.rho = interp(self.altitude, AIR_DENS_FROM_ELEV_BP, AIR_DENS_FROM_ELEV_V)
    self.drag_force = 0.
    self.accel_force = 0. 
    self.regen_force = 0.
    self.brake_force = 0.
    self.drag_power = 0.
    self.accel_power = 0. 
    self.regen_power = 0.
    self.drive_power = 0. 
    self.ice_power = 0.
    self.brake_power = 0.
    self.pitch_power = 0.
    self.rolling_resistance_force = 0.
    self.rolling_resistance_power = 0.
    self.observed_efficiency = FirstOrderFilter(float(self._params.get("EVDriveTrainEfficiency", encoding="utf8")), 50., 0.05)
    self.brake_cmd = 0
    
    self.t = 0.
    self.is_ev = (self.car_fingerprint in [CAR.VOLT, CAR.VOLT18])
    self.do_sng = (self.car_fingerprint in [CAR.VOLT])
    
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
    self.cruise_enabled_last_t = 0.
    self.cruise_enabled_last = False
    self.cruise_enabled_neg_accel_ramp_bp = [0.5, 1.0] # ramp up negative accel when engaging behind a lead over 0.75s with a .25s delay
    self.cruise_enabled_neg_accel_ramp_v = [0., 1.]
    self.engineRPM = 0
    self.lastAutoHoldTime = 0.0
    self.sessionInitTime = sec_since_boot()
    self.params_check_last_t = 0.
    self.params_check_freq = 0.1 # check params at 10Hz
    
    self.resume_button_pressed = False
    self.resume_required = False
    
    self.accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco; 3 = creep
    
    self.coasting_enabled = self._params.get_bool("Coasting")
    self.coasting_dl_enabled = self.is_ev and self._params.get_bool("CoastingDL")
    self.coasting_enabled_last = self.coasting_enabled
    self.no_friction_braking = self._params.get_bool("RegenBraking")
    self.coasting_brake_over_speed_enabled = self._params.get_bool("CoastingBrakeOverSpeed")
    self.coasting_over_speed_vEgo_BP = [[1.3, 1.2], [1.35, 1.25]]
    self.coasting_over_speed_regen_vEgo_BP = [[1.20, 1.10], [1.25, 1.15]]
    self.coasting_over_speed_vEgo_BP_BP = [i * CV.MPH_TO_MS for i in [20., 80.]]
    self.coasting_long_plan = ""
    self.coasting_lead_d = -1. # [m] lead distance. -1. if no lead
    self.coasting_lead_v = -1.
    self.coasting_lead_a = 10.
    self.tr = 1.8
    self.coast_one_pedal_mode_active = False
    self.coast_one_pedal_mode_active_last = False
    self.pause_long_on_gas_press = False
    self.last_pause_long_on_gas_press_t = 0.
    self.gasPressed = False
    
    self.lead_accel = 0.
    
    self.one_pedal_mode_enabled = self._params.get_bool("OnePedalMode") and not self.disengage_on_gas
    self.one_pedal_mode_op_braking_allowed = not self._params.get_bool("OnePedalModeSimple")
    self.one_pedal_mode_engage_on_gas_enabled = self._params.get_bool("OnePedalModeEngageOnGas") and (self.one_pedal_mode_enabled or not self.disengage_on_gas)
    self.one_pedal_dl_engage_on_gas_enabled = self.is_ev and self._params.get_bool("OnePedalDLEngageOnGas") and (self.one_pedal_mode_enabled or not self.disengage_on_gas)
    self.one_pedal_dl_coasting_enabled = self.is_ev and self._params.get_bool("OnePedalDLCoasting") and (self.one_pedal_mode_enabled or not self.disengage_on_gas)
    self.one_pedal_mode_engage_on_gas = False
    self.one_pedal_mode_engage_on_gas_min_speed = 2.5 * CV.MPH_TO_MS # gas press at or above this speed with engage on gas enabled and one-pedal mode will activate
    self.one_pedal_mode_max_set_speed = 3 * CV.MPH_TO_MS #  one pedal mode activates if cruise set at or below this speed
    self.one_pedal_mode_stop_apply_brake_bp = [[i * CV.MPH_TO_MS for i in [1., 4., 8., 45., 85.]], [i * CV.MPH_TO_MS for i in [1., 7., 14., 45., 85.]], [1.]]
    self.one_pedal_mode_stop_apply_brake_v = [[82., 90., 95., 115., 90.], [110., 150., 165., 185., 140.], [280.]] # three levels. 1-2 are cycled using follow distance press, and 3 by holding
    self.one_pedal_mode_apply_brake = 0.
    self.one_pedal_mode_active_last = False
    self.one_pedal_mode_last_gas_press_t = 0.
    self.one_pedal_mode_engaged_with_button = False
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
    self.one_pedal_coast_lead_dist_apply_brake_bp = [4.5, 15.] # [m] distance to lead
    self.one_pedal_coast_lead_dist_apply_brake_v = [1., 0.] # [unitless] factor of light one-pedal braking
    
    self.drive_mode_button = False
    self.drive_mode_button_last = False
    self.gear_shifter_ev = None
    self.gear_shifter_ev_last = None
          
    self.pitch = 0. # radians
    self.pitch_raw = 0. # radians
    self.pitch_ema = 1/100
    self.pitch_future_time = 0.5 # seconds
    
    
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
    self.hvb_wattage = 0. # [kW]
    self.hvb_wattage_bp = [0., 53.] # [kW], based on the banned user BZZT's testimony at https://www.gm-volt.com/threads/using-regen-paddle-and-l-drive-mode-summary.222289/
    self.apply_brake_percent = 0. if self.showBrakeIndicator else -1. # for brake percent on ui
    self.vEgo = 0.
    self.v_cruise_kph = 1
    self.min_lane_change_speed = 20. * CV.MPH_TO_MS
    self.blinker = False
    self.prev_blinker = self.blinker
    self.lane_change_steer_factor = 1.
    self.lang_change_ramp_up_steer_start_t = 0.
    self.lang_change_ramp_down_steer_start_t = 0.
    self.lang_change_ramp_steer_dur = .5 # [s]
    
    self.lka_steering_cmd_counter = 0

  def update(self, pt_cp, loopback_cp):
    ret = car.CarState.new_message()
    
    t = sec_since_boot()
    self.t = t
    
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
      self.showBrakeIndicator = self._params.get_bool("BrakeIndicator")
      if not self.disengage_on_gas:
        self.one_pedal_pause_steering_enabled = self._params.get_bool("OnePedalPauseBlinkerSteering")
        self.one_pedal_mode_enabled = self._params.get_bool("OnePedalMode")
        self.one_pedal_mode_op_braking_allowed = not self._params.get_bool("OnePedalModeSimple")
        self.one_pedal_mode_engage_on_gas_enabled = self._params.get_bool("OnePedalModeEngageOnGas") and (self.one_pedal_mode_enabled or not self.disengage_on_gas)

    self.angle_steers = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
      
    self.gear_shifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    ret.gearShifter = self.gear_shifter    
    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0 
    ret.brakePressed = ret.brakePressed and pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"] >= 15
    if ret.brakePressed:
      self.user_brake = pt_cp.vl["ECMAcceleratorPos"]['BrakePedalPos']
      ret.brake = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"] / 0xd0
    else:
      self.user_brake = 0.
      ret.brake = 0.
    
    if self.showBrakeIndicator:
      if t - self.sessionInitTime < 13.:
        self.apply_brake_percent = int(round(interp(t - self.sessionInitTime, [i * 3. for i in range(4)], ([100,0]*2))))
    ret.frictionBrakePercent = int(round(self.apply_brake_percent))
    

    ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
    ret.gasPressed = ret.gas > 1e-5
    self.gasPressed = ret.gasPressed
    if self.gasPressed or self.vEgo < 0.2:
      self.resume_required = False

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    self.lka_steering_cmd_counter = loopback_cp.vl["ASCMLKASteeringCmd"]["RollingCounter"]
    
    ret.lateralAcceleration = pt_cp.vl["EBCMVehicleDynamic"]["LateralAcceleration"]
    ret.yawRate = pt_cp.vl["EBCMVehicleDynamic"]["YawRate"]
    ret.yawRate2 = pt_cp.vl["EBCMVehicleDynamic"]["YawRate2"]

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
    if (self.coast_one_pedal_mode_active or self.one_pedal_mode_active) and (self.pause_long_on_gas_press or self.v_cruise_kph * CV.KPH_TO_MPH <= 10.) and self.one_pedal_pause_steering_enabled:
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
    ret.cruiseState.available = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0
    self.cruiseMain = ret.cruiseState.available
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]

    # Regen braking is braking
    if self.is_ev:
      self.regenPaddlePressed = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0
      ret.brakePressed = ret.brakePressed or self.regenPaddlePressed
      hvb_current = pt_cp.vl["BECMBatteryVoltageCurrent"]['HVBatteryCurrent']
      hvb_voltage = pt_cp.vl["BECMBatteryVoltageCurrent"]['HVBatteryVoltage']
      self.hvb_wattage = hvb_current * hvb_voltage
      ret.hvbVoltage = hvb_voltage
      ret.hvbCurrent = hvb_current
      ret.hvbWattage = self.hvb_wattage
      self.gear_shifter_ev_last = self.gear_shifter_ev
      self.gear_shifter_ev = pt_cp.vl["ECMPRDNL2"]['PRNDL2']
    
    if self.iter % self.uiframe == 0:
      # drag, drive, brake, and ice power
      self.rho = interp(self.altitude, AIR_DENS_FROM_ELEV_BP, AIR_DENS_FROM_ELEV_V) # [kg/m^3]
      self.drag_force = 0.5 * self.drag_cd * self.drag_csa * self.rho * self.vEgo**2 # [N]
      pitch_accel = ACCELERATION_DUE_TO_GRAVITY * sin(self.pitch)
      pitch_force = pitch_accel * self.cp_mass
      self.pitch_power = pitch_force * ret.aEgo
      pitch_adjusted_accel = ret.aEgo + pitch_accel # [m/s^2]
      self.accel_force = self.cp_mass * pitch_adjusted_accel # [N]
      self.drag_power = self.drag_force * self.vEgo # [W]
      self.accel_power = self.accel_force * self.vEgo  # [W]
      rolling_resistance = interp(self.vEgo, ROLLING_RESISTANCE_FROM_VEGO_BP, ROLLING_RESISTANCE_FROM_VEGO_V)
      self.rolling_resistance_force = rolling_resistance * self.cp_mass * ACCELERATION_DUE_TO_GRAVITY * cos(self.pitch)
      self.rolling_resistance_power = self.rolling_resistance_force * self.vEgo
      self.drive_power = self.drag_power + self.accel_power + self.rolling_resistance_power # [W]
      if self.is_ev:
        if self.drive_power > 0.:
          self.brake_power = 0.
          self.regen_power = (self.hvb_wattage * self.observed_efficiency.x) if self.hvb_wattage > 0. else 0.
          self.regen_force = (self.regen_power / self.vEgo) if self.vEgo > 0.3 else 0.
          if self.engineRPM > 0:
            self.ice_power = self.drive_power / max(0.1,self.observed_efficiency.x) + self.hvb_wattage  # [W]
            self.ev_power = -((self.hvb_wattage + self.ice_power) * self.observed_efficiency.x) if self.hvb_wattage <= 0. else 0.
          else:
            self.ev_power = -(self.hvb_wattage * self.observed_efficiency.x) if self.hvb_wattage <= 0. else 0.
            self.ice_power = 0.
            if self.vEgo > 0.3 and ret.gearShifter != GearShifter.reverse and self.drive_power > 10000. and self.hvb_wattage < -10000.:
              eff = self.drive_power / (-self.hvb_wattage)
              if eff < 1.0:
                self.observed_efficiency.update(eff)
        else:
          if self.engineRPM == 0:
            self.ice_power = 0.
            self.regen_power = (self.hvb_wattage * self.observed_efficiency.x) if self.hvb_wattage > 0. else 0.
            self.regen_force = (self.regen_power / self.vEgo) if self.vEgo > 0.3 else 0.
            self.ev_power = -(self.hvb_wattage * self.observed_efficiency.x) if self.hvb_wattage <= 0. else 0.
            self.brake_power = -(self.drive_power + self.regen_power)
            if self.vEgo > 0.3 and ret.gearShifter != GearShifter.reverse and self.drive_power < -5000. and self.hvb_wattage > 5000. and self.brake_cmd == 0 and not ret.brakePressed:
              eff = -self.hvb_wattage / self.drive_power
              if eff < 1.0:
                self.observed_efficiency.update(eff)
          else: # assume full regen from dynamic gas/brake accel threshold which is a measure of available regen brake force
            self.regen_force = self.cp_mass * ev_regen_accel(self.vEgo+2.5, self.engineRPM > 0)
            self.regen_power = self.regen_force * self.vEgo  # [W]
            self.brake_power = self.drive_power - self.regen_power
            self.ice_power = -self.hvb_wattage - self.regen_power
            self.ev_power = -((self.hvb_wattage + self.ice_power) * self.observed_efficiency.x) if self.hvb_wattage <= 0. else 0.
        self.drive_power /= max(0.1, self.observed_efficiency.x) # [W]
      else:        
        self.ev_power = 0.
        self.ev_force = 0.
        if self.drive_power > 0.:
          self.ice_power = self.drive_power
          self.brake_power = 0.
        else:
          if self.brake_cmd == 0 and not ret.brakePressed:
            self.ice_power = self.drive_power
            self.brake_power = 0.
          else:
            self.brake_power = -(self.drive_power - self.ice_power * min([1., self.engineRPM / 6000., self.vEgo / 12.])) # when brakes are in use, assume the ongoing presence of whatever engine brake force was available when brakes were first applied(the last value of ice_power), and then assume it diminishes linearly with speed and engine rpms; whichever is "lower"

      self.brake_force = (self.brake_power / self.vEgo) if self.vEgo > 0.3 else 0.
      self.ev_force = (self.ev_power / self.vEgo) if self.vEgo > 0.3 else 0.
        
      if (self.iter // self.uiframe) % 20 == 0:
        put_nonblocking("EVDriveTrainEfficiency", str(self.observed_efficiency.x))
        self.iter = 0
        
    
    ret.dragForce = self.drag_force
    ret.dragPower = self.drag_power
    ret.accelForce = self.accel_force
    ret.accelPower = self.accel_power
    ret.regenForce = self.regen_force
    ret.regenPower = self.regen_power
    ret.brakeForce = self.brake_force
    ret.brakePower = self.brake_power
    ret.drivePower = self.drive_power
    ret.pitchPower = self.pitch_power
    ret.icePower = self.ice_power
    ret.evForce = self.ev_force
    ret.evPower = self.ev_power
    ret.rollingForce = self.rolling_resistance_force
    ret.rollingPower = self.rolling_resistance_power
    ret.observedEVDrivetrainEfficiency = self.observed_efficiency.x
    
    
    if self.is_ev and self.coasting_dl_enabled:
      if not self.coasting_enabled and self.gear_shifter_ev == GEAR_SHIFTER2.DRIVE:
        self.coasting_enabled = True
        put_nonblocking("Coasting", "1")
      elif self.coasting_enabled and self.gear_shifter_ev == GEAR_SHIFTER2.LOW and (self.vEgo <= self.v_cruise_kph * CV.KPH_TO_MS or self.no_friction_braking):
        self.coasting_enabled = False
        put_nonblocking("Coasting", "0")
    else:
      if self.coasting_enabled != self.coasting_enabled_last:
        if not self.coasting_enabled and self.vEgo > self.v_cruise_kph * CV.KPH_TO_MS and not self.no_friction_braking:
          # prevent disable of coasting if over set speed and friction brakes not disabled.
          put_nonblocking("Coasting", "1")
          self.coasting_enabled = True
    ret.coastingActive = self.coasting_enabled
    
    if self.is_ev and self.one_pedal_dl_engage_on_gas_enabled:
      if not self.one_pedal_mode_engage_on_gas_enabled and self.gear_shifter_ev == GEAR_SHIFTER2.LOW:
        self.one_pedal_mode_engage_on_gas_enabled = True
        put_nonblocking("OnePedalModeEngageOnGas", "1")
      elif self.one_pedal_mode_engage_on_gas_enabled and self.gear_shifter_ev == GEAR_SHIFTER2.DRIVE:
        self.one_pedal_mode_engage_on_gas_enabled = False
        put_nonblocking("OnePedalModeEngageOnGas", "0")

    
    cruise_enabled = self.pcm_acc_status != AccState.OFF
    ret.cruiseState.enabled = cruise_enabled
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
    
    if (cruise_enabled and not self.cruise_enabled_last) \
       or (not one_pedal_mode_active and self.one_pedal_mode_active) \
       or (not coast_one_pedal_mode_active and self.coast_one_pedal_mode_active):
      if self.is_ev and self.gear_shifter_ev == GEAR_SHIFTER2.LOW:
        self.cruise_enabled_neg_accel_ramp_v[0] = 0.15
      else:
        self.cruise_enabled_neg_accel_ramp_v[0] = 0.
      self.cruise_enabled_last_t = t
      
    self.cruise_enabled_last = cruise_enabled
        
    self.coast_one_pedal_mode_active = coast_one_pedal_mode_active
    ret.coastOnePedalModeActive = self.coast_one_pedal_mode_active
    self.one_pedal_mode_active = one_pedal_mode_active
    ret.onePedalModeActive = self.one_pedal_mode_active
    ret.onePedalBrakeMode = self.one_pedal_brake_mode
    
    self.pitch = self.pitch_ema * self.pitch_raw + (1 - self.pitch_ema) * self.pitch 
    ret.pitch = self.pitch

    ret.autoHoldActivated = self.autoHoldActivated
    
    ret.lkMode = self.lkMode
    
    self.iter += 1
    return ret

  def get_follow_level(self):
    return self.follow_level

  @staticmethod
  def get_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("FrontLeftDoor", "BCMDoorBeltStatus", 0),
      ("FrontRightDoor", "BCMDoorBeltStatus", 0),
      ("RearLeftDoor", "BCMDoorBeltStatus", 0),
      ("RearRightDoor", "BCMDoorBeltStatus", 0),
      ("LeftSeatBelt", "BCMDoorBeltStatus", 0),
      ("RightSeatBelt", "BCMDoorBeltStatus", 0),
      ("TurnSignals", "BCMTurnSignals", 0),
      ("AcceleratorPedal2", "AcceleratorPedal2", 0),
      ("BrakePedalPos", "ECMAcceleratorPos", 0),
      ("Brake_Pressed", "ECMEngineStatus", 0),
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
      ("LateralAcceleration", "EBCMVehicleDynamic", 0),
      ("YawRate", "EBCMVehicleDynamic", 0),
      ("YawRate2", "EBCMVehicleDynamic", 0),
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
      ("AcceleratorPedal2", 33),
      ("ECMAcceleratorPos", 100),
      ("ASCMSteeringButton", 33),
      ("ECMEngineStatus", 100),
      ("PSCMSteeringAngle", 100),
      ("ECMEngineCoolantTemp", 1),
      ("EBCMVehicleDynamic", 100),
    ]

    if CP.carFingerprint in [CAR.VOLT, CAR.VOLT18]:
      signals += [
        ("RegenPaddle", "EBCMRegenPaddle", 0),
        ("PRNDL2", "ECMPRDNL2", 0),
        ("HVBatteryVoltage", "BECMBatteryVoltageCurrent", 0),
        ("HVBatteryCurrent", "BECMBatteryVoltageCurrent", 0),
      ]
      checks += [
        ("EBCMRegenPaddle", 50),
        ("BECMBatteryVoltageCurrent", 10),
        ("ECMPRDNL2", 10),
      ]
      
    signals += [
      ("TractionControlOn", "ESPStatus", 0),
      ("EPBClosed", "EPBStatus", 0),
      ("CruiseMainOn", "ECMEngineStatus", 0),
      ("CruiseState", "AcceleratorPedal2", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "ASCMLKASteeringCmd", 0),
    ]

    checks = [
      ("ASCMLKASteeringCmd", 50),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK)
