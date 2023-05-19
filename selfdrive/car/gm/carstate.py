from cereal import car
from common.filter_simple import FirstOrderFilter
from common.op_params import opParams, UI_METRICS
from common.params import Params, put_nonblocking
from common.numpy_fast import mean, interp, clip
from common.realtime import sec_since_boot, DT_CTRL
from math import sin, cos
from selfdrive.config import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, \
                                    CruiseButtons, STEER_THRESHOLD, CarControllerParams
from selfdrive.controls.lib.drive_helpers import set_v_cruise_offset, ClusterSpeed
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

GAS_PRESSED_THRESHOLD = 0.15

GearShifter = car.CarState.GearShifter
class GEAR_SHIFTER2:
  IN_BETWEEN_PARK_AND_REVERSE = 0
  PARK = 1
  REVERSE = 2
  NEUTRAL = 3
  DRIVE = 4
  REGEN_PADDLE_DRIVE = 5
  LOW = 6
  REGEN_PADDLE_LOW = 7
  
def ev_regen_accel(v_ego, ice_on):
  gas_brake_threshold = interp(v_ego, CarControllerParams.EV_GAS_BRAKE_THRESHOLD_BP, CarControllerParams.EV_GAS_BRAKE_THRESHOLD_ICE_V if ice_on else CarControllerParams.EV_GAS_BRAKE_THRESHOLD_V)
  return gas_brake_threshold

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["PRNDL"]
    self.steering_pressed_filter = FirstOrderFilter(0, 0.05, 0.01)
    self._params = Params()
    self._op_params = opParams("gm CarState")
    self.hvb_wattage = FirstOrderFilter(0.0, self._op_params.get('MET_power_meter_smoothing_factor'), DT_CTRL) # [kW]
    self.ui_metrics_params = [int(self._params.get(f'MeasureSlot{i:02d}', encoding="utf8")) for i in range(10)]
    self.cruise_resume_high_accel_ramp_bp = [3.0, 6.0] # seconds of using accel_mode + 1 accel after resuming
    self.cruise_resume_high_accel_ramp_v = [1.0, 0.0]
    self.accel_limits_rate_limits = [i * DT_CTRL for i in [-1.0, 0.5]]
    self.update_op_params()
    
    self.gear_shifter = 'park'
    self.cluster_speed = ClusterSpeed(is_metric=self._params.get_bool("IsMetric"))
    
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
    self.park_assist_active = False
    
    self.t = 0.
    self.is_ev = (self.car_fingerprint in [CAR.VOLT, CAR.VOLT18])
    self.do_sng = (self.car_fingerprint in [CAR.VOLT])
    
    self.parked_timer = 0
    self.parked_timer_min_time = 120
    
    self.sessionInitTime = sec_since_boot()
    self.prev_distance_button = 0
    self.prev_lka_button = 0
    self.lka_button = 0
    self.distance_button = 0
    self.distance_button_last_press_t = 0.
    self.follow_level = int(self._params.get("FollowLevel", encoding="utf8"))
    self.follow_level_change_last_t = self.sessionInitTime
    self.lkaEnabled = True
    self.cruise_offset_enabled = self._params.get_bool("CruiseSpeedOffset")
    set_v_cruise_offset(self._op_params.get('MISC_set_speed_offset_mph', force_update=True) if self.cruise_offset_enabled else 0)
    self.autoHold = self._params.get_bool("GMAutoHold")
    self.MADS_enabled = self._params.get_bool("MADSEnabled")
    self.disengage_on_gas = not self.MADS_enabled and not Params().get_bool("DisableDisengageOnGas")
    self.autoHoldActive = False
    self.autoHoldActivated = False
    self.regen_paddle_pressed = False
    self.regen_paddle_pressed_last_t = 0.0
    self.regen_paddle_released_last_t = 0.0
    self.cruiseMain = False
    self.standstill_time_since_t = 0.0
    self.engineRPM = 0
    self.lastAutoHoldTime = 0.0
    self.time_in_drive_autohold = 0.0
    self.time_in_drive_one_pedal = 0.0
    self.MADS_long_min_time_in_drive = 3.0 # [s]
    self.params_check_last_t = 0.
    self.params_check_freq = 0.25 # check params at 10Hz
    
    self.resume_button_pressed = False
    self.resume_required = False
    
    self.accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco
    self.accel_mode_change_last_t = self.sessionInitTime
    
    self.coasting_allowed = self._params.get_bool("Coasting")
    self.coasting_enabled = self.coasting_allowed and self._params.get_bool("CoastingActive")
    self.coasting_dl_enabled = self.is_ev and self._params.get_bool("CoastingDL")
    self.coasting_enabled_last = self.coasting_enabled
    self.no_friction_braking = self._params.get_bool("RegenBraking")
    self.coasting_brake_over_speed_enabled = self._params.get_bool("CoastingBrakeOverSpeed")
    base_BP = self._op_params.get('MISC_coasting_speed_over', force_update=True)
    self.coasting_over_speed_vEgo_BP = [[i + 0.1 for i in base_BP], [i + 0.15 for i in base_BP]]
    self.coasting_over_speed_regen_vEgo_BP = [base_BP, [i + 0.05 for i in base_BP]]
    self.coasting_over_speed_vEgo_BP_BP = [i * CV.MPH_TO_MS for i in [20., 80.]]
    self.coasting_long_plan = ""
    self.coasting_lead_d = -1. # [m] lead distance. -1. if no lead
    self.coasting_lead_v = -1.
    self.coasting_lead_a = 10.
    self.tr = 1.8
    self.pause_long_on_gas_press = False
    self.last_pause_long_on_gas_press_t = 0.
    self.gasPressed = False
    
    self.one_pedal_mode_enabled = self.MADS_enabled and self._params.get_bool("MADSOnePedalMode")
    self.MADS_lead_braking_enabled = self.MADS_enabled and self._params.get_bool("MADSLeadBraking")
    self.MADS_lead_braking_active = False
    self.one_pedal_dl_coasting_enabled = True
    self.one_pedal_mode_active = self.one_pedal_mode_enabled
    self.long_active = False
    self.one_pedal_mode_temporary = False
    self.one_pedal_mode_regen_paddle_double_press_time = 0.7
    self.regen_paddle_under_speed_pressed_time = 0.0
    self.v_ego_prev = 0.0
    self.MADS_pause_steering_enabled = self._params.get_bool("MADSPauseBlinkerSteering")
    
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
    self.hvb_wattage_bp = [0., 53.] # [kW], based on the banned user BZZT's testimony at https://www.gm-volt.com/threads/using-regen-paddle-and-l-drive-mode-summary.222289/
    self.apply_brake_percent = 0. if self.showBrakeIndicator else -1. # for brake percent on ui
    self.vEgo = 0.
    self.v_cruise_kph = 1
    self.min_lane_change_speed = self._op_params.get('MADS_steer_pause_speed_mph', force_update=True) * CV.MPH_TO_MS
    self.blinker = False
    self.prev_blinker = self.blinker
    self.lane_change_steer_factor = 1.
    self.steer_unpaused = False # used to make is so it will only pause once per blinker use
    self.min_steer_speed = CP.minSteerSpeed
    self.steer_pause_rate = 0.6 * DT_CTRL # 0.6 seconds to ramp up/down steering for pause
    self.steer_pause_a_ego_min = 0.1
    self.a_ego_filtered_rc = 1.0
    self.a_ego_filtered = FirstOrderFilter(0.0, self.a_ego_filtered_rc, DT_CTRL)
    
    self.low_visibility_active = False
    self.slippery_roads_active = False
    self.low_visibility_activated_t = 0.0
    self.slippery_roads_activated_t = 0.0
    
    self.reboot_in_N_seconds = -1
    
    self.lka_steering_cmd_counter = 0
    
  
  def update_op_params(self, t = sec_since_boot()):
    global GAS_PRESSED_THRESHOLD
    self.REGEN_PADDLE_STOP_SPEED = self._op_params.get('MADS_OP_one_time_stop_threshold_mph') * CV.MPH_TO_MS
    self.REGEN_PADDLE_STOP_PRESS_TIME = self._op_params.get('MADS_OP_one_time_stop_hold_s')
    
    base_BP = self._op_params.get('MISC_coasting_speed_over')
    self.coasting_over_speed_vEgo_BP = [[i + 0.1 for i in base_BP], [i + 0.15 for i in base_BP]]
    self.coasting_over_speed_regen_vEgo_BP = [base_BP, [i + 0.05 for i in base_BP]]
    
    self.hvb_wattage.update_alpha(self._op_params.get('MET_power_meter_smoothing_factor'))
    self.one_pedal_mode_regen_paddle_double_press_time = self._op_params.get('MADS_OP_double_press_time_s')
    self.min_lane_change_speed = self._op_params.get('MADS_steer_pause_speed_mph') * CV.MPH_TO_MS
    GAS_PRESSED_THRESHOLD = max(1e-5, self._op_params.get('TUNE_LONG_gas_overlap_cutoff') * 0.01)
    
    self.parked_timer_min_time = self._op_params.get('MISC_parked_timer_min_time_s')
    self.accel_limits_rate_limits = [i * DT_CTRL for i in self._op_params.get('AP_accel_limit_rates_ms3')]
    self.accel_limits_rate_speed_cutoff = self._op_params.get('AP_accel_limit_rates_speed_cutoff_mph') * CV.MPH_TO_MS
    
    cruise_resume_high_accel_ramp_dur = self._op_params.get('AP_post_resume_fast_accel_s')
    if cruise_resume_high_accel_ramp_dur != self.cruise_resume_high_accel_ramp_bp[-1]:
      self.cruise_resume_high_accel_ramp_bp = [cruise_resume_high_accel_ramp_dur/2, cruise_resume_high_accel_ramp_dur] # seconds of using accel_mode + 1 accel after resuming
    
    for i in range(10):
      key_op = f'MET_{i:02d}'
      key = f'MeasureSlot{i:02d}'
      metric_param = int(self._params.get(key, encoding="utf8"))
      if metric_param != self.ui_metrics_params[i]:
        if len(UI_METRICS) > self.ui_metrics_params[i]:
          cloudlog.info(f"opParams: UI metric in slot {i} '{UI_METRICS[self.ui_metrics_params[i]]}' ({self.ui_metrics_params[i]}) updated to '{UI_METRICS[metric_param]}' ({metric_param}) onroad. Copying new metric to opParams '{key_op}'")
          self._op_params.put(key_op, UI_METRICS[metric_param])
          self.ui_metrics_params[i] = metric_param
        else:
          cloudlog.error(f"Failed to sync opparams and params in gm/carstate due to out of bounds error. {len(UI_METRICS) = } ≤ {self.ui_metrics_params[i] = }")

    
  def update(self, pt_cp, loopback_cp, chassis_cp):
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
    if ret.vEgo < 3.0:
      self.a_ego_filtered.x = ret.aEgo
    else:
      self.a_ego_filtered.update(ret.aEgo)
    
    self.vEgo = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.01
    
    if ret.standstill:
      self.standstill_time_since_t = 0.0
    else:
      self.standstill_time_since_t += DT_CTRL

    self.coasting_enabled_last = self.coasting_enabled
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      self.reboot_in_N_seconds = int(self._params.get("OPParamsRebootInNSeconds", encoding="utf8"))
      self.update_op_params(t)
      set_v_cruise_offset(self._op_params.get('MISC_set_speed_offset_mph') if self.cruise_offset_enabled else 0)
      self.coasting_allowed = self._params.get_bool("Coasting")
      self.coasting_enabled = self.coasting_allowed and self._params.get_bool("CoastingActive")
      accel_mode = int(self._params.get("AccelMode", encoding="utf8"))  # 0 = normal, 1 = sport; 2 = eco
      if accel_mode != self.accel_mode:
        self.accel_mode_change_last_t = t
      self.accel_mode = accel_mode
      self.showBrakeIndicator = self._params.get_bool("BrakeIndicator")
      if not self.disengage_on_gas:
        self.MADS_pause_steering_enabled = self._params.get_bool("MADSPauseBlinkerSteering")
        self.one_pedal_mode_enabled = self.MADS_enabled and self._params.get_bool("MADSOnePedalMode")
        self.MADS_lead_braking_enabled = self.MADS_enabled and self._params.get_bool("MADSLeadBraking")
    
    self.angle_steers = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
      
    gear_shifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    ret.clusterSpeed = self.cluster_speed.update(ret.vEgo, do_reset=self.gear_shifter != gear_shifter)
    self.gear_shifter = gear_shifter
    ret.gearShifter = self.gear_shifter
    ret.brakePressure = chassis_cp.vl["EBCMFrictionBrakeStatus"]["FrictionBrakePressure"]
    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0 
    ret.brakePressed = ret.brakePressed and pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"] >= 15
    if ret.brakePressed:
      self.user_brake = pt_cp.vl["ECMAcceleratorPos"]['BrakePedalPos']
      ret.brake = pt_cp.vl["ECMAcceleratorPos"]["BrakePedalPos"] / 0xd0
    else:
      self.user_brake = 0.
      ret.brake = 0.
    
    if ret.gearShifter in ['drive','low']:
      if ret.brakePressed:
        self.time_in_drive_autohold = self.MADS_long_min_time_in_drive
      else:
        self.time_in_drive_autohold += DT_CTRL
      self.time_in_drive_one_pedal += DT_CTRL
    else:
      self.time_in_drive_autohold = 0.0
      self.time_in_drive_one_pedal = 0.0
    
    
    if self.showBrakeIndicator:
      if t - self.sessionInitTime < 13.:
        self.apply_brake_percent = int(round(interp(t - self.sessionInitTime, [i * 3. for i in range(4)], ([100,0]*2))))
    ret.frictionBrakePercent = int(round(self.apply_brake_percent))
    
    self.park_assist_active = chassis_cp.vl["PACMParkAssitCmd"]["RequestActive"] == 1
    

    ret.gas = pt_cp.vl["AcceleratorPedal2"]["AcceleratorPedal2"] / 254.
    ret.gasPressed = ret.gas > GAS_PRESSED_THRESHOLD
    self.gasPressed = ret.gasPressed
    if self.gasPressed or self.vEgo < 0.2:
      self.resume_required = False

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(self.steering_pressed_filter.update(ret.steeringTorque)) > STEER_THRESHOLD
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
    

    self.park_brake = pt_cp.vl["EPBStatus"]["EPBClosed"]
    ret.cruiseState.available = pt_cp.vl["ECMEngineStatus"]["CruiseMainOn"] != 0
    self.prev_cruise_main = self.cruiseMain
    self.cruiseMain = ret.cruiseState.available
    if self.cruiseMain and not self.prev_cruise_main:
      if self.MADS_enabled:
        self.lkaEnabled = self._params.get_bool("MADSAutosteerEnabled")
    elif not self.cruiseMain:
      self.lkaEnabled = False
    ret.espDisabled = pt_cp.vl["ESPStatus"]["TractionControlOn"] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]["CruiseState"]
    

    # Regen braking is braking
    if self.is_ev:
      regen_paddle_pressed = pt_cp.vl["EBCMRegenPaddle"]["RegenPaddle"] != 0
      ret.regenPaddlePressed = regen_paddle_pressed
      hvb_current = pt_cp.vl["BECMBatteryVoltageCurrent"]['HVBatteryCurrent']
      hvb_voltage = pt_cp.vl["BECMBatteryVoltageCurrent"]['HVBatteryVoltage']
      self.hvb_wattage.update(hvb_current * hvb_voltage)
      ret.hvbVoltage = hvb_voltage
      ret.hvbCurrent = hvb_current
      ret.hvbWattage = self.hvb_wattage.x
      self.gear_shifter_ev_last = self.gear_shifter_ev
      self.gear_shifter_ev = pt_cp.vl["ECMPRDNL2"]['PRNDL2']
      
      if ret.gas > 1e-5 and self.one_pedal_mode_temporary:
        self.one_pedal_mode_active = False
        self.one_pedal_mode_temporary = False
        cloudlog.info("Deactivating temporary one-pedal mode with gas press")
      
      if regen_paddle_pressed and not self.regen_paddle_pressed:
        if self.MADS_enabled and t - self.regen_paddle_pressed_last_t <= self.one_pedal_mode_regen_paddle_double_press_time:
          self.one_pedal_mode_active = not self.one_pedal_mode_active
          self.one_pedal_mode_temporary = False
          put_nonblocking("MADSOnePedalMode", str(int(self.one_pedal_mode_active))) # persists across drives
          cloudlog.info(f"Toggling one-pedal mode with double-regen press. New value: {self.one_pedal_mode_active}")
        self.regen_paddle_pressed_last_t = t
      elif self.MADS_enabled \
          and not self.one_pedal_mode_active \
          and regen_paddle_pressed \
          and self.regen_paddle_pressed \
          and ((ret.vEgo < self.REGEN_PADDLE_STOP_SPEED \
            and self.v_ego_prev >= self.REGEN_PADDLE_STOP_SPEED) \
            or self.regen_paddle_under_speed_pressed_time >= self.REGEN_PADDLE_STOP_PRESS_TIME):
        self.one_pedal_mode_active = True
        self.one_pedal_mode_temporary = True
        cloudlog.info("Activating temporary one-pedal mode")
      
      if not regen_paddle_pressed and self.regen_paddle_pressed:
        self.regen_paddle_released_last_t = t
      
      if regen_paddle_pressed and self.regen_paddle_pressed and ret.vEgo < self.REGEN_PADDLE_STOP_SPEED:
        self.regen_paddle_under_speed_pressed_time += DT_CTRL
      else:
        self.regen_paddle_under_speed_pressed_time = 0.0
      
      self.regen_paddle_pressed = regen_paddle_pressed
    
    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    self.blinker = (ret.leftBlinker or ret.rightBlinker)
    if self.blinker and self.vEgo <= self.min_lane_change_speed \
        and (self.a_ego_filtered.x <= self.steer_pause_a_ego_min \
          or self.vEgo <= self.min_steer_speed * 0.5) \
        and not self.long_active \
        and (self.lkaEnabled and self.cruiseMain and self.MADS_enabled) \
        and self.MADS_pause_steering_enabled \
        and not self.steer_unpaused:
      lane_change_steer_factor = 0.0
    else:
      if self.blinker and self.lane_change_steer_factor < 0.5:
        self.steer_unpaused = True
      lane_change_steer_factor = 1.0
    
    if not self.blinker:
      self.steer_unpaused = False
    
    self.lane_change_steer_factor = clip(lane_change_steer_factor, 
                                         self.lane_change_steer_factor - self.steer_pause_rate, 
                                         self.lane_change_steer_factor + self.steer_pause_rate)
    
    self.prev_blinker = self.blinker
    
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
          self.regen_power = (self.hvb_wattage.x * self.observed_efficiency.x) if self.hvb_wattage.x > 0. else 0.
          self.regen_force = (self.regen_power / self.vEgo) if self.vEgo > 0.3 else 0.
          if self.engineRPM > 0:
            self.ice_power = self.drive_power / max(0.1,self.observed_efficiency.x) + self.hvb_wattage.x  # [W]
            self.ev_power = -((self.hvb_wattage.x + self.ice_power) * self.observed_efficiency.x) if self.hvb_wattage.x <= 0. else 0.
          else:
            self.ev_power = -(self.hvb_wattage.x * self.observed_efficiency.x) if self.hvb_wattage.x <= 0. else 0.
            self.ice_power = 0.
            if self.vEgo > 0.3 and ret.gearShifter != GearShifter.reverse and self.drive_power > 10000. and self.hvb_wattage.x < -10000.:
              eff = self.drive_power / (-self.hvb_wattage.x)
              if eff < 1.0:
                self.observed_efficiency.update(eff)
        else:
          if self.engineRPM == 0:
            self.ice_power = 0.
            self.regen_power = (self.hvb_wattage.x * self.observed_efficiency.x) if self.hvb_wattage.x > 0. else 0.
            self.regen_force = (self.regen_power / self.vEgo) if self.vEgo > 0.3 else 0.
            self.ev_power = -(self.hvb_wattage.x * self.observed_efficiency.x) if self.hvb_wattage.x <= 0. else 0.
            self.brake_power = -(self.drive_power + self.regen_power)
            if self.vEgo > 0.3 and ret.gearShifter != GearShifter.reverse and self.drive_power < -5000. and self.hvb_wattage.x > 5000. and self.brake_cmd == 0 and not ret.brakePressed:
              eff = -self.hvb_wattage.x / self.drive_power
              if eff < 1.0:
                self.observed_efficiency.update(eff)
          else: # assume full regen from dynamic gas/brake accel threshold which is a measure of available regen brake force
            self.regen_force = self.cp_mass * ev_regen_accel(self.vEgo+2.5, self.engineRPM > 0)
            self.regen_power = self.regen_force * self.vEgo  # [W]
            self.brake_power = self.drive_power - self.regen_power
            self.ice_power = -self.hvb_wattage.x - self.regen_power
            self.ev_power = -((self.hvb_wattage.x + self.ice_power) * self.observed_efficiency.x) if self.hvb_wattage.x <= 0. else 0.
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
    
    
    if self.coasting_allowed and self.gear_shifter in ['drive', 'low'] and self.long_active:
      if self.is_ev and self.coasting_dl_enabled:
        if not self.coasting_enabled and self.gear_shifter_ev == GEAR_SHIFTER2.DRIVE:
          self.coasting_enabled = True
          put_nonblocking("CoastingActive", "1")
        elif self.coasting_enabled and self.gear_shifter_ev == GEAR_SHIFTER2.LOW and (self.vEgo <= self.v_cruise_kph * CV.KPH_TO_MS or self.no_friction_braking):
          self.coasting_enabled = False
          put_nonblocking("CoastingActive", "0")
      else:
        if self.coasting_enabled != self.coasting_enabled_last:
          if not self.coasting_enabled and self.vEgo > self.v_cruise_kph * CV.KPH_TO_MS and not self.no_friction_braking:
            # prevent disable of coasting if over set speed and friction brakes not disabled.
            put_nonblocking("CoastingActive", "1")
            self.coasting_enabled = True
    else:
      self.coasting_enabled = False
      
    ret.coastingActive = self.coasting_enabled

    
    cruise_enabled = self.pcm_acc_status != AccState.OFF
    ret.cruiseState.enabled = cruise_enabled
    ret.cruiseState.standstill = False
    ret.cruiseMain = self.cruiseMain
    
    ret.onePedalModeActive = self.one_pedal_mode_active and not self.long_active and self.time_in_drive_one_pedal >= self.MADS_long_min_time_in_drive and not self.park_assist_active
    if self.long_active:
      ret.brakePressed = ret.brakePressed or self.regen_paddle_pressed
    ret.onePedalModeTemporary = self.one_pedal_mode_temporary
    ret.madsLeadBrakingActive = self.MADS_lead_braking_active and ret.vEgo > 0.02 and ret.gearShifter in ['drive','low'] and ret.gas < 1e-5 and not ret.brakePressed and ret.cruiseMain
    
    ret.slipperyRoadsActive = self.slippery_roads_active
    ret.lowVisibilityActive = self.low_visibility_active
    ret.rebootInNSeconds = int(self.reboot_in_N_seconds)
    
    if self.iter % 20 == 0:
      if self.slippery_roads_active:
        if self.follow_level != 2 and self.slippery_roads_activated_t > self.follow_level_change_last_t:
          self.follow_level = 2
          put_nonblocking("FollowLevel", str(int(self.follow_level)))
        if self.accel_mode != 2 and self.slippery_roads_activated_t > self.accel_mode_change_last_t:
          self.accel_mode = 2
          put_nonblocking("AccelMode", str(int(self.accel_mode)))
      if self.low_visibility_active:
        if self.follow_level != 2 and self.low_visibility_activated_t > self.follow_level_change_last_t:
          self.follow_level = 2
          put_nonblocking("FollowLevel", str(int(self.follow_level)))
    
    self.pitch = self.pitch_ema * self.pitch_raw + (1 - self.pitch_ema) * self.pitch 
    ret.pitch = self.pitch

    ret.autoHoldActivated = self.autoHoldActivated
    
    ret.lkaEnabled = self.lkaEnabled
    self.v_ego_prev = ret.vEgo
    
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
  def get_chassis_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
        # sig_name, sig_address, default
        ("FrictionBrakePressure", "EBCMFrictionBrakeStatus", 0),
        ("RequestActive", "PACMParkAssitCmd", 0),
    ]
    
    checks = [
      ("EBCMFrictionBrakeStatus", 1),
      ("PACMParkAssitCmd", 1),
    ]

    return CANParser(DBC[CP.carFingerprint]['chassis'], signals, checks, CanBus.CHASSIS)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "ASCMLKASteeringCmd", 0),
    ]

    checks = [
      ("ASCMLKASteeringCmd", 50),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK)
