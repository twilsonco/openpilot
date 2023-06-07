#!/usr/bin/env python3
from math import fabs, erf, atan, cos
from cereal import car
from common.numpy_fast import interp, clip, sign
from common.realtime import sec_since_boot
from common.op_params import opParams
from common.params import Params, put_nonblocking
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car.gm.values import CAR, CruiseButtons, \
                                    AccState, CarControllerParams, \
                                    FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase, FluxModel
from selfdrive.controls.lib.longitudinal_planner import _A_CRUISE_HIGHER_ACCEL_INDEX, \
                                                        calc_cruise_accel_limits
                                                        
GearShifter = car.CarState.GearShifter

FOLLOW_AGGRESSION = 0.15 # (Acceleration/Decel aggression) Lower is more aggressive

# revert to stock max negative accel based on relative lead velocity
_A_MIN_V_STOCK_FACTOR_BP = [-5. * CV.MPH_TO_MS, 1. * CV.MPH_TO_MS]
_A_MIN_V_STOCK_FACTOR_V = [0., 1.]


ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName


# meant for traditional ff fits
def get_steer_feedforward_sigmoid1(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (angle) / max(0.01,speed)
  sigmoid = x / (1. + fabs(x))
  return ((SIGMOID_COEF_RIGHT if angle > 0. else SIGMOID_COEF_LEFT) * sigmoid) * (0.01 + speed + SPEED_OFFSET) ** ANGLE_COEF2 + ANGLE_OFFSET * (angle * SPEED_COEF - atan(angle * SPEED_COEF))

# meant for torque fits
def get_steer_feedforward_erf(angle, speed,
                              ANGLE_COEF, 
                              ANGLE_COEF2, 
                              ANGLE_OFFSET, 
                              SPEED_OFFSET, 
                              SIGMOID_COEF_RIGHT, 
                              SIGMOID_COEF_LEFT, 
                              SPEED_COEF):
  x = ANGLE_COEF * (angle) * (40.0 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = erf(x) * (SIGMOID_COEF_RIGHT if angle < 0. else SIGMOID_COEF_LEFT)
  linear = ANGLE_COEF2 * angle
  return sigmoid + linear

def get_steer_feedforward_erf1(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = erf(x)
  sigmoid *= sigmoid_factor * sigmoid_factor
  sigmoid *= (40.23 / (max(0.05,speed + SPEED_OFFSET2))**SPEED_COEF2)
  linear = ANGLE_COEF2 * (angle + ANGLE_OFFSET)
  return sigmoid + linear


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    if CarState is not None:
      self.cp_chassis = self.CS.get_chassis_can_parser(CP)
    self.mads_one_pedal_enabled = False
    self.mads_one_pedal_temporary = False
    self.mads_lead_braking_enabled = False
    self.mads_cruise_main = False
    self.autosteer_enabled = False
    self.accel_limits_last = [-3.0, 1.5]
    self.cruise_enabled_last = False
    
  params_check_last_t = 0.
  params_check_freq = 0.1 # check params at 10Hz
  params = CarControllerParams()
  
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed, CI = None):
    following = CI.CS.coasting_lead_d > 0. and CI.CS.coasting_lead_d < 45.0 and CI.CS.coasting_lead_v > current_speed
    accel_limits = calc_cruise_accel_limits(current_speed, following, CI.CS.accel_mode)
    
    # decrease min accel as necessary based on lead conditions
    stock_min_factor = interp(current_speed - CI.CS.coasting_lead_v, _A_MIN_V_STOCK_FACTOR_BP, _A_MIN_V_STOCK_FACTOR_V) if CI.CS.coasting_lead_d > 0. else 0.
    accel_limits[0] = stock_min_factor * CI.params.ACCEL_MIN + (1. - stock_min_factor) * accel_limits[0]
      
    if CI.CS.accel_mode != 1 and CI.CS.standstill_time_since_t < CI.CS.cruise_resume_high_accel_ramp_bp[-1]:
      k = interp(CI.CS.standstill_time_since_t, CI.CS.cruise_resume_high_accel_ramp_bp, CI.CS.cruise_resume_high_accel_ramp_v)
      higher_max_accel = calc_cruise_accel_limits(current_speed, False, _A_CRUISE_HIGHER_ACCEL_INDEX[CI.CS.accel_mode])[1]
      accel_limits[1] = k * higher_max_accel + (1.0 - k) * accel_limits[1]
    
    if current_speed > CI.CS.accel_limits_rate_speed_cutoff:
      accel_limits = [
        clip(max(CI.params.ACCEL_MIN, accel_limits[0]), 
            CI.accel_limits_last[0] + CI.CS.accel_limits_rate_limits[0], 
            CI.accel_limits_last[0] + CI.CS.accel_limits_rate_limits[1]), 
        clip(min(accel_limits[1], CI.params.ACCEL_MAX), 
            CI.accel_limits_last[1] + CI.CS.accel_limits_rate_limits[0], 
            CI.accel_limits_last[1] + CI.CS.accel_limits_rate_limits[1])
        ]
    
    CI.accel_limits_last = accel_limits
    
    return accel_limits

  # Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_acadia(desired_angle, v_ego):
    ANGLE_COEF = 5.00000000
    ANGLE_COEF2 = 1.90844451
    ANGLE_OFFSET = 0.03401073
    SPEED_OFFSET = 13.72019138
    SIGMOID_COEF_RIGHT = 0.00100000
    SIGMOID_COEF_LEFT = 0.00101873
    SPEED_COEF = 0.36844505
    return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  
  # Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_acadia_torque(desired_lateral_accel, v_ego):
    ANGLE_COEF = 0.18280978
    ANGLE_COEF2 = 0.33000368
    ANGLE_OFFSET = 0.02769761
    SPEED_OFFSET = -0.63851608
    SIGMOID_COEF_RIGHT = 0.11531735
    SIGMOID_COEF_LEFT = 0.10717206
    SPEED_COEF = 0.15605190
    SPEED_COEF2 = 0.23676024
    SPEED_OFFSET2 = -1.49250269
    return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
  
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    ANGLE_COEF = 1.23514093
    ANGLE_COEF2 = 2.00000000
    ANGLE_OFFSET = 0.03891270
    SPEED_OFFSET = 8.58272983
    SIGMOID_COEF_RIGHT = 0.00154548
    SIGMOID_COEF_LEFT = 0.00168327
    SPEED_COEF = 0.16283995
    return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  
  # Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt_torque(desired_lateral_accel, v_ego):
    ANGLE_COEF = 0.10513119
    ANGLE_COEF2 = 0.10000000
    ANGLE_OFFSET = -0.01230698
    SPEED_OFFSET = -0.42555387
    SIGMOID_COEF_RIGHT = 0.65189115
    SIGMOID_COEF_LEFT = 0.57953135
    SPEED_COEF = 0.65295089
    return get_steer_feedforward_erf(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  
  @staticmethod
  def get_steer_feedforward_torque_roll_volt(g_lat_accel, v_ego):
    ANGLE_COEF = 0.48780343
    SPEED_COEF = 0.24314538
    return ANGLE_COEF * g_lat_accel / (max(1.0, v_ego))**SPEED_COEF
  
  @staticmethod
  def get_steer_feedforward_torque_lat_jerk_volt(jerk, speed, lateral_acceleration, friction, friction_threshold, g_lat_accel):
    if sign(lateral_acceleration) == sign(jerk):
      # entering curve
      ANGLE_COEF = 5.00000000
      ANGLE_COEF2 = 0.18950076
      ANGLE_OFFSET = 15.00059323
      SPEED_OFFSET = -1.32036601
      SIGMOID_COEF_1 = 0.11900164
      SIGMOID_COEF_2 = 1.42607235
      SPEED_COEF = 0.47329761
      SPEED_COEF2 = 1.88198222

      x = ANGLE_COEF * (jerk) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF)
      sigmoid1 = x / (1. + fabs(x))
      sigmoid1 *= SIGMOID_COEF_1
      
      x = ANGLE_COEF2 * (jerk) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF2)
      sigmoid2 = x / (1. + fabs(x))
      sigmoid2 *= SIGMOID_COEF_2 / (fabs(speed)+1)

      out = sigmoid1 + sigmoid2
    else:
      # exiting curve
      ANGLE_COEF = 4.99683211
      ANGLE_COEF2 = 0.03618608
      ANGLE_OFFSET = 15.00000144
      SPEED_OFFSET = -1.23191266
      SIGMOID_COEF_1 = 0.22412302
      SIGMOID_COEF_2 = 1.99877713
      SPEED_COEF = 1.35854322
      SPEED_COEF2 = 1.50516678
      
      x = ANGLE_COEF * (jerk) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF)
      sigmoid1 = x / (1. + fabs(x))
      sigmoid1 *= SIGMOID_COEF_1
      
      x = ANGLE_COEF2 * (jerk) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF2)
      sigmoid2 = x / (1. + fabs(x))
      sigmoid2 *= SIGMOID_COEF_2
      
      max_speed = ANGLE_OFFSET
      speed_norm = 0.5 * cos(clip(speed / max_speed, 0., 1.) * 3.14) + 0.5
      
      out = (1-speed_norm) * sigmoid1 + speed_norm * sigmoid2
    
    # if sign(out) == sign(g_lat_accel):
    #   out = max(0.0, abs(out) - abs(g_lat_accel)) * sign(out)
    
    return out * friction


  @staticmethod
  def get_steer_feedforward_bolt_euv(angle, speed):
    ANGLE_COEF = 4.80745391
    ANGLE_COEF2 = 0.47214969
    ANGLE_OFFSET = 0.#-0.32202861
    SPEED_OFFSET = 2.85629120
    SIGMOID_COEF_RIGHT = 0.33536781
    SIGMOID_COEF_LEFT = 0.40555956
    SPEED_COEF = 0.02123313

    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2)))
  
  @staticmethod
  def get_steer_feedforward_bolt_euv_torque(desired_lateral_accel, speed):
    ANGLE_COEF = 0.16179233
    ANGLE_COEF2 = 0.20691964
    ANGLE_OFFSET = 0.#0.04420955
    SPEED_OFFSET = -7.94958973
    SIGMOID_COEF_RIGHT = 0.34906506
    SIGMOID_COEF_LEFT = 0.20000000
    SPEED_COEF = 0.38748798

    x = ANGLE_COEF * (desired_lateral_accel + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
    sigmoid = erf(x)
    return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * (desired_lateral_accel + ANGLE_OFFSET)
  
  @staticmethod
  def get_steer_feedforward_lacrosse(angle, speed):
    ANGLE_COEF = 0.58539783
    ANGLE_COEF2 = 0.32765082
    ANGLE_OFFSET = 0.00460531
    SPEED_OFFSET = 13.23075991
    SIGMOID_COEF_RIGHT = 0.13719471
    SIGMOID_COEF_LEFT = 0.13309956
    SPEED_COEF = 0.06147823
    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2)))
  
  def get_steer_feedforward_lacrosse_torque(desired_lateral_accel, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = ANGLE_COEF * (desired_lateral_accel + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
    sigmoid = erf(x)
    return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * (desired_lateral_accel + ANGLE_OFFSET)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint in [CAR.VOLT, CAR.VOLT18]:
      return self.get_steer_feedforward_volt
    elif self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia
    elif self.CP.carFingerprint == CAR.BOLT_EUV:
      return self.get_steer_feedforward_bolt_euv
    elif self.CP.carFingerprint == CAR.BUICK_LACROSSE:
      return self.get_steer_feedforward_lacrosse
    else:
      return CarInterfaceBase.get_steer_feedforward_default
  
  def get_steer_feedforward_function_torque(self):
    if self.CP.carFingerprint in [CAR.VOLT, CAR.VOLT18]:
      return self.get_steer_feedforward_volt_torque
    elif self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia_torque
    elif self.CP.carFingerprint == CAR.BOLT_EUV:
      return self.get_steer_feedforward_bolt_euv_torque
    elif self.CP.carFingerprint == CAR.BUICK_LACROSSE:
      return self.get_steer_feedforward_lacrosse_torque
    else:
      return CarInterfaceBase.get_steer_feedforward_torque_default
    
  def initialize_feedforward_function_torque_nn(self):
    if self.CP.carFingerprint in [CAR.VOLT, CAR.VOLT18]:
      self.ff_nn_model = FluxModel("/data/openpilot/selfdrive/car/gm/models/CHEVROLET VOLT PREMIER 2017.json")
    elif self.CP.carFingerprint == CAR.BUICK_LACROSSE:
      self.ff_nn_model = FluxModel("/data/openpilot/selfdrive/car/gm/models/BUICK LACROSSE 2017.json")
    elif self.CP.carFingerprint == CAR.ACADIA:
      self.ff_nn_model = FluxModel("/data/openpilot/selfdrive/car/gm/models/GMC ACADIA DENALI 2018.json")
  
  def initialize_feedforward_function_nn(self):
    if self.CP.carFingerprint in [CAR.VOLT, CAR.VOLT18]:
      self.ff_nn_model = FluxModel("/data/openpilot/selfdrive/car/gm/models/CHEVROLET VOLT PREMIER 2017 steer angle.json")
    elif self.CP.carFingerprint == CAR.BUICK_LACROSSE:
      self.ff_nn_model = FluxModel("/data/openpilot/selfdrive/car/gm/models/BUICK LACROSSE 2017 steer angle.json")
    elif self.CP.carFingerprint == CAR.ACADIA:
      self.ff_nn_model = FluxModel("/data/openpilot/selfdrive/car/gm/models/GMC ACADIA DENALI 2018 steer angle.json")
  
  def get_steer_feedforward_function_torque_lat_jerk(self):
    if self.CP.carFingerprint in [CAR.VOLT, CAR.VOLT18]:
      return self.get_steer_feedforward_torque_lat_jerk_volt
    else:
      return CarInterfaceBase.get_steer_feedforward_function_torque_lat_jerk_default
    
  def get_steer_feedforward_function_torque_roll(self):
    if self.CP.carFingerprint in [CAR.VOLT, CAR.VOLT18]:
      return self.get_steer_feedforward_torque_roll_volt
    else:
      return CarInterfaceBase.get_steer_feedforward_function_torque_roll_default

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "gm"
    ret.safetyModel = car.CarParams.SafetyModel.gm
    ret.pcmCruise = False  # stock cruise control is kept off
    ret.stoppingControl = True
    ret.startAccel = 0.8
    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 1/15  # GM radar runs at 15Hz instead of standard 20Hz

    # GM port is a community feature
    # TODO: make a port that uses a car harness and it only intercepts the camera
    ret.communityFeature = True

    # Presence of a camera on the object bus is ok.
    # Have to go to read_only if ASCM is online (ACC-enabled cars),
    # or camera is on powertrain bus (LKA cars without ACC).
    ret.openpilotLongitudinalControl = True
    tire_stiffness_factor = 0.444  # not optimized yet
    
    ret.longitudinalActuatorDelayLowerBound = 0.42
    ret.longitudinalActuatorDelayUpperBound = 0.5

    # Default lateral controller params.
    ret.minSteerSpeed = 6.7 * CV.MPH_TO_MS
    ret.lateralTuning.pid.kpBP = [0.]
    ret.lateralTuning.pid.kpV = [0.2]
    ret.lateralTuning.pid.kiBP = [0.]
    ret.lateralTuning.pid.kiV = [0.]
    ret.lateralTuning.pid.kf = 0.00004   # full torque for 20 deg at 80mph means 0.00007818594
    ret.steerRateCost = 1.0
    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet

    # Default longitudinal controller params.
    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.36]

    if candidate in [CAR.VOLT, CAR.VOLT18]:
      # supports stop and go, but initial engage must be above 18mph (which include conservatism)
      ret.minEnableSpeed = -1
      ret.mass = 1607. + STD_CARGO_KG
      ret.wheelbase = 2.69
      ret.steerRatio = 17.7  # Stock 15.7, LiveParameters
      ret.steerRateCost = 1.0
      tire_stiffness_factor = 0.469 # Stock Michelin Energy Saver A/S, LiveParameters
      ret.steerRatioRear = 0.
      ret.centerToFront = 0.45 * ret.wheelbase # from Volt Gen 1
      ret.steerActuatorDelay = 0.18
      if (Params().get_bool("EnableTorqueControl")):
        ret.lateralTuning.init('torque')
        ret.lateralTuning.torque.useSteeringAngle = True
        ret.lateralTuning.torque.kp = 0.48
        ret.lateralTuning.torque.ki = 0.15
        ret.lateralTuning.torque.kd = 0.04
        ret.lateralTuning.torque.kf = 1.0 # use with custom torque ff
        ret.lateralTuning.torque.friction = 1.0 # for custom lateral jerk ff
      else:
        ret.lateralTuning.pid.kpBP = [0., 40.]
        ret.lateralTuning.pid.kpV = [0., .16]
        ret.lateralTuning.pid.kiBP = [0., 40.]
        ret.lateralTuning.pid.kiV = [.015, 0.02]
        ret.lateralTuning.pid.kdBP = [0.]
        ret.lateralTuning.pid.kdV = [0.007]
        ret.lateralTuning.pid.kf = 1.0 # !!! ONLY for sigmoid feedforward !!!
      

      # Only tuned to reduce oscillations. TODO.
      ret.longitudinalTuning.kpBP = [5., 15., 35.]
      ret.longitudinalTuning.kpV = [0.8, .9, 0.8]
      ret.longitudinalTuning.kiBP = [5., 15., 35.]
      ret.longitudinalTuning.kiV = [0.08, 0.13, 0.13]
      ret.longitudinalTuning.kdBP = [5., 25.]
      ret.longitudinalTuning.kdV = [0.004, 0.0]
      ret.stoppingDecelRate = 0.2 # brake_travel/s while trying to stop

    elif candidate == CAR.MALIBU:
      # supports stop and go, but initial engage must be above 18mph (which include conservatism)
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.mass = 1496. + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 15.8
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4  # wild guess

    elif candidate == CAR.HOLDEN_ASTRA:
      ret.mass = 1363. + STD_CARGO_KG
      ret.wheelbase = 2.662
      # Remaining parameters copied from Volt for now
      ret.centerToFront = ret.wheelbase * 0.4
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.steerRatio = 15.7
      ret.steerRatioRear = 0.

    elif candidate == CAR.ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 4353. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.86
      ret.steerRatio = 16.0 #14.4  # end to end is 13.46
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerActuatorDelay = 0.24

      if (Params().get_bool("EnableTorqueControl")):
        ret.lateralTuning.init('torque')
        ret.lateralTuning.torque.useSteeringAngle = True
        ret.lateralTuning.torque.kp = 0.66
        ret.lateralTuning.torque.ki = 0.15
        ret.lateralTuning.torque.kd = 0.03
        ret.lateralTuning.torque.kf = 1. # custom ff
        ret.lateralTuning.torque.friction = 0.01
      else:
        ret.lateralTuning.pid.kpBP = [i * CV.MPH_TO_MS for i in [0., 80.]]
        ret.lateralTuning.pid.kpV = [0., 0.16]
        ret.lateralTuning.pid.kiBP = [0., 35.]
        ret.lateralTuning.pid.kiV = [0.005, 0.01]
        ret.lateralTuning.pid.kdBP = [0.]
        ret.lateralTuning.pid.kdV = [0.008]
        ret.lateralTuning.pid.kf = 1.0 # get_steer_feedforward_acadia()

      ret.longitudinalTuning.kdBP = [5., 25.]
      ret.longitudinalTuning.kdV = [0.009, 0.005]
      ret.longitudinalTuning.kiBP = [5., 35.]
      ret.longitudinalTuning.kiV = [0.31, 0.34]
      
    elif candidate == CAR.BUICK_LACROSSE:
      ret.mass = 1741. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.steerRatio = 15.8
      ret.centerToFront = ret.wheelbase * 0.4  # wild guess
      ret.steerActuatorDelay = 0.22
      if (Params().get_bool("EnableTorqueControl")):
        ret.lateralTuning.init('torque')
        ret.lateralTuning.torque.useSteeringAngle = True
        ret.lateralTuning.torque.kp = 0.48
        ret.lateralTuning.torque.ki = 0.13
        ret.lateralTuning.torque.kd = 0.03
        ret.lateralTuning.torque.kf = 1.0 # use with custom torque ff
        ret.lateralTuning.torque.friction = 0.14
      else:
        ret.lateralTuning.pid.kpBP = [0., 40.]
        ret.lateralTuning.pid.kpV = [0.05, 0.2]
        ret.lateralTuning.pid.kiBP = [0., 40.]
        ret.lateralTuning.pid.kiV = [0.01, 0.005]
        ret.lateralTuning.pid.kdBP = [0.]
        ret.lateralTuning.pid.kdV = [0.007]
        ret.lateralTuning.pid.kf = 1.0 # !!! ONLY for sigmoid feedforward !!!

    elif candidate == CAR.BUICK_REGAL:
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.mass = 3779. * CV.LB_TO_KG + STD_CARGO_KG  # (3849+3708)/2
      ret.wheelbase = 2.83  # 111.4 inches in meters
      ret.steerRatio = 14.4  # guess for tourx
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4  # guess for tourx

    elif candidate == CAR.CADILLAC_ATS:
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS
      ret.mass = 1601. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 15.3
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.49

    elif candidate == CAR.ESCALADE:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 2645. + STD_CARGO_KG
      ret.wheelbase = 2.95
      ret.steerRatio = 17.3  # end to end is 13.46
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
      ret.lateralTuning.pid.kf = 0.000045
      tire_stiffness_factor = 1.0

    elif candidate == CAR.ESCALADE_ESV:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 2739. + STD_CARGO_KG
      ret.wheelbase = 3.302
      ret.steerRatio = 17.3
      ret.centerToFront = ret.wheelbase * 0.49
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
      ret.lateralTuning.pid.kf = 0.000045
      tire_stiffness_factor = 1.0
    elif candidate == CAR.BOLT_EUV:
      ret.minEnableSpeed = -1
      ret.mass = 1669. + STD_CARGO_KG
      ret.wheelbase = 2.675
      ret.steerRatio = 16.8
      ret.centerToFront = ret.wheelbase * 0.4
      tire_stiffness_factor = 1.0
      ret.steerActuatorDelay = 0.2
      if (Params().get_bool("EnableTorqueControl")):
        max_lateral_accel = 3.0
        ret.lateralTuning.init('torque')
        ret.lateralTuning.torque.useSteeringAngle = True
        ret.lateralTuning.torque.kp = 1.8 / max_lateral_accel
        ret.lateralTuning.torque.ki = 0.4 / max_lateral_accel
        ret.lateralTuning.torque.kd = 0.05 / max_lateral_accel
        ret.lateralTuning.torque.kf = 1.0 # use with custom torque ff
        ret.lateralTuning.torque.friction = 0.005
      else:
        ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[10., 40.0], [0., 40.]]
        ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.1, 0.22], [0.01, 0.021]]
        ret.lateralTuning.pid.kdBP = [0.]
        ret.lateralTuning.pid.kdV = [0.007]
        ret.lateralTuning.pid.kf = 1. # use with get_feedforward_bolt_euv
    
    if Params().get_bool('OPParamsLateralOverride'):
      op_params = opParams("gm car_interface.py for lateral override")
      lat_type = op_params.get('TUNE_LAT_type')
      ret.minSteerSpeed = op_params.get('TUNE_LAT_min_steer_speed_mph') * CV.MPH_TO_MS
      if lat_type == 'torque':
        ret.lateralTuning.init('torque')
        ret.lateralTuning.torque.useSteeringAngle = op_params.get('TUNE_LAT_TRX_use_steering_angle', force_update=True)
        ret.lateralTuning.torque.kp = op_params.get('TUNE_LAT_TRX_kp', force_update=True)
        ret.lateralTuning.torque.ki = op_params.get('TUNE_LAT_TRX_ki', force_update=True)
        ret.lateralTuning.torque.kd = op_params.get('TUNE_LAT_TRX_kd', force_update=True)
        ret.lateralTuning.torque.kf = op_params.get('TUNE_LAT_TRX_kf', force_update=True)
        ret.lateralTuning.torque.friction = 0.0
      elif lat_type == 'pid':
        ret.lateralTuning.init('pid')
        bp = [i * CV.MPH_TO_MS for i in op_params.get(f"TUNE_LAT_PID_bp_mph", force_update=True)]
        ret.lateralTuning.pid.kpV = op_params.get("TUNE_LAT_PID_kp", force_update=True)
        ret.lateralTuning.pid.kiV = op_params.get("TUNE_LAT_PID_ki", force_update=True)
        ret.lateralTuning.pid.kdV = op_params.get("TUNE_LAT_PID_kd", force_update=True)
        ret.lateralTuning.pid.kpBP = bp
        ret.lateralTuning.pid.kiBP = bp
        ret.lateralTuning.pid.kdBP = bp
        ret.lateralTuning.pid.kf = op_params.get('TUNE_LAT_PID_kf', force_update=True)
      elif lat_type == 'indi':
        ret.lateralTuning.init('indi')
        bp = [i * CV.MPH_TO_MS for i in op_params.get("TUNE_LAT_INDI_bp_mph", force_update=True)]
        ret.lateralTuning.indi.innerLoopGainBP = bp
        ret.lateralTuning.indi.innerLoopGainV = op_params.get("TUNE_LAT_INDI_inner_gain", force_update=True)
        ret.lateralTuning.indi.outerLoopGainBP = bp
        ret.lateralTuning.indi.outerLoopGainV = op_params.get("TUNE_LAT_INDI_outer_gain", force_update=True)
        ret.lateralTuning.indi.timeConstantBP = bp
        ret.lateralTuning.indi.timeConstantV = op_params.get("TUNE_LAT_INDI_time_constant", force_update=True)
        ret.lateralTuning.indi.actuatorEffectivenessBP = bp
        ret.lateralTuning.indi.actuatorEffectivenessV = op_params.get("TUNE_LAT_INDI_actuator_effectiveness", force_update=True)
      elif lat_type == 'lqr':
        ret.lateralTuning.init('lqr')
        ret.lateralTuning.lqr.scale = op_params.get('TUNE_LAT_LQR_scale', force_update=True)
        ret.lateralTuning.lqr.ki = op_params.get('TUNE_LAT_LQR_ki', force_update=True)
        ret.lateralTuning.lqr.dcGain = op_params.get('TUNE_LAT_LQR_dc_gain', force_update=True)
        ret.lateralTuning.lqr.a = op_params.get('TUNE_LAT_LQR_a', force_update=True)
        ret.lateralTuning.lqr.b = op_params.get('TUNE_LAT_LQR_b', force_update=True)
        ret.lateralTuning.lqr.c = op_params.get('TUNE_LAT_LQR_c', force_update=True)
        ret.lateralTuning.lqr.k = op_params.get('TUNE_LAT_LQR_k', force_update=True)
        ret.lateralTuning.lqr.l = op_params.get('TUNE_LAT_LQR_l', force_update=True)
      elif lat_type == 'torqueindi':
        ret.lateralTuning.init('torqueIndi')
        bp = [i * CV.MPH_TO_MS for i in op_params.get("TUNE_LAT_TRXINDI_bp_mph", force_update=True)]
        ret.lateralTuning.torqueIndi.innerLoopGainBP = bp
        ret.lateralTuning.torqueIndi.innerLoopGainV = op_params.get("TUNE_LAT_TRXINDI_inner_gain", force_update=True)
        ret.lateralTuning.torqueIndi.outerLoopGainBP = bp
        ret.lateralTuning.torqueIndi.outerLoopGainV = op_params.get("TUNE_LAT_TRXINDI_outer_gain", force_update=True)
        ret.lateralTuning.torqueIndi.timeConstantBP = bp
        ret.lateralTuning.torqueIndi.timeConstantV = op_params.get("TUNE_LAT_TRXINDI_time_constant", force_update=True)
        ret.lateralTuning.torqueIndi.actuatorEffectivenessBP = bp
        ret.lateralTuning.torqueIndi.actuatorEffectivenessV = op_params.get("TUNE_LAT_TRXINDI_actuator_effectiveness", force_update=True)
        ret.lateralTuning.torqueIndi.friction = op_params.get('TUNE_LAT_TRXINDI_friction', force_update=True)
        ret.lateralTuning.torqueIndi.kf = op_params.get('TUNE_LAT_TRXINDI_kf', force_update=True)
      elif lat_type == 'torquelqr':
        ret.lateralTuning.init('torqueLqr')
        ret.lateralTuning.torqueLqr.scale = op_params.get('TUNE_LAT_TRXLQR_scale', force_update=True)
        ret.lateralTuning.torqueLqr.ki = op_params.get('TUNE_LAT_TRXLQR_ki', force_update=True)
        ret.lateralTuning.torqueLqr.dcGain = op_params.get('TUNE_LAT_TRXLQR_dc_gain', force_update=True)
        ret.lateralTuning.torqueLqr.a = op_params.get('TUNE_LAT_TRXLQR_a', force_update=True)
        ret.lateralTuning.torqueLqr.b = op_params.get('TUNE_LAT_TRXLQR_b', force_update=True)
        ret.lateralTuning.torqueLqr.c = op_params.get('TUNE_LAT_TRXLQR_c', force_update=True)
        ret.lateralTuning.torqueLqr.k = op_params.get('TUNE_LAT_TRXLQR_k', force_update=True)
        ret.lateralTuning.torqueLqr.l = op_params.get('TUNE_LAT_TRXLQR_l', force_update=True)
        ret.lateralTuning.torqueLqr.friction = op_params.get('TUNE_LAT_TRXLQR_friction', force_update=True)
        ret.lateralTuning.torqueLqr.kf = op_params.get('TUNE_LAT_TRXLQR_kf', force_update=True)
        ret.lateralTuning.torqueLqr.useSteeringAngle = op_params.get('TUNE_LAT_TRXLQR_use_steering_angle', force_update=True)
    
    if Params().get_bool('OPParamsLongitudinalOverride'):
      bp = [i * CV.MPH_TO_MS for i in op_params.get('TUNE_LONG_speed_mph', force_update=True)]
      ret.longitudinalTuning.kpBP = bp
      ret.longitudinalTuning.kpV = op_params.get('TUNE_LONG_kp', force_update=True)
      ret.longitudinalTuning.kiBP = bp
      ret.longitudinalTuning.kiV = op_params.get('TUNE_LONG_ki', force_update=True)
      ret.longitudinalTuning.kdBP = bp
      ret.longitudinalTuning.kdV = op_params.get('TUNE_LONG_kd', force_update=True)
      ret.longitudinalTuning.deadzoneBP = bp
      ret.longitudinalTuning.deadzoneV = op_params.get('TUNE_LONG_deadzone_ms2', force_update=True)

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)
    
    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_loopback.update_strings(can_strings)
    self.cp_chassis.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_loopback, self.cp_chassis)

    t = sec_since_boot()

    cruiseEnabled = self.CS.pcm_acc_status != AccState.OFF
    ret.cruiseState.enabled = cruiseEnabled

    ret.canValid = self.cp.can_valid and self.cp_loopback.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    ret.engineRPM = self.CS.engineRPM

    buttonEvents = []
    
    self.driver_interacted = self.driver_interacted \
                            or self.CS.out.leftBlinker or self.CS.out.rightBlinker \
                            or self.CS.distance_button != self.CS.prev_distance_button \
                            or self.CS.cruise_buttons != self.CS.prev_cruise_buttons \
                            or self.CS.gear_shifter_ev != self.CS.gear_shifter_ev_last

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == CruiseButtons.RES_ACCEL:
        self.CS.resume_required = False
        if not (ret.cruiseState.enabled and ret.standstill):
          be.type = ButtonType.accelCruise  # Suppress resume button if we're resuming from stop so we don't adjust speed.
      elif but == CruiseButtons.DECEL_SET:
        if not cruiseEnabled and not self.CS.lkaEnabled:
          self.lkaEnabled = True
        be.type = ButtonType.decelCruise
      elif but == CruiseButtons.CANCEL:
        be.type = ButtonType.cancel
      elif but == CruiseButtons.MAIN:
        be.type = ButtonType.altButton3
      buttonEvents.append(be)

    if self.CS.lka_button and self.CS.lka_button != self.CS.prev_lka_button:
      self.CS.lkaEnabled = not self.CS.lkaEnabled
      if self.CS.MADS_enabled:
        put_nonblocking("MADSAutosteerEnabled", str(int(self.CS.lkaEnabled)))
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = True
      be.type = ButtonType.altButton1
      buttonEvents.append(be)
      cloudlog.info("button press event: LKA button. new value: %i" % self.CS.lkaEnabled)
    elif not cruiseEnabled and self.cruise_enabled_last:
      self.CS.lkaEnabled = self.CS._params.get_bool("MADSAutosteerEnabled")
    self.cruise_enabled_last = cruiseEnabled
    
    ret.buttonEvents = buttonEvents
    
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      self.one_pedal_mode = self.CS._params.get_bool("MADSOnePedalMode")

    if self.CS.long_active and self.CS.distance_button != self.CS.prev_distance_button:
      if self.CS.distance_button:
        self.CS.distance_button_last_press_t = t
        cloudlog.info("button press event: Distance button pressed in cruise mode.")
      else: # apply change on button lift
        self.CS.follow_level -= 1
        if self.CS.follow_level < 1:
          self.CS.follow_level = 3
        put_nonblocking("FollowLevel", str(self.CS.follow_level))
        self.CS.follow_level_change_last_t = t
        cloudlog.info("button press event: cruise follow distance button. new value: %r" % self.CS.follow_level)
    elif self.CS.MADS_enabled and not self.CS.distance_button and self.CS.prev_distance_button:
      put_nonblocking("MADSLeadBraking", str(int(not self.CS.MADS_lead_braking_enabled)))
      cloudlog.info("button press event: press distance button for MADS lead braking. new value: %i" % self.CS.MADS_lead_braking_enabled)

    ret.readdistancelines = self.CS.follow_level

    events = self.create_common_events(ret, pcm_enable=False)
    
    if self.CS.reboot_in_N_seconds >= 0:
      events.add(EventName.rebootImminent)
    
    if self.CS.cruiseMain:
      if ret.vEgo < self.CP.minEnableSpeed:
        events.add(EventName.belowEngageSpeed)
      if self.CS.pause_long_on_gas_press:
        events.add(EventName.gasPressed)
      if self.CS.park_brake:
        events.add(EventName.parkBrake)
      steer_paused = False
      if cruiseEnabled and t - self.CS.last_pause_long_on_gas_press_t < 0.5 and t - self.CS.sessionInitTime > 10.:
        events.add(car.CarEvent.EventName.pauseLongOnGasPress)
      if not ret.standstill and self.CS.lane_change_steer_factor < 1.:
        events.add(car.CarEvent.EventName.blinkerSteeringPaused)
        steer_paused = True
      if ret.vEgo <= self.CP.minSteerSpeed and (not self.CS.autoHoldActivated or self.CS.out.onePedalModeActive):
        if ret.standstill and (self.CS.parked_timer > self.CS.parked_timer_min_time or (cruiseEnabled or self.CS.out.onePedalModeActive) and t - self.CS.sessionInitTime > 10. and not self.CS.resume_required):
          events.add(car.CarEvent.EventName.stoppedWaitForGas)
        elif not ret.standstill and self.CS.out.gearShifter in ['drive','low'] and not steer_paused and self.CS.lkaEnabled:
          events.add(car.CarEvent.EventName.belowSteerSpeed)
      if self.CS.autoHoldActivated:
        self.CS.lastAutoHoldTime = t
        events.add(car.CarEvent.EventName.autoHoldActivated)
      if self.CS.pcm_acc_status == AccState.FAULTED and t - self.CS.sessionInitTime > 10.0 and t - self.CS.lastAutoHoldTime > 1.0:
        events.add(EventName.accFaulted)
      if self.CS.resume_required:
        events.add(EventName.resumeRequired)

    if self.MADS_enabled and not self.CS.long_active and t - self.CS.sessionInitTime > 30.:
      if self.CS.MADS_lead_braking_enabled != self.mads_lead_braking_enabled:
        if self.CS.MADS_lead_braking_enabled:
          events.add(EventName.madsLeadBrakingEnabled)
        else:
          events.add(EventName.madsLeadBrakingDisabled)
      
      if self.CS.one_pedal_mode_active != self.mads_one_pedal_enabled:
        if self.CS.one_pedal_mode_active:
          if self.CS.one_pedal_mode_temporary:
            if self.CS.out.vEgo > 0.1:
              events.add(EventName.madsOnePedalTemporary)
          else:
            events.add(EventName.madsOnePedalEnabled)
        else:
          if not self.mads_one_pedal_temporary:
            events.add(EventName.madsOnePedalDisabled)
      
      if self.CS.cruiseMain != self.mads_cruise_main:
        if self.CS.cruiseMain:
          events.add(EventName.madsEnabled)
        else:
          events.add(EventName.madsDisabled)
      
      if self.CS.lkaEnabled != self.autosteer_enabled:
        if self.CS.lkaEnabled:
          events.add(EventName.madsAutosteerEnabled)
        else:
          events.add(EventName.madsAutosteerDisabled)
    else:
      if self.CS.lkaEnabled != self.autosteer_enabled:
        if not self.CS.lkaEnabled:
          events.add(EventName.manualSteeringRequired)
          
    self.mads_one_pedal_enabled = self.CS.one_pedal_mode_active
    self.mads_one_pedal_temporary = self.CS.one_pedal_mode_temporary
    self.mads_cruise_main = self.CS.cruiseMain
    self.autosteer_enabled = self.CS.lkaEnabled
    self.mads_lead_braking_enabled = self.CS.MADS_lead_braking_enabled

    # handle button presses
    for b in ret.buttonEvents:
      # do enable on both accel and decel buttons

      # do disable on LFA button if ACC is disabled
      if self.MADS_enabled:
        if b.type in [ButtonType.altButton1] and b.pressed:
          if not self.CS.lkaEnabled: #disabled LFA
            if not ret.cruiseState.enabled:
              events.add(EventName.buttonCancel)
            
            
      
      # The ECM will fault if resume triggers an enable while speed is set to 0
      if b.type == ButtonType.accelCruise and c.hudControl.setSpeed > 0 and c.hudControl.setSpeed < 70 and not b.pressed:
        events.add(EventName.buttonEnable)
        if not self.CS.cruiseMain:
          events.add(EventName.wrongCarMode)
      if b.type == ButtonType.decelCruise and not b.pressed:
        events.add(EventName.buttonEnable)
        if not self.CS.cruiseMain:
          events.add(EventName.wrongCarMode)
      # do disable on button down
      if b.type == ButtonType.cancel and b.pressed:
        events.add(EventName.buttonCancel)
      # The ECM independently tracks a ‘speed is set’ state that is reset on main off.
      # To keep controlsd in sync with the ECM state, generate a RESET_V_CRUISE event on main cruise presses.
      if b.type == ButtonType.altButton3 and b.pressed:
        events.add(EventName.buttonMainCancel)

    ret.events = events.to_msg()

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  def apply(self, c):
    hud_v_cruise = c.hudControl.setSpeed
    if hud_v_cruise > 70:
      hud_v_cruise = 0

    # For Openpilot, "enabled" includes pre-enable.
    # In GM, PCM faults out if ACC command overlaps user gas, so keep that from happening inside CC.update().
    pause_long_on_gas_press = c.enabled and self.CS.gasPressed and not self.CS.out.brake > 0. and not self.disengage_on_gas
    t = sec_since_boot()
    if pause_long_on_gas_press and not self.CS.pause_long_on_gas_press:
      if t - self.CS.last_pause_long_on_gas_press_t > 300.:
        self.CS.last_pause_long_on_gas_press_t = t

    self.CS.pause_long_on_gas_press = pause_long_on_gas_press
    enabled = c.enabled or self.CS.pause_long_on_gas_press
    
    self.CS.long_active = enabled
    
    can_sends = self.CC.update(enabled, self.CS, self.frame,
                               c.actuators,
                               hud_v_cruise, c.hudControl.lanesVisible,
                               c.hudControl.leadVisible, c.hudControl.visualAlert)

    self.frame += 1

    # Release Auto Hold and creep smoothly when regenpaddle pressed
    if t - self.CS.regen_paddle_released_last_t < 1.0 and self.CS.autoHold and not self.CS.one_pedal_mode_active:
      self.CS.autoHoldActive = False

    if ((self.CS.autoHold and not self.CS.regen_paddle_pressed) or self.CS.one_pedal_mode_active) and not self.CS.autoHoldActive:
      if self.CS.out.vEgo > 0.03:
        self.CS.autoHoldActive = True
      elif self.CS.out.vEgo < 0.02 and self.CS.out.brakePressed and self.CS.time_in_drive_autohold >= self.CS.MADS_long_min_time_in_drive:
        self.CS.autoHoldActive = True

    return can_sends
