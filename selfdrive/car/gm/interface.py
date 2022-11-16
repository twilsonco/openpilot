#!/usr/bin/env python3
from cereal import car
from math import fabs, erf, atan
from panda import Panda

from common.conversions import Conversions as CV
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from selfdrive.car import STD_CARGO_KG, scale_tire_stiffness, get_safety_config
from selfdrive.car.gm.values import CAR, CruiseButtons, CarControllerParams, EV_CAR, CAMERA_ACC_CAR
from selfdrive.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}

FRICTION_THRESHOLD_LAT_JERK = 2.0

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def get_steer_feedforward_acadia(desired_angle, v_ego):
    ANGLE_COEF = 5.00000000
    ANGLE_COEF2 = 1.90844451
    ANGLE_COEF3 = 0.03401073
    SPEED_OFFSET = 13.72019138
    SIGMOID_COEF_RIGHT = 0.00100000
    SIGMOID_COEF_LEFT = 0.00101873
    SPEED_COEF = 0.36844505
    x = ANGLE_COEF * (desired_angle) / max(0.01,v_ego)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if desired_angle > 0. else SIGMOID_COEF_LEFT) * sigmoid) * (0.01 + v_ego + SPEED_OFFSET) ** ANGLE_COEF2 + ANGLE_COEF3 * (desired_angle * SPEED_COEF - atan(desired_angle * SPEED_COEF))

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia
    else:
      return CarInterfaceBase.get_steer_feedforward_default
    
  @staticmethod
  def torque_from_lateral_accel_volt(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 0.08617848
    ANGLE_COEF2 = 0.12568428
    SPEED_OFFSET = -3.48009247
    SIGMOID_COEF_RIGHT = 0.56664089
    SIGMOID_COEF_LEFT = 0.50360594
    SPEED_COEF = 0.55322718
    x = ANGLE_COEF * (lateral_accel_value) * (40.23 / (max(0.2,v_ego + SPEED_OFFSET))**SPEED_COEF)
    sigmoid = erf(x)
    out = ((SIGMOID_COEF_RIGHT if lateral_accel_value < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * lateral_accel_value
    friction = interp(
      lateral_jerk_desired,
      [-FRICTION_THRESHOLD_LAT_JERK, FRICTION_THRESHOLD_LAT_JERK],
      [-torque_params.friction, torque_params.friction]
    )
    return out + friction + g_lat_accel * 0.3
  
  @staticmethod
  def torque_from_lateral_accel_bolt_euv(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 0.16179233
    ANGLE_COEF2 = 0.20691964
    SPEED_OFFSET = -7.94958973
    SIGMOID_COEF_RIGHT = 0.34906506
    SIGMOID_COEF_LEFT = 0.20000000
    SPEED_COEF = 0.38748798

    x = ANGLE_COEF * lateral_accel_value * (40.23 / (max(0.2,v_ego + SPEED_OFFSET))**SPEED_COEF)
    sigmoid = erf(x)
    out = ((SIGMOID_COEF_RIGHT if lateral_accel_value < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * lateral_accel_value
    friction = interp(
      lateral_jerk_desired,
      [-FRICTION_THRESHOLD_LAT_JERK, FRICTION_THRESHOLD_LAT_JERK],
      [-torque_params.friction, torque_params.friction]
    )
    return out + friction + g_lat_accel * 0.3
  
  @staticmethod
  def torque_from_lateral_accel_bolt(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 0.18708832
    ANGLE_COEF2 = 0.28818528
    SPEED_OFFSET = 20.00000000
    SIGMOID_COEF_RIGHT = 0.36997215
    SIGMOID_COEF_LEFT = 0.43181054
    SPEED_COEF = 0.34143006
    x = ANGLE_COEF * lateral_accel_value * (40.23 / (max(0.2,v_ego + SPEED_OFFSET))**SPEED_COEF)
    sigmoid = erf(x)
    out = ((SIGMOID_COEF_RIGHT if lateral_accel_value < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * lateral_accel_value
    friction = interp(
      lateral_jerk_desired,
      [-FRICTION_THRESHOLD_LAT_JERK, FRICTION_THRESHOLD_LAT_JERK],
      [-torque_params.friction, torque_params.friction]
    )
    return out + friction + g_lat_accel * 0.3
  
  @staticmethod
  def torque_from_lateral_accel_silverado(lateral_accel_value, torque_params, lateral_accel_error, lateral_accel_deadzone, friction_compensation, v_ego, g_lat_accel, lateral_jerk_desired):
    ANGLE_COEF = 0.40612450
    ANGLE_COEF2_RIGHT = 0.14742903
    ANGLE_COEF2_LEFT = 0.07317035
    SIGMOID_COEF_RIGHT = 0.35
    SIGMOID_COEF_LEFT = 0.35
    x = ANGLE_COEF * (lateral_accel_value) * (40.23 / (max(0.2,v_ego)))
    sigmoid = erf(x)
    if lateral_accel_value < 0.:
      sigmoid_coef = SIGMOID_COEF_RIGHT 
      slope_coef = ANGLE_COEF2_RIGHT
    else:
      sigmoid_coef = SIGMOID_COEF_LEFT
      slope_coef = ANGLE_COEF2_LEFT
    out = sigmoid_coef * sigmoid + slope_coef * lateral_accel_value
    friction = interp(
      lateral_jerk_desired,
      [-FRICTION_THRESHOLD_LAT_JERK, FRICTION_THRESHOLD_LAT_JERK],
      [-torque_params.friction, torque_params.friction]
    )
    return out + friction + g_lat_accel * 0.3
  
  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint == CAR.VOLT:
      return self.torque_from_lateral_accel_volt
    elif self.CP.carFingerprint == CAR.BOLT_EV:
      return self.torque_from_lateral_accel_bolt
    elif self.CP.carFingerprint == CAR.BOLT_EUV:
      return self.torque_from_lateral_accel_bolt_euv
    else:
      return CarInterfaceBase.torque_from_lateral_accel_linear

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long):
    ret.carName = "gm"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.gm)]
    ret.autoResumeSng = False

    if candidate in EV_CAR:
      ret.transmissionType = TransmissionType.direct
    else:
      ret.transmissionType = TransmissionType.automatic

    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.15]

    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kiBP = [0.]

    if candidate in CAMERA_ACC_CAR:
      ret.experimentalLongitudinalAvailable = True
      ret.networkLocation = NetworkLocation.fwdCamera
      ret.radarOffCan = True  # no radar
      ret.pcmCruise = True
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM
      ret.minEnableSpeed = 5 * CV.KPH_TO_MS

      if experimental_long:
        ret.pcmCruise = False
        ret.openpilotLongitudinalControl = True
        ret.safetyConfigs[0].safetyParam |= Panda.FLAG_GM_HW_CAM_LONG

        # Tuning
        ret.stopAccel = -2.0
        ret.stoppingDecelRate = 2.0  # reach brake quickly after enabling
        ret.vEgoStopping = 0.25
        ret.vEgoStarting = 0.25
        ret.longitudinalActuatorDelayUpperBound = 0.5

    else:  # ASCM, OBD-II harness
      ret.openpilotLongitudinalControl = True
      ret.networkLocation = NetworkLocation.gateway
      ret.radarOffCan = False
      ret.pcmCruise = False  # stock non-adaptive cruise control is kept off
      # supports stop and go, but initial engage must (conservatively) be above 18mph
      ret.minEnableSpeed = 18 * CV.MPH_TO_MS

    # Tuning
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiV = [0.36]

    # These cars have been put into dashcam only due to both a lack of users and test coverage.
    # These cars likely still work fine. Once a user confirms each car works and a test route is
    # added to selfdrive/car/tests/routes.py, we can remove it from this list.
    ret.dashcamOnly = candidate in {CAR.CADILLAC_ATS, CAR.HOLDEN_ASTRA, CAR.MALIBU, CAR.BUICK_REGAL, CAR.EQUINOX}

    # Start with a baseline tuning for all GM vehicles. Override tuning as needed in each model section below.
    # Some GMs need some tolerance above 10 kph to avoid a fault
    ret.minSteerSpeed = 7 * CV.MPH_TO_MS
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.00]]
    ret.lateralTuning.pid.kf = 0.00004   # full torque for 20 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
    tire_stiffness_factor = 0.444  # not optimized yet

    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    if candidate == CAR.VOLT:
      ret.minEnableSpeed = -1
      ret.mass = 1607. + STD_CARGO_KG
      ret.wheelbase = 2.69
      ret.steerRatio = 17.7  # Stock 15.7, LiveParameters
      tire_stiffness_factor = 0.469  # Stock Michelin Energy Saver A/S, LiveParameters
      ret.centerToFront = ret.wheelbase * 0.45  # Volt Gen 1, TODO corner weigh

      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      ret.steerActuatorDelay = 0.2
      
      ret.longitudinalTuning.kpBP = [5., 15., 35.]
      ret.longitudinalTuning.kpV = [0.8, .9, 0.8]
      ret.longitudinalTuning.kiBP = [5., 15., 35.]
      ret.longitudinalTuning.kiV = [0.08, 0.13, 0.13]
      ret.longitudinalTuning.kdBP = [5., 25.]
      ret.longitudinalTuning.kdV = [0.3, 0.0]
      ret.stoppingDecelRate = 0.2 # brake_travel/s while trying to stop
      ret.longitudinalActuatorDelayLowerBound = 0.41
      ret.longitudinalActuatorDelayUpperBound = 0.41

    elif candidate == CAR.MALIBU:
      ret.mass = 1496. + STD_CARGO_KG
      ret.wheelbase = 2.83
      ret.steerRatio = 15.8
      ret.centerToFront = ret.wheelbase * 0.4  # wild guess

    elif candidate == CAR.HOLDEN_ASTRA:
      ret.mass = 1363. + STD_CARGO_KG
      ret.wheelbase = 2.662
      # Remaining parameters copied from Volt for now
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerRatio = 15.7

    elif candidate == CAR.ACADIA:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 4353. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.86
      ret.steerRatio = 16.0 #14.4  # end to end is 13.46
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4
      ret.steerActuatorDelay = 0.24
        
      ret.lateralTuning.pid.kpBP = [i * CV.MPH_TO_MS for i in [0., 80.]]
      ret.lateralTuning.pid.kpV = [0., 0.16]
      ret.lateralTuning.pid.kiBP = [0., 35.]
      ret.lateralTuning.pid.kiV = [0.008, 0.012]
      ret.lateralTuning.pid.kdBP = [0.]
      ret.lateralTuning.pid.kdV = [0.6]
      ret.lateralTuning.pid.kf = 1. # get_steer_feedforward_acadia()

      ret.longitudinalTuning.kdBP = [5., 25.]
      ret.longitudinalTuning.kdV = [0.8, 0.4]
      ret.longitudinalTuning.kiBP = [5., 35.]
      ret.longitudinalTuning.kiV = [0.31, 0.34]
      ret.longitudinalActuatorDelayUpperBound = 0.5  # large delay to initially start braking

    elif candidate == CAR.BUICK_REGAL:
      ret.mass = 3779. * CV.LB_TO_KG + STD_CARGO_KG  # (3849+3708)/2
      ret.wheelbase = 2.83  # 111.4 inches in meters
      ret.steerRatio = 14.4  # guess for tourx
      ret.centerToFront = ret.wheelbase * 0.4  # guess for tourx

    elif candidate == CAR.CADILLAC_ATS:
      ret.mass = 1601. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 15.3
      ret.centerToFront = ret.wheelbase * 0.5

    elif candidate == CAR.ESCALADE_ESV:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 2739. + STD_CARGO_KG
      ret.wheelbase = 3.302
      ret.steerRatio = 17.3
      ret.centerToFront = ret.wheelbase * 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[10., 41.0], [10., 41.0]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.13, 0.24], [0.01, 0.02]]
      ret.lateralTuning.pid.kf = 0.000045
      tire_stiffness_factor = 1.0
  
    elif candidate == CAR.ESCALADE_ESV_2019:
      ret.minEnableSpeed = -1.  # engage speed is decided by pcm
      ret.mass = 2739. + STD_CARGO_KG
      ret.wheelbase = 3.302
      ret.steerRatio = 17.3
      ret.centerToFront = ret.wheelbase * 0.5
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
      tire_stiffness_factor = 1.0

    elif candidate == CAR.BOLT_EUV:
      ret.mass = 1669. + STD_CARGO_KG
      ret.wheelbase = 2.63779
      ret.steerRatio = 16.8
      ret.centerToFront = 2.15  # measured
      tire_stiffness_factor = 1.0
      ret.steerActuatorDelay = 0.2
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.SILVERADO:
      ret.minEnableSpeed = -1.
      ret.minSteerSpeed = -1 * CV.MPH_TO_MS
      ret.mass = 2400. + STD_CARGO_KG
      ret.wheelbase = 3.745
      ret.steerRatio = 16.3
      ret.centerToFront = ret.wheelbase * .49
      ret.steerActuatorDelay = 0.11
      tire_stiffness_factor = 1.0
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    elif candidate == CAR.EQUINOX:
      ret.mass = 3500. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.72
      ret.steerRatio = 14.4
      ret.centerToFront = ret.wheelbase * 0.4
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)
    
    t = sec_since_boot()

    ret.madsEnabled = self.CS.madsEnabled
    ret.accEnabled = self.CS.accEnabled
    ret.leftBlinkerOn = self.CS.leftBlinkerOn
    ret.rightBlinkerOn = self.CS.rightBlinkerOn
    ret.belowLaneChangeSpeed = self.CS.belowLaneChangeSpeed

    buttonEvents = []

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
      buttonEvents.append(create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS))
      # Handle ACCButtons changing buttons mid-press
      if self.CS.cruise_buttons != CruiseButtons.UNPRESS and self.CS.prev_cruise_buttons != CruiseButtons.UNPRESS:
        buttonEvents.append(create_button_event(CruiseButtons.UNPRESS, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS))

    # MADS BUTTON
    if self.CS.out.madsEnabled != self.CS.madsEnabled:
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = True
      be.type = ButtonType.altButton1
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents

    # The ECM allows enabling on falling edge of set, but only rising edge of resume
    events = self.create_common_events(ret, extra_gears=[GearShifter.sport, GearShifter.low,
                                                         GearShifter.eco, GearShifter.manumatic],
                                       pcm_enable=False, enable_buttons=(ButtonType.decelCruise,))

    # Enabling at a standstill with brake is allowed
    # TODO: verify 17 Volt can enable for the first time at a stop and allow for all GMs
    below_min_enable_speed = ret.vEgo < self.CP.minEnableSpeed or self.CS.moving_backward
    if below_min_enable_speed and not (ret.standstill and ret.brake >= 20 and
                                       self.CP.networkLocation == NetworkLocation.fwdCamera) and \
                                       ret.cruiseState.enabled and not self.CS.out.cruiseState.enabled and \
                                       not self.CS.madsEnabled:
      events.add(EventName.belowEngageSpeed)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if ret.vEgo < self.CP.minSteerSpeed and self.CS.madsEnabled and ret.vEgo > 0.05:
      events.add(EventName.belowSteerSpeed)

    if self.CS.autoHoldActivated:
      self.CS.lastAutoHoldTime = t
    if EventName.accFaulted in events.events and \
        (t - self.CS.sessionInitTime < 10.0 or
        t - self.CS.lastAutoHoldTime < 1.0):
      events.events.remove(EventName.accFaulted)

    self.CS.disengageByBrake = self.CS.disengageByBrake or ret.disengageByBrake

    enable_pressed = False
    enable_from_brake = False

    if self.CS.disengageByBrake and not ret.brakePressed and not ret.brakeHoldActive and not ret.parkingBrake and self.CS.madsEnabled:
      enable_pressed = True
      enable_from_brake = True

    if not ret.brakePressed and not ret.brakeHoldActive and not ret.parkingBrake:
      self.CS.disengageByBrake = False
      ret.disengageByBrake = False

    if self.CP.pcmCruise:
      # do enable on both accel and decel buttons
      if ret.cruiseState.enabled and not self.CS.out.cruiseState.enabled:
        enable_pressed = True

    for b in ret.buttonEvents:
      # do disable on button down
      if b.type == ButtonType.cancel:
        if not self.CS.madsEnabled:
          events.add(EventName.buttonCancel)
        elif not self.cruise_cancelled_btn:
          self.cruise_cancelled_btn = True
          events.add(EventName.manualLongitudinalRequired)
      # do enable on both accel and decel buttons
      if (b.type == ButtonType.accelCruise and b.pressed) or (b.type == ButtonType.decelCruise and not b.pressed) and not self.CP.pcmCruise:
        enable_pressed = self.CS.resumeAllowed
      # do disable on MADS button if ACC is disabled
      if b.type == ButtonType.altButton1 and b.pressed:
        if not self.CS.madsEnabled: # disabled MADS
          if not ret.cruiseState.enabled:
            events.add(EventName.buttonCancel)
          else:
            events.add(EventName.manualSteeringRequired)
        else: # enabled MADS
          if not ret.cruiseState.enabled:
            enable_pressed = True
    if (ret.cruiseState.enabled or self.CS.madsEnabled) and enable_pressed:
      if enable_from_brake:
        events.add(EventName.silentButtonEnable)
      else:
        events.add(EventName.buttonEnable)

    if ret.cruiseState.enabled:
      self.cruise_cancelled_btn = False

    ret.events = events.to_msg()

    return ret

  def apply(self, c):
    can_sends = self.CC.update(c, self.CS)
    # Release Auto Hold and creep smoothly when regenpaddle pressed
    if self.CS.regenPaddlePressed and self.CS.autoHold:
      self.CS.autoHoldActive = False

    if self.CS.autoHold and not self.CS.autoHoldActive and not self.CS.regenPaddlePressed:
      if self.CS.out.vEgo > 0.03:
        self.CS.autoHoldActive = True
      elif self.CS.out.vEgo < 0.02 and self.CS.out.brakePressed:
        self.CS.autoHoldActive = True
        
    return can_sends
