#!/usr/bin/env python3
from math import fabs
from cereal import car
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car.gm.values import CAR, CruiseButtons, \
                                    AccState, CarControllerParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

FOLLOW_AGGRESSION = 0.15 # (Acceleration/Decel aggression) Lower is more aggressive


ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    params = CarControllerParams()
    return params.ACCEL_MIN, params.ACCEL_MAX

  # Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  def get_steer_feedforward(self, desired_angle, v_ego):
    if self.CP.carFingerprint in [CAR.VOLT]:
      # Sigmoid maps [-inf,inf] to [-1,1].
      # This scalar gives sigmoid(34.4 deg) = sigmoid(1) = 0.5.
      # 1 / 0.02904609 = 34.4 deg ~= 36 deg ~= 1/10 circle? Arbitrary scaling?
      desired_angle *= 0.02904609
      sigmoid = desired_angle / (1 + fabs(desired_angle))
      return 0.10006696 * sigmoid * (v_ego + 3.12485927)
    else:
      return CarInterfaceBase.get_steer_feedforward(desired_angle, v_ego)

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

    # Default lateral controller params.
    ret.minSteerSpeed = 7 * CV.MPH_TO_MS
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

    if candidate == CAR.VOLT:
      # supports stop and go, but initial engage must be above 18mph (which include conservatism)
      ret.minEnableSpeed = -1
      ret.mass = 1607. + STD_CARGO_KG
      ret.wheelbase = 2.69
      ret.steerRatio = 17.7  # Stock 15.7, LiveParameters
      tire_stiffness_factor = 0.469 # Stock Michelin Energy Saver A/S, LiveParameters
      ret.steerRatioRear = 0.
      ret.centerToFront = 0.45 * ret.wheelbase # from Volt Gen 1

      ret.lateralTuning.pid.kpBP = [10. * CV.MPH_TO_MS,
                                    20. * CV.MPH_TO_MS,
                                    25. * CV.MPH_TO_MS,
                                    30. * CV.MPH_TO_MS,
                                    55. * CV.MPH_TO_MS,
                                    65. * CV.MPH_TO_MS,
                                    85. * CV.MPH_TO_MS]
      ret.lateralTuning.pid.kpV = [0.025,
                                   0.03,
                                   0.04,
                                   0.055,
                                   0.14,
                                   0.18,
                                   0.25]
      ret.lateralTuning.pid.kiBP = [0.]
      ret.lateralTuning.pid.kiV = [0.01]
      ret.lateralTuning.pid.kf = 1. # !!! ONLY for sigmoid feedforward !!!
      ret.steerActuatorDelay = 0.2

      # Only tuned to reduce oscillations. TODO.
      ret.longitudinalTuning.kpV = [1.7, 1.3]
      ret.longitudinalTuning.kiV = [0.36]

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
      ret.mass = 3956. * CV.LB_TO_KG + STD_CARGO_KG # from vin decoder
      ret.wheelbase = 2.86 # Confirmed from vin decoder
      ret.steerRatio = 16.5  # end to end is 13.46 - seems to be undocumented, using JYoung value
      ret.steerRatioRear = 0.
      ret.centerToFront = ret.wheelbase * 0.4
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.5], [0.1]] # using JYoung overrides
      ret.lateralTuning.pid.kf = 0.00004   # Using JYoung value - full torque for 20 deg at 80mph means 0.00007818594

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

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.36]

    ret.startAccel = 0.8

    ret.steerLimitTimer = 0.4
    ret.radarTimeStep = 0.0667  # GM radar runs at 15Hz instead of standard 20Hz

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)

    cruiseEnabled = self.CS.pcm_acc_status != AccState.OFF
    ret.cruiseState.enabled = cruiseEnabled


    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    ret.engineRPM = self.CS.engineRPM

    buttonEvents = []

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
        if not (ret.cruiseState.enabled and ret.standstill):
          be.type = ButtonType.accelCruise  # Suppress resume button if we're resuming from stop so we don't adjust speed.
      elif but == CruiseButtons.DECEL_SET:
        if not cruiseEnabled and not self.CS.lkMode:
          self.lkMode = True
        be.type = ButtonType.decelCruise
      elif but == CruiseButtons.CANCEL:
        be.type = ButtonType.cancel
      elif but == CruiseButtons.MAIN:
        be.type = ButtonType.altButton3
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents

    if cruiseEnabled and self.CS.lka_button and self.CS.lka_button != self.CS.prev_lka_button:
      self.CS.lkMode = not self.CS.lkMode
      cloudlog.info("button press event: LKA button. new value: %i" % self.CS.lkMode)

    if self.CS.distance_button and self.CS.distance_button != self.CS.prev_distance_button:
       self.CS.follow_level -= 1
       if self.CS.follow_level < 1:
         self.CS.follow_level = 3
       cloudlog.info("button press event: cruise follow distance button. new value: %r" % self.CS.follow_level)

    ret.readdistancelines = self.CS.follow_level

    events = self.create_common_events(ret, pcm_enable=False)

    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if self.CS.pause_long_on_gas_press:
      events.add(EventName.gasPressed)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    if ret.vEgo < self.CP.minSteerSpeed:
      if ret.vEgo < 0.1 and cruiseEnabled and not ret.brakePressed:
        events.add(car.CarEvent.EventName.stoppedWaitForGas)
      else:
        events.add(car.CarEvent.EventName.belowSteerSpeed)
    t = sec_since_boot()
    if cruiseEnabled:
      if t - self.CS.last_pause_long_on_gas_press_t < 0.5 and t - self.CS.sessionInitTime > 10.:
        events.add(car.CarEvent.EventName.pauseLongOnGasPress)
      if self.CS.lane_change_steer_factor < 1.:
        events.add(car.CarEvent.EventName.blinkerSteeringPaused)
    if self.CS.autoHoldActivated:
      self.CS.lastAutoHoldTime = t
      events.add(car.CarEvent.EventName.autoHoldActivated)
    if self.CS.pcm_acc_status == AccState.FAULTED and t - self.CS.sessionInitTime > 3.0 and t - self.CS.lastAutoHoldTime > 1.0:
      events.add(EventName.accFaulted)

    # handle button presses
    for b in ret.buttonEvents:
      # do enable on both accel and decel buttons
      if b.type in [ButtonType.accelCruise, ButtonType.decelCruise] and not b.pressed:
        events.add(EventName.buttonEnable)
      # do disable on button down
      if b.type == ButtonType.cancel and b.pressed:
        events.add(EventName.buttonCancel)

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
    pause_long_on_gas_press = c.enabled and self.CS.gasPressed and not self.disengage_on_gas
    t = sec_since_boot()
    if pause_long_on_gas_press and not self.CS.pause_long_on_gas_press:
      if t - self.CS.last_pause_long_on_gas_press_t > 300.:
        self.CS.last_pause_long_on_gas_press_t = t
    if self.CS.gasPressed:
      self.CS.one_pedal_mode_last_gas_press_t = t
      
    self.CS.pause_long_on_gas_press = pause_long_on_gas_press
    enabled = c.enabled or self.CS.pause_long_on_gas_press

    can_sends = self.CC.update(enabled, self.CS, self.frame,
                               c.actuators,
                               hud_v_cruise, c.hudControl.lanesVisible,
                               c.hudControl.leadVisible, c.hudControl.visualAlert)

    self.frame += 1

    # Release Auto Hold and creep smoothly when regenpaddle pressed
    if self.CS.regenPaddlePressed and self.CS.autoHold:
      self.CS.autoHoldActive = False

    if self.CS.autoHold and not self.CS.autoHoldActive and not self.CS.regenPaddlePressed:
      if self.CS.out.vEgo > 0.02:
        self.CS.autoHoldActive = True
      elif self.CS.out.vEgo < 0.01 and self.CS.out.brakePressed:
        self.CS.autoHoldActive = True

    return can_sends
