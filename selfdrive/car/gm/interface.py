#!/usr/bin/env python3
from math import fabs, sin
from cereal import car
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from common.params import Params
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car.gm.values import CAR, CruiseButtons, \
                                    AccState, CarControllerParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.controls.lib.longitudinal_planner import _A_CRUISE_MAX_V_SPORT, \
                                                        _A_CRUISE_MAX_BP, \
                                                        calc_cruise_accel_limits

FOLLOW_AGGRESSION = 0.15 # (Acceleration/Decel aggression) Lower is more aggressive

# revert to stock max negative accel based on relative lead velocity
_A_MIN_V_STOCK_FACTOR_BP = [-5. * CV.MPH_TO_MS, 1. * CV.MPH_TO_MS]
_A_MIN_V_STOCK_FACTOR_V = [0., 1.]

# increase/decrease max accel based on vehicle pitch
INCLINE_ACCEL_SCALE_BP = [i * CV.MPH_TO_MS for i in [25., 75]] # [mph] lookup speeds for additional offset
INCLINE_ACCEL_SCALE_V = [1.5, 1.4] # [m/s^2] additional scale factor to change how incline affects accel based on speed
INCLINE_ACCEL_MAX_SPORT_FACTOR = 0.9 # acceleration will never be increased to more than this factor of the "sport" acceleration at the current speed
DECLINE_ACCEL_FACTOR = 0.5 # this factor of g accel is used to lower max accel limit so you don't floor it downhill
DECLINE_ACCEL_MIN = 0.2 # [m/s^2] don't decrease acceleration limit due to decline below this total value

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
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
    
    # decrease/increase max accel based on vehicle pitch
    g_accel = 9.81 * sin(CI.CS.pitch)
    if g_accel > 0.:
      accel_limits[1] = max(accel_limits[1], min(INCLINE_ACCEL_MAX_SPORT_FACTOR * interp(current_speed, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_SPORT), g_accel * interp(current_speed, INCLINE_ACCEL_SCALE_BP, INCLINE_ACCEL_SCALE_V)))
    else:
      accel_limits[1] = max(DECLINE_ACCEL_MIN, accel_limits[1] + g_accel * DECLINE_ACCEL_FACTOR)
    
    time_since_engage = CI.CS.t - CI.CS.cruise_enabled_last_t
    if time_since_engage < CI.CS.cruise_enabled_neg_accel_ramp_bp[-1]:
      accel_limits[0] *= interp(time_since_engage, CI.CS.cruise_enabled_neg_accel_ramp_bp, CI.CS.cruise_enabled_neg_accel_ramp_v)
      
    return [max(CI.params.ACCEL_MIN, accel_limits[0]), min(accel_limits[1], CI.params.ACCEL_MAX)]

  # Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
  @staticmethod
  def get_steer_feedforward_volt(desired_angle, v_ego):
    # maps [-inf,inf] to [-1,1]: sigmoid(34.4 deg) = sigmoid(1) = 0.5
    # 1 / 0.02904609 = 34.4 deg ~= 36 deg ~= 1/10 circle? Arbitrary?
    desired_angle *= 0.02904609
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.10006696 * sigmoid * (v_ego + 3.12485927)

  @staticmethod
  def get_steer_feedforward_acadia(desired_angle, v_ego):
    desired_angle *= 0.09760208
    sigmoid = desired_angle / (1 + fabs(desired_angle))
    return 0.04689655 * sigmoid * (v_ego + 10.028217)

  def get_steer_feedforward_function(self):
    if self.CP.carFingerprint == CAR.VOLT:
      return self.get_steer_feedforward_volt
    elif self.CP.carFingerprint == CAR.ACADIA:
      return self.get_steer_feedforward_acadia
    else:
      return CarInterfaceBase.get_steer_feedforward_default

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
      ret.steerRateCost = 1.0
      tire_stiffness_factor = 0.469 # Stock Michelin Energy Saver A/S, LiveParameters
      ret.steerRatioRear = 0.
      ret.centerToFront = 0.45 * ret.wheelbase # from Volt Gen 1

      ret.lateralTuning.pid.kpBP = [0., 40.]
      ret.lateralTuning.pid.kpV = [0.0, .20]
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kiV = [0.02]
      ret.lateralTuning.pid.kdBP = [i * CV.MPH_TO_MS for i in [15., 30., 55.]]
      ret.lateralTuning.pid.kdV = [0.1, 0.25, 0.3]
      ret.lateralTuning.pid.kf = 1. # !!! ONLY for sigmoid feedforward !!!
      ret.steerActuatorDelay = 0.2

      # Only tuned to reduce oscillations. TODO.
      ret.longitudinalTuning.kpV = [1.7, 1.3]
      ret.longitudinalTuning.kiBP = [5., 35.]
      ret.longitudinalTuning.kiV = [0.31, 0.34]
      ret.longitudinalTuning.kdV = [0.8, 0.4]
      ret.longitudinalTuning.kdBP = [5., 25.]

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
      ret.steerActuatorDelay = 0.4
      ret.centerToFront = ret.wheelbase * 0.4
      ret.lateralTuning.pid.kpBP = [i * CV.MPH_TO_MS for i in [0., 80.]]
      ret.lateralTuning.pid.kpV = [0., 0.2]
      ret.lateralTuning.pid.kiBP = [0.0]
      ret.lateralTuning.pid.kiV = [0.025]
      ret.lateralTuning.pid.kdV = [0.35]
      ret.lateralTuning.pid.kf = 1. # get_steer_feedforward_acadia()
      ret.longitudinalTuning.kdBP = [5., 25.]
      ret.longitudinalTuning.kdV = [0.8, 0.2]
      ret.longitudinalTuning.kiBP = [5., 35.]
      ret.longitudinalTuning.kiV = [0.3, 0.34]

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

    ret = self.CS.update(self.cp, self.cp_loopback)

    t = sec_since_boot()

    cruiseEnabled = self.CS.pcm_acc_status != AccState.OFF
    ret.cruiseState.enabled = cruiseEnabled

    ret.canValid = self.cp.can_valid and self.cp_loopback.can_valid
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
    
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      self.one_pedal_mode = self.CS._params.get_bool("OnePedalMode")

    # distance button is also used to toggle braking modes when in one-pedal-mode
    if self.CS.one_pedal_mode_active or self.CS.coast_one_pedal_mode_active:
      if self.CS.distance_button != self.CS.prev_distance_button:
        if not self.CS.distance_button and self.CS.one_pedal_mode_engaged_with_button and t - self.CS.distance_button_last_press_t < 0.8: #user just engaged one-pedal with distance button hold and immediately let off the button, so default to regen/engine braking. If they keep holding, it does hard braking
          cloudlog.info("button press event: Engaging one-pedal mode with distance button.")
          self.CS.one_pedal_brake_mode = 0
          self.one_pedal_last_brake_mode = self.CS.one_pedal_brake_mode
          self.CS.one_pedal_mode_enabled = False
          self.CS.one_pedal_mode_active = False
          self.CS.coast_one_pedal_mode_active = True
          tmp_params = Params()
          tmp_params.put("OnePedalBrakeMode", str(self.CS.one_pedal_brake_mode))
          tmp_params.put_bool("OnePedalMode", self.CS.one_pedal_mode_enabled)
        else:
          if not self.one_pedal_mode and self.CS.distance_button: # user lifted press of distance button while in coast-one-pedal mode, so turn on braking
            cloudlog.info("button press event: Engaging one-pedal braking.")
            self.CS.one_pedal_last_switch_to_friction_braking_t = t
            self.CS.distance_button_last_press_t = t + 0.5
            self.CS.one_pedal_brake_mode = 0
            self.one_pedal_last_brake_mode = self.CS.one_pedal_brake_mode
            self.CS.one_pedal_mode_enabled = True
            self.CS.one_pedal_mode_active = True
            tmp_params = Params()
            tmp_params.put("OnePedalBrakeMode", str(self.CS.one_pedal_brake_mode))
            tmp_params.put_bool("OnePedalMode", self.CS.one_pedal_mode_enabled)
          elif self.CS.distance_button and (self.CS.pause_long_on_gas_press or self.CS.out.standstill) and t - self.CS.distance_button_last_press_t < 0.4 and t - self.CS.one_pedal_last_switch_to_friction_braking_t > 1.: # on the second press of a double tap while the gas is pressed, turn off one-pedal braking
            # cycle the brake mode back to nullify the first press
            cloudlog.info("button press event: Disengaging one-pedal mode with distace button double-press.")
            self.CS.distance_button_last_press_t = t + 0.5
            self.CS.one_pedal_brake_mode = 0
            self.one_pedal_last_brake_mode = self.CS.one_pedal_brake_mode
            self.CS.one_pedal_mode_enabled = False
            self.CS.one_pedal_mode_active = False
            self.CS.coast_one_pedal_mode_active = True
            tmp_params = Params()
            tmp_params.put("OnePedalBrakeMode", str(self.CS.one_pedal_brake_mode))
            tmp_params.put_bool("OnePedalMode", self.CS.one_pedal_mode_enabled)
          else:
            if self.CS.distance_button:
              self.CS.distance_button_last_press_t = t
              cloudlog.info("button press event: Distance button pressed in one-pedal mode.")
            else: # only make changes when user lifts press
              if self.CS.one_pedal_brake_mode == 2:
                cloudlog.info("button press event: Disengaging one-pedal hard braking. Switching to moderate braking")
                self.CS.one_pedal_brake_mode = 1
                tmp_params = Params()
                tmp_params.put("OnePedalBrakeMode", str(self.CS.one_pedal_brake_mode))
              elif t - self.CS.distance_button_last_press_t > 0. and t - self.CS.distance_button_last_press_t < 0.4: # only switch braking on a single tap (also allows for ignoring presses by setting last_press_t to be greater than t)
                self.CS.one_pedal_brake_mode = (self.CS.one_pedal_brake_mode + 1) % 2
                cloudlog.info(f"button press event: one-pedal braking. New value: {self.CS.one_pedal_brake_mode}")
                tmp_params = Params()
                tmp_params.put("OnePedalBrakeMode", str(self.CS.one_pedal_brake_mode))
          self.CS.one_pedal_mode_engaged_with_button = False
      elif self.CS.distance_button and t - self.CS.distance_button_last_press_t > 0.3:
        if self.CS.one_pedal_brake_mode < 2:
          cloudlog.info("button press event: Engaging one-pedal hard braking.")
          self.one_pedal_last_brake_mode = self.CS.one_pedal_brake_mode
        self.CS.one_pedal_brake_mode = 2
      self.CS.follow_level = self.CS.one_pedal_brake_mode + 1
    else: # cruis is active, so just modify follow distance
      if self.CS.distance_button != self.CS.prev_distance_button:
        if self.CS.distance_button:
          self.CS.distance_button_last_press_t = t
          cloudlog.info("button press event: Distance button pressed in cruise mode.")
        else: # apply change on button lift
          self.CS.follow_level -= 1
          if self.CS.follow_level < 1:
            self.CS.follow_level = 3
          tmp_params = Params()
          tmp_params.put("FollowLevel", str(self.CS.follow_level))
          cloudlog.info("button press event: cruise follow distance button. new value: %r" % self.CS.follow_level)
      elif self.CS.distance_button and t - self.CS.distance_button_last_press_t > 0.5 and not (self.CS.one_pedal_mode_active or self.CS.coast_one_pedal_mode_active):
          # user held follow button while in normal cruise, so engage one-pedal mode
          cloudlog.info("button press event: distance button hold to engage one-pedal mode.")
          self.CS.one_pedal_mode_engage_on_gas = True
          self.CS.one_pedal_mode_engaged_with_button = True
          self.CS.distance_button_last_press_t = t + 0.2 # gives the user X+0.3 seconds to release the distance button before hard braking is applied (which they may want, so don't want too long of a delay)

    ret.readdistancelines = self.CS.follow_level

    events = self.create_common_events(ret, pcm_enable=False)

    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if self.CS.pause_long_on_gas_press:
      events.add(EventName.gasPressed)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    steer_paused = False
    if cruiseEnabled:
      if t - self.CS.last_pause_long_on_gas_press_t < 0.5 and t - self.CS.sessionInitTime > 10.:
        events.add(car.CarEvent.EventName.pauseLongOnGasPress)
      if not ret.standstill and self.CS.lkMode and self.CS.lane_change_steer_factor < 1.:
        events.add(car.CarEvent.EventName.blinkerSteeringPaused)
        steer_paused = True
    if ret.vEgo < self.CP.minSteerSpeed:
      if ret.standstill and cruiseEnabled and not ret.brakePressed and not self.CS.pause_long_on_gas_press and not self.CS.autoHoldActivated and not self.CS.disengage_on_gas and t - self.CS.sessionInitTime > 10.:
        events.add(car.CarEvent.EventName.stoppedWaitForGas)
      elif not steer_paused and self.CS.lkMode:
        events.add(car.CarEvent.EventName.belowSteerSpeed)
    if self.CS.autoHoldActivated:
      self.CS.lastAutoHoldTime = t
      events.add(car.CarEvent.EventName.autoHoldActivated)
    if self.CS.pcm_acc_status == AccState.FAULTED and t - self.CS.sessionInitTime > 10.0 and t - self.CS.lastAutoHoldTime > 1.0:
      events.add(EventName.accFaulted)

    # handle button presses
    for b in ret.buttonEvents:
      # do enable on both accel and decel buttons
      # The ECM will fault if resume triggers an enable while speed is set to 0
      if b.type == ButtonType.accelCruise and c.hudControl.setSpeed > 0 and c.hudControl.setSpeed < 70 and not b.pressed:
        events.add(EventName.buttonEnable)
      if b.type == ButtonType.decelCruise and not b.pressed:
        events.add(EventName.buttonEnable)
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
    self.CS.one_pedal_mode_engage_on_gas = False
    if pause_long_on_gas_press and not self.CS.pause_long_on_gas_press:
      self.CS.one_pedal_mode_engage_on_gas = (self.CS.one_pedal_mode_engage_on_gas_enabled and self.CS.vEgo >= self.CS.one_pedal_mode_engage_on_gas_min_speed and not self.CS.one_pedal_mode_active and not self.CS.coast_one_pedal_mode_active)
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
      if self.CS.out.vEgo > 0.03:
        self.CS.autoHoldActive = True
      elif self.CS.out.vEgo < 0.02 and self.CS.out.brakePressed:
        self.CS.autoHoldActive = True

    return can_sends
