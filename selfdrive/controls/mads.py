# PFEIFER - mads

from cereal import log, car
from common.params import Params
from selfdrive.controls.lib.latcontrol import MIN_LATERAL_CONTROL_SPEED
from panda import ALTERNATIVE_EXPERIENCE
from selfdrive.controls.lib.events import EngagementAlert, AudibleAlert

State = log.ControlsState.OpenpilotState
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)

# Speeds below this will deactivate lateral controls
STOP_SPEED = 0.2 # m/s

class Mads:
  def __init__(self, CI, braking = False, blinkers_active = False, op_active = False, standstill = False, steer_fault = False, invalid_gear = False):
    self.CI = CI
    self.braking = braking
    self.blinkers_active = blinkers_active
    self.op_active = op_active
    self.standstill = standstill
    self.steer_fault = steer_fault
    self.invalid_gear = invalid_gear
    self.last_lat_allowed = False
    self.stopped = True

    self.params = Params()

  def update(self, car_state, op_state, car_params, sm, alert_manager):
    panda_states = sm['pandaStates']
    lateral_allowed = self.params.get_bool("LateralAllowed")
    if self.last_lat_allowed != lateral_allowed:
      alert = None
      if lateral_allowed:
        alert = EngagementAlert(AudibleAlert.engage)
      else:
        alert = EngagementAlert(AudibleAlert.disengage)
      alert_manager.add_many(sm.frame, [alert])
    self.last_lat_allowed = lateral_allowed
    self.braking = car_state.brakePressed or car_state.regenBraking
    self.blinkers_active = car_state.leftBlinker or car_state.rightBlinker
    self.standstill = car_state.vEgo <= max(car_params.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or car_state.standstill
    self.op_active = op_state in ACTIVE_STATES
    self.steer_fault = car_state.steerFaultTemporary or car_state.steerFaultPermanent
    self.car_lat_active = car_state.cruiseState.available
    self.invalid_gear = car_state.gearShifter not in [car.CarState.GearShifter.drive, car.CarState.GearShifter.sport, car.CarState.GearShifter.low, car.CarState.GearShifter.eco]
    self.stopped = car_state.vEgo < STOP_SPEED

    # Always allow lateral when controls are allowed
    if any(ps.controlsAllowed for ps in panda_states) and not lateral_allowed:
      self.params.put_bool("LateralAllowed", True)

  @property
  def lat_active(self):

    # If car is in a gear that does not move forward do not engage lateral
    if self.invalid_gear:
      return False

    # If there is a steer fault lat is not available
    if self.steer_fault:
      return False

    # Disable lateral when vehicle is stopped to prevent "twitching" steering wheel
    if self.stopped:
      return False

    # If the car lateral control is not active lat cannot be active
    if self.params.get_bool('MadsEnabled') and not self.params.get_bool("LateralAllowed"):
      return False

    # If mads is disabled then lat is only active when openpilot is active
    if not self.params.get_bool('MadsEnabled') and not self.op_active:
      return False

    # If DisengageLatOnBrake is enabled we disable lat when braking
    if self.params.get_bool('DisengageLatOnBrake') and self.braking:
      return False

    # If DisengageLatOnBlinker is enabled we disable lat when blinkers are on
    if self.params.get_bool('DisengageLatOnBlinker') and self.blinkers_active:
      return False

    # Lat is enabled if we pass all previous tests for disengagement
    return True

  @property
  def alternative_experience(self):
    mads_enabled = self.params.get_bool('MadsEnabled')
    dlob = self.params.get_bool('DisengageLatOnBrake')
    experience = 0
    if mads_enabled:
      experience |= ALTERNATIVE_EXPERIENCE.ENABLE_MADS
    if not dlob:
      experience |= ALTERNATIVE_EXPERIENCE.MADS_DISABLE_DISENGAGE_LATERAL_ON_BRAKE
    return experience

