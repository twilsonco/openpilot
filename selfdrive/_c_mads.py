# PFEIFER - mads

from typing import Any
from cereal import log, car
from common.params import put_bool_nonblocking
from selfdrive.controls.lib.latcontrol import MIN_LATERAL_CONTROL_SPEED
from panda import ALTERNATIVE_EXPERIENCE
from selfdrive.controls.lib.events import EngagementAlert, AudibleAlert
from selfdrive._c_mem import get, update

State = log.ControlsState.OpenpilotState
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
STOP_SPEED = 0.1 # m/s, disable below this speed

class Mads:
  def __init__(self, braking = False, blinkers_active = False, op_active = False, standstill = False, steer_fault = False, invalid_gear = False):
    self.braking = braking
    self.blinkers_active = blinkers_active
    self.op_active = op_active
    self.standstill = standstill
    self.steer_fault = steer_fault
    self.invalid_gear = invalid_gear
    self.controls: Any = None
    self.last_lat_allowed = False
    self.stopped = True


  def setup(self, controls):
    self.controls = controls

  @property
  def enabled(self) -> bool:
    return bool(get("MadsEnabled", False, True))

  @property
  def disable_lat_on_brake(self) -> bool:
    return bool(get("DisengageLatOnBrake", False, True))

  @property
  def disable_lat_on_blinker(self) -> bool:
    return bool(get("DisengageLatOnBlinker", False, True))

  @property
  def lateral_allowed(self) -> bool:
    return bool(get("LateralAllowed", False))

  @lateral_allowed.setter
  def lateral_allowed(self, lateral_allowed: bool):
    update("LateralAllowed", bool(lateral_allowed))


  def update(self, car_state):
    lateral_allowed = self.lateral_allowed
    panda_states = self.controls.sm['pandaStates']


    # Update states used for determining if lat controls should be active
    self.braking = car_state.brakePressed or car_state.regenBraking
    self.blinkers_active = car_state.leftBlinker or car_state.rightBlinker
    self.standstill = car_state.vEgo <= max(self.controls.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or car_state.standstill
    self.op_active = self.controls.state in ACTIVE_STATES
    self.steer_fault = car_state.steerFaultTemporary or car_state.steerFaultPermanent
    self.car_lat_active = car_state.cruiseState.available
    self.invalid_gear = car_state.gearShifter not in [car.CarState.GearShifter.drive, car.CarState.GearShifter.neutral, car.CarState.GearShifter.sport, car.CarState.GearShifter.low, car.CarState.GearShifter.eco, car.CarState.GearShifter.manumatic]
    self.stopped = car_state.vEgo < STOP_SPEED

    # Panda controls allowed state refers to the longitudinal allowed state. Whenever
    # longitudinal is allowed we change the lateral_allowed state to true as it
    # helps keep the lateral allowed state in sync with panda without having to
    # change the panda messages to include the lateral allowed state.
    if any(ps.controlsAllowed for ps in panda_states) and not lateral_allowed:
      update("LateralAllowed", True)
      put_bool_nonblocking("LateralAllowed", True)

  def update_alert(self):
    """
    Checks for a change of lat allowed state and sends the appropriate
    AudibleAlert to the alert manager.
    """
    lateral_allowed = self.lateral_allowed
    if self.last_lat_allowed != lateral_allowed:
      alert = None
      if lateral_allowed:
        alert = EngagementAlert(AudibleAlert.engage)
      else:
        alert = EngagementAlert(AudibleAlert.disengage)
      self.controls.AM.add_many(self.controls.sm.frame, [alert])
    self.last_lat_allowed = lateral_allowed

  @property
  def lat_active(self) -> bool:
    """
    Whether lateral controls should be active for the car.
    """

    # If car is in a gear that does not move forward do not engage lateral
    if self.invalid_gear:
      return False

    # If the car is stopped do not engage lateral
    if self.stopped:
      return False

    # If there is a steer fault lat is not available
    if self.steer_fault:
      return False

    # If the car lateral control is not active lat cannot be active
    if self.enabled and not self.lateral_allowed:
      return False

    # If mads is disabled then lat is only active when openpilot is active
    if not self.enabled and not self.op_active:
      return False

    # If DisengageLatOnBrake is enabled we disable lat when braking
    if self.disable_lat_on_brake and self.braking:
      return False

    # If DisengageLatOnBlinker is enabled we disable lat when blinkers are on
    if self.disable_lat_on_blinker and self.blinkers_active:
      return False

    # Lat is enabled if we pass all previous tests for disengagement
    return True

  def toggle_lateral_allowed(self):
    self.lateral_allowed = not self.lateral_allowed
    self.update_alert()

  @property
  def alternative_experience(self):
    """
    Gets the alternative experience to use based on the current settings. Should
    "logical or" with other alternative experiences in openpilot before setting
    the alternative experience in panda.
    """
    dlob = self.disable_lat_on_brake

    if self.enabled and dlob:
      return ALTERNATIVE_EXPERIENCE.ENABLE_MADS
    if self.enabled and not dlob:
      return ALTERNATIVE_EXPERIENCE.MADS_DISABLE_DISENGAGE_LATERAL_ON_BRAKE
    return 0

mads = Mads()
