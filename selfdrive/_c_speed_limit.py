from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from selfdrive._c_mem import get, update
from selfdrive._c_button_manager import ButtonManager, ButtonState
import numpy as np

DISTANCE_V_PERCENT = [0, 0.2, 1] # 1=100%
DISTANCE_V_PERCENT_BP = [8, 4, 0] # seconds

class SpeedLimitController:
  def __init__(self):
    self.last_cruise = V_CRUISE_MAX
    self.bm = ButtonManager()
    self.gap_button = self.bm.getObserver("SpeedLimitControl", "gap")

  @property
  def speed_limit(self) -> float:
    return float(get("speedLimit", 0))

  @speed_limit.setter
  def speed_limit(self, value: float):
    update('speedLimit', float(value))

  @property
  def upcoming_speed_limit(self) -> float:
    return float(get('upcomingSpeedLimit', 0))

  @property
  def upcoming_speed_limit_with_offset(self) -> float:
    if self.upcoming_speed_limit < 1:
      return 0
    return self.upcoming_speed_limit + self.speed_offset


  @property
  def upcoming_speed_limit_distance(self) -> float:
    return float(get('upcomingSpeedLimitDistance', 0))

  @property
  def speed_offset(self) -> float:
    return float(get("SpeedLimitOffset", 0))

  @speed_offset.setter
  def speed_offset(self, value: float):
    return update("SpeedLimitOffset", float(value))

  @property
  def enabled(self) -> bool:
    return bool(get("SpeedLimitControlEnabled", False, True))

  @property
  def limit_with_offset(self) -> float:
    if self.speed_limit < 1:
      return 0
    return self.speed_limit + self.speed_offset

  def get_max(self, cruise):
    if not self.enabled:
      return cruise


    gap_button_status = self.gap_button.simple_status()
    if gap_button_status is not None and gap_button_status.state == ButtonState.DOUBLE_PRESS and self.speed_limit != 0:
      # Use the difference between the speed limit and cruise as offset for future changes in speed limit
      self.speed_offset = cruise - self.speed_limit


    if (self.limit_with_offset == 0 or self.limit_with_offset > cruise) \
      and (self.upcoming_speed_limit_with_offset == 0 or self.upcoming_speed_limit_with_offset > cruise):
      return cruise

    # change to upcoming speed limit
    if self.upcoming_speed_limit_distance > 0:
      distance_s = self.upcoming_speed_limit_distance / cruise
      if distance_s < 6 and self.upcoming_speed_limit > 0 and self.upcoming_speed_limit < self.speed_limit:
        return self.upcoming_speed_limit_with_offset

    if self.speed_limit > 0:
      # return current speed limit
      return self.limit_with_offset

    return cruise

slc = SpeedLimitController()
