# PFEIFER - CMS
from selfdrive._c_mem import get, update

class CurrentMaxSpeed:
  @property
  def max_speed(self) -> float:
    return float(get("CurrentMaxSpeed", 0.0))

  @max_speed.setter
  def max_speed(self, value: float):
    update("CurrentMaxSpeed", float(value))

cms = CurrentMaxSpeed()
