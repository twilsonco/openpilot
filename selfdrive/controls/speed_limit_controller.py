from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
import json

params = Params()
params_memory = Params("/dev/shm/params")

class SpeedLimitController:
  car_speed_limit: float = 0  # m/s
  map_speed_limit: float = 0  # m/s
  nav_speed_limit: float = 0  # m/s
  prv_speed_limit: float = 0  # m/s

  def __init__(self) -> None:
    self.update_toggles()
    self.write_map_state()
    self.write_nav_state()

  def update_current_max_velocity(self, car_speed_limit: float, max_v: float, frogpilot_toggles_updated: bool = False, load_state: bool = True, write_state: bool = True) -> None:
    self.car_speed_limit = car_speed_limit
    if load_state:
      self.load_state()
      if frogpilot_toggles_updated:
        self.update_toggles()

  @property
  def offset(self) -> float:
    if self.speed_limit < 15.6464:
      return self.offset1
    elif self.speed_limit < 24.5872:
      return self.offset2
    elif self.speed_limit < 29.0576:
      return self.offset3
    else:
      return self.offset4

  @property
  def speed_limit(self) -> float:
    priority_orders = [
      ["nav", "car", "map"],
      ["nav", "map", "car"],
      ["nav", "map"],
      ["nav", "car"],
      ["nav"],
      ["map", "car", "nav"],
      ["map", "nav", "car"],
      ["map", "nav"],
      ["map", "car"],
      ["map"],
      ["car", "nav", "map"],
      ["car", "map", "nav"],
      ["car", "map"],
      ["car", "nav"],
      ["car"]
    ]

    if self.highest:
      return max(filter(lambda x: x > 0, [self.car_speed_limit, self.map_speed_limit, self.nav_speed_limit]))
    elif self.lowest:
      nonzero_limits = list(filter(lambda x: x > 0, [self.car_speed_limit, self.map_speed_limit, self.nav_speed_limit]))
      return min(nonzero_limits) if nonzero_limits else 0
    elif self.speed_limit_priority < len(priority_orders):
      for source in priority_orders[self.speed_limit_priority]:
        speed_limit = getattr(self, f"{source}_speed_limit", None)
        if speed_limit:
          self.prv_speed_limit = speed_limit
          return speed_limit

    if self.use_experimental_mode:
      params_memory.put_int("ConditionalStatus", 2)
    elif self.use_previous_limit:
      return self.prv_speed_limit

    return 0

  @property
  def desired_speed_limit(self):
    return self.speed_limit + self.offset if self.speed_limit else 0

  def load_state(self):
    self.nav_speed_limit = json.loads(params_memory.get("NavSpeedLimit"))
    self.map_speed_limit = json.loads(params_memory.get("MapSpeedLimit"))

  def write_map_state(self):
    params_memory.put("MapSpeedLimit", json.dumps(self.map_speed_limit))

  def write_nav_state(self):
    params_memory.put("NavSpeedLimit", json.dumps(self.nav_speed_limit))

  def update_toggles(self):
    conversion = CV.KPH_TO_MS if params.get_bool("IsMetric") else CV.MPH_TO_MS

    self.offset1 = params.get_int("Offset1") * conversion
    self.offset2 = params.get_int("Offset2") * conversion
    self.offset3 = params.get_int("Offset3") * conversion
    self.offset4 = params.get_int("Offset4") * conversion

    self.highest = params.get_int("SLCPriority") == 15
    self.lowest = params.get_int("SLCPriority") == 16
    self.speed_limit_priority = params.get_int("SLCPriority")

    slc_fallback = params.get_int("SLCFallback")
    self.use_experimental_mode = slc_fallback == 1
    self.use_previous_limit = slc_fallback == 2

slc = SpeedLimitController()
