import json

from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params

params = Params()
params_memory = Params("/dev/shm/params")

class SpeedLimitController:
  car_speed_limit: float = 0  # m/s
  lst_speed_limit: float = 0  # m/s
  map_speed_limit: float = 0  # m/s
  nav_speed_limit: float = 0  # m/s

  def __init__(self) -> None:
    self.update_frogpilot_params()

    self.write_car_state()
    self.write_map_state()
    self.write_nav_state()

  def update_speed_limits(self):
    self.car_speed_limit = json.loads(params_memory.get("CarSpeedLimit"))
    self.map_speed_limit = json.loads(params_memory.get("MapSpeedLimit"))
    self.nav_speed_limit = json.loads(params_memory.get("NavSpeedLimit"))

  @property
  def offset(self) -> float:
    if self.speed_limit < 14:
      return self.offset1
    elif self.speed_limit < 24:
      return self.offset2
    elif self.speed_limit < 29:
      return self.offset3
    else:
      return self.offset4

  @property
  def speed_limit(self) -> float:
    limits = [self.car_speed_limit, self.map_speed_limit, self.nav_speed_limit]
    filtered_limits = [limit for limit in limits if limit != 0]

    if self.highest:
      return max(filtered_limits)
    if self.lowest:
      return min(filtered_limits)

    priority_map = {"Dashboard": "car", "Navigation": "nav", "Offline Maps": "map"}
    priorities = [self.speed_limit_priority1, self.speed_limit_priority2, self.speed_limit_priority3]
    valid_priorities = [priority_map[p] for p in priorities if p in priority_map]

    for source in valid_priorities:
      limit = getattr(self, f"{source}_speed_limit", 0)
      if limit != 0:
        self.prv_speed_limit = limit
        return limit

    if self.use_previous_limit:
      if self.lst_speed_limit != self.prv_speed_limit:
        params.put_float("PreviousSpeedLimit", self.prv_speed_limit)
      self.lst_speed_limit = self.prv_speed_limit
      return self.prv_speed_limit

    return 0

  @property
  def desired_speed_limit(self):
    return self.speed_limit + self.offset if self.speed_limit != 0 else 0

  @property
  def experimental_mode(self):
    return self.speed_limit == 0 and self.use_experimental_mode

  def write_car_state(self):
    params_memory.put("CarSpeedLimit", json.dumps(self.car_speed_limit))

  def write_map_state(self):
    params_memory.put("MapSpeedLimit", json.dumps(self.map_speed_limit))

  def write_nav_state(self):
    params_memory.put("NavSpeedLimit", json.dumps(self.nav_speed_limit))

  def update_frogpilot_params(self):
    conversion = CV.KPH_TO_MS if params.get_bool("IsMetric") else CV.MPH_TO_MS

    self.offset1 = params.get_int("Offset1") * conversion
    self.offset2 = params.get_int("Offset2") * conversion
    self.offset3 = params.get_int("Offset3") * conversion
    self.offset4 = params.get_int("Offset4") * conversion

    self.speed_limit_priority1 = params.get("SLCPriority1", encoding='utf-8')
    self.speed_limit_priority2 = params.get("SLCPriority2", encoding='utf-8')
    self.speed_limit_priority3 = params.get("SLCPriority3", encoding='utf-8')

    self.highest = self.speed_limit_priority1 == "Highest"
    self.lowest = self.speed_limit_priority1 == "Lowest"

    slc_fallback = params.get_int("SLCFallback")
    self.use_experimental_mode = slc_fallback == 1
    self.use_previous_limit = slc_fallback == 2

    self.prv_speed_limit = params.get_float("PreviousSpeedLimit")

SpeedLimitController = SpeedLimitController()
