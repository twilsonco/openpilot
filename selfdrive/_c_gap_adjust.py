# PFEIFER - GA
from typing import Optional
from selfdrive._c_button_manager import ButtonManager, ButtonState, ButtonObserver
from selfdrive._c_mem import get, update
from common.numpy_fast import interp
from time import time
from selfdrive._c_mem import get, update

GA_ENABLED_PARAM = "GapAdjustEnabled"
GA_LEVEL_PARAM = "GapAdjustLevel"
GAP_BUTTON = "gap"

A_CHANGE_COST = 200.
X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0

LIMIT_COST = 1e6
DANGER_ZONE_COST = 100.

PARAM_CHECK_INTERVAL = 5 # seconds, how long before checking if enabled

class GapAdjust:
  def __init__(self):
    self.multiplier_breakpoints = [1.0, 1.50, 2.50]

    self.button_manager: Optional[ButtonManager] = None
    self.gap_button: Optional[ButtonObserver] = None
    self._enabled = get(GA_ENABLED_PARAM, False, True)
    self.last_check = time()
    self.gap_times = [1.2, 1.4, 1.8, 2.5]
    self.setup() # set default values

  def setup(self, default_gap: float = 1.45, max_level: int = 4):
    self.default_gap = default_gap
    self.max_level = max_level
    self.button_manager = ButtonManager()
    self.gap_button = self.button_manager.getObserver("gap_adjust", GAP_BUTTON)

  @property
  def enabled(self) -> bool:
    if time() - self.last_check > PARAM_CHECK_INTERVAL:
      self.last_check = time()
      self._enabled = get(GA_ENABLED_PARAM, False, True)
    return True # TODO: self._enabled


  @property
  def gap_level(self) -> int:
    """
    The current gap level, 0 is the lowest, max_level - 1 is the highest. Lower
    gap levels result in a smaller gap to the lead car.
    """
    if not self.enabled:
      return self.max_level - 1
    try:
      return int(get(GA_LEVEL_PARAM)) % self.max_level
    except:
      return self.max_level - 1

  @gap_level.setter
  def gap_level(self, value: int):
    update(GA_LEVEL_PARAM, int(value) % self.max_level)

  @property
  def gap_time(self) -> float:
    if not self.enabled:
      return self.default_gap
    return self.gap_times[self.gap_level]

  @property
  def next_gap_level(self) -> int:
    try:
      return (self.gap_level + 1) % self.max_level
    except:
      return self.max_level - 1


  # multipliers / weights pulled from https://github.com/FrogAi/FrogPilot
  # @KRKeegan adjustments to costs for different TFs
  # further tuned by @FrogAi
  @property
  def a_change_multiplier(self) -> float:
    return interp(self.gap_time, self.multiplier_breakpoints, [.1, .8, 2.])

  @property
  def j_ego_multiplier(self) -> float:
    return interp(self.gap_time, self.multiplier_breakpoints, [.6, 1., 2.])

  @property
  def d_zone_multiplier(self) -> float:
    return interp(self.gap_time, self.multiplier_breakpoints, [1.6, 1., .5])

  def cost_weights(self, prev_accel_constraint):
    a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
    if self.enabled:
      return [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_cost * self.a_change_multiplier, J_EGO_COST * self.j_ego_multiplier]
    else:
      return [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, a_change_cost, J_EGO_COST]

  def constraint_cost_weights(self):
    if self.enabled:
      return [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST * self.d_zone_multiplier]
    else:
      return [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]


  def update(self):
    if self.gap_button is not None:
      status = self.gap_button.simple_status()
      if status is not None and status.state == ButtonState.SINGLE_PRESS:
        self.gap_level = self.next_gap_level

ga = GapAdjust()
