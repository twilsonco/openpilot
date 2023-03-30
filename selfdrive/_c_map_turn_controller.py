import numpy as np
from common.conversions import Conversions as CV
from common.numpy_fast import clip
from common.realtime import sec_since_boot, DT_MDL
from time import time
from selfdrive._c_mem import get


# Maximum rate of change of acceleration this controller will request
MAX_JERK = 1.5 # m/s^3

# The maximum this controller will request to decelerate
MAX_DECEL = -3.5 # m/s^2

DISTANCE_V_PERCENT = [0, 0.2, 1] # 1=100%
DISTANCE_V_PERCENT_BP = [8, 4, 0] # seconds

V_TARGET_OFFSET = [0, 8, 3, 2.5, 0] # m/s
V_TARGET_OFFSET_BP = [0, 10, 20, 25, 30] # m/s

MAX_ACTIVE_ACCEL = 0.5

MIN_TARGET_V = 5 # m/s

def v_offset(target_vs):
  def offset(v):
    offset = np.interp(v, V_TARGET_OFFSET_BP, V_TARGET_OFFSET)
    v_o = v + offset
    return v_o

  return np.vectorize(offset)(target_vs)

class MapTurnController():
  def __init__(self, VM):
    self.VM = VM
    self.op_enabled = False
    self.gas_pressed = False
    self.last_params_update = 0
    self.v_cruise_setpoint = 0
    self.v_ego = 0
    self.a_ego = 0
    self.a_target = 0
    self.v_target = 0
    self.enabled = get("TurnMapControl", False, True)
    self.has_lead = False
    self.last_update = time()
    self.update_time_diff = DT_MDL # seconds, updated each update loop
    self.turn_speed_limits = np.array([])
    self.turn_speed_distances = np.array([])
    self.v_distance = 0

    self.reset()

  @property
  def turn_speed_limit(self) -> float:
    return float(get("mapTurnSpeedLimit", 0))

  @property
  def turn_speed_end_distance(self) -> float:
    return float(get("mapTurnSpeedLimitEndDistance", 0))

  @property
  def active(self):
    limit_speed = self.v_target < self.v_cruise_setpoint
    return self.op_enabled and not self.gas_pressed and self.enabled and not self.has_lead and limit_speed

  @property
  def plan(self):
    v_target = self.v_target if self.v_target < self.v_cruise_setpoint else self.v_cruise_setpoint
    return (self.a_target, v_target)

  def reset(self):
    self.current_lat_accel = 0
    self.max_pred_lat_acc = 0
    self.x_plan = []
    self.y_plan = []

  def update_params(self):
    time = sec_since_boot()
    if time > self.last_params_update + 5.0:
      self.enabled = get("TurnMapControl", False, True)
      self.last_params_update = time

  def update_current_state(self, sm):
    """
    Uses the current state of the car to calculate the curvature based off the
    angle of the wheels and store the max acceptable velocity for the curve as
    well as the current lateral acceleration.
    """
    self.has_lead = sm['radarState'].leadOne.status
    lp = sm['liveParameters']
    sa = sm['carState'].steeringAngleDeg * CV.DEG_TO_RAD
    current_curvature = self.VM.calc_curvature(sa, self.v_ego, lp.roll)

    self.current_lat_accel = current_curvature * self.v_ego**2
    self.current_curvature = current_curvature
    now = time()
    self.update_time_diff = now - self.last_update
    self.last_update = now


  def apply_limits(self):
    # VTSC not needed or disabled, do not change targets
    if not self.enabled:
      self.a_target = self.a_ego
      self.v_target = self.v_cruise_setpoint
      return

    self.a_target = min(max(self.a_target, MAX_DECEL), MAX_ACTIVE_ACCEL)
    self.v_target = min(self.v_target, self.v_cruise_setpoint) # Never request more velocity than the set max

    # limit accel based on MAX_JERK
    min_accel = self.a_ego - (MAX_JERK * self.update_time_diff)
    max_accel = self.a_ego + (MAX_JERK * self.update_time_diff)
    self.a_target = clip(self.a_target, min_accel, max_accel)


  def update_calculations(self, sm):
    self.update_current_state(sm)

    v_ego = max(self.v_ego, 0.1) # ensure a value greater than 0 for calculations

    if len(self.turn_speed_limits) == 0:
      self.turn_speed_limits = np.array([self.v_cruise_setpoint])
      self.turn_speed_distances = np.array([self.v_cruise_setpoint])

    # Get target velocity for curve
    v_target_idx = np.where(self.turn_speed_distances > 0)
    v_targets = np.array([self.turn_speed_limits[i] for i in v_target_idx[0]])
    v_targets = np.maximum(v_targets, MIN_TARGET_V)
    if self.turn_speed_end_distance > 0:
      v_targets = np.minimum(self.turn_speed_limit + 2, v_targets)
    distances = np.array([self.turn_speed_distances[i] for i in v_target_idx[0]])
    a_targets = ((v_targets ** 2) - (v_ego ** 2)) / (2 * distances)
    max_idx = np.argmin(a_targets)

    distance_s = distances[max_idx] / self.v_ego
    v_percent = np.interp(distance_s, DISTANCE_V_PERCENT_BP, DISTANCE_V_PERCENT)
    v_diff = v_targets[max_idx] - self.v_cruise_setpoint
    self.v_target = self.v_cruise_setpoint + (v_diff * v_percent)
    if self.turn_speed_end_distance > 0:
      self.v_target = min(self.v_target, self.turn_speed_limit + 2)

    self.a_target = a_targets[max_idx] * v_percent
    self.a_target = min(self.a_ego, self.a_target, MAX_ACTIVE_ACCEL)
    self.v_distance = distances[max_idx]

    if self.v_target <= 0:
      self.v_target = self.v_cruise_setpoint
      self.a_target = self.a_ego

    # update targets based on limits
    self.apply_limits()


  def update(self, enabled, v_ego, a_ego, v_cruise_setpoint, sm):
    self.op_enabled = enabled
    self.gas_pressed = sm['carState'].gasPressed
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint
    self.turn_speed_limits = np.array(get('mapTurnSpeedLimitsAhead', []))
    self.turn_speed_distances = np.array(get('mapTurnSpeedLimitsAheadDistances', []))

    if len(self.turn_speed_limits) == 0 or len(self.turn_speed_limits) != len(self.turn_speed_distances):
      self.v_target = self.v_cruise_setpoint
      self.a_target = a_ego
      return

    self.update_params()
    self.update_calculations(sm)
