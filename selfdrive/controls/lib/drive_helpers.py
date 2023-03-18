from cereal import car
from common.filter_simple import FirstOrderFilter
from common.op_params import opParams
from common.numpy_fast import clip, interp
from common.realtime import DT_MDL, DT_CTRL
from common.params import Params
from selfdrive.config import Conversions as CV
from selfdrive.modeld.constants import T_IDXS


# kph
V_CRUISE_MAX = 145
V_CRUISE_MIN = 1
V_CRUISE_DELTA = 5
V_CRUISE_OFFSET = 3
V_CRUISE_ENABLE_MIN = 5
LAT_MPC_N = 16
LON_MPC_N = 32
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
MIN_SPEED = 1.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

# this corresponds to 80deg/s and 20deg/s steering angle in a toyota corolla
MAX_CURVATURE_RATES = [0.03762194918267951, 0.003441203371932992]
MAX_CURVATURE_RATE_SPEEDS = [0, 35]

# Constants for Limit controllers.
LIMIT_ADAPT_ACC = -1.  # m/s^2 Ideal acceleration for the adapting (braking) phase when approaching speed limits.
LIMIT_MIN_ACC = -1.5  # m/s^2 Maximum deceleration allowed for limit controllers to provide.
LIMIT_MAX_ACC = 3.0   # m/s^2 Maximum acelration allowed for limit controllers to provide while active.
LIMIT_MIN_SPEED = 8.33  # m/s, Minimum speed limit to provide as solution on limit controllers.
LIMIT_SPEED_OFFSET_TH = -1.  # m/s Maximum offset between speed limit and current speed for adapting state.
LIMIT_MAX_MAP_DATA_AGE = 10.  # s Maximum time to hold to map data, then consider it invalid inside limits controllers.


class MPC_COST_LAT:
  PATH = 1.0
  HEADING = 1.0
  STEER_RATE = 1.0


class MPC_COST_LONG:
  TTC = 5.0
  DISTANCE = 0.1
  ACCELERATION = 10.0
  JERK = 20.0

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)

class ClusterSpeed:
  def __init__(self, is_metric):
    self._op_params = opParams(calling_function='drive_helpers.py ClusterSpeed')
    self.v_ego = FirstOrderFilter(0.0, self._op_params.get('MISC_cluster_speed_smoothing_factor', force_update=True), DT_CTRL)
    self.deadzone = self._op_params.get('MISC_cluster_speed_deadzone', force_update=True)
    self.is_metric = is_metric
    self.cluster_speed_last = 0
    self.frame = 0
  
  def update(self, v_ego, do_reset=False):
    if self.frame > 350:
      self.frame = 0
      self.v_ego.update_alpha(self._op_params.get('MISC_cluster_speed_smoothing_factor'))
      self.deadzone = self._op_params.get('MISC_cluster_speed_deadzone')
    self.frame += 1
      
    if do_reset:
      self.v_ego.x = v_ego
    else:
      self.v_ego.update(v_ego)
      
    out = max(self.v_ego.x * (CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH), 0.0)
    if do_reset or out < 0.5 - self.deadzone or abs(out - self.cluster_speed_last) > 1.0 + self.deadzone:
      self.cluster_speed_last = int(round(out))
    
    return int(self.cluster_speed_last)
      
def get_cluster_speed(v_ego, cluster_speed_last, is_metric):
  out = v_ego * (CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH)
  if abs(out - cluster_speed_last) > 1.25:
    return int(round(out))
  else:
    return cluster_speed_last

def set_v_cruise_offset(offset):
  global V_CRUISE_OFFSET
  V_CRUISE_OFFSET = offset

def update_v_cruise(v_cruise_kph, buttonEvents, enabled, cur_time, accel_pressed,decel_pressed,accel_pressed_last,decel_pressed_last, fastMode, stock_speed_adjust, vEgo_kph, gas_pressed):
  
  if stock_speed_adjust:
    if enabled:
      if accel_pressed:
        if ((cur_time-accel_pressed_last) >= 0.6667 or (fastMode and (cur_time-accel_pressed_last) >= 0.5)):
          v_cruise_kph += V_CRUISE_DELTA - (v_cruise_kph % V_CRUISE_DELTA)
      elif decel_pressed:
        if ((cur_time-decel_pressed_last) >= 0.6667 or (fastMode and (cur_time-decel_pressed_last) >= 0.5)):
          v_cruise_kph -= V_CRUISE_DELTA - ((V_CRUISE_DELTA - v_cruise_kph) % V_CRUISE_DELTA)
      else:
        for b in buttonEvents:
          if not b.pressed:
            if b.type == car.CarState.ButtonEvent.Type.accelCruise:
              if (not fastMode):
                v_cruise_kph += 1
            elif b.type == car.CarState.ButtonEvent.Type.decelCruise:
              if (not fastMode):
                if gas_pressed:
                  v_cruise_kph = vEgo_kph
                else:
                  v_cruise_kph -= 1
      v_cruise_kph = clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX)
  else:    
    if enabled:
      if accel_pressed:
        if ((cur_time-accel_pressed_last) >= 0.6667 or (fastMode and (cur_time-accel_pressed_last) >= 0.5)):
          v_cruise_kph += 1
      elif decel_pressed:
        if ((cur_time-decel_pressed_last) >= 0.6667 or (fastMode and (cur_time-decel_pressed_last) >= 0.5)):
          v_cruise_kph -= 1
      else:
        for b in buttonEvents:
          if not b.pressed:
            if b.type == car.CarState.ButtonEvent.Type.accelCruise:
              if (not fastMode):
                v_cruise_kph += V_CRUISE_DELTA - (v_cruise_kph % V_CRUISE_DELTA - V_CRUISE_OFFSET)
            elif b.type == car.CarState.ButtonEvent.Type.decelCruise:
              if (not fastMode):
                if gas_pressed:
                  v_cruise_kph = vEgo_kph
                else:
                  v_cruise_kph -= V_CRUISE_DELTA - ((V_CRUISE_DELTA - v_cruise_kph + V_CRUISE_OFFSET) % V_CRUISE_DELTA)
      v_cruise_kph = clip(v_cruise_kph, V_CRUISE_MIN, V_CRUISE_MAX)

  return v_cruise_kph

def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last):
  for b in buttonEvents:
    # 250kph or above probably means we never had a set speed
    if b.type == car.CarState.ButtonEvent.Type.accelCruise and v_cruise_last < 250:
      return v_cruise_last

  return int(round(clip(v_ego * CV.MS_TO_KPH, V_CRUISE_ENABLE_MIN, V_CRUISE_MAX)))

def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, curvature_rates):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    curvature_rates = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = CP.steerActuatorDelay + .2

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = interp(delay, T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  desired_curvature_rate = curvature_rates[0]
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature_rate = clip(desired_curvature_rate,
                                     -max_curvature_rate,
                                     max_curvature_rate)
  safe_desired_curvature = clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)

  return safe_desired_curvature, safe_desired_curvature_rate
