import numpy as np

from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params

params = Params()
params_memory = Params("/dev/shm/params")

# Acceleration profiles - Credit goes to the DragonPilot team!
                 # MPH = [0.,  35,   35,  40,    40,  45,    45,  67,    67,   67, 123]
A_CRUISE_MIN_BP_CUSTOM = [0., 2.0, 2.01, 11., 11.01, 18., 18.01, 28., 28.01,  33., 55.]
                 # MPH = [0., 6.71, 13.4, 17.9, 24.6, 33.6, 44.7, 55.9, 67.1, 123]
A_CRUISE_MAX_BP_CUSTOM = [0.,    3,   6.,   8.,  11.,  15.,  20.,  25.,  30., 55.]

A_CRUISE_MIN_VALS_ECO = [-0.480, -0.480, -0.40, -0.40, -0.40, -0.36, -0.32, -0.28, -0.28, -0.25, -0.25]
A_CRUISE_MAX_VALS_ECO = [3.5, 3.3, 1.7, 1.1, .76, .62, .47, .36, .28, .09]

A_CRUISE_MIN_VALS_SPORT = [-0.500, -0.500, -0.42, -0.42, -0.42, -0.42, -0.40, -0.35, -0.35, -0.30, -0.30]
A_CRUISE_MAX_VALS_SPORT = [3.5, 3.5, 3.0, 2.6, 1.4, 1.0, 0.7, 0.6, .38, .2]

class FrogPilotFunctions:
  @staticmethod
  def get_min_accel_eco(v_ego):
    return interp(v_ego, A_CRUISE_MIN_BP_CUSTOM, A_CRUISE_MIN_VALS_ECO)

  @staticmethod
  def get_max_accel_eco(v_ego):
    return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_ECO)

  @staticmethod
  def get_min_accel_sport(v_ego):
    return interp(v_ego, A_CRUISE_MIN_BP_CUSTOM, A_CRUISE_MIN_VALS_SPORT)

  @staticmethod
  def get_max_accel_sport(v_ego):
    return interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_SPORT)

  @staticmethod
  def calculate_lane_width(lane, current_lane, road_edge):
    lane_x, lane_y = np.array(lane.x), np.array(lane.y)
    edge_x, edge_y = np.array(road_edge.x), np.array(road_edge.y)
    current_x, current_y = np.array(current_lane.x), np.array(current_lane.y)

    lane_y_interp = np.interp(current_x, lane_x[lane_x.argsort()], lane_y[lane_x.argsort()])
    road_edge_y_interp = np.interp(current_x, edge_x[edge_x.argsort()], edge_y[edge_x.argsort()])

    distance_to_lane = np.mean(np.abs(current_y - lane_y_interp))
    distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y_interp))

    return min(distance_to_lane, distance_to_road_edge)

  @staticmethod
  def lkas_button_function(conditional_experimental_mode):
    if conditional_experimental_mode:
      # Set "CEStatus" to work with "Conditional Experimental Mode"
      conditional_status = params_memory.get_int("CEStatus")
      override_value = 0 if conditional_status in (1, 2, 3, 4) else 1 if conditional_status >= 5 else 2
      params_memory.put_int("CEStatus", override_value)
    else:
      experimental_mode = params.get_bool("ExperimentalMode")
      # Invert the value of "ExperimentalMode"
      params.put_bool("ExperimentalMode", not experimental_mode)

  @staticmethod
  def road_curvature(modelData, v_ego):
    predicted_velocities = np.array(modelData.velocity.x)
    curvature_ratios = np.abs(np.array(modelData.acceleration.y)) / (predicted_velocities**2)
    return np.amax(curvature_ratios * (v_ego**2))
