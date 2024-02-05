import cereal.messaging as messaging
import numpy as np

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN, get_max_accel

from openpilot.selfdrive.frogpilot.functions.frogpilot_functions import FrogPilotFunctions

from openpilot.selfdrive.frogpilot.functions.conditional_experimental_mode import ConditionalExperimentalMode
from openpilot.selfdrive.frogpilot.functions.map_turn_speed_controller import MapTurnSpeedController

class FrogPilotPlanner:
  def __init__(self, params, params_memory):
    self.cem = ConditionalExperimentalMode()
    self.fpf = FrogPilotFunctions()
    self.mtsc = MapTurnSpeedController()

    self.mtsc_target = 0
    self.road_curvature = 0
    self.v_cruise = 0

    self.accel_limits = [A_CRUISE_MIN, get_max_accel(0)]

    self.update_frogpilot_params(params, params_memory)

  def update(self, carState, controlsState, modelData, mpc, sm, v_ego):
    enabled = controlsState.enabled

    v_cruise_kph = controlsState.vCruiseCluster if controlsState.vCruiseCluster != 0.0 else controlsState.vCruise
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    # Acceleration profiles
    v_cruise_changed = (self.mtsc_target) + 1 < v_cruise  # Use stock acceleration profiles to handle MTSC more precisely
    if v_cruise_changed:
      self.accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
    elif self.acceleration_profile == 1:
      self.accel_limits = [self.fpf.get_min_accel_eco(v_ego), self.fpf.get_max_accel_eco(v_ego)]
    elif self.acceleration_profile in (2, 3):
      self.accel_limits = [self.fpf.get_min_accel_sport(v_ego), self.fpf.get_max_accel_sport(v_ego)]
    else:
      self.accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]

    # Conditional Experimental Mode
    if self.conditional_experimental_mode and enabled:
      self.cem.update(carState, sm['frogpilotNavigation'], modelData, mpc, sm['radarState'], self.road_curvature, carState.standstill, v_ego)

    # Update the max allowed speed
    if enabled:
      self.v_cruise = self.update_v_cruise(carState, controlsState, enabled, modelData, v_cruise, v_ego)
    else:
      self.mtsc_target = v_cruise
      self.v_cruise = v_cruise

    # Lane detection
    check_lane_width = self.adjacent_lanes or self.blind_spot_path
    if check_lane_width and v_ego >= LANE_CHANGE_SPEED_MIN:
      self.lane_width_left = float(self.fpf.calculate_lane_width(modelData.laneLines[0], modelData.laneLines[1], modelData.roadEdges[0]))
      self.lane_width_right = float(self.fpf.calculate_lane_width(modelData.laneLines[3], modelData.laneLines[2], modelData.roadEdges[1]))
    else:
      self.lane_width_left = 0
      self.lane_width_right = 0

    # Update the current road curvature
    self.road_curvature = self.fpf.road_curvature(modelData, v_ego)

  def update_v_cruise(self, carState, controlsState, enabled, modelData, v_cruise, v_ego):
    # Pfeiferj's Map Turn Speed Controller
    if self.map_turn_speed_controller and enabled:
      self.mtsc_target = np.clip(self.mtsc.target_speed(v_ego, carState.aEgo), MIN_TARGET_V, v_cruise)
      if self.mtsc_target / v_ego > self.mtsc_limit:
        self.mtsc_target = v_cruise
    else:
      self.mtsc_target = v_cruise

    return min(v_cruise, self.mtsc_target)

  def publish(self, sm, pm, mpc):
    frogpilot_plan_send = messaging.new_message('frogpilotPlan')
    frogpilot_plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])
    frogpilotPlan = frogpilot_plan_send.frogpilotPlan

    frogpilotPlan.adjustedCruise = self.mtsc_target * (CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH)
    frogpilotPlan.conditionalExperimental = self.cem.experimental_mode

    frogpilotPlan.desiredFollowDistance = mpc.safe_obstacle_distance - mpc.stopped_equivalence_factor
    frogpilotPlan.safeObstacleDistance = mpc.safe_obstacle_distance
    frogpilotPlan.safeObstacleDistanceStock = mpc.safe_obstacle_distance_stock
    frogpilotPlan.stoppedEquivalenceFactor = mpc.stopped_equivalence_factor

    frogpilotPlan.laneWidthLeft = self.lane_width_left
    frogpilotPlan.laneWidthRight = self.lane_width_right

    frogpilotPlan.redLight = self.cem.red_light_detected

    pm.send('frogpilotPlan', frogpilot_plan_send)

  def update_frogpilot_params(self, params, params_memory):
    self.is_metric = params.get_bool("IsMetric")

    self.conditional_experimental_mode = params.get_bool("ConditionalExperimental")
    if self.conditional_experimental_mode:
      self.cem.update_frogpilot_params(self.is_metric, params)
      params.put_bool("ExperimentalMode", True)

    self.custom_personalities = params.get_bool("CustomPersonalities")
    self.aggressive_follow = params.get_float("AggressiveFollow")
    self.standard_follow = params.get_float("StandardFollow")
    self.relaxed_follow = params.get_float("RelaxedFollow")
    self.aggressive_jerk = params.get_float("AggressiveJerk")
    self.standard_jerk = params.get_float("StandardJerk")
    self.relaxed_jerk = params.get_float("RelaxedJerk")

    custom_ui = params.get_bool("CustomUI")
    self.adjacent_lanes = params.get_bool("AdjacentPath") and custom_ui
    self.blind_spot_path = params.get_bool("BlindSpotPath") and custom_ui

    longitudinal_tune = params.get_bool("LongitudinalTune")
    self.acceleration_profile = params.get_int("AccelerationProfile") if longitudinal_tune else 0
    self.aggressive_acceleration = params.get_bool("AggressiveAcceleration") and longitudinal_tune
    self.increased_stopping_distance = params.get_int("StoppingDistance") * (1 if self.is_metric else CV.FOOT_TO_METER) if longitudinal_tune else 0

    self.map_turn_speed_controller = params.get_bool("MTSCEnabled")
    if self.map_turn_speed_controller:
      self.mtsc_limit = params.get_float("MTSCLimit") / 100
      params_memory.put_float("MapTargetLatA", 2 * (params.get_int("MTSCAggressiveness") / 100))
