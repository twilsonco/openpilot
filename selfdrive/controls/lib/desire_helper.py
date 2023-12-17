from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
import numpy as np

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
TurnDirection = log.LateralPlan.Desire

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  TurnDirection.none: log.LateralPlan.Desire.none,
  TurnDirection.turnLeft: log.LateralPlan.Desire.turnLeft,
  TurnDirection.turnRight: log.LateralPlan.Desire.turnRight,
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none

    # FrogPilot variables
    self.turn_direction = TurnDirection.none
    self.lane_change_completed = False
    self.turn_completed = False
    self.lane_change_wait_timer = 0

  # Lane detection
  def calculate_lane_width(self, lane, current_lane, road_edge):
    # Interpolate lane values at current_lane.x positions
    lane_x, lane_y = np.array(lane.x), np.array(lane.y)
    edge_x, edge_y = np.array(road_edge.x), np.array(road_edge.y)
    current_x, current_y = np.array(current_lane.x), np.array(current_lane.y)

    # Interpolate lane and road edge values at current_lane.x positions
    lane_y_interp = np.interp(current_x, lane_x[lane_x.argsort()], lane_y[lane_x.argsort()])
    road_edge_y_interp = np.interp(current_x, edge_x[edge_x.argsort()], edge_y[edge_x.argsort()])

    # Calculate the mean absolute distances
    distance_to_lane = np.mean(np.abs(current_y - lane_y_interp))
    distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y_interp))

    # Return the smallest between the two
    return min(distance_to_lane, distance_to_road_edge)

  def update(self, carstate, modeldata, lateral_active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Calculate left and right lane widths for the blindspot path
    self.lane_width_left = 0
    self.lane_width_right = 0
    turning = abs(carstate.steeringAngleDeg) >= 60
    if self.blindspot_path and not below_lane_change_speed and not turning:
      # Calculate left and right lane widths
      self.lane_width_left = self.calculate_lane_width(modeldata.laneLines[0], modeldata.laneLines[1], modeldata.roadEdges[0])
      self.lane_width_right = self.calculate_lane_width(modeldata.laneLines[3], modeldata.laneLines[2], modeldata.roadEdges[1])

    # Calculate the desired lane width for nudgeless lane change with lane detection
    if not (self.lane_detection and one_blinker) or below_lane_change_speed or turning:
      lane_available = True
    else:
      # Set the minimum lane threshold to 2.8 meters
      min_lane_threshold = 2.8
      # Set the blinker index based on which signal is on
      blinker_index = 0 if carstate.leftBlinker else 1
      current_lane = modeldata.laneLines[blinker_index + 1]
      desired_lane = modeldata.laneLines[blinker_index if carstate.leftBlinker else blinker_index + 2]
      road_edge = modeldata.roadEdges[blinker_index]
      # Check if the lane width exceeds the threshold
      lane_available = self.calculate_lane_width(desired_lane, current_lane, road_edge) >= min_lane_threshold

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    elif one_blinker and below_lane_change_speed and self.turn_desires:
      self.turn_direction = TurnDirection.turnLeft if carstate.leftBlinker else TurnDirection.turnRight
      # Set the "turn_completed" flag to prevent lane changes after completing a turn
      self.turn_completed = True
    else:
      # TurnDirection.turnLeft / turnRight
      self.turn_direction = TurnDirection.none

      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        # Conduct a nudgeless lane change if all the conditions are true
        self.lane_change_wait_timer += DT_MDL
        if self.nudgeless and lane_available and not self.lane_change_completed and self.lane_change_wait_timer >= self.lane_change_delay:
          torque_applied = True
          self.lane_change_wait_timer = 0

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif torque_applied and not blindspot_detected:
          # Set the "lane_change_completed" flag to prevent any more lane changes if the toggle is on
          self.lane_change_completed = self.one_lane_change
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    # Reset the flags
    self.lane_change_completed &= one_blinker
    self.turn_completed &= one_blinker

    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
    elif not self.turn_completed:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]
    else:
      self.desire = log.LateralPlan.Desire.none

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none

  def update_frogpilot_params(self, params):
    self.blindspot_path = params.get_bool("CustomUI") and params.get_bool("BlindSpotPath")

    self.nudgeless = params.get_bool("NudgelessLaneChange")
    self.lane_change_delay = params.get_int("LaneChangeTime") if self.nudgeless else 0
    self.lane_detection = params.get_bool("LaneDetection") if self.nudgeless else False
    self.one_lane_change = params.get_bool("OneLaneChange") if self.nudgeless else False

    self.turn_desires = params.get_bool("TurnDesires")
