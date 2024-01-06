import numpy as np
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc

# Constants
PROBABILITY = 0.6  # 60% chance of condition being true
THRESHOLD = 5      # Time threshold (0.25s)

SPEED_LIMIT = 25   # Speed limit for turn signal check
TURN_ANGLE = 60    # Angle for turning check

# Lookup table for stop sign / stop light detection
SLOW_DOWN_BP = [0., 10., 20., 30., 40., 50., 55.]
SLOW_DOWN_DISTANCE = [10, 30., 50., 70., 80., 90., 120.]


class GenericMovingAverageCalculator:
  def __init__(self):
    self.data = []
    self.total = 0

  def add_data(self, value):
    if len(self.data) == THRESHOLD:
      self.total -= self.data.pop(0)
    self.data.append(value)
    self.total += value

  def get_moving_average(self):
    if len(self.data) == 0:
      return None
    return self.total / len(self.data)

  def reset_data(self):
    self.data = []
    self.total = 0


class ConditionalExperimentalMode:
  def __init__(self):
    self.mpc = LongitudinalMpc()

    self.params = Params()
    self.params_memory = Params("/dev/shm/params")

    self.curve_detected = False
    self.experimental_mode = False
    self.lead_detected = False
    self.lead_detected_previously = False
    self.lead_slowing_down = False
    self.red_light_detected = False
    self.slower_lead_detected = False
    self.slowing_down = False

    self.previous_status_value = 0
    self.previous_v_ego = 0
    self.previous_v_lead = 0
    self.status_value = 0

    self.curvature_gmac = GenericMovingAverageCalculator()
    self.lead_detection_gmac = GenericMovingAverageCalculator()
    self.lead_slowing_down_gmac = GenericMovingAverageCalculator()
    self.slow_down_gmac = GenericMovingAverageCalculator()
    self.slow_lead_gmac = GenericMovingAverageCalculator()
    self.slowing_down_gmac = GenericMovingAverageCalculator()

  def update(self, carState, frogpilotNavigation, modelData, radarState, standstill, v_ego):
    # Set the value of "overridden"
    if self.experimental_mode_via_press:
      overridden = self.params_memory.get_int("CEStatus")
    else:
      overridden = 0

    # Update Experimental Mode based on the current driving conditions
    condition_met = self.check_conditions(carState, frogpilotNavigation, modelData, standstill, v_ego)
    if (not self.experimental_mode and condition_met and overridden not in (1, 3)) or overridden in (2, 4):
      self.experimental_mode = True
    elif (self.experimental_mode and not condition_met and overridden not in (2, 4)) or overridden in (1, 3):
      self.experimental_mode = False
      self.status_value = 0

    # Update the onroad status bar
    self.status_value = overridden if overridden in (1, 2, 3, 4) else self.status_value
    if self.status_value != self.previous_status_value:
      self.previous_status_value = self.status_value
      self.params_memory.put_int("CEStatus", self.status_value)

    self.update_conditions(modelData, radarState, v_ego)

  # Check conditions for the appropriate state of Experimental Mode
  def check_conditions(self, carState, frogpilotNavigation, modelData, standstill, v_ego):
    if standstill:
      return self.experimental_mode

    # Keep Experimental Mode active if slowing down for a red light
    if self.slowing_down and self.status_value == 12 and not self.lead_slowing_down:
      return True

    # Navigation check
    if self.navigation and modelData.navEnabled and frogpilotNavigation.navigationConditionMet:
      self.status_value = 5
      return True

    # Speed Limit Controller check
    if self.params_memory.get_bool("SLCExperimentalMode"):
      self.status_value = 6
      return True

    # Speed check
    if (not self.lead_detected and v_ego < self.limit) or (self.lead_detected and v_ego < self.limit_lead):
      self.status_value = 7 if self.lead_detected else 8
      return True

    # Slower lead check
    if self.slower_lead and self.slower_lead_detected:
      self.status_value = 9
      return True

    # Turn signal check
    if self.signal and v_ego < SPEED_LIMIT and (carState.leftBlinker or carState.rightBlinker):
      self.status_value = 10
      return True

    # Road curvature check
    if self.curves and self.curve_detected and (self.curves_lead or not self.lead_detected):
      self.status_value = 11
      return True

    # Stop sign and light check
    if self.stop_lights and self.red_light_detected and (self.stop_lights_lead or not self.lead_detected):
      self.status_value = 12
      return True

    return False

  def update_conditions(self, modelData, radarState, v_ego):
    self.lead_detection(radarState)
    self.road_curvature(modelData, v_ego)
    self.slow_lead(radarState, v_ego)
    self.stop_sign_and_light(modelData, v_ego)
    self.v_ego_slowing_down(v_ego)
    self.v_lead_slowing_down(radarState)

  def v_ego_slowing_down(self, v_ego):
    self.slowing_down_gmac.add_data(v_ego < self.previous_v_ego)
    self.slowing_down = self.slowing_down_gmac.get_moving_average() >= PROBABILITY
    self.previous_v_ego = v_ego

  def v_lead_slowing_down(self, radarState):
    if self.lead_detected:
      v_lead = radarState.leadOne.vLead
      self.lead_slowing_down_gmac.add_data(v_lead < self.previous_v_lead)
      self.lead_slowing_down = self.lead_slowing_down_gmac.get_moving_average() >= PROBABILITY
      self.previous_v_lead = v_lead
    else:
      self.lead_slowing_down_gmac.reset_data()
      self.lead_slowing_down = False
      self.previous_v_lead = 0

  # Lead detection
  def lead_detection(self, radarState):
    self.lead_detection_gmac.add_data(radarState.leadOne.status)
    self.lead_detected = self.lead_detection_gmac.get_moving_average() >= PROBABILITY

  # Determine the road curvature - Credit goes to to Pfeiferj!
  def road_curvature(self, modelData, v_ego):
    predicted_velocities = np.array(modelData.velocity.x)
    if predicted_velocities.size:
      curvature_ratios = np.abs(np.array(modelData.acceleration.y)) / (predicted_velocities**2)
      curvature = np.amax(curvature_ratios * (v_ego**2))
      # Setting an upper limit of "5.0" helps prevent it activating at stop lights
      if curvature > 5.0:
        self.curvature_gmac.reset_data()
        self.curve_detected = False
      else:
        self.curvature_gmac.add_data(curvature > 1.6 or self.curve_detected and curvature > 1.1)
        self.curve_detected = self.curvature_gmac.get_moving_average() >= PROBABILITY
    else:
      self.curvature_gmac.reset_data()

  # Slower lead detection - Credit goes to the DragonPilot team!
  def slow_lead(self, radarState, v_ego):
    if not self.lead_detected and self.lead_detected_previously:
      self.slow_lead_gmac.reset_data()
      self.slower_lead_detected = False

    if self.lead_detected and v_ego >= 0.01:
      self.slow_lead_gmac.add_data(radarState.leadOne.dRel < (v_ego - 1) * self.mpc.t_follow)
      self.slower_lead_detected = self.slow_lead_gmac.get_moving_average() >= PROBABILITY

    self.lead_detected_previously = self.lead_detected

  # Stop sign/stop light detection - Credit goes to the DragonPilot team!
  def stop_sign_and_light(self, modelData, v_ego):
    # Check if the model data is consistent and wants to stop
    model_check = len(modelData.orientation.x) == len(modelData.position.x) == TRAJECTORY_SIZE
    model_stopping = modelData.position.x[TRAJECTORY_SIZE - 1] < interp(v_ego * CV.MS_TO_KPH, SLOW_DOWN_BP, SLOW_DOWN_DISTANCE)
    self.slow_down_gmac.add_data(model_check and model_stopping and not self.curve_detected and not (self.slower_lead_detected or self.lead_slowing_down))

    self.red_light_detected = self.slow_down_gmac.get_moving_average() >= PROBABILITY

  def update_frogpilot_params(self, is_metric):
    self.curves = self.params.get_bool("CECurves")
    self.curves_lead = self.params.get_bool("CECurvesLead")
    self.experimental_mode_via_press = self.params.get_bool("ExperimentalModeViaPress")
    self.limit = self.params.get_int("CESpeed") * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    self.limit_lead = self.params.get_int("CESpeedLead") * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    self.navigation = self.params.get_bool("CENavigation")
    self.signal = self.params.get_bool("CESignal")
    self.slower_lead = self.params.get_bool("CESlowerLead")
    self.stop_lights = self.params.get_bool("CEStopLights")
    self.stop_lights_lead = self.params.get_bool("CEStopLightsLead")

ConditionalExperimentalMode = ConditionalExperimentalMode()
