import numpy as np
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lateral_planner import TRAJECTORY_SIZE

# Constants
THRESHOLD = 5     # Time threshold (0.25s)
SPEED_LIMIT = 25  # Speed limit for turn signal check
TURN_ANGLE = 60   # Angle for turning check

# Lookup table for approaching slower leads - Credit goes to Henryccy!
LEAD_DISTANCE = [10., 100.]
LEAD_SPEED_DIFF = [-1., -10.]

# Lookup table for stop sign / stop light detection - Credit goes to the DragonPilot team!
STOP_SIGN_BP = [0., 10., 20., 30., 40., 50., 55.]
STOP_SIGN_DISTANCE = [10, 30., 50., 70., 80., 90., 120.]

class ConditionalExperimentalMode:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.is_metric = self.params.get_bool("IsMetric")

    self.curve_detected = False
    self.experimental_mode = False

    self.curvature_count = 0
    self.lead_status_count = 0
    self.previous_ego_speed = 0
    self.previous_lead_speed = 0
    self.previous_status_bar = 0
    self.previous_yRel = 0
    self.status_value = 0
    self.stop_light_count = 0

    self.update_frogpilot_params()

  def update(self, carstate, modeldata, radarstate, v_ego, v_lead, v_offset):
    # Set the current driving states
    lead = self.detect_lead(radarstate)
    lead_distance = radarstate.leadOne.dRel
    speed_difference = radarstate.leadOne.vRel * 3.6
    standstill = carstate.standstill

    # Set the value of "overridden"
    if self.experimental_mode_via_press:
      overridden = self.params_memory.get_int("CEStatus")
    else:
      overridden = 0

    # Update Experimental Mode based on the current driving conditions
    condition_met = self.check_conditions(carstate, lead, lead_distance, modeldata, speed_difference, standstill, v_ego, v_lead, v_offset)
    if (not self.experimental_mode and condition_met and overridden not in (1, 3)) or overridden in (2, 4):
      self.experimental_mode = True
    elif (self.experimental_mode and not condition_met and overridden not in (2, 4)) or overridden in (1, 3):
      self.experimental_mode = False

    # Set parameter for on-road status bar
    status_bar = overridden if overridden in (1, 2, 3, 4) else (self.status_value if self.status_value >= 5 and self.experimental_mode else 0)
    if status_bar != self.previous_status_bar:
      self.previous_status_bar = status_bar
      self.params_memory.put_int("CEStatus", status_bar)

    self.previous_ego_speed = v_ego

  # Check conditions for the appropriate state of Experimental Mode
  def check_conditions(self, carstate, lead, lead_distance, modeldata, speed_difference, standstill, v_ego, v_lead, v_offset):
    # Prevent Experimental Mode from deactivating at a standstill/slowing down so we don't accidentally run red lights/stop signs
    stopping_for_light = v_ego <= self.previous_ego_speed and self.status_value == 11
    if (standstill or stopping_for_light) and self.experimental_mode:
      return True

    # Navigation check
    if self.navigation and modeldata.navEnabled and self.params_memory.get_bool("NavigationConditionMet"):
      self.status_value = 5
      return True

    # Speed Limit Controller check
    if self.params_memory.get_bool("SLCExperimentalMode"):
      self.status_value = 6
      return True

    # Speed check
    if not standstill and (not lead and v_ego < self.limit or lead and v_ego < self.limit_lead):
      self.status_value = 7 if lead else 8
      return True

    # Slower lead check
    if self.slower_lead and lead and speed_difference < interp(lead_distance, LEAD_DISTANCE, LEAD_SPEED_DIFF):
      self.status_value = 9
      return True

    # Turn signal check
    if self.signal and v_ego < SPEED_LIMIT and (carstate.leftBlinker or carstate.rightBlinker):
      self.status_value = 10
      return True

    # Stop sign and light check
    if self.stop_lights and self.stop_sign_and_light(carstate, lead, lead_distance, modeldata, v_ego, v_lead):
      self.status_value = 11
      return True

    # Road curvature check
    if self.curves and self.road_curvature(modeldata, v_ego) and (self.curves_lead or not lead) and v_offset == 0:
      self.status_value = 12
      return True

    return False

  def detect_lead(self, radarstate):
    # Check to make sure the lead isn't crossing the intersection
    if radarstate.leadOne.status and abs(radarstate.leadOne.yRel - self.previous_yRel) < 0.25:
      self.lead_status_count = max(10, self.lead_status_count + 1)
    else:
      self.lead_status_count = 0 if radarstate.leadOne.status else min(0, self.lead_status_count - 1)
    self.previous_yRel = radarstate.leadOne.yRel
    return self.lead_status_count >= THRESHOLD

  # Determine the road curvature - Credit goes to to Pfeiferj!
  def road_curvature(self, modeldata, v_ego):
    predicted_velocities = np.array(modeldata.velocity.x)
    if predicted_velocities.size:
      curvature_ratios = np.abs(np.array(modeldata.acceleration.y)) / (predicted_velocities ** 2)
      curvature = np.amax(curvature_ratios * (v_ego ** 2))
      # Setting an upper limit of "5.0" helps prevent it activating at stop lights
      if 5.0 > curvature >= 1.6 or (self.curve_detected and 5.0 > curvature >= 1.1):
        self.curvature_count = min(10, self.curvature_count + 1)
      else:
        self.curvature_count = max(0, self.curvature_count - 1)
      return self.curvature_count >= THRESHOLD
    return False

  def stop_sign_and_light(self, carstate, lead, lead_distance, modeldata, v_ego, v_lead):
    following_lead = lead and (self.previous_lead_speed >= v_lead or lead_distance < v_ego or v_lead < 1)
    model_check = len(modeldata.orientation.x) == len(modeldata.position.x) == TRAJECTORY_SIZE
    model_stopping = model_check and modeldata.position.x[-1] < interp(v_ego * 3.6, STOP_SIGN_BP, STOP_SIGN_DISTANCE)
    red_light_detected = self.stop_light_count >= THRESHOLD
    turning = abs(carstate.steeringAngleDeg) >= TURN_ANGLE

    if not turning or red_light_detected:
      if not following_lead and model_stopping:
        self.stop_light_count = min(10, self.stop_light_count + 1)
      else:
        self.stop_light_count = max(0, self.stop_light_count - 1)
    else:
      self.stop_light_count = max(0, self.stop_light_count - 1)

    self.previous_lead_speed = v_lead
    return red_light_detected

  def update_frogpilot_params(self):
    self.curves = self.params.get_bool("CECurves")
    self.curves_lead = self.params.get_bool("CECurvesLead")
    self.experimental_mode_via_press = self.params.get_bool("ExperimentalModeViaPress")
    self.limit = self.params.get_int("CESpeed") * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
    self.limit_lead = self.params.get_int("CESpeedLead") * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
    self.navigation = self.params.get_bool("CENavigation")
    self.signal = self.params.get_bool("CESignal")
    self.slower_lead = self.params.get_bool("CESlowerLead")
    self.stop_lights = self.params.get_bool("CEStopLights")

ConditionalExperimentalMode = ConditionalExperimentalMode()
