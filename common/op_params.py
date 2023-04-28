#!/usr/bin/env python3
import os
import json, csv
from atomicwrites import atomic_write
from common.colors import COLORS
from common.numpy_fast import clip
from common.params import Params
from common.travis_checker import BASEDIR
from datetime import datetime
from enum import Enum
import re
from selfdrive.hardware import TICI
from selfdrive.swaglog import cloudlog
import threading
try:
  from common.realtime import sec_since_boot
except ImportError:
  import time
  sec_since_boot = time.time

warning = lambda msg: print('{}opParams WARNING: {}{}'.format(COLORS.WARNING, msg, COLORS.ENDC))
error = lambda msg: print('{}opParams ERROR: {}{}'.format(COLORS.FAIL, msg, COLORS.ENDC))

NUMBER = [float, int]  # value types
NONE_OR_NUMBER = [type(None), float, int]

BASEDIR = os.path.dirname(BASEDIR)
PARAMS_DIR = os.path.join(BASEDIR, 'community', 'params')
IMPORTED_PATH = os.path.join(PARAMS_DIR, '.imported')
OLD_PARAMS_FILE = os.path.join(BASEDIR, 'op_params.json')

HISTORY_FILE = os.path.join(BASEDIR, 'community', 'op_params_history.csv')
HISTORY_DATETIME_FORMAT = "%Y/%m/%d %H:%M:%S"
HISTORY_COLUMN_HEADINGS = [
  "time of change",
  "name",
  "value",
  "old value",
  "reason"
]

# write history. cancel write by setting reason to False
def write_history(data):
  if isinstance(data, dict) and "reason" in data and data["reason"] != False:
    if not os.path.exists(HISTORY_FILE):
      with open(HISTORY_FILE, 'w') as f:
        w = csv.DictWriter(f, fieldnames=HISTORY_COLUMN_HEADINGS, delimiter=';')
        w.writeheader()
        w.writerow(data)
    else:
      with open(HISTORY_FILE, 'a') as f:
        w = csv.DictWriter(f, fieldnames=HISTORY_COLUMN_HEADINGS, delimiter=';')
        w.writerow(data)

def read_history(param_name=None):
  out = []
  if os.path.exists(HISTORY_FILE):
    with open(HISTORY_FILE, 'r') as f:
      r = csv.DictReader(f, delimiter=';')
      for row in r:
        out.append({k:row[k] for k in HISTORY_COLUMN_HEADINGS})
    if param_name is not None:
      out = [p for p in out if p["name"] == param_name]
  return out

UI_DOT_H_PATH = '/data/openpilot/selfdrive/ui/ui.h'
def parse_UIMeasure_enum(ui_dot_h_path):
  try:
    with open(ui_dot_h_path, 'r') as f:
      data = f.read()
  except:
    return []
  
  if 'typedef enum UIMeasure {' not in data:
    return []
  # Find the UIMeasure enum definition
  enum_defs = data.split('typedef enum UIMeasure {')[-1]
  enum_defs = enum_defs.split('} UIMeasure;')[0]

  # Get a list of enum names from the definition   
  enum_names = [l.strip().split(',')[0].split(' = ')[0] for l in enum_defs.split('\n') if len(l.strip()) > 2 and '//' not in l.strip().split(',')[0].split(' = ')[0]]
  
  return enum_names[:-1] if len(enum_names) > 1 else enum_names

UI_METRICS = parse_UIMeasure_enum(UI_DOT_H_PATH)

def _read_param(key):  # Returns None, False if a json error occurs
  try:
    with open(os.path.join(PARAMS_DIR, key), 'r') as f:
      value = json.loads(f.read())
    return value, True
  except json.decoder.JSONDecodeError:
    return None, False


def _write_param(key, value, reason=None, old_value=None, do_log=True):
  if do_log:
    log = {"time of change": datetime.now().strftime(HISTORY_DATETIME_FORMAT),
          "name": key,
          "value": value,
          "old value": old_value,
          "reason": reason}
    write_history(log)
  param_path = os.path.join(PARAMS_DIR, key)
  with atomic_write(param_path, overwrite=True) as f:
    f.write(json.dumps(value))

class Param:
  def __init__(self, 
               default, 
               allowed_types=[], 
               description=None, 
               *, 
               static=False, 
               live=False, 
               hidden=False,
               allowed_vals=[],
               min_val=None,
               max_val=None,
               is_common=False,
               param_param='',
               param_param_use_ord=False,
               unit='',
               linked_op_param_check_param='',
               param_param_read_on_startup=False,
               show_op_param='',
               show_op_param_check_val=None,
               fake_live=False):  # pylint: disable=dangerous-default-value
    self.default_value = default  # value first saved and returned if actual value isn't a valid type
    if not isinstance(allowed_types, list):
      allowed_types = [allowed_types]
    self.allowed_types = allowed_types  # allowed python value types for opEdit
    self.description = description  # description to be shown in opEdit
    self.hidden = hidden  # hide this param to user in opEdit
    self.live = live  # show under the live menu in opEdit
    self.static = static  # use cached value, never reads to update
    self.fake_live = fake_live # param is live but only read on OP startup
    self.allowed_vals = allowed_vals # optional to specify a set of discrete allowed values (of any type)
    self.min_val = min_val # specify minimum value
    self.max_val = max_val # specify maximum value
    self.is_common = is_common
    self.unit = unit
    self.show_op_param = show_op_param # if set, this param's value is compared to see if param should be shown or not
    self.show_op_param_check_val = show_op_param_check_val # the value against which it is compared
    self.linked_op_param_check_param = linked_op_param_check_param # this specified bool param can be changed by the user to control whether the list items are synced
    self.param_param = param_param # op_params can also write to regular "params" when put(), and op_params always overwrite regular params
    self.param_param_use_ord = param_param_use_ord # store the index of the value in allowed values when writing to corresponding param
    self.param_param_read_on_startup = param_param_read_on_startup # if true, get the param_param when initializing to override own value
    self._get_thread = None # non-static params are fetched regularly in a separate thread
    self._create_attrs()

  def type_is_valid(self, value):
    if not self.has_allowed_types:  # always valid if no allowed types, otherwise checks to make sure
      return True
    return all([not self.is_list,
                type(value) in self.allowed_types]) \
      or all([self.is_list,
              isinstance(value, list) and all([type(i) in self.allowed_types for i in value])])
  
  def value_is_valid(self, value):
    if not self.has_allowed_vals:  # always valid if no allowed types, otherwise checks to make sure
      return True
    return value in self.allowed_vals
  
  
  def clip_val(self, v):
    if self.max_val is not None and self.min_val is not None:
      return clip(v, self.min_val, self.max_val)
    elif self.max_val is not None:
      return min(self.max_val, v)
    elif self.min_val is not None:
      return max(self.min_val, v)
    return v
    
  def value_clipped(self, value):
    
    if isinstance(value, list):
      val = [self.clip_val(v) for v in value]
    else:
      val = self.clip_val(value)
    
    if val != value:
      value = val
      return value, True
    else:
      return value, False

  def _create_attrs(self):  # Create attributes and check Param is valid
    self.has_allowed_types = isinstance(self.allowed_types, list) and len(self.allowed_types) > 0
    self.has_allowed_vals = isinstance(self.allowed_vals, list) and len(self.allowed_vals) > 0
    self.has_description = self.description is not None
    self.is_list = list in self.allowed_types
    self.read_frequency = None if self.static else (1 if self.live else 10)  # how often to read param file (sec)
    if self.has_allowed_types:
      if not type(self.default_value) in self.allowed_types and not isinstance(self.default_value, list):
        try:
          self.default_value = self.allowed_types[0](self.default_value)
        except ValueError:
          assert type(self.default_value) in self.allowed_types, 'Default value type must be in specified allowed_types!'
    if self.has_allowed_vals:
      assert self.default_value in self.allowed_vals, 'Default value must be in specified allowed_vals!'
    self.default_value = self.value_clipped(self.default_value)[0]
    self.updated = False
    self.last_read = 0
    self._params = None
    self.value = None
    if self.param_param != '':
      self._params = Params()
      try:
        val = None
        self._params.check_key(self.param_param)
        if self.param_param_read_on_startup:
          if self.param_param_use_ord and self.has_allowed_vals:
            val = int(self._params.get(self.param_param, encoding="utf8"))
            if val < len(self.allowed_vals):
              self.value = self.allowed_vals[val]
            else:
              self.value = self._params.get(self.param_param)
          else:
            self.value = self._params.get(self.param_param)
          if self.has_allowed_types and type(self.value) not in self.allowed_types:
            for t in self.allowed_types:
              try:
                self.value = t(self.value)
                break
              except:
                continue
            if type(self.value) not in self.allowed_types:
              raise ValueError 
          # cloudlog.info(f"opParams: Read in param value '{self.value}' from param '{self.param_param}' on startup")
        else:
          self.value = self.default_value
      except Exception as e:
        cloudlog.warning(f"Corresponding param '{self.param_param}' for this op_param is invalid! Read {val = }, {self.value = }. Exception: {e}, {self.allowed_types = }")
        self.param_param = ''
        self._params = None
        self.value = self.default_value
    else:
      self.value = self.default_value
    
    if self.is_list:
      self.allowed_types.remove(list)

  def _get_val(self, key, force_update=False, time_cur=-1.0):
    do_update = not self.static and time_cur >= 0.0 and time_cur - self.last_read >= self.read_frequency
    if do_update:
      self.last_read = time_cur
    if (self.static and self.updated) \
        or (self.updated and not do_update) \
        or (self._get_thread is not None and self._get_thread.is_alive()):
      return self.value
    
    def _update_val():
      self.updated = True
      value, success = _read_param(key)
      if not success:
        err_str = "read failure"
      if success and not (success := self.type_is_valid(value)):
        err_str = f"invalid type of {value = }"
      if success and not (success := self.value_is_valid(value)):
        err_str = f"invalid {value = }"
      if not success:
        value = self.default_value
        _write_param(key, value, reason="failed to update value, so writing default value")
        cloudlog.warning(f"{err_str} for '{key}'. Writing default value = {value}")
      self.value = value
      value, was_clipped = self.value_clipped(value)
      if was_clipped:
        cloudlog.warning(f"clipping {self.value} to {value} for '{key}'")
        self.value = value
    
    self._get_thread = threading.Thread(target=_update_val)
    self._get_thread.start()
    
    if force_update:
      self._get_thread.join()

    return self.value


def _import_params():
  if os.path.exists(OLD_PARAMS_FILE) and not os.path.exists(IMPORTED_PATH):  # if opParams needs to import from old params file
    try:
      with open(OLD_PARAMS_FILE, 'r') as f:
        old_params = json.loads(f.read())
      for key in old_params:
        _write_param(key, old_params[key], reason="importing params")
      open(IMPORTED_PATH, 'w').close()
    except:  # pylint: disable=bare-except
      pass


class opParams:
  def __init__(self, calling_function='', check_for_reset=False):
    """
      To add your own parameter to opParams in your fork, simply add a new entry in self.fork_params, instancing a new Param class with at minimum a default value.
      The allowed_types and description args are not required but highly recommended to help users edit their parameters with opEdit safely.
        - The description value will be shown to users when they use opEdit to change the value of the parameter.
        - The allowed_types arg is used to restrict what kinds of values can be entered with opEdit so that users can't crash openpilot with unintended behavior.
          (setting a param intended to be a number with a boolean, or viceversa for example)
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the arg with `isinstance()`.
        - If you want your param to update within a second, specify live=True. If your param is designed to be read once, specify static=True.
          Specifying neither will have the param update every 10 seconds if constantly .get()
          If the param is not static, call the .get() function on it in the update function of the file you're reading from to use live updating

      Here's an example of a good fork_param entry:
      self.fork_params = {'camera_offset': Param(0.06, allowed_types=NUMBER), live=True}  # NUMBER allows both floats and ints
    """

    kf_desc = 'Feedforward is the part of the steering controller that only cares about the desire steering angle (how sharp the curve is). So feedforward only comes into play in curves when the desired steering angle is non-zero, and the greater the angle, the greater the feedforward response, which is scaled by kf.\nTo tune kf, you observe if OpenPilot enters curves too early/late and rides curves too far inside/outside. If it enters too early (late) and/or rides too far inside (outside), then kf is too high (low) and should be lowered (raised) in 10% increments until it enters correctly and rides center.'
    kp_desc = 'Proportional gain responds proportionally to the instantaneous error being controlled. The greater the error, the greater the corrective response, linearly, and scaled according to kp. In this case, where we\'re controlling the steering angle, the proportional gain alone cannot completely correct for error, becuase when the error is close to zero, so is the proportional response.\nThe best way to tune kp is then using nudgeless lane change on straight roads (no feedforward response), which creates a sudden (so doesn\'t trigger the integral response) change in course that results in a reproducible error source that triggers the proportional and derivative responses. Set kd to zero to best assess kp. If the lane change feels too assertive or jerky, lower kp. If too weak, increase kp.'
    ki_desc = 'Integral gain responds based on the accumulated error, so if you\'re missing the target continually, the integral response builds the longer you\'re off in the same direction. This corrects for things like persistent crosswinds, inconsistent tire pressures, or dramatic road roll that roll compensation fails to fully compensate for. The drawback is that integral gain can "wind up", overshooting the desired angle, causing lateral oscillations, ping-ponging back and forth about lane center.\nTune kf and kp with ki set to zero, then set ki to 1/3rd the value of kp or less, and kd to 10-20x the value of kp (see the default values here). If lateral oscillations occur, lower ki in 10% increments until they are no longer observed.'
    kd_desc = 'Derivative gain responds to the rate of change of error. The benefits are two-fold. First, note that the proportional and integral responses always push against the error until the error is zero (and due to integral wind-up, integral can push past zero even), which necessarily results in overshoot and oscillations. In such an overshooting case, when you\'re returning to lane center and the error (let\'s say positive) is decreasing, the error rate wil be negative even though the error is still positive, so the derivative response is pushing against the proportional and integral overshoot. Second, if you\'re quickly leaving lane center then the rate of change of error is positive along with the error, so the derivative here helps along with the proportional and integral responses to correct for the error. Too high of kd is indicated by a jerky initial correction when using nudgeles lane change on straight roads.'
    
    self.fork_params = {      
      'LP_camera_offset_m': Param(-0.04 if TICI else 0.06, float, 'Adjust your default lane position. Increase it to move your position to the left, or decrease it to move to the right. This only shifts the perceived position of lanelines for laneline-based planning. You need to also change the LP_path_offset_m below in order to adjust lateral position when using laneless. (Your camera offset to use in lane_planner.py)', live=True, min_val=-0.5, max_val=0.5, is_common=True, unit='meters'),
      
      'LP_path_offset_m': Param(-0.04 if TICI else 0.0, float, 'Adjust your default path position. Increase it to move your position to the left, or decrease it to move to the right. This is like camera offset, but applies to lateral position when no lanelines are present or they are not being used.', live=True, min_val=-0.5, max_val=0.5, is_common=True, unit='meters'),
      
      'MISC_set_speed_offset_mph': Param(3, int, 'When adjusting 5mph at a time, this offset will be added to the result, so if set to 3mph, then your set speed will always be 3mph over the closest 5, so 10+3, 15+3, 20+3 = 13, 18, 23mph instead of 10, 15, 20', min_val=1, max_val=4, is_common=True, unit='mph'),
      
      'MISC_coasting_speed_over': Param([0.2, 0.1], [list, float], 'When over-speed coasting is enabled along with the 15% cap, use this to adjust the actual cap used at low and high speeds (20 and 80mph). (0.2 corresponds to 20% over the set speed)', min_val=0.01, max_val=0.5),
      
      'MISC_car_12v_pause_charging_v': Param(11.0, float, 'Lower voltage threshold for your car\'s 12v system, below which the device will shutdown (well, stop charging, but there\'s no battery in the C2 or C3, so it shuts down)', min_val=0, max_val=10000, unit='volts'),
      
      'MISC_offroad_shutdown_time_hr': Param(5, int, 'The amount of time after turning off your car before your device shuts down to conserve power (and not drain your car battery)', min_val=0, unit='hours'),
      
      'MISC_car_12v_pause_charging_v': Param(11.0, float, 'Lower voltage threshold for your car\'s 12v system, below which the device will shutdown (well, stop charging, but there\'s no battery in the C2 or C3, so it shuts down)', min_val=6, max_val=20, unit='volts'),
      
      'MISC_weather_simple_update_freq_s': Param(4, int, 'If weather is enabled, you can tap it to toggle simple/full display. If there\'s wind and rain/snow, simple display will alternate between showing wind and precipitation information. Set the amount of time between alternations here.', min_val=1, unit='seconds', param_param='WeatherAlternateFrequency'),
      
      'MISC_open_weather_map_api_key': Param(None, [str, None], 'You can provide your own api key for fetching weather information after signing up at https://home.openweathermap.org/users/sign_up. This is optional.', static=True),
      
      'MISC_cluster_speed_smoothing_factor': Param(0.4, float, 'Adjust the smoothing used to determine the "current speed" shown on the Comma device', min_val=0.0, max_val=100.0),
      
      'MISC_cluster_speed_deadzone': Param(0.01, float, 'Adjust the deadzone used to prevent the current speed from quickly alternating back and forth. Higher value means a more delayed response with less alternating. Units of this parameter depend on whether you have "metric" units enabled in OpenPilot.', min_val=0.0, max_val=0.45, unit='mph or km/h'),
      
      'MISC_speed_limit_offset': Param([0.2, 0.12], [list, float], '"Low" and "high" speed speed limit offsets over the current speed limit if seed limit offset is enabled in OpenPilot settings. The speed limit offset changes from low-speed amount (eg. 0.2 = 20% over) at 20mph to the high_speed offset amount by 80mph', live=True, min_val=0.0, max_val=0.5),
      
      'MISC_parked_timer_min_time_s': Param(80, int, 'Controls how long you have to be sitting in park before the parked timer will begin to show. Set to a very high value to basically disable the timer.', min_val=0, unit='seconds'),
      
      #####
      
      'AP_accel_limit_rates_ms3': Param([-1.0, 0.5], [list, float], 'Rate at which acceleration profile is allowed to change. FIRST VALUE MUST BE NEGATIVE AND SECOND VALUE POSITIVE. IF FIRST VALUE IS TOO CLOSE TO ZERO, OPENPILOT MAY NOT BRAKE SOON ENOUGH WHEN APPROACHING A LEAD IN ECO ACCELERATION MODE.', live=True, min_val=-10.0, max_val=10.0, unit="m/s³"),
      
      'AP_accel_limit_rates_speed_cutoff_mph': Param(4, float, 'Only apply accel limit rate limits below this speed.', live=True, min_val=0.0, max_val=20.0, unit="mph"),
      
      'AP_eco_accel_factor': Param(1.2, float, 'Scale eco acceleration.', live=True, min_val=0.01, max_val=20.0),
      
      'AP_stock_accel_factor': Param(1.0, float, 'Scale stock acceleration. This is also the acceleration used if the acceleration profiles button is disabled', live=True, min_val=0.01, max_val=20.0, is_common=True),
      
      'AP_sport_accel_factor': Param(1.0, float, 'Scale sport acceleration.', live=True, min_val=0.01, max_val=20.0),
      
      'AP_following_accel_factor': Param(1.0, float, 'Scale acceleration when behind a pulling away lead.', live=True, min_val=0.01, max_val=20.0),
      
      'AP_positive_accel_post_resume_smoothing_factor': Param(1.0, float, 'Adjust the smoothing applied to positive acceleration as OpenPilot resumes long control after you let off the gas pedal or otherwise resume long control. Higher value is more smoothing.', live=True, min_val=0.0, max_val=20.0),
      
      'AP_positive_accel_post_resume_smoothing_time_s': Param(3.0, float, 'How long should positive acceleration smoothing be applied after resuming following a resume event?', live=True, min_val=0.0, max_val=20.0, unit='seconds'),
      
      'AP_positive_accel_post_resume_smoothing_max_speed_mph': Param(30.0, float, 'Post-resume smoothing will only be used at or below this speed.', live=True, min_val=0.0, max_val=90.0, unit="mph"),
      
      'AP_positive_accel_post_lead_smoothing_factor': Param(1.5, float, 'Adjust the smoothing applied to positive acceleration immediately after a lead "disappears", say, after they slow for a hard right turn, so that OpenPilot doesn\'t abruptly let off the brakes and accelerate uncomfortably. Higher value is more smoothing.', live=True, min_val=0.0, max_val=20.0),
      
      'AP_positive_accel_post_lead_smoothing_time_s': Param(4.0, float, 'How long should positive acceleration smoothing be applied after a lead having "disappeared"?', live=True, min_val=0.0, max_val=20.0, unit='seconds'),
      
      'AP_positive_accel_smoothing_factor': Param(0.0, float, 'Adjust the smoothing applied to positive acceleration at all other times. Higher value is more smoothing.', live=True, min_val=0.0, max_val=20.0),
      
      'AP_positive_accel_smoothing_min_speed_mph': Param(2.0, float, 'Below this speed, no smoothing is applied to positive acceleration under any circumstances.', live=True, min_val=0.0, max_val=90.0, unit='mph'),
      
      'AP_post_resume_fast_accel_s': Param(6.0, float, 'This controls the amount of time higher acceleration is used after resuming from standstill.', min_val=0.0, max_val=20.0, unit='mph'),
      
      #####
      
      'FP_close_gas_factor': Param(1.0, float, 'Controls how proactively OpenPilot will adjust in response to a change in the lead car velocity when using the close follow profile. Increase to make close follow more responsive.', live=True, min_val=0.1, max_val=2.0),
      
      'FP_close_distance_offset_s': Param(0.0, float, 'Add or subtract follow distance from the close follow profile', live=True, min_val=-0.5, max_val=3.0, unit='seconds follow distance'),
      
      'FP_medium_distance_offset_s': Param(0.0, float, 'Add or subtract follow distance from the medium follow profile', live=True, min_val=-1.0, max_val=2.5, is_common=True, unit='seconds follow distance'),
      
      'FP_far_distance_offset_s': Param(0.0, float, 'Add or subtract follow distance from the far follow profile', live=True, min_val=-1.5, max_val=2.0, unit='seconds follow distance'),
      
      # dynamic follow
      
      'FP_DF_distance_gain_factor': Param(1.0, float, 'Scale the rate at which the follow profile increases to a higher, farther following profile. A larger number means it will return to medium/far follow more quickly after a cut-in.', min_val=0.01, max_val=20.0),
      
      'FP_DF_distance_penalty_factor': Param(1.0, float, 'Scale the follow profile penalty incurred based on cut-in forward distance. A higher value will result in more defensive driving after a cut-in, and for longer.', min_val=0.01, max_val=20.0),
      
      'FP_DF_velocity_penalty_factor': Param(1.0, float, 'Scale the follow profile penalty incurred based on relative velocity of cut-in. A higher value will result in more defensive driving after a cut-in, and for longer.', min_val=0.01, max_val=20.0),
      
      'FP_DF_traffic_penalty_factor': Param(1.0, float, 'Scale the amount of follow profile penalty per second incurred based on the follow distances of adjacent traffic. A higher value will cause defensive driving more rapidly in the presence of close-following adjacent traffic.', min_val=0.01, max_val=20.0),
      
      #####
      
      'XR_v_lat_derivative_period_s': Param(1.0, float, 'Lead relative lateral velocity is calculated as a numerical derivative (change in lateral position with respect to time). This parameter controls the time used to calculate the derivative.', min_val=0.1, max_val=10.0, unit="seconds"),
      
      'XR_v_lat_vLead_factor_bp_mph': Param([15.0, 25.0], [list, float], 'Lead relative lateral velocity only really needs to be calculated to better deal with leads turning out of your lane, which is only typically an issue at low speeds. Meanwhile, the lead lateral velocity data can be noisy compared to radar range data, so we don\'t want to use it when we don\'t need to. Therefore, we decrease the contribution of lateral velocity to total lead velocity at "high" lead speeds and use it at "low" lead speeds. Here you define the "low" and "high" speeds. Below the "low" speed, 100% of lateral velocity is used, and it decreases to 0% by the time the "high" speed is reached.', min_val=0.0, max_val=90.0, unit="mph"),
      
      'XR_LRL_max_detection_distance_m': Param(150, int, 'Max distance [in meters] at which radar + vision data will be used to consider tracks (a.k.a. points) as a potential lead. WARNING: RADAR DETECTION DISTANCE IS ~150m, AND ABOVE THAT VISION DATA FROM THE VOLT LKA CAMERA WILL BE USED, WHICH CAN BE VERY NOISY, LEADING TO PHANTOM BRAKING. USE THE SMOOTHING PARAMETER TO TRY AND SMOOTH IT OR JUST DON\'T GO ABOVE 150m!', min_val=50, max_val=250, unit="meters"),
      
      'XR_LRL_smoothing_factor': Param(1.0, float, 'This adjusts the smoothing used for vision-only long-range leads, in order to reduce unnecessary braking from noise lead velocity data.', min_val=0.0, max_val=50.0),
      
      'XR_LRL_min_smoothing_distance_m': Param(145, int, 'Vision-only long-range lead distance/velocity data is noisier at higher distances. At this lead distance, no smoothing is applied. At the max detection distance, the full amount of smoothing is applied. Change this to control the distance at which smoothing begins to ramp up.', min_val=50, max_val=250, unit="meters"),
      
      'XR_LRL_min_detection_distance_m': Param(60, int, 'Below this distance, radar tracks aren\'t considered as leads. The OpenPilot model is very good at lead detection up to around 120m, but it becomes more inconsistent as you approach that value. Setting a value below 120m allows for a smooth transition, as you approach a long range lead on the highway, from long-rage lead to regular OpenPilot model lead, avoiding interruptions in planned acceleration. Don\'t use too low of a value, however, because at closer distances the OpenPilot model is very good at lead detection, and it\'s best to rely entirely on it and not risk false positives from long-range lead detection. If you experience false positives, for example detecting the lead in the adjacent lane while in a curve, and the lead marker has a blue dot (long-range lead) then you can increase this value to prevent those.', min_val=20, max_val=120, unit="meters"),
      
      'XR_LRL_max_relative_y_distance_m': Param(7.0, float, 'Max distance [in meters] in the lateral (y) direction at which long-range radar + vision tracks will be considered as leads.  Long range leads are qualified by comparing them to the OpenPilot model lanelines and planned path, which becomes less accurate at far distances and more so in curves, so you can reduce false positives (detecting the wrong lead) in curves by decreasing this value.', min_val=3.0, max_val=50.0, unit="meters"),
      
      'XR_LRL_path_y_distance_bp_m': Param([0.0, 200.0], [list, float], 'The "low-distance" and "high-distance" values used to determine path lateral (y) cutoff for long-range lead detection. At these distances, the respective values of LRL_path_y_distance_v_m will be used', min_val=0.0, max_val=220.0, unit="meters"),
      
      'XR_LRL_path_y_distance_v_m': Param([1.2, 1.2], [list, float], 'The "low-distance" and "high-distance" values of path lateral (y) cutoff for long-range lead detection. If a long-range lead is within than this many meters of lane center or the planned OpenPilot path, then it is considered to be in your path. Note that lanes are never more than 4m wide, so you will have false positives in adjacent lanes if too high of a value is used.', min_val=0.0, max_val=2.5, unit="meters"),
      
      'XR_LRL_path_y_distance_low_tol_filter_m': Param(0.7, float, 'Long-range lead detection prioritizes leads based on lateral distance to the planned path or lane center. In the event there are two lead candidates, if the closer of the two is within this distance of path/lane-center, it will take priority over the farther lead, even if the farther lead is technically closer to the path. This guarantees OpenPilot is following based on the closest lead that is on the path.', min_val=0.0, max_val=1.5, unit="meters"),
      
      #####
      
      'LC_minimum_speed_mph': Param(20.0, float, 'No OpenPilot assisted lane change below this speed', min_val=8.0, max_val=90.0, is_common=True, unit='mph'),
      
      'LC_nudgeless_minimum_speed_mph': Param(38.0, float, 'No nudgeless lane change below this speed', min_val=8.0, max_val=90.0, is_common=True, unit='mph'),
      
      'LC_nudgeless_delay_s': Param(1.5, float, 'Auto "nudgeless" lane change after this delay', min_val=0.0, max_val=10.0, is_common=True, unit='seconds'),
      
      #####
      
      'MADS_steer_pause_speed_mph': Param(20.0, float, 'When blinker is on below this speed and you\'re decelerating, autosteering will pause', min_val=0.0, max_val=90.0, unit='mph'),
      
      'MADS_steer_allow_nudgeless_lane_change': Param(False, bool, 'If true, nudgeless lane changes will apply when in MADS mode'),
      
      'MADS_OP_decel_ms2': Param([-1.0, -1.1], [list, float], 'The amount of desired one-pedal deceleration at "low" and "high" speeds when the regen paddle is not pressed.', min_val=-2.5, max_val=0.0, unit='m/s²'),
      
      'MADS_OP_regen_paddle_decel_factor': Param(1.3, float, 'When the regen paddle is pressed, the deceleration values defined above are scaled according to this value.', min_val=1.0, max_val=3.0),
      
      'MADS_OP_one_time_stop_decel_factor': Param(0.8, float, 'One pedal driving "one time stop" is when one pedal driving mode is temporarily enabled, to bring you to a stop one time, activated by holding the regen paddle as you come to a stop, and indicated by a "warning"-color behind the MADS/one-pedal icon. The one-pedal mode deceleration values defined above are scaled according to this value.', min_val=0.1, max_val=3.0),
      
      'MADS_OP_rate_ramp_up': Param(0.8, float, 'The rate at which one-pedal brake force increases (applying)', min_val=0.1, max_val=3.0, unit='m/s³'),
      
      'MADS_OP_rate_ramp_down': Param(0.8, float, 'The rate at which one-pedal brake force decreases (releasing brakes)', min_val=0.1, max_val=3.0, unit='m/s³'),
      
      'MADS_OP_rate_low_speed_factor': Param(0.5, float, 'At low speed, the one-pedal ramp-up rate is scaled by this value. If you want one-pedal to apply the brakes sooner/later after releasing the gas/brakes at low speed, increase/decrease this value.', min_val=0.01, max_val=1.0),
      
      'MADS_OP_rate_low_speed_factor_bp': Param([2.0, 10.0], [list, float], 'Here you determine what constitutes "low" and "high" speed in regards to the MADS_OP_rate_low_speed_factor parameter.', min_val=0.0, max_val=100.0, unit="mph"),
      
      'MADS_OP_rate_high_steer_factor': Param(0.2, float, 'The one-pedal ramp-up rate is scaled down at high steer angles to prevent it from applying brakes too quickly for example when turning sharply in a parking lot. If you want one-pedal to apply the brakes more/less quickly while the steering wheel is at "high" angles, then increase/decrease this value.', min_val=0.01, max_val=1.0),
      
      'MADS_OP_rate_high_steer_factor_bp': Param([20.0, 120.0], [list, float], 'Here you determine what constitutes "low" and "high" steering angles in regards to the MADS_OP_rate_high_steer_factor parameter.', min_val=0.0, max_val=720, unit="degrees"),
      
      'MADS_OP_low_speed_pitch_factor_incline': Param(0.2, float, 'At higher speeds, one-pedal driving corrects for road pitch so that, for example, if you\'re already slowing due to an incline it doesn\'t need to apply additional brakes. At low speed this can be unwanted, so control it here when on incline. A value of 0.0 cancels out all pitch correction', min_val=0.0, max_val=1.0),
      
      'MADS_OP_low_speed_pitch_factor_decline': Param(0.4, float, 'At higher speeds, one-pedal driving corrects for road pitch so that, for example, if you\'re already slowing due to an incline it doesn\'t need to apply additional brakes. At low speed this can be unwanted, so control it here when on decline. A value of 0.0 cancels out all pitch correction', min_val=0.0, max_val=1.0),
      
      'MADS_OP_speed_error_factor': Param([0.2, 0.4], [list, float], 'One-pedal driving uses the same longitudinal PID controller tune as your car, which is too high for the smooth response we want from one-pedal, so we scale down the error (in terms of acceleration) for the one-pedal controller to dampen its response. Set to 0.0 for a "dumb" one-pedal that applies "constant" braking regardless of whether you\'re slowing or not, or increase for more responsive correction and consistent deceleration. This parameter sets the "low" and "high" speed error factor.', min_val=0.0, max_val=2.0),
      
      'MADS_OP_one_time_stop_threshold_mph': Param(5.0, float, 'Adjust the speed threshold used for one-pedal one-time stop. If you are holding the regen paddle while your speed goes from above to below this speed, one-pedal one-time stop will activate. Note that you can continue to hold the regen paddle and one-pedal brakes will still blend in, bringing you to a stop more quickly.', min_val=0.5, max_val=100.0, unit='mph'),
      
      'MADS_OP_one_time_stop_hold_s': Param(0.5, float, 'The one-time stop threshold speed above is explicit, in that you must cross that threshold speed with the regen paddle pressed, otherwise one-pedal one-time stop will not activate. However, one-pedal one-time stop will still activate if you hold the regen paddle for long enough while already below the threshold speed. This parameter lets you adjust the amount of time regen paddle needs to be held while below the threshold speed for one-pedal one-time stop to activate, at which point you can release the regen paddle and come to a smooth stop, or continue holding the regen paddle to stop sooner, as one-pedal brakes are still blended in.', min_val=0.1, max_val=15.0, unit='seconds'),
      
      'MADS_OP_double_press_time_s': Param(0.7, float, 'Adjust how quickly you must double-press the regen paddle to toggle one-pedal mode', min_val=0.1, max_val=2.0, unit='seconds'),
      
      #####
      
      'CB_VTSC_smoothing_factor': Param(0.3, float, 'The vision turn controller output acceleration is smoothed. Increase/decrease for more/less smoothing.', live=True, min_val=0.01, max_val=3.0),
      
      'CB_VTSC_lat_accel_factor': Param(1.0, float, 'The vision turn controller uses the car\'s lateral acceleration in order to lookup corresponding desired values of output longitudinal acceleration. Use this to scale the lateral acceleration values used in the lookup. A value less/greater than 1.0 will make curve braking less/more sensitive to lateral acceleration and apply braking later/sooner.', live=True, min_val=0.01, max_val=3.0),
      
      'CB_VTSC_long_accel_factor': Param(1.0, float, 'The vision turn controller uses the car\'s lateral acceleration in order to lookup corresponding desired values of output longitudinal acceleration. Use this to scale the output values of longitudinal acceleration. A value less/greater than 1.0 will decrease/increase the brake intensity for a given curve.', live=True, min_val=0.01, max_val=3.0),
      
      'CB_VTSC_low_speed_scale_interchange': Param(0.83, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds on freeway/highway interchanges. By 55mph, no scaling is applied. A value less/greater than 1.0 will increase/decrease the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      'CB_VTSC_low_speed_scale_freeway': Param(0.9, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds on freeways. By 55mph, no scaling is applied. A value less/greater than 1.0 will increase/decrease the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      'CB_VTSC_low_speed_scale_state_highway': Param(0.9, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds on freeways. By 55mph, no scaling is applied. A value less/greater than 1.0 will increase/decrease the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      'CB_VTSC_low_speed_scale_default': Param(0.95, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds for all other types of roads (e.g. neighborhood and most city streets). By 55mph, no scaling is applied. A value less/greater than 1.0 will increase/decrease the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      #####
      
      'CB_MTSC_smoothing_factor': Param(0.3, float, 'The map turn controller output acceleration is smoothed. Increase/decrease for more/less smoothing.', live=True, min_val=0.01, max_val=3.0),
      
      'CB_MTSC_speed_scale_interchange': Param(1.4, float, 'This scales the speed limit for a curve on interchanges. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      'CB_MTSC_speed_scale_freeway': Param(1.15, float, 'This scales the speed limit for a curve on freeways. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      'CB_MTSC_speed_scale_state_highway': Param(1.15, float, 'This scales the speed limit for a curve on state highways. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      'CB_MTSC_speed_scale_default': Param(1.15, float, 'This scales the speed limit for a curve for all other types of roads (e.g. neighborhood and most city streets). By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.', live=True, min_val=0.01, max_val=2.0),
      
      'CB_MTSC_cutoff_speed_freeway_mph': Param(35.0, float, 'If you\'re on a freeway then map curve braking won\'t activate for turns whose curve speed is less than this', live=True, min_val=0.0, max_val=90.0),
      
      'CB_MTSC_cutoff_speed_state_highway_mph': Param(0.0, float, 'If you\'re on a state highway then map curve braking won\'t activate for turns whose curve speed is less than this', live=True, min_val=0.0, max_val=90.0, unit='mph'),
      
      #####
      
      'LP_auto_auto_minimum_speed_mph': Param(10.0, float, 'Minimum speed at which traffic-based "auto auto lane position" will activate', min_val=5.0, max_val=90.0, unit='mph'),
      
      'LP_offset': Param(0.11, float, 'This controls the offset distance of the left and right lane positions, as a factor of current lane width, when manual adjustable lane position is used', live=True, min_val=0.01, max_val=0.3, is_common=True),
      
      'LP_offset_maximum_m': Param(0.7, float, 'Adjustable lane positioning picks the offset based on the current lane width. Use this to cap the max offset possible', min_val=0.01, max_val=1.5, unit='meters'),
      
      'LP_offset_factor_automatic': Param(0.8, float, 'This controls the offset distance of the left and right lane positions when automatic lane positioning is used. This is as a factor of the manual adjustable lane positioning offset, so the default of 0.8 means 80% of that offset (20% less offset) ', live=True, min_val=0.01, max_val=2.0),
      
      'LP_transition_duration_manual_s': Param(2.0, float, 'The amount of time it takes to change from one lane position to another when you manually change lane position using the onscreen buttons', min_val=0.5, max_val=60., unit='seconds'),
      
      'LP_transition_duration_automatic_s': Param(8.0, float, 'The amount of time it takes to change from one lane position to another when automatic lane position changes lane position', min_val=0.5, max_val=60., unit='seconds'),
      
      #####
      
      'XR_TD_oncoming_timeout_s': Param(60.0, float, 'OpenPilot will remember that there is oncoming traffic in an adjacent lane for this amount of time', min_val=1.0, max_val=1200.0, unit='seconds'),
      
      'XR_TD_ongoing_timeout_s': Param(5.0, float, 'OpenPilot will remember that there is ongoing traffic in an adjacent lane for this amount of time', min_val=1.0, max_val=1200.0, unit='seconds'),
      
      'XR_TD_traffic_presence_cutoff_s': Param(1.1, float, 'A moving car in an adjacent lane needs to be present for this amount of time for there to be "traffic"', min_val=0.1, max_val=10.0, unit='seconds'),
      
      'XR_TD_min_traffic_moving_speed_mph': Param(20.0, float, 'A car in an adjacent lane needs to be moving faster than this to be "moving"', min_val=0.1, max_val=50.0, unit='mph'),
      
      'XR_TD_reset_steer_angle_deg': Param(110.0, float, 'Reset traffic detection when steering wheel is above this absolute angle', min_val=10.0, max_val=720.0, unit='degrees'),
      
      'XR_TD_cutoff_steer_angle_deg': Param(9.0, float, 'Don\'t detect new traffic when steering wheel is above this absolute angle', min_val=0.0, max_val=720.0, unit='degrees'),
      
      #####
      
      'TUNE_LAT_do_override': Param(False, bool, 'If true, the other params here will override the hardcoded lateral tune settings for any gm car. Changes to this opParam will also apply to the "Custom lateral override" toggle in OpenPilot settings.', param_param='OPParamsLateralOverride', fake_live=True, param_param_read_on_startup=True),
      
      'TUNE_LAT_min_steer_speed_mph': Param(6.7, float, 'Lateral (steering) cannot engage below this speed.', min_val=0.0, max_val=60.0, unit='mph', fake_live=True),
      
      'TUNE_LAT_low_speed_extra_actuator_delay_s': Param(0.0, float, 'At very low speed, look out further into future curvatures to more proactively make sharp, low-speed curves.', min_val=0.0, max_val=2.0, unit='s', live=True),
      
      'TUNE_LAT_mpc_path_cost': Param(1.1, float, 'This value represents the weight given to the path tracking error, i.e., the deviation of the vehicle from the desired path. Increasing this value will prioritize staying close to the desired path, while decreasing it may result in larger deviations from the path.', min_val=0.0, max_val=1000.0, live=True),
      
      'TUNE_LAT_mpc_heading_cost': Param(1.1, float, 'This value is the weight given to the lateral motion of the vehicle, specifically the heading error. Increasing this value will prioritize minimizing the heading error and aligning the vehicle with the desired path. Decreasing it may result in larger heading errors.', min_val=0.0, max_val=1000.0, live=True),
      
      'TUNE_LAT_mpc_steer_rate_cost': Param(0.8, float, 'This value represents the weight given to the rate of change of the steering angle. Increasing this value will prioritize minimizing the rate of steering angle changes, resulting in smoother steering movements. Decreasing it may lead to more abrupt steering movements.', min_val=0.0, max_val=1000.0, live=True),
      
      'TUNE_LAT_type': Param('torque', [str, int], 'Type of lateral controller that will be used with the corresponding parameters. The default torque and pid tunes are for Volt, the indi tune is from Hyundai Genesis, and the lqr is from Toyota Rav4. Consult selfdrive/car/gm/interface.py to see the default values for your car for the "pid" (and possibly also the "torque") controllers.  torque: lateral acceleration-based pid controller.  pid: steering angle-based pid controller.  indi: incremental non-linear dynamic inversion controller.  lqr: linear quadratic regulator.  There are also "torque" versions of indi and lqr to experiment with. The torque INDI needs tuning, but the torque LQR needs it more. Let me know if you get those working well! The provided torque and pid tunes for Volt are the same very good tunes as hardcoded in the this fork of OpenPilot', live=True, fake_live=True, allowed_vals=['torque','pid','indi','lqr','torqueindi','torquelqr']),
      
      'TUNE_sensor_lockout_time_s': Param(60, int, 'The device sensors are used to compensate steering and long control. This can be problematic shortly after startup before they\'ve calibrated and can give crazy values. Set the amount of time after device start until sensor readings are allowed to affect controls.', live=True, min_val=0, max_val=600, unit="seconds"),
      
      #####
      
      'TUNE_LAT_TRX_roll_compensation': Param(0.4, float, 'Scale the amount of roll compensation for the torque controller', live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_use_steering_angle': Param(True, bool, 'The torque controller computes current lateral acceleration using the steering angle and current roll. Set this to false to instead use the internal Comma device sensors to measure lateral acceleration (it\'s terrible)', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_use_NN_FF': Param(False, bool, 'Use the experimental neural network feedforward for the torque controller.', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque', param_param='EnableTorqueNNFF', param_param_read_on_startup=True, fake_live=True),
      
      'TUNE_LAT_TRX_error_downscale_in_curves': Param(3.0, float, 'Downscale error when under high lateral acceleration, thereby allowing feedforward to control everything. The error is downscaled to a little as the reciprocal of this value, so if it\'s set to 5, then the error will scale down to 1/5th in sharp corners.', live=True, min_val=1.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_error_downscale_smoothing': Param(0.5, float, 'The error scaling factor is filtered so that it changes smoothly. A higher value here is more smoothing (and more lag). This factor also controls the use of instantaneous vs "lookahead" lateral jerk for calculating friction or NNFF, such that when error is downscaled, more instantaneous lateral jerk is used in order to let feedforward be as responsive as possible.', live=True, min_val=0.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_kf': Param(1.0, float, kf_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_friction': Param(1.0, float, '(For custom friction FF, this scales the response. For regular friction, this is the max value of torque that friction will send.) The torque controller has two components to the feedforward, one based solely on desired lateral acceleration and is scaled by kf. The other is based on desired lateral jerk (rate of desired lateral acceleration) and is also called "friction" to depict the idea of overcoming the friction in the steering assembly. The concept it simple: the faster the desired lateral acceleration changes (i.e. high rate of change), the greater the friction response. This provides much smoother steering, especially when the steering angle is decreasing (returning to center).', live=True, min_val=0.0, max_val=1.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_friction_lookahead_v': Param([0.3, 1.2], [list, float], "The instantaneous desired lateral jerk is too erratic, so it is checked against future planned lateral jerk so that short-lived jerk does not produce a feedforward response. This is done by taking the future lateral jerk value of minimum absolute value. If the sign of this minimized lateral jerk value is opposite that of the current desired jerk, then it is set to zero. So it can only limit the value of friction, never increase it. Here you are deciding how long a planned lateral jerk must persist before the lateral jerk feedforward acts on it. Change this in multiples of 0.2s, since lesser changes will likely have no effect do to the discretization of future lateral jerk values.", live=True, min_val=0.3, max_val=2.2, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque', unit='seconds'),
      
      'TUNE_LAT_TRX_friction_lookahead_bp': Param([20.0, 65.0], [list, float], "The instantaneous desired lateral jerk is too erratic, so it is checked against future planned lateral jerk so that short-lived jerk does not produce a feedforward response. This is done by taking the future lateral jerk value of minimum absolute value. If the sign of this minimized lateral jerk value is opposite that of the current desired jerk, then it is set to zero. So it can only limit the value of friction, never increase it. Here you are deciding how long a planned lateral jerk must persist before the lateral jerk feedforward acts on it. Change this in multiples of 0.2s, since lesser changes will likely have no effect do to the discretization of future lateral jerk values.", live=True, min_val=0.0, max_val=90.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque', unit='mph'),
      
      'TUNE_LAT_TRX_friction_curve_exit_ramp_bp': Param([0.6, 1.8], [list, float], 'Lateral jerk feedforward can push with or against the lateral acceleration feedforward, so it can help push into a curve, or help push out of a curve. At high values of lateral acceleration, in tight curves, you do not need any additional help to return to straight, and in fact the lateral jerk feedforward can cause steering issues by "helping" when it is not necessary. To address this, lateral jerk FF is decreased according to lateral acceleration when exiting curves. Here you set the "low" and "high" values, below and above which there is no damping or full damping.', live=True, min_val=0.0, max_val=3.0, unit='m/s²', show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_friction_curve_exit_ramp_v': Param(0.7, float, 'Lateral jerk feedforward can push with or against the lateral acceleration feedforward, so it can help push into a curve, or help push out of a curve. At high values of lateral acceleration, in tight curves, you do not need any additional help to return to straight, and in fact the lateral jerk feedforward can cause steering issues by "helping" when it is not necessary. To address this, lateral jerk FF is decreased according to lateral acceleration when exiting curves. Here you set the "full" damping amount, applied for lateral acceleration values at or above the "high" value set above.', live=True, min_val=0.0, max_val=1.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_kp': Param(0.48, float, kp_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_ki': Param(0.15, float, ki_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_kd': Param(0.04, float, kd_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_kp_e': Param(1.0, float, "This fork uses an \"autotuned\" PID controller, where the kp, ki, and kd values change based on the rate of change of controller error (actually the output, but that's based on the error). This controls how much kp is able to change.", live=True, min_val=0.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_ki_e': Param(1.0, float, "This fork uses an \"autotuned\" PID controller, where the kp, ki, and kd values change based on the rate of change of controller error (actually the output, but that's based on the error). This controls how much ki is able to change.", live=True, min_val=0.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_kd_e': Param(12.0, float, "This fork uses an \"autotuned\" PID controller, where the kp, ki, and kd values change based on the rate of change of controller error (actually the output, but that's based on the error). This controls how much kd is able to change.", live=True, min_val=0.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_low_speed_factor_v': Param([15.0, 5.0], [list, float], 'The torque controller corrects based on desired vs. actual lateral acceleration (curvature ⨉ speed²), so at low speeds both become very small, making for insufficient error correction. The "low speed factor" increases the perceived error in proportion to the curvature of the current curve. It should be higher at "low" speeds and then get lower at "high" speeds where lateral acceleration is sufficient. This parameter sets the "low" and "high" scaling factors. If OP fails to achieve low-speed curves, increase the value. If too high a value is used, OP will have a jerky, overshoot+correct response when turning at low speeds. This value is squared when used.', live=True, min_val=0.0, max_val=1000., show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_low_speed_factor_bp': Param([0.0, 67.0], [list, float], 'The torque controller corrects based on desired vs. actual lateral acceleration (curvature ⨉ speed²), so at low speeds both become very small, making for insufficient error correction. The "low speed factor" increases the perceived error in proportion to the curvature of the current curve. It should be higher at "low" speeds and then get lower at "high" speeds where lateral acceleration is sufficient. This parameter defines "low" and "high" speed.', live=True, min_val=0.0, max_val=90.0, unit='mph', show_op_param='TUNE_LAT_type', show_op_param_check_val='torque'),
      
      'TUNE_LAT_TRX_low_speed_factor_lookahead_s': Param(0.3, float, "Because high instantaneous curvature can make the error values very high and cause uncomfortable corrections, the curvature used to determine the lowspeed factor is checked against future planned curvature to ensure that sharp, brief curvature does not cause too harsh of steering. Here you set the amount of time in the future to consider planned curvature for this process. You are deciding how long high curvature should be sustained in order to affect the lateral controller error response.", live=True, min_val=0.3, max_val=2.2, show_op_param='TUNE_LAT_type', show_op_param_check_val='torque', unit='seconds'),
      
      #####
      
      'TUNE_LAT_PID_link_ls_hs': Param(False, bool, 'Set to true to make changes to a low-speed (ls) parameter also apply to its high-speed (hs) counterpart. With PID it seems the k values need to be higher at high speed, but in INDI other (Hyundai) car tunes only use one value, so the ls and hs values are the same. When linked, redundant params are not shown.', show_op_param='TUNE_LAT_type', show_op_param_check_val='pid', fake_live=True),
      
      'TUNE_LAT_PID_roll_compensation': Param(1.0, float, 'Scale the amount of roll compensation for the pid controller', live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_kf': Param(1.0, float, kf_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_kp': Param([0.0, 0.16], [list, float], kp_desc + 'This scales the low-speed response.', live=True, min_val=0.0, max_val=10.0, linked_op_param_check_param='TUNE_LAT_PID_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_ki': Param([0.015, 0.02], [list, float], ki_desc + 'This scales the low-speed response.', live=True, min_val=0.0, max_val=10.0,  linked_op_param_check_param='TUNE_LAT_PID_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_kd': Param([0.007, 0.007], [list, float], kd_desc + 'This scales the low-speed response.', live=True, min_val=0.0, max_val=10.0, linked_op_param_check_param='TUNE_LAT_PID_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_kp_e': Param(0.5, float, "This fork uses an \"autotuned\" PID controller, where the kp, ki, and kd values change based on the rate of change of controller error (actually the output, but that's based on the error). This controls how much kp is able to change. ", live=True, min_val=0.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_ki_e': Param(1.0, float, "This fork uses an \"autotuned\" PID controller, where the kp, ki, and kd values change based on the rate of change of controller error (actually the output, but that's based on the error). This controls how much ki is able to change.", live=True, min_val=0.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_kd_e': Param(2.0, float, "This fork uses an \"autotuned\" PID controller, where the kp, ki, and kd values change based on the rate of change of controller error (actually the output, but that's based on the error). This controls how much kd is able to change.", live=True, min_val=0.0, max_val=1000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      'TUNE_LAT_PID_bp_mph': Param([0.0, 90.0], [list, float], 'These speeds corresponds to the low- and high-speed kp, ki, and kd values.', live=True, min_val=0.0, max_val=100.0, unit="mph", show_op_param='TUNE_LAT_type', show_op_param_check_val='pid'),
      
      #####
      
      'TUNE_LAT_INDI_link_ls_hs': Param(True, bool, 'Set to true to make changes to a low-speed (ls) parameter also apply to its high-speed (hs) counterpart. With PID it seems the k values need to be higher at high speed so this wouldn\'t be used, but in INDI other (Hyundai) car tunes only use one value, so the ls and hs values are the same and this makes tuning easier. When linked, redundant params are not shown.', show_op_param='TUNE_LAT_type', show_op_param_check_val='indi'),
      
      'TUNE_LAT_INDI_roll_compensation': Param(1.0, float, 'Scale the amount of roll compensation for the indi controller', live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='indi'),
      
      'TUNE_LAT_INDI_outer_gain': Param([2.0, 2.0], [list, float], 'This scales the low-speed  (and maybe high-speed if linked) Steer error gain.  Too high: twitchy hyper lane centering, oversteering.  Too low: sloppy, long hugging in turns (not to be confused with over/understeering), all over lane (no tendency to approach the center).  Just right: crisp lane centering', live=True, min_val=0.01, max_val=10.0, linked_op_param_check_param='TUNE_LAT_INDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='indi'),
      
      'TUNE_LAT_INDI_inner_gain': Param([3.5, 3.5], [list, float], 'This scales the low-speed (and maybe high-speed if linked) Steer rate error gain.  Too high: jerky oscillation in high curvature.  Too low: sloppy, cannot accomplish desired steer angle.  Just right: brief snap on entering high curvature', live=True, min_val=0.01, max_val=10.0, linked_op_param_check_param='TUNE_LAT_INDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='indi'),
      
      'TUNE_LAT_INDI_time_constant': Param([1.4, 1.4], [list, float], 'This scales the low-speed (and maybe high-speed if linked) exponential moving average of prior output steer.  Too high: sloppy lane centering.  Too low: noisy actuation, responds to every bump, maybe unable to maintain lane center due to rapid actuation.  Just right: above noisy actuation and lane centering instability', live=True, min_val=0.01, max_val=10.0, linked_op_param_check_param='TUNE_LAT_INDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='indi'),
      
      'TUNE_LAT_INDI_actuator_effectiveness': Param([2.3, 2.3], [list, float], 'This scales the low-speed (and maybe high-speed if linked) actuator effectiveness. As it increases, actuation strength decreases.  Too high: weak, sloppy lane centering, slow oscillation, can\'t follow high curvature, high steering error causes snappy corrections.  Too low: overpower, saturation, jerky, fast oscillation, bang-bang control.  Just right: Highest value able to maintain good lane centering.', live=True, min_val=0.01, max_val=10.0, linked_op_param_check_param='TUNE_LAT_INDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='indi'),
      
      'TUNE_LAT_INDI_bp_mph': Param([0.0, 90.0], [list, float], 'At this speed, the low-speed values are used', live=True, min_val=0.0, max_val=100.0, unit="mph", show_op_param='TUNE_LAT_type', show_op_param_check_val='indi'),
      
      #####
      
      'TUNE_LAT_TRXINDI_link_ls_hs': Param(True, bool, 'Set to true to make changes to a low-speed (ls) parameter also apply to its high-speed (hs) counterpart. With PID it seems the k values need to be higher at high speed so this wouldn\'t be used, but in INDI other (Hyundai) car tunes only use one value, so the ls and hs values are the same and this makes tuning easier. When linked, redundant params are not shown.', show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_roll_compensation': Param(0.0, float, 'Scale the amount of roll compensation for the torque indi controller', live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_kf': Param(0.5, float, kf_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_friction': Param(0.05, float, 'Scale the amount of friction for the torque lqr controller. Friction applies additional steer output linearly based on the amount of desired lateral jerk (rate of change of desired lateral acceleration)', live=True, min_val=0.0, max_val=1.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_outer_gain': Param([2.0, 2.0], [list, float], 'This scales the low-speed (and maybe high-speed if linked) Steer error gain.  Too high: twitchy hyper lane centering, oversteering.  Too low: sloppy, long hugging in turns (not to be confused with over/understeering), all over lane (no tendency to approach the center).  Just right: crisp lane centering', live=True, min_val=0.01, max_val=50.0, linked_op_param_check_param='TUNE_LAT_TRXINDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_inner_gain': Param([3.5, 3.5], [list, float], 'This scales the low-speed (and maybe high-speed if linked) Steer rate error gain.  Too high: jerky oscillation in high curvature.  Too low: sloppy, cannot accomplish desired steer angle.  Just right: brief snap on entering high curvature', live=True, min_val=0.01, max_val=50.0, linked_op_param_check_param='TUNE_LAT_TRXINDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_time_constant': Param([1.4, 1.4], [list, float], 'This scales the low-speed (and maybe high-speed if linked) Exponential moving average of prior output steer.  Too high: sloppy lane centering.  Too low: noisy actuation, responds to every bump, maybe unable to maintain lane center due to rapid actuation.  Just right: above noisy actuation and lane centering instability', live=True, min_val=0.01, max_val=50.0, linked_op_param_check_param='TUNE_LAT_TRXINDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_actuator_effectiveness': Param([2.3, 2.3], [list, float], 'This scales the low-speed (and maybe high-speed if linked) actuator effectiveness. As it increases, actuation strength decreases.  Too high: weak, sloppy lane centering, slow oscillation, can\'t follow high curvature, high steering error causes snappy corrections.  Too low: overpower, saturation, jerky, fast oscillation, bang-bang control.  Just right: Highest value able to maintain good lane centering.', live=True, min_val=0.01, max_val=50.0,  linked_op_param_check_param='TUNE_LAT_TRXINDI_link_ls_hs', show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      'TUNE_LAT_TRXINDI_bp_mph': Param([0.0, 90.0], [list, float], 'At this speed, the low-speed values are used', live=True, min_val=0.0, max_val=100.0, unit="mph", show_op_param='TUNE_LAT_type', show_op_param_check_val='torqueindi'),
      
      #####
      
      'TUNE_LAT_LQR_roll_compensation': Param(1.0, float, 'Scale the amount of roll compensation for the lqr controller', live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_scale': Param(1500.0, float, 'Steer output scales inversely with this "scale" parameter, so increse to lower oversteering or early entry into curves in 10% increments', live=True, min_val=0.0, max_val=10000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_ki': Param(0.05, float, 'Like the ki in PID controllers, responds to accumulated error in order to correct for persistent sources of lateral error like crosswinds or dramatic road roll. If too high, will cause ping-pong oscillations back and fourth in the lane. Keep it as high as you can without triggering oscillations.', live=True, min_val=0.0, max_val=5.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_dc_gain': Param(0.00224, float, 'Steer output scales inversely with this "scale" parameter, so increse to lower oversteering or early entry into curves in 10% increments', live=True, min_val=0.0, max_val=5.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_a': Param([0., 1., -0.22619643, 1.21822268], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_b': Param([-1.92006585e-04, 3.95603032e-05], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_c': Param([1., 0.], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_k': Param([-110.73572306, 451.22718255], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      'TUNE_LAT_LQR_l': Param([0.3233671, 0.3185757], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='lqr'),
      
      #####
      
      'TUNE_LAT_TRXLQR_use_steering_angle': Param(True, bool, 'The torque controller computes current lateral acceleration using the steering angle and current roll. Set this to false to instead use the internal Comma device sensors to measure lateral acceleration (it\'s terrible)', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_roll_compensation': Param(0.0, float, 'Scale the amount of roll compensation for the torque lqr controller', live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_kf': Param(1.0, float, kf_desc, live=True, min_val=0.0, max_val=10.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_friction': Param(0.0, float, 'Scale the amount of friction for the torque lqr controller. Friction applies additional steer output linearly based on the amount of desired lateral jerk (rate of change of desired lateral acceleration)', live=True, min_val=0.0, max_val=1.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_scale': Param(1500.0, float, 'Steer output scales inversely with this "scale" parameter, so increse to lower oversteering or early entry into curves in 10% increments', live=True, min_val=0.0, max_val=10000.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_ki': Param(0.05, float, 'Like the ki in PID controllers, responds to accumulated error in order to correct for persistent sources of lateral error like crosswinds or dramatic road roll. If too high, will cause ping-pong oscillations back and fourth in the lane. Keep it as high as you can without triggering oscillations.', live=True, min_val=0.0, max_val=5.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_dc_gain': Param(0.00224, float, 'Steer output scales inversely with this "scale" parameter, so increse to lower oversteering or early entry into curves in 10% increments', live=True, min_val=0.0, max_val=5.0, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_a': Param([0., 1., -0.22619643, 1.21822268], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_b': Param([-1.92006585e-04, 3.95603032e-05], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_c': Param([1., 0.], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_k': Param([-110.73572306, 451.22718255], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      'TUNE_LAT_TRXLQR_l': Param([0.3233671, 0.3185757], [list, float], 'Don\'t adjust unless you know what you\'re doing, or adjust one element at a time up/down by 10-20% increments in each direction and hold onto the wheel. Once I do I will update this description!', live=True, show_op_param='TUNE_LAT_type', show_op_param_check_val='torquelqr'),
      
      #####
      
      'TUNE_LONG_gas_overlap_cutoff': Param(15, int, 'Adjust the percent throttle necessary to override OpenPilot gas control. (any amount of throttle overrides brake control.)', live=True, min_val=0, max_val=15, unit='%'),
      
      'TUNE_LONG_do_override': Param(False, bool, 'If true, the other params here will override the hardcoded longitudinal tune settings for any gm car. The default is for Volt. Changes to this opParam will also apply to the "Custom long override" toggle in OpenPilot settings. There is no tunable feedforward; instead you would adjust acceleration profiles.', param_param='OPParamsLongitudinalOverride', param_param_read_on_startup=True, fake_live=True),
      
      'TUNE_LONG_speed_mph': Param([12.0, 35.0, 80.0], [list, float], 'Lookup speeds used for corresponding values of kp, ki, and kd, such that the first value of kp,ki,kd is used when driving at the first speed here.', live=True, min_val=0.0, max_val=100.0, unit="mph"),
      
      'TUNE_LONG_kp': Param([0.8, .9, 0.8], [list, float], 'Values of kp used at the corresponding speeds in TUNE_LONG_mph. For longitudinal (gas/brake) control, too high of kp and/or ki results in overshooting and oscillations, which feel like OpenPilot is pumping the brakes. Lowering both in 5-10% increments will reduce oscillations. If kp,ki are too low, the braking response will be insufficient and OpenPilot will fail to stop. Kd at low speeds helps to reduce oscillations, allowing for higher values of kp and ki.', live=True, min_val=0.0, max_val=5.0),
      
      'TUNE_LONG_ki': Param([0.08, 0.13, 0.13], [list, float], 'Values of ki used at the corresponding speeds in TUNE_LONG_mph. For longitudinal (gas/brake) control, too high of kp and/or ki results in overshooting and oscillations, which feel like OpenPilot is pumping the brakes. Lowering both in 5-10% increments will reduce oscillations. If kp,ki are too low, the braking response will be insufficient and OpenPilot will fail to stop. Kd at low speeds helps to reduce oscillations, allowing for higher values of kp and ki.', live=True, min_val=0.0, max_val=5.0),
      
      'TUNE_LONG_kd': Param([0.004, 0.0, 0.0], [list, float], 'Values of kd used at the corresponding speeds in TUNE_LONG_mph. For longitudinal (gas/brake) control, too high of kp and/or ki results in overshooting and oscillations, which feel like OpenPilot is pumping the brakes. Lowering both in 5-10% increments will reduce oscillations. If kp,ki are too low, the braking response will be insufficient and OpenPilot will fail to stop. Kd at low speeds helps to reduce oscillations, allowing for higher values of kp and ki.', live=True, min_val=0.0, max_val=5.0),
      
      'TUNE_LONG_deadzone_ms2': Param([0.0, 0.0, 0.0], [list, float], 'Values of deadzone used at the corresponding speeds in TUNE_LONG_mph. Deadzone sets a minimum amount of desired acceleration before the gas or brakes are actually actuated. Deadzones are used to smooth jerky long control, if the gas/brake controls are too sensitive or if the planning is noisy.', live=True, min_val=0.0, max_val=5.0, unit='m/s²'),
      
      #####
      
      'MET_00': Param('PERCENT_GRADE_DEVICE', [int,str], 'UI metric in top row right column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot00', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_01': Param('ALTITUDE', [int,str], 'UI metric in second row from top, right column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot01', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_02': Param('ENGINE_RPM_TEMPF', [int,str], 'UI metric in third row from top, right column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot02', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_03': Param('EV_EFF_RECENT', [int,str], 'UI metric in fourth row from top, right column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot03', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_04': Param('CPU_TEMP_AND_PERCENTC', [int,str], 'UI metric in bottom row right column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot04', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_05': Param('DISTANCE_ENGAGED_PERCENT_TOTAL', [int,str], 'UI metric in top row left column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot05', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_06': Param('TIME_ENGAGED_PERCENT_TOTAL', [int,str], 'UI metric in second row from top, left column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot06', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_07': Param('LANE_DIST_FROM_CENTER', [int,str], 'UI metric in third row from top, left column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot07', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_08': Param('FANSPEED_PERCENT', [int,str], 'UI metric in fourth row from top, left column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot08', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_09': Param('MEMORY_USAGE_PERCENT', [int,str], 'UI metric in bottom row left column. Enter the name of the metric or it\'s number.', allowed_vals=UI_METRICS, param_param='MeasureSlot09', param_param_use_ord=True, param_param_read_on_startup=True),
      
      'MET_reset_trip_metrics': Param(False, bool, 'Set this to true in order to, the next time you start your car, reset trip and EV "total" efficiency metrics. This sets the UI metric reset toggle in OpenPilot settings, so you can reset on-device or here using opparams.', param_param='MetricResetSwitch', param_param_read_on_startup=True, is_common=True, fake_live=True),
      
      'MET_power_meter_smoothing_factor': Param(10.0, float, 'Use this to control the amount of smoothing applied to the wattage reading in order to smooth the power meter. On Volt, the power meter for power output is determined using the high-voltage battery wattage. Only total battery wattage is available to OpenPilot, so the power use is not exclusively that of the powertrain, and things like the AC/heater frequently cycle on/off, causing rapid spikes in the wattage.', min_val=0.0, max_val=1000.0),
    }
    
    # params in a group must start with the group's short name
    self._param_sections = {
      'AP': 'Acceleration Profiles',
      'FP': 'Follow Profiles',
      'LC': 'assisted Lane Change',
      'MADS': "Modified Assistive Driving Safety",
      'CB': "Curve Braking (slowing down for turns)",
      'VTSC': 'Vision Turn Speed Controller',
      'MTSC': 'Map Turn Speed Controller',
      'SLC': 'Speed Limit Controller',
      'LP': 'adjustable/automatic Lane Positioning',
      'TD': 'Traffic Detection',
      'TUNE': 'lateral/longitudinal tuning',
      'MISC': 'MISCellaneous OpenPilot settings',
      'MET': 'on-screen UI METrics',
      'LAT': "LATeral planning/control (steering)",
      'TRX': "Lateral acceleration \"torque\" controller",
      'PID': "Angle PID controller",
      'INDI': "Incremental Non-linear Dynamic Inversion controller",
      'LQR': "Linear Quadratic Regulator controller",
      'LONG': "LONGitudinal control (gas/brake)",
      'OP': "One-Pedal driving",
      'XR': "eXtended Radar lead/traffic detection",
      'LRL': "Long Range Lead detection",
    }

    self._to_delete = []  # a list of unused params you want to delete from users' params file
    self._to_reset = {
      '2023/01/26-09:30': [r'MISC_cluster_speed_.*',
                           'AP_positive_accel_post_resume_smoothing_max_speed_mph',
                           'AP_positive_accel_post_lead_smoothing_factor',
                           'AP_positive_accel_smoothing_factor',
                           'TUNE_LAT_TRX_roll_compensation',
                           'TUNE_LAT_TRX_ki',
                           'TUNE_LAT_TRX_low_speed_factor_v',
                           'TUNE_LAT_TRX_low_speed_factor_bp',
                           'TUNE_LAT_TRXINDI_roll_compensation',
                           'TUNE_LAT_TRXINDI_kf',
                           'TUNE_LAT_TRXINDI_friction',
                           'TUNE_LAT_TRXLQR_roll_compensation',
                           'MET_power_meter_smoothing_factor'],
      '2023/02/26-22:00': [r'MISC_cluster_speed_.*',
                           'AP_positive_accel_post_resume_smoothing_max_speed_mph',
                           'AP_positive_accel_post_lead_smoothing_factor',
                           'AP_positive_accel_smoothing_factor',
                           'LC_nudgeless_minimum_speed_mph',
                           'MADS_OP_rate_low_speed_factor',
                           'MADS_OP_rate_low_speed_factor',
                           'TUNE_LAT_min_steer_speed_mph',
                           'TUNE_LAT_TRX_roll_compensation',
                           'TUNE_LAT_TRX_friction',
                           'TUNE_LAT_TRX_kp',
                           'TUNE_LAT_TRX_kp_e',
                           'TUNE_LAT_TRX_ki_e',
                           'TUNE_LAT_TRX_kd_e',
                           'TUNE_LAT_PID_kp',
                           'TUNE_LONG_kp',
                           'MET_power_meter_smoothing_factor'],
      '2023/03/13-20:00': [r'.*_kd_period_s',
                           'TUNE_LAT_TRX_ki',
                           r'TUNE_LAT_TRX_k._e',
                           'TUNE_LAT_TRX_roll_compensation',
                           'MET_power_meter_smoothing_factor',
                           r'CB_.*_speed_scale.*',
                           'XR_v_lat_derivative_period_s'],
      '2023/03/16-21:00': [r'TUNE_LAT_TRX_.*'],
      '2023/03/19-02:00': ['TUNE_LAT_TRX_friction',
                           r'.*low_speed_factor_.*'],
      '2023/04/27-09:00': [r'TUNE_LAT_TRX_.*',
                           'TUNE_LAT_low_speed_extra_actuator_delay_s'],
      '2023/04/28-2:40:00': [r'TUNE_LAT_mpc_.*'],
      }  # a dict where each key is a date in 'yyyy/mm/dd-hh:mm' (24-hour) format, and the value is a list of names of params OR regular expressions to match params you want reset to their default values if the modification date is before the key date
      # use something that doesn't match the date string format and the associated list of param names or regex's will apply no matter the modified date of the param
    self._calling_function = calling_function
    self._run_init(calling_function=calling_function, check_for_reset=check_for_reset)  # restores, reads, and updates params

  def _run_init(self, calling_function = '', check_for_reset=False):  # does first time initializing of default params
    # Two required parameters for opEdit
    self.live_tuning_enabled = Params().get_bool("OPParamsLiveTuneEnabled")
    do_reset = check_for_reset and Params().get_bool("OPParamsReset")
    if do_reset:
      Params().put_bool("OPParamsReset", False)
    cloudlog.info(f"opParams: loading opParams{f' from {calling_function}' if calling_function else ''}.\n   Live tuning: {self.live_tuning_enabled}")
    
    self.fork_params['op_params_live_tune_enabled'] = Param(self.live_tuning_enabled, bool, 'Used to see if live tuning is enabled when using opparams.py (formerly op_edit.py)', hidden=True)
    self.fork_params['MISC_username'] = Param(False, [type(None), str, bool], 'Your identifier provided with any crash logs sent to Sentry.\nHelps the developer reach out to you if anything goes wrong', is_common=False, fake_live=True)
    self.fork_params['op_edit_live_mode'] = Param(False, bool, 'This parameter controls which mode opEdit starts in', hidden=True)
    self.fork_params['op_edit_section'] = Param('', str, 'This parameter controls display of param sections', hidden=True)
    self.fork_params['op_edit_compact_view'] = Param(True, bool, 'This parameter controls display of more or less parameters on the home screen', hidden=True)
    self.fork_params['op_edit_param_changed'] = Param(False, bool, 'This triggers openpilot to show an alert with the changed param', hidden=True, live=True)
    self.fork_params['op_edit_param_changed_name'] = Param("", str, 'Name of changed parameter', hidden=True, live=True)
    self.fork_params['op_edit_param_changed_val_old'] = Param("", str, 'Changed parameter old value', hidden=True, live=True)
    self.fork_params['op_edit_param_changed_val_new'] = Param("", str, 'Changed parameter new value', hidden=True, live=True)
    if not self.live_tuning_enabled:
      for p in self.fork_params:
        self.fork_params[p].live = False
        self.fork_params[p].static = True
    self.params = self._load_params(can_import=True)
    # cloudlog.info(f"opParams: {calling_function}:   Loaded op_params")
    self._add_default_params()  # adds missing params and resets values with invalid types to self.params
    # cloudlog.info(f"opParams: {calling_function}:   added default params")
    self._delete_and_reset(full_reset=do_reset)  # removes old params
    # cloudlog.info(f"opParams: {calling_function}:   delete and reset applied")
    if self.live_tuning_enabled:
      self.put('op_params_live_tune_enabled', True, reason=False)
      
    # cloudlog.info(f"opParams: {calling_function}:   Putting params with linked param_params")
    for k,v in self.fork_params.items():
      val = v.value
      type_valid, val_valid = [v.type_is_valid(v.value), v.value_is_valid(v.value)]
      if not type_valid:
        for t in v.allowed_types:
          try:
            val = t(val)
            type_valid = v.type_is_valid(val)
            if type_valid:
              v.value = val
              break
          except:
            continue
          
      if not all([type_valid, val_valid]):
        val = v.default_value
        cloudlog.info(f"opParams: {calling_function}:   Putting params with invalid initial value: {k} -> {v.value} to default value '{val}'. {type_valid = }, {val_valid = }")
        self.put(k, val, reason='replacing invalid value on startup')
      # elif v.param_param != '':
      #   cloudlog.info(f"opParams: {calling_function}:   Putting params with linked param_params: {k} -> {v.value}")
      #   self.put(k, val)

  def get(self, key=None, *, force_update=False):  # key=None returns dict of all params
    if key is None:
      return self._get_all_params(to_update=force_update)
    self._check_key_exists(key, 'get')
    return self.fork_params[key]._get_val(key, 
                                          force_update=force_update, 
                                          time_cur=sec_since_boot())

  def put(self, key, value, reason="changed by user", show_alert=False, do_log=True):
    self._check_key_exists(key, 'put')
    old_val = self.fork_params[key].value
    if not self.fork_params[key].type_is_valid(value):
      raise Exception(f'opParams: Tried to put a value of invalid type! {key = }, {value = }')
    if not self.fork_params[key].value_is_valid(value):
      raise Exception(f'opParams: Tried to put an invalid value! {key = }, {value = }')
    value, clipped = self.fork_params[key].value_clipped(value)
    if clipped:
      print(warning(f'Provided value was clipped to param bounds. {key = }, {value = }'))
    self.params.update({key: value})
    self.fork_params[key].value = value
    if self.fork_params[key].param_param != '':
      if self.fork_params[key].param_param_use_ord and value in self.fork_params[key].allowed_vals:
        ind = self.fork_params[key].allowed_vals.index(value)
        self.fork_params[key]._params.put(self.fork_params[key].param_param, str(ind))
        cloudlog.info(f"opParams: putting value in linked param {self.fork_params[key].param_param} using ordinal {ind} for {value = }. {key = }")
      else:
        put_val = value if type(value) == str \
          else str(int(value)) if type(value) == bool \
          else str(value)
        cloudlog.info(f"opParams: putting value in linked param {self.fork_params[key].param_param}: {value = }. {key = }")
        self.fork_params[key]._params.put(self.fork_params[key].param_param, put_val)
    _write_param(key, value, reason=reason, old_value=old_val, do_log=do_log)
    if show_alert:
      _write_param('op_edit_param_changed', True, reason=reason, old_value=old_val, do_log=False)
      _write_param('op_edit_param_changed_name', key, reason=reason, old_value=old_val, do_log=False)
      _write_param('op_edit_param_changed_val_old', str(old_val), reason=reason, old_value=old_val, do_log=False)
      _write_param('op_edit_param_changed_val_new', str(value), reason=reason, old_value=old_val, do_log=False)

  def _load_params(self, can_import=False):
    if not os.path.exists(PARAMS_DIR):
      os.makedirs(PARAMS_DIR)
      if can_import:
        _import_params()  # just imports old params. below we read them in

    params = {}
    for key in os.listdir(PARAMS_DIR):  # PARAMS_DIR is guaranteed to exist
      if key.startswith('.') or key not in self.fork_params:
        continue
      value, success = _read_param(key)
      if not success:
        val_default = self.fork_params[key].default_value
        cloudlog.warning(f'opParams: Failed to initially load param "{key}" (got "{value}" from the read operation), loading with default value "{val_default}"')
        value = val_default
        _write_param(key, value, reason="failure when loading params, so writing default value")
      params[key] = value
    return params

  def _get_all_params(self, to_update=False):
    if to_update:
      self.params = self._load_params()
    return {k: self.params[k] for k, p in self.fork_params.items() if k in self.params and not p.hidden}

  def _check_key_exists(self, key, met):
    if key not in self.fork_params:
      raise Exception('opParams {}: Tried to {} an unknown parameter! Key not in fork_params: {}'.format(self._calling_function, met, key))

  def _add_default_params(self):
    for key, param in self.fork_params.items():
      if key not in self.params:
        self.params[key] = param.default_value
        _write_param(key, self.params[key], reason="adding new param defaults")
      elif not param.type_is_valid(self.params[key]):
        print(warning('Value type of user\'s {} param not in allowed types, replacing with default!'.format(key)))
        self.params[key] = param.default_value
        _write_param(key, self.params[key], reason="adding new param defaults (was corrected due to invalid value)")

  def _delete_and_reset(self, full_reset=False):
    for key in list(self.params):
      if key in self._to_delete:
        del self.params[key]
        os.remove(os.path.join(PARAMS_DIR, key))
    if full_reset:
      cloudlog.warning(warning("opParams: Performing a full reset of all params!"))
      for k,v in self.fork_params.items():
        if k in self.params and v.default_value != self.params[k]:
          self.params[k] = v.default_value
          _write_param(k, self.params[k], reason="full reset")
    else:
      for k,v in self._to_reset.items():
        try:
          dt = datetime.strptime(k, '%Y/%m/%d-%H:%M') # cutoff date 'yyyy/mm/dd-hh:mm' 24-hour format
        except:
          dt = None
        for key in v:
          p = os.path.join(PARAMS_DIR, key)
          if key in self.fork_params and os.path.exists(p): # key is exact match for param
            dm = datetime.fromtimestamp(os.path.getmtime(p)) # modification date
            if (dt is None or dm < dt) and self.params[key] != self.fork_params[key].default_value: # if param modification date older than cutoff, overwrite
              cloudlog.warning(warning('Replacing value of param {}: {}, with updated default value: {}'.format(key, self.params[key], self.fork_params[key].default_value)))
              old_val = self.fork_params[key].value
              self.params[key] = self.fork_params[key].default_value
              _write_param(key, self.params[key], reason="overwriting due to fork update (exact match)", old_value=old_val)
          else: # not exact match, try as regex
            try:
              r = re.compile(key)
            except Exception as e:
              cloudlog.warning(f"Failed to apply opParam reset for {key = }")
              r = None
            if r is not None:
              for key2 in list(self.params):
                m = r.match(key2)
                if m is not None:
                  key1 = m.group()
                  p = os.path.join(PARAMS_DIR, key1)
                  if key1 in self.fork_params and os.path.exists(p):
                    dm = datetime.fromtimestamp(os.path.getmtime(p)) # modification date
                    if (dt is None or dm < dt) and self.params[key1] != self.fork_params[key1].default_value: # if param modification date older than cutoff, overwrite
                      cloudlog.warning(warning('Replacing value of param {}: {}, with updated default value: {}'.format(key1, self.params[key1], self.fork_params[key1].default_value)))
                      old_val = self.fork_params[key1].value
                      self.params[key1] = self.fork_params[key1].default_value
                      _write_param(key1, self.params[key1], reason="overwriting due to fork update (regex match)", old_value=old_val)
