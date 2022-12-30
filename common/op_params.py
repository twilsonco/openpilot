#!/usr/bin/env python3
import os
import json
from atomicwrites import atomic_write
from common.colors import COLORS
from common.numpy_fast import clip
from common.params import Params
from common.travis_checker import BASEDIR
from enum import Enum
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

UI_METRICS = [
  'STEERING_ANGLE',
  'DESIRED_STEERING_ANGLE',
  'STEERING_ANGLE_ERROR',
  'STEERING_TORQUE_EPS',
  'ENGINE_RPM',
  'ENGINE_RPM_TEMPC',
  'ENGINE_RPM_TEMPF',
  'COOLANT_TEMPC',
  'COOLANT_TEMPF',
  'ACCELERATION',
  'LAT_ACCEL',
  'DRAG_FORCE',
  'DRAG_POWER',
  'DRAG_POWER_HP',
  'DRAG_LOSSES',
  'ACCEL_FORCE',
  'ACCEL_POWER',
  'ACCEL_POWER_HP',
  'EV_FORCE',
  'EV_POWER',
  'EV_POWER_HP',
  'REGEN_FORCE',
  'REGEN_POWER',
  'REGEN_POWER_HP',
  'BRAKE_FORCE',
  'BRAKE_POWER',
  'BRAKE_POWER_HP',
  'DRIVE_POWER',
  'DRIVE_POWER_HP',
  'ICE_POWER',
  'ICE_POWER_HP',
  #// Location/road info
  'ALTITUDE',
  'BEARING',
  'PERCENT_GRADE',
  'PERCENT_GRADE_DEVICE',
  'ROLL',
  'ROLL_DEVICE',
  'LANE_WIDTH',
  'LANE_DIST_FROM_CENTER',
  'DISTANCE_TRAVELED_SESSION',
  'DISTANCE_TRAVELED_TOTAL',
  'DISTANCE_ENGAGED',
  'DISTANCE_ENGAGED_SESSION',
  'DISTANCE_ENGAGED_TOTAL',
  'DISTANCE_ENGAGED_PERCENT_SESSION',
  'DISTANCE_ENGAGED_PERCENT_TOTAL',
  'TIME_CAR_RUNNING_SESSION',
  'TIME_CAR_RUNNING_TOTAL',
  'TIME_OPENPILOT_ENGAGED_SESSION',
  'TIME_OPENPILOT_ENGAGED_TOTAL',
  'TIME_OPENPILOT_ENGAGED',
  'TIME_ENGAGED_PERCENT_SESSION',
  'TIME_ENGAGED_PERCENT_TOTAL',
  'DISENGAGEMENT_COUNT_SESSION',
  'DISENGAGEMENT_COUNT_TOTAL',
  'INTERACTION_COUNT_SESSION',
  'INTERACTION_COUNT_TOTAL',
  'INTERVENTION_COUNT_SESSION',
  'INTERVENTION_COUNT_TOTAL',
  'DISTRACTION_COUNT_SESSION',
  'DISTRACTION_COUNT_TOTAL',
  'INTERACTION_DISTANCE',
  'INTERVENTION_DISTANCE',
  'DISTRACTION_DISTANCE',
  'DISTANCE_PER_DISENGAGEMENT_SESSION',
  'DISTANCE_PER_DISENGAGEMENT_TOTAL',
  'DISTANCE_PER_INTERACTION_SESSION',
  'DISTANCE_PER_INTERACTION_TOTAL',
  'DISTANCE_PER_INTERVENTION_SESSION',
  'DISTANCE_PER_INTERVENTION_TOTAL',
  'DISTANCE_PER_DISTRACTION_SESSION',
  'DISTANCE_PER_DISTRACTION_TOTAL',
  'TIME_PER_DISENGAGEMENT_SESSION',
  'TIME_PER_DISENGAGEMENT_TOTAL',
  'TIME_PER_INTERACTION_SESSION',
  'TIME_PER_INTERACTION_TOTAL',
  'TIME_PER_INTERVENTION_SESSION',
  'TIME_PER_INTERVENTION_TOTAL',
  'TIME_PER_DISTRACTION_SESSION',
  'TIME_PER_DISTRACTION_TOTAL',
  'INTERACTION_TIMER',
  'INTERVENTION_TIMER',
  'DISTRACTION_TIMER',
  #// Lead/traffic info
  'FOLLOW_LEVEL',
  'LEAD_TTC',
  'LEAD_DISTANCE_LENGTH',
  'LEAD_DISTANCE_TIME',
  'LEAD_DESIRED_DISTANCE_LENGTH',
  'LEAD_DESIRED_DISTANCE_TIME',
  'LEAD_COSTS',
  'LEAD_VELOCITY_RELATIVE',
  'LEAD_VELOCITY_ABS',
  'LANE_POSITION',
  'LANE_OFFSET',
  'TRAFFIC_COUNT_TOTAL',
  'TRAFFIC_COUNT_ONCOMING',
  'TRAFFIC_COUNT_ONGOING',
  'TRAFFIC_COUNT_STOPPED',
  'TRAFFIC_COUNT_ADJACENT_ONGOING',
  'TRAFFIC_ADJ_ONGOING_MIN_DISTANCE',
  #// EV info
  'HVB_VOLTAGE',
  'HVB_CURRENT',
  'HVB_WATTAGE',
  'HVB_WATTVOLT',
  'EV_EFF_NOW',
  'EV_EFF_RECENT',
  'EV_EFF_TRIP',
  'EV_CONSUM_NOW',
  'EV_CONSUM_RECENT',
  'EV_CONSUM_TRIP',
  'EV_BOTH_NOW',
  'EV_OBSERVED_DRIVETRAIN_EFF',
  #// Device info
  'CPU_TEMP_AND_PERCENTF',
  'CPU_TEMP_AND_PERCENTC',
  'CPU_TEMPF',
  'CPU_TEMPC',
  'CPU_PERCENT',
  'MEMORY_TEMPF',
  'MEMORY_TEMPC',
  'AMBIENT_TEMPF',
  'AMBIENT_TEMPC',
  'FANSPEED_PERCENT',
  'FANSPEED_RPM',
  'MEMORY_USAGE_PERCENT',
  'FREESPACE_STORAGE',
  'DEVICE_BATTERY',
  'GPS_ACCURACY',
  #// vision turn speed controller
  'VISION_CURLATACCEL',
  'VISION_MAXVFORCURCURV',
  'VISION_MAXPREDLATACCEL',
  'VISION_VF',

  # 'NUM_MEASURES'
]

def _read_param(key):  # Returns None, False if a json error occurs
  try:
    with open(os.path.join(PARAMS_DIR, key), 'r') as f:
      value = json.loads(f.read())
    return value, True
  except json.decoder.JSONDecodeError:
    return None, False


def _write_param(key, value):
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
               unit=''):  # pylint: disable=dangerous-default-value
    self.default_value = default  # value first saved and returned if actual value isn't a valid type
    if not isinstance(allowed_types, list):
      allowed_types = [allowed_types]
    self.allowed_types = allowed_types  # allowed python value types for opEdit
    self.description = description  # description to be shown in opEdit
    self.hidden = hidden  # hide this param to user in opEdit
    self.live = live  # show under the live menu in opEdit
    self.static = static  # use cached value, never reads to update
    self.allowed_vals = allowed_vals # optional to specify a set of discrete allowed values (of any type)
    self.min_val = min_val # specify minimum value
    self.max_val = max_val # specify maximum value
    self.is_common = is_common
    self.unit = unit
    self.param_param = param_param # op_params can also write to regular "params" when put(), and op_params always overwrite regular params
    self.param_param_use_ord = param_param_use_ord # store the index of the value in allowed values when writing to corresponding param
    self._get_thread = None # non-static params are fetched regularly in a separate thread
    self._create_attrs()

  def type_is_valid(self, value):
    if not self.has_allowed_types:  # always valid if no allowed types, otherwise checks to make sure
      return True
    return type(value) in self.allowed_types
  
  def value_is_valid(self, value):
    if not self.has_allowed_vals:  # always valid if no allowed types, otherwise checks to make sure
      return True
    return value in self.allowed_vals
  
  def value_clipped(self, value):
    val = None
    if self.max_val is not None and self.min_val is not None:
      val = clip(value, self.min_val, self.max_val)
    elif self.max_val is not None:
      val = min(self.max_val, value)
    elif self.min_val is not None:
      val = max(self.min_val, value)
      
    if val is not None and val != value:
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
      if not type(self.default_value) in self.allowed_types:
        try:
          self.default_value = self.allowed_types[0](self.default_value)
        except ValueError:
          assert type(self.default_value) in self.allowed_types, 'Default value type must be in specified allowed_types!'
    if self.has_allowed_vals:
      assert self.default_value in self.allowed_vals, 'Default value must be in specified allowed_vals!'
    self.default_value = self.value_clipped(self.default_value)[0]
    self.value = self.default_value
    self.updated = False
    self.last_read = 0
    self._params = None
    if self.param_param != '':
      self._params = Params()
      try:
        self._params.check_key(self.param_param)
      except:
        warning("Corresponding param for this op_param is invalid!")
        self.param_param = ''
        self._params = None
    
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
      elif not (success := self.type_is_valid(value)):
        err_str = f"invalid type of {value = }"
      elif not (success := self.value_is_valid(value)):
        err_str = f"invalid {value = }"
      if not success:
        value = self.default_value
        _write_param(key, value)
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
        _write_param(key, old_params[key])
      open(IMPORTED_PATH, 'w').close()
    except:  # pylint: disable=bare-except
      pass


class opParams:
  def __init__(self, calling_function=''):
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

    self.fork_params = {
      'camera_offset_m': Param(-0.04 if TICI else 0.06, float, 'Adjust your default lane position. (Your camera offset to use in lane_planner.py)\n', live=True, min_val=-0.5, max_val=0.5, is_common=True, unit='meters'),
      'offroad_shutdown_time_hr': Param(5, int, 'The amount of time after turning off your car before your device shuts down to conserve power (and not drain your car battery)\n', min_val=0, is_common=True, unit='hours'),
      'car_12v_pause_charging_v': Param(11.0, float, 'Lower voltage threshold for your car\'s 12v system, below which the device will shutdown (well, stop charging, but there\'s no battery in the C2 or C3, so it shuts down)\n', min_val=0, max_val=10000, is_common=True, unit='volts'),
      'set_speed_offset_mph': Param(3, int, 'When adjusting 5mph at a time, this offset will be added to the result, so if set to 3mph, then your set speed will always be 3mph over the closest 5, so 10+3, 15+3, 20+3 = 13, 18, 23mph instead of 10, 15, 20\n', live=True, min_val=1, max_val=4, is_common=True, unit='mph'),
      'coasting_low_speed_over': Param(0.2, float, 'When over-speed coasting is enabled along with the 15% cap, use this to adjust the low-speed value. (0.2 corresponds to 20% over the set speed)\n', live=True, min_val=0.01, max_val=0.5, is_common=True),
      'coasting_high_speed_over': Param(0.1, float, 'When over-speed coasting is enabled along with the 15% cap, use this to adjust the low-speed value. (0.1 corresponds to 10% over the set speed)\n', live=True, min_val=0.01, max_val=0.5, is_common=True),
      'open_weather_map_api_key': Param(None, [str, None], 'You can provide your own api key for fetching weather information after signing up at https://home.openweathermap.org/users/sign_up. This is optional.\n', static=True),
      
      'AP_eco_accel_factor': Param(1.0, float, 'Scale eco acceleration.\n', live=True, min_val=0.01, max_val=20.0),
      'AP_stock_accel_factor': Param(1.0, float, 'Scale stock acceleration. This is also the acceleration used if the acceleration profiles button is disabled\n', live=True, min_val=0.01, max_val=20.0, is_common=True),
      'AP_sport_accel_factor': Param(1.0, float, 'Scale sport acceleration.\n', live=True, min_val=0.01, max_val=20.0),
      'AP_following_accel_factor': Param(1.0, float, 'Scale acceleration when behind a pulling away lead.\n', live=True, min_val=0.01, max_val=20.0),
      
      'FP_close_gas_factor': Param(1.0, float, 'Controls how proactively OpenPilot will adjust in response to a change in the lead car velocity when using the close follow profile. Increase to make close follow more responsive.\n', live=True, min_val=0.1, max_val=2.0),
      'FP_close_distance_offset_s': Param(0.0, float, 'Add or subtract follow distance from the close follow profile\n', live=True, min_val=-0.5, max_val=3.0, is_common=True, unit='seconds follow distance'),
      'FP_medium_distance_offset_s': Param(0.0, float, 'Add or subtract follow distance from the medium follow profile\n', live=True, min_val=-1.0, max_val=2.5, is_common=True, unit='seconds follow distance'),
      'FP_far_distance_offset_s': Param(0.0, float, 'Add or subtract follow distance from the far follow profile\n', live=True, min_val=-1.5, max_val=2.0, is_common=True, unit='seconds follow distance'),
      'FP_DF_distance_gain_factor': Param(1.0, float, 'Scale the rate at which the follow profile increases to a higher, farther following profile. A larger number means it will return to medium/far follow more quickly after a cut-in.\n', live=True, min_val=0.01, max_val=20.0),
      'FP_DF_distance_penalty_factor': Param(1.0, float, 'Scale the follow profile penalty incurred based on cut-in forward distance. A higher value will result in more defensive driving after a cut-in, and for longer.\n', live=True, min_val=0.01, max_val=20.0),
      'FP_DF_velocity_penalty_factor': Param(1.0, float, 'Scale the follow profile penalty incurred based on relative velocity of cut-in. A higher value will result in more defensive driving after a cut-in, and for longer.\n', live=True, min_val=0.01, max_val=20.0),
      'FP_DF_traffic_penalty_factor': Param(1.0, float, 'Scale the amount of follow profile penalty per second incurred based on the follow distances of adjacent traffic. A higher value will cause defensive driving more rapidly in the presence of close-following adjacent traffic.\n', live=True, min_val=0.01, max_val=20.0),
      
      'LC_minimum_speed_mph': Param(20.0, float, 'No OpenPilot assisted lane change below this speed\n', static=True, min_val=8.0, max_val=90.0, is_common=True, unit='mph'),
      'LC_nudgeless_minimum_speed_mph': Param(40.0, float, 'No nudgeless lane change below this speed\n', static=True, min_val=8.0, max_val=90.0, is_common=True, unit='mph'),
      'LC_nudgeless_delay_s': Param(1.5, float, 'Auto "nudgeless" lane change after this delay\n', static=True, min_val=0.0, max_val=10.0, is_common=True, unit='seconds'),
      
      'MADS_steer_pause_speed_mph': Param(20.0, float, 'When blinker is on below this speed and you\'re decelerating, autosteering will pause\n', live=True, min_val=0.0, max_val=90.0, unit='mph'),
      'MADS_steer_allow_nudgeless_lane_change': Param(False, bool, 'If true, nudgeless lane changes will apply when in MADS mode\n', static=True),
      'MADS_OP_low_speed_decel_mss': Param(-1.0, float, 'The amount of desired one-pedal deceleration at and below 1mph\n', live=True, min_val=-2.5, max_val=0.0, is_common=True, unit='m/s²'),
      'MADS_OP_high_speed_decel_mss': Param(-1.1, float, 'The amount of desired one-pedal deceleration above 14mph\n', live=True, min_val=-2.5, max_val=0.0, is_common=True, unit='m/s²'),
      'MADS_OP_rate_ramp_up': Param(0.8, float, 'The rate at which one-pedal brake force increases (applying)\n', live=True, min_val=0.1, max_val=3.0, unit='m/s³'),
      'MADS_OP_rate_ramp_down': Param(0.8, float, 'The rate at which one-pedal brake force decreases (releasing brakes)\n', live=True, min_val=0.1, max_val=3.0, unit='m/s³'),
      'MADS_OP_low_speed_pitch_factor_incline': Param(0.2, float, 'At higher speeds, one-pedal driving corrects for road pitch so that, for example, if you\'re already slowing due to an incline it doesn\'t need to apply additional brakes. At low speed this can be unwanted, so control it here when on incline. A value of 0.0 cancels out all pitch correction\n', live=True, min_val=0.0, max_val=1.0),
      'MADS_OP_low_speed_pitch_factor_decline': Param(0.4, float, 'At higher speeds, one-pedal driving corrects for road pitch so that, for example, if you\'re already slowing due to an incline it doesn\'t need to apply additional brakes. At low speed this can be unwanted, so control it here when on decline. A value of 0.0 cancels out all pitch correction\n', live=True, min_val=0.0, max_val=1.0),
      'MADS_OP_low_speed_error_factor': Param(0.4, float, 'One-pedal driving uses the same longitudinal PID controller tune as your car, which is too high for the smooth response we want from one-pedal, so we scale down the error (in terms of acceleration) for the one-pedal controller to dampen its response. This is the low-speed dampening. Set to 0.0 for a "dumb" one-pedal that applies "constant" braking regardless of whether you\'re slowing or not, or increase for more responsive correction\n', live=True, min_val=0.0, max_val=2.0),
      'MADS_OP_high_speed_error_factor': Param(0.2, float, 'One-pedal driving uses the same longitudinal PID controller tune as your car, which is too high for the smooth response we want from one-pedal, so we scale down the error (in terms of acceleration) for the one-pedal controller to dampen its response. This is the high-speed dampening. Set to 0.0 for a "dumb" one-pedal that applies "constant" braking regardless of whether you\'re slowing or not, or increase for more responsive correction\n', live=True, min_val=0.0, max_val=2.0),
      'MADS_OP_one_time_stop_threshold_mph': Param(5.0, float, 'Adjust the speed threshold used for one-pedal one-time stop\n', live=True, min_val=0.5, max_val=100.0, unit='mph'),
      'MADS_OP_one_time_stop_hold_s': Param(3.0, float, 'Adjust the amount of time regen paddle needs to be held while below the threshold speed for one-pedal one-time stop\n', live=True, min_val=0.5, max_val=15.0, unit='seconds'),
      
      'VTSC_smoothing_factor': Param(0.3, float, 'The vision turn controller output acceleration is smoothed. Increase/decrease for more/less smoothing.\n', live=True, min_val=0.01, max_val=3.0),
      'VTSC_lat_accel_factor': Param(1.0, float, 'The vision turn controller uses the car\'s lateral acceleration in order to lookup corresponding desired values of output longitudinal acceleration. Use this to scale the lateral acceleration values used in the lookup. A value less/greater than 1.0 will make curve braking less/more sensitive to lateral acceleration and apply braking later/sooner.\n', live=True, min_val=0.01, max_val=3.0),
      'VTSC_long_accel_factor': Param(1.0, float, 'The vision turn controller uses the car\'s lateral acceleration in order to lookup corresponding desired values of output longitudinal acceleration. Use this to scale the output values of longitudinal acceleration. A value less/greater than 1.0 will decrease/increase the brake intensity for a given curve.\n', live=True, min_val=0.01, max_val=3.0, is_common=True),
      'VTSC_low_speed_scale_interchange': Param(0.85, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds on freeway/highway interchanges. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      'VTSC_low_speed_scale_freeway': Param(1.0, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds on freeways. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      'VTSC_low_speed_scale_state_highway': Param(0.9, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds on freeways. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      'VTSC_low_speed_scale_default': Param(1.05, float, 'This scales the perceived car speed used by the vision turn speed controller at low speeds for all other types of roads (e.g. neighborhood and most city streets). By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      
      'MTSC_smoothing_factor': Param(0.3, float, 'The map turn controller output acceleration is smoothed. Increase/decrease for more/less smoothing.\n', live=True, min_val=0.01, max_val=3.0),
      'MTSC_speed_scale_interchange': Param(1.34, float, 'This scales the perceived car speed used by the vision turn speed controller at all speeds on interchanges. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      'MTSC_speed_scale_freeway': Param(1.08, float, 'This scales the perceived car speed used by the vision turn speed controller at all speeds on freeways. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      'MTSC_speed_scale_state_highway': Param(1.08, float, 'This scales the perceived car speed used by the vision turn speed controller at all speeds on state highways. By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      'MTSC_speed_scale_default': Param(1.08, float, 'This scales the perceived car speed used by the vision turn speed controller at all speeds for all other types of roads (e.g. neighborhood and most city streets). By 55mph, no scaling is applied. A value less/greater than 1.0 will decrease/increase the speed at which curves are taken at low speeds.\n', live=True, min_val=0.01, max_val=2.0),
      'MTSC_cutoff_speed_freeway_mph': Param(35.0, float, 'If you\'re on a freeway then map curve braking won\'t activate for turns whose curve speed is less than this\n', live=True, min_val=0.0, max_val=90.0),
      'MTSC_cutoff_speed_state_highway_mph': Param(0.0, float, 'If you\'re on a state highway then map curve braking won\'t activate for turns whose curve speed is less than this\n', live=True, min_val=0.0, max_val=90.0, unit='mph'),
      
      'SLC_offset_low_speed': Param(0.2, float, 'The speed limit offset changes from this amount over (0.2 = 20% over) at 20mph to the high_speed offset amount by 80mph\n', live=True, min_val=0.0, max_val=0.5),
      'SLC_offset_high_speed': Param(0.12, float, 'The speed limit offset changes from the low_speed amount at 20mph to the this amount (0.12 = 12% over) by 80mph\n', live=True, min_val=0.0, max_val=0.5),
      
      'LP_auto_auto_minimum_speed_mph': Param(10.0, float, 'Minimum speed at which traffic-based "auto auto lane position" will activate\n', static=True, min_val=5.0, max_val=90.0, is_common=True, unit='mph'),
      'LP_offset': Param(0.11, float, 'This controls the offset distance of the left and right lane positions, as a factor of current lane width, when manual adjustable lane position is used\n', static=True, min_val=0.01, max_val=0.3, is_common=True),
      'LP_offset_maximum_m': Param(0.7, float, 'Adjustable lane positioning picks the offset based on the current lane width. Use this to cap the max offset possible\n', static=True, min_val=0.01, max_val=1.5, unit='meters'),
      'LP_offset_factor_automatic': Param(0.8, float, 'This controls the offset distance of the left and right lane positions when automatic lane positioning is used. This is as a factor of the manual adjustable lane positioning offset, so the default of 0.8 means 80% of that offset (20% less offset) \n', static=True, min_val=0.01, max_val=2.0),
      'LP_transition_duration_manual_s': Param(2.0, float, 'The amount of time it takes to change from one lane position to another when you manually change lane position using the onscreen buttons\n', static=True, min_val=0.5, max_val=60., unit='seconds'),
      'LP_transition_duration_automatic_s': Param(8.0, float, 'The amount of time it takes to change from one lane position to another when automatic lane position changes lane position\n', static=True, min_val=0.5, max_val=60., unit='seconds'),
      
      'TD_oncoming_timeout_s': Param(60.0, float, 'OpenPilot will remember that there is oncoming traffic in an adjacent lane for this amount of time\n', static=True, min_val=1.0, max_val=1200.0, unit='seconds'),
      'TD_ongoing_timeout_s': Param(5.0, float, 'OpenPilot will remember that there is ongoing traffic in an adjacent lane for this amount of time\n', static=True, min_val=1.0, max_val=1200.0, unit='seconds'),
      'TD_traffic_presence_cutoff_s': Param(1.1, float, 'A moving car in an adjacent lane needs to be present for this amount of time for there to be "traffic"\n', static=True, min_val=0.1, max_val=10.0, unit='seconds'),
      'TD_min_traffic_moving_speed_mph': Param(20.0, float, 'A car in an adjacent lane needs to be moving faster than this to be "moving"\n', static=True, min_val=0.1, max_val=50.0, unit='mph'),
      'TD_reset_steer_angle_deg': Param(110.0, float, 'Reset traffic detection when steering wheel is above this absolute angle\n', static=True, min_val=10.0, max_val=720.0, unit='degrees'),
      'TD_cutoff_steer_angle_deg': Param(9.0, float, 'Don\'t detect new traffic when steering wheel is above this absolute angle\n', static=True, min_val=0.0, max_val=720.0, unit='degrees'),
      
      'MET_00': Param('PERCENT_GRADE_DEVICE', [int,str], 'UI metric in top row right column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot00', param_param_use_ord=True),
      'MET_01': Param('ALTITUDE', [int,str], 'UI metric in second row from top, right column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot01', param_param_use_ord=True),
      'MET_02': Param('ENGINE_RPM_TEMPF', [int,str], 'UI metric in third row from top, right column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot02', param_param_use_ord=True),
      'MET_03': Param('EV_EFF_RECENT', [int,str], 'UI metric in fourth row from top, right column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot03', param_param_use_ord=True),
      'MET_04': Param('CPU_TEMP_AND_PERCENTC', [int,str], 'UI metric in bottom row right column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot04', param_param_use_ord=True),
      'MET_05': Param('DISTANCE_ENGAGED_PERCENT_TOTAL', [int,str], 'UI metric in top row left column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot05', param_param_use_ord=True),
      'MET_06': Param('TIME_ENGAGED_PERCENT_TOTAL', [int,str], 'UI metric in second row from top, left column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot06', param_param_use_ord=True),
      'MET_07': Param('LANE_DIST_FROM_CENTER', [int,str], 'UI metric in third row from top, left column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot07', param_param_use_ord=True),
      'MET_08': Param('FANSPEED_PERCENT', [int,str], 'UI metric in fourth row from top, left column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot08', param_param_use_ord=True),
      'MET_09': Param('MEMORY_USAGE_PERCENT', [int,str], 'UI metric in bottom row left column. Enter the name of the metric or it\'s number.\n', allowed_vals=UI_METRICS, static=True, param_param='MeasureSlot09', param_param_use_ord=True),
    }
    
    # params in a group must start with the group's short name
    self._param_sections = {
      'AP': 'Acceleration Profiles',
      'FP': 'Follow Profiles',
      'LC': 'assisted Lane Change',
      'MADS': "Modified Assistive Driving Safety",
      'VTSC': 'Vision Turn Speed Controller',
      'MTSC': 'Map Turn Speed Controller',
      'SLC': 'Speed Limit Controller',
      'LP': 'adjustable/automatic Lane Positioning',
      'TD': 'Traffic Detection',
      'MET': 'on-screed UI METrics',
    }

    self._to_delete = []  # a list of unused params you want to delete from users' params file
    self._to_reset = []  # a list of params you want reset to their default values
    self._calling_function = calling_function
    self._run_init(calling_function=calling_function)  # restores, reads, and updates params

  def _run_init(self, calling_function = ''):  # does first time initializing of default params
    # Two required parameters for opEdit
    self.live_tuning_enabled = Params().get_bool("OPParamsLiveTuneEnabled")
    cloudlog.info(f"opParams: loading opParams{f' from {calling_function}' if calling_function else ''}.\n   Live tuning: {self.live_tuning_enabled}")
    
    self.fork_params['op_params_live_tune_enabled'] = Param(False, bool, 'Used to see if live tuning is enabled when using opparams.py (formerly op_edit.py)', hidden=True)
    self.fork_params['username'] = Param(False, [type(None), str, bool], 'Your identifier provided with any crash logs sent to Sentry.\nHelps the developer reach out to you if anything goes wrong')
    self.fork_params['op_edit_live_mode'] = Param(False, bool, 'This parameter controls which mode opEdit starts in', hidden=True)
    self.fork_params['op_edit_section'] = Param('', str, 'This parameter controls display of param sections', hidden=True)
    self.fork_params['op_edit_compact_view'] = Param(True, bool, 'This parameter controls display of more or less parameters on the home screen', hidden=True)
    if not self.live_tuning_enabled:
      for p in self.fork_params:
        self.fork_params[p].live = False
        self.fork_params[p].static = True
    self.params = self._load_params(can_import=True)
    self._add_default_params()  # adds missing params and resets values with invalid types to self.params
    self._delete_and_reset()  # removes old params
    if self.live_tuning_enabled:
      self.put('op_params_live_tune_enabled', True)
    for k,v in self.fork_params.items():
      if v.param_param != '':
        self.put(k, v.value)

  def get(self, key=None, *, force_update=False):  # key=None returns dict of all params
    if key is None:
      return self._get_all_params(to_update=force_update)
    self._check_key_exists(key, 'get')
    return self.fork_params[key]._get_val(key, 
                                          force_update=force_update, 
                                          time_cur=sec_since_boot())

  def put(self, key, value):
    self._check_key_exists(key, 'put')
    if not self.fork_params[key].type_is_valid(value):
      raise Exception('opParams: Tried to put a value of invalid type!')
    if not self.fork_params[key].value_is_valid(value):
      raise Exception('opParams: Tried to put an invalid value!')
    value, clipped = self.fork_params[key].value_clipped(value)
    if clipped:
      print(warning('Provided value was clipped to param bounds'))
    self.params.update({key: value})
    if self.fork_params[key].param_param != '':
      if self.fork_params[key].param_param_use_ord and value in self.fork_params[key].allowed_vals:
        self.fork_params[key]._params.put(self.fork_params[key].param_param, str(self.fork_params[key].allowed_vals.index(value)))
      else:
        self.fork_params[key]._params.put(self.fork_params[key].param_param, str(value))
    _write_param(key, value)

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
        value = self.fork_params[key].default_value
        _write_param(key, value)
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
        _write_param(key, self.params[key])
      elif not param.type_is_valid(self.params[key]):
        print(warning('Value type of user\'s {} param not in allowed types, replacing with default!'.format(key)))
        self.params[key] = param.default_value
        _write_param(key, self.params[key])

  def _delete_and_reset(self):
    for key in list(self.params):
      if key in self._to_delete:
        del self.params[key]
        os.remove(os.path.join(PARAMS_DIR, key))
      elif key in self._to_reset and key in self.fork_params:
        self.params[key] = self.fork_params[key].default_value
        _write_param(key, self.params[key])
