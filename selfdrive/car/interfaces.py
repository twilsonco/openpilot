import ast
import re
import os
import time
import shutil
import numpy as np
import json
from typing import Dict

from cereal import car
from common.kalman.simple_kalman import KF1D
from common.numpy_fast import interp
from common.realtime import DT_CTRL
from common.params import Params
from selfdrive.car import gen_empty_fingerprint
from selfdrive.config import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from selfdrive.controls.lib.events import Events
from selfdrive.controls.lib.vehicle_model import VehicleModel

GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
MAX_CTRL_SPEED = (V_CRUISE_MAX + 4) * CV.KPH_TO_MS  # 135 + 4 = 86 mph
ACCEL_MAX = 2.0
ACCEL_MIN = -3.5

class FluxModel:
  # dict used to rename activation functions whose names aren't valid python identifiers
  activation_function_names = {'σ': 'sigmoid'}
  def __init__(self, params_file, zero_bias=False):
    with open(params_file, "r") as f:
      params = json.load(f)

    self.input_size = params["input_size"]
    self.output_size = params["output_size"]
    self.input_mean = np.array(params["input_mean"], dtype=np.float32).T
    self.input_std = np.array(params["input_std"], dtype=np.float32).T
    test_dict = params["test_dict_zero_bias"] if zero_bias else params["test_dict"]
    self.layers = []

    for layer_params in params["layers"]:
      W = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_W'))], dtype=np.float32).T
      b = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_b'))], dtype=np.float32).T
      if zero_bias:
        b = np.zeros_like(b)
      activation = layer_params["activation"]
      for k, v in self.activation_function_names.items():
        activation = activation.replace(k, v)
      self.layers.append((W, b, activation))
    
    self.test(test_dict)
    if not self.test_passed:
      raise ValueError(f"NN FF model failed test: {params_file}")

    cloudlog.info(f"NN FF model loaded")
    cloudlog.info(self.summary(do_print=False))
  # Begin activation functions.
  # These are called by name using the keys in the model json file
  def sigmoid(self, x):
    return 1 / (1 + np.exp(-x))
    
  def tanh(self, x):
    return np.tanh(x)

  def sigmoid_fast(self, x):
    return 0.5 * (x / (1 + np.abs(x)) + 1)
    # return x / (1 + np.abs(x))

  def identity(self, x):
    return x
  # End activation functions

  def forward(self, x):
    for W, b, activation in self.layers:
      if hasattr(self, activation):
        x = getattr(self, activation)(x.dot(W) + b)
      else:
        raise ValueError(f"Unknown activation: {activation}")
    return x

  def evaluate(self, input_array):
    if len(input_array) != self.input_size:
      # This can be used to discern between different "versions" of the NNFF model
      # v1 has an input of 4 (v_ego, lateral_accel, lateral_jerk, roll)
      # v2 has an input of 20 (v_ego, a_ego, lateral_accel, lateral_jerk, roll, <then three groups of five points with lat accel, lat jerk, and roll data for at one past point -0.3s, and four future points 0.3, 0.6, 1.1, 2.0s, where the 0.3s values are actually the "desired" values when calling the model>) 
      if len(input_array) == 23 and self.input_size == 4: # leave out a_ego and anything after the first 5 values
        input_array = [input_array[0], input_array[2], input_array[3], input_array[4]]
      else:
        raise ValueError(f"Input array length {len(input_array)} does not match the expected length {self.input_size}")
        
    input_array = np.array(input_array, dtype=np.float32)#.reshape(1, -1)

    # Rescale the input array using the input_mean and input_std
    input_array = (input_array - self.input_mean) / self.input_std

    output_array = self.forward(input_array)

    return float(output_array[0, 0])
  
  def test(self, test_data: dict) -> str:
    num_passed = 0
    num_failed = 0
    allowed_chars = r'^[-\d.,\[\] ]+$'

    for input_str, expected_output in test_data.items():
      if not re.match(allowed_chars, input_str):
        raise ValueError(f"Invalid characters in NN FF model testing input string: {input_str}")

      input_list = ast.literal_eval(input_str)
      model_output = self.evaluate(input_list)

      if abs(model_output - expected_output) <= 5e-5:
        num_passed += 1
      else:
        num_failed += 1
        raise ValueError(f"NN FF model failed test at value {input_list}: expected {expected_output}, got {model_output}")

    summary_str = (
      f"Test results: PASSED ({num_passed} inputs tested) "
    )
    
    self.test_passed = num_failed == 0
    self.test_str = summary_str

  def summary(self, do_print=True):
    summary_lines = [
      "FluxModel Summary:",
      f"Input size: {self.input_size}",
      f"Output size: {self.output_size}",
      f"Number of layers: {len(self.layers)}",
      self.test_str,
      "Layer details:"
    ]

    for i, (W, b, activation) in enumerate(self.layers):
      summary_lines.append(
          f"  Layer {i + 1}: W: {W.shape}, b: {b.shape}, f: {activation}"
      )
    
    summary_str = "\n".join(summary_lines)

    if do_print:
      print(summary_str)

    return summary_str

# generic car and radar interfaces

class CarInterfaceBase():
  def __init__(self, CP, CarController, CarState):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.steer_warning = 0
    self.steering_unpressed = 0
    self.low_speed_alert = False
    self.driver_interacted = False
    self.screen_tapped = False
    
    self.ff_nn_model = None

    self.MADS_enabled = Params().get_bool("MADSEnabled")
    self.MADS_alert_mode = 0
    self.MADS_alerts = [EventName.madsAlert1, EventName.madsAlert2, EventName.madsAlert5, EventName.madsAlert3, EventName.madsAlert6, EventName.madsAlert4]
    self.MADS_alert_dur = int(4 / DT_CTRL)
    self.MADS_alery_delay = int(1.3 / DT_CTRL)
    self.disengage_on_gas = not self.MADS_enabled and not Params().get_bool("DisableDisengageOnGas")

    if CarState is not None:
      self.CS = CarState(CP)
      self.cp = self.CS.get_can_parser(CP)
      self.cp_cam = self.CS.get_cam_can_parser(CP)
      self.cp_body = self.CS.get_body_can_parser(CP)
      self.cp_loopback = self.CS.get_loopback_can_parser(CP)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP, self.VM)
  
  def get_ff_nn(self, x):
    return self.ff_nn_model.evaluate(x)

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed, CI = None):
    return ACCEL_MIN, ACCEL_MAX

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    raise NotImplementedError

  @staticmethod
  def init(CP, logcan, sendcan):
    pass

  @staticmethod
  def get_steer_feedforward_default(desired_angle, v_ego):
    # Proportional to realigning tire momentum: lateral acceleration.
    # TODO: something with lateralPlan.curvatureRates
    return desired_angle * (v_ego**2)
  
  @staticmethod
  def get_steer_feedforward_torque_default(desired_lateral_accel, v_ego):
    return desired_lateral_accel
  
  @staticmethod
  def get_steer_feedforward_torque_nn_default(v_ego, desired_lateral_accel, desired_lateral_jerk, lateral_accel_g):
    return desired_lateral_accel
  
  @staticmethod
  def get_steer_feedforward_nn_default(v_ego, desired_lateral_accel, desired_lateral_jerk, lateral_accel_g):
    return desired_lateral_accel
  
  @staticmethod
  def get_steer_feedforward_function_torque_roll_default(g_lat_accel, v_ego):
    return g_lat_accel
  
  @staticmethod
  def get_steer_feedforward_function_torque_lat_jerk_default(desired_lateral_jerk, v_ego, desired_lateral_acceleration, friction, friction_threshold, g_lat_accel):
    if friction < 0.0:
      f = 0.0
    else:
      f = friction
    return interp(desired_lateral_jerk,
                  [-friction_threshold, friction_threshold],
                  [-f, f])

  @staticmethod
  def get_steer_feedforward_function():
    return CarInterfaceBase.get_steer_feedforward_default
  
  @staticmethod
  def get_steer_feedforward_function_torque():
    return CarInterfaceBase.get_steer_feedforward_torque_default
  
  @staticmethod
  def initialize_feedforward_function_torque_nn():
    return CarInterfaceBase.get_steer_feedforward_torque_nn_default
  
  @staticmethod
  def initialize_feedforward_function_nn():
    return CarInterfaceBase.get_steer_feedforward_nn_default
  
  @staticmethod
  def get_steer_feedforward_function_torque_lat_jerk():
    return CarInterfaceBase.get_steer_feedforward_function_torque_lat_jerk_default
  
  @staticmethod
  def get_steer_feedforward_function_torque_roll():
    return CarInterfaceBase.get_steer_feedforward_function_torque_roll_default

  # returns a set of default params to avoid repetition in car specific params
  @staticmethod
  def get_std_params(candidate, fingerprint):
    ret = car.CarParams.new_message()
    ret.carFingerprint = candidate

    # standard ALC params
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [1.]
    ret.minSteerSpeed = 0.

    ret.pcmCruise = True     # openpilot's state is tied to the PCM's cruise state on most cars
    ret.minEnableSpeed = -1. # enable is done by stock ACC, so ignore this
    ret.steerRatioRear = 0.  # no rear steering, at least on the listed cars aboveA
    ret.openpilotLongitudinalControl = False
    ret.startAccel = 0.0
    ret.minSpeedCan = 0.3
    ret.stoppingDecelRate = 0.8 # brake_travel/s while trying to stop
    ret.startingAccelRate = 3.2 # brake_travel/s while releasing on restart
    ret.stoppingControl = True
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [1.]
    # TODO estimate car specific lag, use .15s for now
    ret.longitudinalActuatorDelayLowerBound = 0.15
    ret.longitudinalActuatorDelayUpperBound = 0.15
    ret.steerLimitTimer = 1.0
    return ret

  # returns a car.CarState, pass in car.CarControl
  def update(self, c, can_strings):
    raise NotImplementedError

  # return sendcan, pass in a car.CarControl
  def apply(self, c):
    raise NotImplementedError

  def create_common_events(self, cs_out, extra_gears=None, gas_resume_speed=-1, pcm_enable=True):
    events = Events()

    if cs_out.cruiseMain:
      if cs_out.doorOpen:
        events.add(EventName.doorOpen)
      if cs_out.seatbeltUnlatched:
        events.add(EventName.seatbeltNotLatched)
      if cs_out.gearShifter != GearShifter.drive and (extra_gears is None or
        cs_out.gearShifter not in extra_gears):
        if cs_out.gearShifter == GearShifter.park:
          events.add(EventName.silentWrongGear)
        else:
          events.add(EventName.wrongGear)
      if cs_out.gearShifter == GearShifter.reverse:
        events.add(EventName.reverseGear)
      if not cs_out.cruiseState.available:
        events.add(EventName.wrongCarMode)
      if cs_out.espDisabled:
        events.add(EventName.espDisabled)
      if cs_out.gasPressed and self.disengage_on_gas:
        events.add(EventName.gasPressed)
      if cs_out.stockFcw:
        events.add(EventName.stockFcw)
      if cs_out.stockAeb:
        events.add(EventName.stockAeb)
      if cs_out.vEgo > MAX_CTRL_SPEED:
        events.add(EventName.speedTooHigh)
      if cs_out.cruiseState.nonAdaptive:
        events.add(EventName.wrongCruiseMode)
      
    if self.screen_tapped:
      self.MADS_alert_mode = len(self.MADS_alerts)
    if self.MADS_enabled and self.MADS_alert_mode < len(self.MADS_alerts) and self.frame >= self.MADS_alery_delay and (self.frame - self.MADS_alery_delay) % self.MADS_alert_dur == 0:
      events.add(self.MADS_alerts[self.MADS_alert_mode])
      self.MADS_alert_mode += 1

    self.steer_warning = self.steer_warning + 1 if cs_out.steerWarning else 0
    self.steering_unpressed = 0 if cs_out.steeringPressed else self.steering_unpressed + 1
    
    if cs_out.cruiseMain:
      # Handle permanent and temporary steering faults
      if cs_out.steerError:
        events.add(EventName.steerUnavailable)
      elif cs_out.steerWarning:
        # only escalate to the harsher alert after the condition has
        # persisted for 0.5s and we're certain that the user isn't overriding
        if self.steering_unpressed > int(0.5/DT_CTRL) and self.steer_warning > int(0.5/DT_CTRL):
          events.add(EventName.steerTempUnavailable)
        else:
          events.add(EventName.steerTempUnavailableSilent)

      # Disable on rising edge of gas or brake. Also disable on brake when speed > 0.
      # Optionally allow to press gas at zero speed to resume.
      # e.g. Chrysler does not spam the resume button yet, so resuming with gas is handy. FIXME!
      if (self.disengage_on_gas and cs_out.gasPressed and (not self.CS.out.gasPressed) and cs_out.vEgo > gas_resume_speed) or \
        (cs_out.brakePressed and (not self.CS.out.brakePressed or not cs_out.standstill)):
        if (cs_out.cruiseState.enabled):
          events.add(EventName.pedalPressed)

      # we engage when pcm is active (rising edge)
      if pcm_enable:
        if cs_out.cruiseState.enabled and not self.CS.out.cruiseState.enabled:
          events.add(EventName.pcmEnable)
        elif not cs_out.cruiseState.enabled:
          events.add(EventName.pcmDisable)

    return events


class RadarInterfaceBase():
  def __init__(self, CP):
    self.pts = {}
    self.delay = 0
    self.radar_ts = CP.radarTimeStep
    self.no_radar_sleep = 'NO_RADAR_SLEEP' in os.environ

  def update(self, can_strings):
    ret = car.RadarData.new_message()
    if not self.no_radar_sleep:
      time.sleep(self.radar_ts)  # radard runs on RI updates
    return ret


class CarStateBase:
  def __init__(self, CP):
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.out = car.CarState.new_message()

    self.cruise_buttons = 0
    self.left_blinker_cnt = 0
    self.right_blinker_cnt = 0
    self.left_blinker_prev = False
    self.right_blinker_prev = False
    
    self.speed_limit_active = False
    self.speed_limit = 0.

    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, DT_CTRL], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])

  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_ego_raw], [0.0]]

    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

  def update_blinker_from_lamp(self, blinker_time: int, left_blinker_lamp: bool, right_blinker_lamp: bool):
    """Update blinkers from lights. Enable output when light was seen within the last `blinker_time`
    iterations"""
    # TODO: Handle case when switching direction. Now both blinkers can be on at the same time
    self.left_blinker_cnt = blinker_time if left_blinker_lamp else max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = blinker_time if right_blinker_lamp else max(self.right_blinker_cnt - 1, 0)
    return self.left_blinker_cnt > 0, self.right_blinker_cnt > 0

  def update_blinker_from_stalk(self, blinker_time: int, left_blinker_stalk: bool, right_blinker_stalk: bool):
    """Update blinkers from stalk position. When stalk is seen the blinker will be on for at least blinker_time,
    or until the stalk is turned off, whichever is longer. If the opposite stalk direction is seen the blinker
    is forced to the other side. On a rising edge of the stalk the timeout is reset."""

    if left_blinker_stalk:
      self.right_blinker_cnt = 0
      if not self.left_blinker_prev:
        self.left_blinker_cnt = blinker_time

    if right_blinker_stalk:
      self.left_blinker_cnt = 0
      if not self.right_blinker_prev:
        self.right_blinker_cnt = blinker_time

    self.left_blinker_cnt = max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = max(self.right_blinker_cnt - 1, 0)

    self.left_blinker_prev = left_blinker_stalk
    self.right_blinker_prev = right_blinker_stalk

    return bool(left_blinker_stalk or self.left_blinker_cnt > 0), bool(right_blinker_stalk or self.right_blinker_cnt > 0)

  @staticmethod
  def parse_gear_shifter(gear: str) -> car.CarState.GearShifter:
    d: Dict[str, car.CarState.GearShifter] = {
        'P': GearShifter.park, 'R': GearShifter.reverse, 'N': GearShifter.neutral,
        'E': GearShifter.eco, 'T': GearShifter.manumatic, 'D': GearShifter.drive,
        'S': GearShifter.sport, 'L': GearShifter.low, 'B': GearShifter.brake
    }
    return d.get(gear, GearShifter.unknown)

  @staticmethod
  def get_cam_can_parser(CP):
    return None

  @staticmethod
  def get_body_can_parser(CP):
    return None

  @staticmethod
  def get_loopback_can_parser(CP):
    return None
