#!/usr/bin/env python3

# f(angle, velocity) = steer command
# Only use units of steer: [-1,1], m/s, and degrees

# remove points with poor distribution: std() threshold? P99?

import math
import argparse
import os
import pickle
from copy import deepcopy
from typing import NamedTuple
import shutil
import tempfile
import bz2
import numpy as np
# import seaborn as sns
from tqdm import tqdm  # type: ignore
from p_tqdm import p_map
import re
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from opendbc.can.parser import CANParser

from tools.tuning.lat_settings import *
if not PREPROCESS_ONLY:
  from scipy.stats import describe
  from scipy.signal import correlate, correlation_lags
  import matplotlib.pyplot as plt
  from tools.tuning.lat_plot import fit, plot, get_steer_feedforward_for_filter
  import sys
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  class Logger(object):
      def __init__(self):
          self.terminal = sys.stdout
          self.log = open("plots/logfile.txt", "a")
      def write(self, message):
          self.terminal.write(message)
          self.log.write(message)  
      def flush(self):
          # this flush method is needed for python 3 compatibility.
          # this handles the flush command by doing nothing.
          # you might want to specify some extra behavior here.
          pass    
  sys.stdout = Logger()
  get_lat_accel_ff = get_steer_feedforward_for_filter()

from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.config import Conversions as CV
from tools.lib.logreader import MultiLogIterator
from tools.lib.route import Route

MULTI_FILE = True

MAX_GAS = 0.0
MAX_EPS_TORQUE = 0.0
MAX_SPEED = 0.0
MIN_GAS = 0.0
MAX_GAS = 0.0
MIN_BRAKE = 0.0
MAX_BRAKE = 0.0

def sign(x):
  if x < 0.0:
    return -1.0
  elif x > 0.0:
    return 1.0
  else:
    return 0.0

# Reduce samples using binning and outlier rejection
def regularize(speed, angle, steer, sort_var):
  print("Regularizing...")
  # Bin by rounding
  speed_bin = np.around(speed*2)/2
  angle_bin = np.around(angle*2, decimals=1)/2

  i = 0
  iter = 0
  std = []
  count = []
  n = len(speed)
  if n > 1e6:
    bin_count1 = 10000
    bin_step = 100
  elif n > 1e5:
    bin_count1 = 2000
    bin_step = 50
  else:
    bin_count1 = 100
    bin_step = 5
  bin_count = BIN_COUNT
  while i != len(speed):
      iter += 1
      # Select bins by mask
      mask = (speed_bin == speed_bin[i]) & (angle_bin == angle_bin[i])

      # Exclude outliers
      sigma = np.std(steer[mask])
      mean = np.mean(steer[mask])
      inliers = mask & (np.fabs(steer - mean) <= BIN_SIGMA * sigma)
      
      c = inliers.sum()
      s = np.std(steer[inliers])
      if iter % 100 == 0:
        print(f"Regulararizing: {iter} steps. {i = } out of {len(speed)}")
      # Use this bin
      if bin_count == 1 or (c > bin_count and s < BIN_STD):
        speed[i] = np.mean(speed[inliers])
        angle[i] = np.mean(angle[inliers])
        steer[i] = np.mean(steer[inliers])

        count.append(c)
        std.append(s)
        mask[i] = False
        bin_count = BIN_COUNT
        i += 1
      # else:
      #   bin_count -= 25

      # Remove samples
      speed = speed[~mask]
      angle = angle[~mask]
      steer = steer[~mask]
      sort_var = sort_var[~mask]
      speed_bin = speed_bin[~mask]
      angle_bin = angle_bin[~mask]

  count = np.log(np.sort(count))
  std = np.sort(std)
  plt.figure(figsize=(12,8))
  plt.plot(std, label='std')
  plt.title('std')
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  plt.savefig('plots/std.png')
  plt.close()

  plt.figure(figsize=(12,8))
  plt.plot(count, label='count')
  plt.title('count')
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  plt.savefig('plots/count.png')
  plt.close()

  print(f'Regularized samples: {len(speed)}')
  return speed, angle, steer

def lag(x, y):
  assert (len(x) == len(y))
  # Normalize
  x = np.array(x)
  x = (x - np.mean(x)) / np.std(x)
  y = np.array(y)
  y = (y - np.mean(y)) / np.std(y)
  corr = correlate(x, y, mode='valid')
  lags = correlation_lags(x.size, y.size, "valid")
  return lags[np.argmax(corr)]

  # # Determine lag of this section
  # x = np.array([line['steer_command'] for line in data[-1]])
  # y = np.array([line['torque_eps'] for line in data[-1]])
  # l = lag(x,y)
  # if -30 < l < -10: # reasonable clipping
  #   lags.append(lag(x,y))

  #   if lags == []:
  # else:
  #   print(lags)
  #   print(describe(lags))
  #   print(f'lag median: {np.median(lags)}')
  #   print(f'Max seq. len: {max([len(line) for line in data])}')

class Sample():
  enabled: bool = False
  v_ego: float = np.nan
  a_ego: float = np.nan
  steer_angle: float = np.nan
  steer_rate: float = np.nan
  steer_offset: float = np.nan
  steer_offset_average: float = np.nan
  torque_eps: float = np.nan # -1,1
  torque_driver: float = np.nan # -1,1
  # curvature_plan: float = np.nan # lag
  # curvature_true: float = np.nan # lag
  desired_curvature: float = np.nan
  desired_curvature_rate: float = np.nan
  lateral_accel: float = np.nan
  lateral_accel_rate_w_roll: float = np.nan
  lateral_jerk: float = np.nan
  roll: float = np.nan
  curvature_device: float = np.nan
  lateral_accel_device: float = np.nan
  steer_cmd: float = np.nan
  desired_steer_angle: float = np.nan
  desired_steer_rate: float = np.nan
  gas_cmd: float = np.nan
  gas: float = np.nan
  gas_pressed: bool = False
  brake_cmd: float = np.nan
  brake: float = np.nan
  brake_pressed: bool = False
  car_make: str = ''
  car_fp: str = ''
  long_actuator_delay: float = np.nan

class CleanSample(NamedTuple):
  angle: float = np.nan
  speed: float = np.nan
  steer: float = np.nan
  sort_var: float = np.nan

def collect(lr):
  s = Sample()
  samples: list[Sample] = []
  section: list[Sample] = []

  section_start: int = 0
  section_end: int = 0
  last_msg_time: int = 0
  
    # Select CAN signals
  # Volt has electric traction motor for accel / regen, and friction brakes.
  signals = [
      ("GasRegenCmd", "ASCMGasRegenCmd"),
  ]
  cp1 = CANParser("gm_global_a_powertrain_generated", signals, enforce_checks=False)
  signals = [
      ("FrictionBrakeCmd", "EBCMFrictionBrakeCmd"),
  ]
  cp2 = CANParser("gm_global_a_chassis", signals, enforce_checks=False)
  can1_updated = can2_updated = False
  CP = None
  VM = None
  lat_angular_velocity = np.nan
  lrd = dict()
  for msg in lr:
    try:
      msgid = f"{msg.logMonoTime}:{msg.which()}"
      if msgid in lrd:
        break
      lrd[msgid] = msg
    except:
      continue
  lr1 = list(lrd.values())
  # if not MULTI_FILE: print(f"{len(lr1)} messages")
  # type_set = set()
  for msg in sorted(lr1, key=lambda msg: msg.logMonoTime) if MULTI_FILE else tqdm(sorted(lr1, key=lambda msg: msg.logMonoTime)):
    # print(f'{msg.which() = }')
    try:
      # type_set.add(msg.which())
      if msg.which() == 'carState':
        s.v_ego  = msg.carState.vEgo
        s.a_ego  = msg.carState.aEgo
        s.steer_angle = msg.carState.steeringAngleDeg
        s.steer_rate = msg.carState.steeringRateDeg
        s.torque_eps = msg.carState.steeringTorqueEps
        s.torque_driver = msg.carState.steeringTorque
        s.gas = msg.carState.gas
        s.gas_pressed = msg.carState.gasPressed
        s.brake = msg.carState.brake
        s.brake_pressed = msg.carState.brakePressed
      elif msg.which() == 'liveParameters':
        s.steer_offset = msg.liveParameters.angleOffsetDeg
        s.steer_offset_average = msg.liveParameters.angleOffsetAverageDeg  
        stiffnessFactor = msg.liveParameters.stiffnessFactor
        steerRatio = msg.liveParameters.steerRatio
        s.roll = msg.liveParameters.roll
        continue
      elif msg.which() == 'carControl':
        s.enabled = msg.carControl.enabled
        s.steer_cmd = msg.carControl.actuatorsOutput.steer
        continue
      elif msg.which() == 'liveLocationKalman':
        yaw_rate = msg.liveLocationKalman.angularVelocityCalibrated.value[2]
      elif msg.which() == 'lateralPlan':
        # logs before 2021-05 don't have this field
        try:
          s.desired_curvature_rate = msg.lateralPlan.curvatureRates[0]
          s.desired_curvature = msg.lateralPlan.curvatures[0]
        except:
          s.desired_curvature_rate = 0
        continue
      elif VM is None and msg.which() == 'carParams':
        CP = msg.carParams
        VM = VehicleModel(CP)
      elif msg.which() == 'can':
        # print(msg.can[0])
        byt = [msg.as_builder().to_bytes()]
        print(byt)
        if not can1_updated or not can2_updated:

          # print(cp1.update_strings(bytes))
          for u in cp1.update_strings(byt):
            print(u)
            if u == 715:  # ASCMGasRegenCmd
              can1_updated = True
              print("gas")
              break
          for u in cp2.update_strings(byt):
            print(u)
            if u == 789:  # EBCMFrictionBrakeCmd
              can2_updated = True
              print("brake")
              break
      else:
        continue

      # assert all messages have been received
      valid = not np.isnan(s.v_ego) and \
              not np.isnan(s.steer_offset) and \
              not np.isnan(s.desired_curvature_rate) and \
              not np.isnan(s.desired_curvature) and \
              VM is not None and \
              not np.isnan(yaw_rate) and \
              can1_updated and \
              can2_updated
      
      if valid:
        can1_updated = can2_updated = False
        s.car_make = CP.carName
        s.car_fp = CP.carFingerprint
        s.long_actuator_delay = (CP.longitudinalActuatorDelayUpperBound + CP.longitudinalActuatorDelayLowerBound) / 2
        s.gas_cmd = cp1.vl["ASCMGasRegenCmd"]["GasRegenCmd"]
        s.brake_cmd = cp2.vl["EBCMFrictionBrakeCmd"]["FrictionBrakeCmd"]
        VM.update_params(max(stiffnessFactor, 0.1), max(steerRatio, 0.1))
        current_curvature = -VM.calc_curvature(math.radians(s.steer_angle - s.steer_offset), s.v_ego, s.roll)
        current_curvature_rate = -VM.calc_curvature(math.radians(s.steer_rate), s.v_ego, s.roll)
        current_curvature_rate_no_roll = -VM.calc_curvature(math.radians(s.steer_rate), s.v_ego, 0.)
        s.lateral_accel = current_curvature * s.v_ego**2
        s.lateral_accel_rate_w_roll = current_curvature_rate * s.v_ego**2
        s.lateral_jerk = current_curvature_rate_no_roll * s.v_ego**2
        s.curvature_device = (yaw_rate / s.v_ego) if s.v_ego > 0.01 else 0.
        s.lateral_accel_device = yaw_rate * s.v_ego  - (np.sin(s.roll) * ACCELERATION_DUE_TO_GRAVITY)
        s.desired_steer_angle = math.degrees(VM.get_steer_from_curvature(-s.desired_curvature, s.v_ego, s.roll))
        s.desired_steer_rate = math.degrees(VM.get_steer_from_curvature(-s.desired_curvature_rate, s.v_ego, 0))
#         if s.steer_cmd != 0 and s.torque_eps != 0 and s.torque_driver == 0:
#           print(f"""{s.v_ego = }
# {s.steer_angle = }
# {s.steer_rate = }
# {s.torque_eps = }
# {s.torque_driver = }
# {s.steer_cmd =}
# {s.steer_offset = }
# {s.steer_offset_average = }
# {s.roll = }
# {s.enabled = }
# {s.desired_curvature_rate = }
# {s.lateral_accel = }
# {s.lateral_accel_device = }
# """)
      
      # if valid:
      #   print(f"{s.v_ego = :0.3f}\t{s.steer_angle = :0.3f}\t{s.steer_rate = :0.3f}\t{s.torque_driver = :0.3f}\t{s.torque_eps = :0.3f}")
      # else:
      #   print("invalid")
      #   pass

      # assert continuous section
      # if last_msg_time:
      #   valid = valid and 0.1 > abs(msg.logMonoTime - last_msg_time) * 1e-9
      # last_msg_time = msg.logMonoTime

      if valid:
        samples.append(deepcopy(s))
        s.v_ego = np.nan
    #     section.append(deepcopy(s))
    #     section_end = msg.logMonoTime
    #     if not section_start:
    #       section_start = msg.logMonoTime
    #   elif section_start:
    #     # end of valid section
    #     if (section_end - section_start) * 1e-9 >= MIN_SECTION_SECONDS:
    #       samples.extend(section)  
    #       lat_angular_velocity = np.nan
    #     section = []
    #     section_start = section_end = 0
    except:
      continue
  
  # print("message types found:\n" + ", ".join(list(type_set)))

  # # Terminated during valid section
  # if (section_end - section_start) * 1e-9 > MIN_SECTION_SECONDS:
  #   samples.extend(section)


  if len(samples) == 0:
    return np.array([])

  return np.array(samples)

def lookahead_lookback_filter(samples, comp_func, n_forward = 0, n_back = 0):
  return np.array([s for i,s in enumerate(samples) if \
    i > n_back and i < len(samples) - n_forward and \
    all([comp_func(s1) for s1 in samples[max(0,i-n_back):min(len(samples), i+n_forward+1)]])])

def filter(samples):
  global MIN_GAS, MAX_GAS, MAX_BRAKE, MIN_BRAKE
  # Order these to remove the most samples first
  
  
  # Some rlogs use [-300,300] for torque, others [-3,3]
  # Scale both from STEER_MAX to [-1,1]
  steer_torque_key = "torque_eps" # gm
  MIN_GAS = min(MIN_GAS, np.min(np.array([s.gas_cmd for s in samples])))
  MAX_GAS = max(MAX_GAS, np.max(np.array([s.gas_cmd for s in samples])))
  MIN_BRAKE = min(MIN_BRAKE, np.min(np.array([s.brake_cmd for s in samples])))
  MAX_BRAKE = max(MAX_BRAKE, np.max(np.array([s.brake_cmd for s in samples])))
  # for s in samples:
  #   if MAX_GAS > 40 or MAX_EPS_TORQUE > 40:
  #     s.torque_driver /= 300
  #     s.torque_eps /= 300
  #   else:
  #     s.torque_driver /= 3
  #     s.torque_eps /= 3
  MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # VW MQB cars, str cmd units 0.01Nm, 3.0Nm max
  # steer_torque_key = "steer_cmd" # vw
  # MAX_GAS = max(MAX_GAS, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  # MIN_GAS = min(MIN_GAS, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_GAS = max(MAX_GAS, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_BRAKE = min(MIN_BRAKE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_BRAKE = max(MAX_BRAKE, np.max(np.array([s.steer_rate for s in samples])))
  # for s in samples:
  #   s.torque_driver /= 300
    
  # VW PQ cars {CAR.PASSAT_NMS, CAR.SHARAN_MK2}, str cmd units 0.01Nm, 3.0Nm max
  # steer_torque_key = "steer_cmd" # vw
  # MAX_GAS = max(MAX_GAS, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MIN_GAS = min(MIN_GAS, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_GAS = max(MAX_GAS, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_BRAKE = min(MIN_BRAKE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_BRAKE = max(MAX_BRAKE, np.max(np.array([s.steer_rate for s in samples])))
  # for s in samples:
  #   s.torque_driver /= 300
  #   setattr(s, steer_torque_key, getattr(s,steer_torque_key) * 3)
  #   if s.v_ego > 89.4:
  #     s.v_ego = 0.0
      
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  #hyundai
  # driver torque units 0.01Nm
  # eps torque units 0.2Nm
  # max torque 4.0Nm
  # steer_torque_key = "torque_eps" # vw
  # MAX_GAS = max(MAX_GAS, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  # MIN_GAS = min(MIN_GAS, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_GAS = max(MAX_GAS, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_BRAKE = min(MIN_BRAKE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_BRAKE = max(MAX_BRAKE, np.max(np.array([s.steer_rate for s in samples])))
  # for s in samples:
  #   s.torque_driver /= 100 * 4
  #   s.torque_eps /= 5 * 4
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # MAX_GAS = max(MAX_GAS, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # all chrysler and ram 1500: steer scaled to 261. 361 for ram hd
  # steer_torque_key = "torque_eps" # vw
  # MAX_GAS = max(MAX_GAS, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  # MIN_GAS = min(MIN_GAS, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_GAS = max(MAX_GAS, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_BRAKE = min(MIN_BRAKE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_BRAKE = max(MAX_BRAKE, np.max(np.array([s.steer_rate for s in samples])))
  # for s in samples:
  #   s.torque_driver /= 261
  #   s.torque_eps /= 261
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # print(f'max eps torque = {eps:0.4f}')
  # print(f"max driver torque = {driver:0.4f}")
  
  # Enabled and no steer pressed or not enabled and driver steer under threshold
  mask = np.array([(s.enabled and s.torque_driver <= STEER_PRESSED_MIN) or (not s.enabled and s.torque_driver <= STEER_PRESSED_MAX) for s in samples])
  samples = samples[mask]
  
  # has lat accel and lat jerk data
  # samples = np.array([s for s in samples if np.isnan(s.lateral_jerk) == False])
  # # matching sign of lateral jerk and accel
  # mask = np.array([sign(s.lateral_jerk) == sign(s.lateral_accel) for s in samples])
  # samples = samples[mask]
  
  # non-matching sign of lateral jerk and accel
  # mask = np.array([sign(s.lateral_jerk) != sign(s.lateral_accel) for s in samples])
  # samples = samples[mask]
  
  if not IS_ANGLE_PLOT:
    # samples = np.array([s for s in samples if \
    #   (sign(s.torque_driver + getattr(s, steer_torque_key)) == sign(-s.lateral_accel) \
    #     or abs(s.torque_driver + getattr(s, steer_torque_key)) < 0.15) \
    #   and abs(s.torque_driver + getattr(s, steer_torque_key)) > 0.15 * abs(s.lateral_accel)])
    samples = np.array([s for s in samples if \
      abs(s.torque_driver + getattr(s, steer_torque_key)) > 0.15 * abs(s.lateral_accel)])
  
  # enabled
  # mask = np.array([s.enabled for s in samples])
  # samples = samples[mask]
  
  if IS_ANGLE_PLOT:
    # only take high angles if there was enough lateral acceleration
    max_lat_accel = 4.0
    curv_per_deg = 1/3000.0
    mask = np.array([s.enabled or np.abs(((s.steer_angle - s.steer_offset) * curv_per_deg) * s.v_ego**2) < max_lat_accel  for s in samples])
    samples = samples[mask]
    
    
  
  # these next two can be used if only driver torque is available
  # driver steer under threshold
  # data = np.array([s.torque_driver for s in samples])
  # mask = np.abs(data) <= STEER_PRESSED_MAX
  # samples = samples[mask]
  
  # NOT Enabled
  # mask = np.array([not s.enabled for s in samples])
  # samples = samples[mask]

  # No steer rate: holding steady curve or straight
  # data = np.array([s.desired_curvature_rate for s in samples])
  # mask = np.abs(data) < CURVATURE_RATE_MIN # determined from plotjuggler
  # samples = samples[mask]

  # No steer rate: holding steady curve or straight
  data = np.array([s.steer_rate for s in samples])
  mask = np.abs(data) < STEER_RATE_MIN
  samples = samples[mask]
  
  # samples = lookahead_lookback_filter(samples, lambda s:abs(s.steer_rate) < STEER_RATE_MIN, 1, 1)
  
  # constant speed
  data = np.array([s.a_ego for s in samples])
  mask = np.abs(data) <= LONG_ACCEL_MAX
  samples = samples[mask]

  # GM no steering below 7 mph
  # data = np.array([s.v_ego for s in samples])
  # mask = SPEED_MIN * CV.MPH_TO_MS <= data
  # mask &= data <= SPEED_MAX * CV.MPH_TO_MS
  # samples = samples[mask]
  
  data = np.array([s.v_ego for s in samples])
  if IS_ANGLE_PLOT:
    mask = SPEED_MIN_ANGLE * CV.MPH_TO_MS <= data
  else:
    mask = SPEED_MIN * CV.MPH_TO_MS <= data
  mask &= data <= SPEED_MAX * CV.MPH_TO_MS
  samples = samples[mask]

  # Not saturated
  # data = np.array([s.torque_eps for s in samples])
  # mask = np.abs(data) < 4.0
  # samples = samples[mask]
  
  # out = []
  # for s in samples:
  #   speed = s.v_ego
  #   angle = -s.lateral_accel if not IS_ANGLE_PLOT else s.steer_angle - s.steer_offset
  #   lat_jerk_ff = get_lat_accel_ff(-s.lateral_jerk, s.v_ego, -s.lateral_accel)
  #   actual_steer = (s.torque_driver + (getattr(s, steer_torque_key) if not np.isnan(getattr(s, steer_torque_key)) else 0.0))
  #   steer = actual_steer - lat_jerk_ff
  #   sort_var = 0.0
  #   out.append(CleanSample(speed=speed, angle=angle, steer=steer, sort_var=sort_var))
  #   # print(out[-1])
  
  # return out

  return [CleanSample(
    speed = s.v_ego,
    angle = -s.lateral_accel if not IS_ANGLE_PLOT else s.steer_angle - s.steer_offset,
    steer = (s.torque_driver + (getattr(s, steer_torque_key) if not np.isnan(getattr(s, steer_torque_key)) else 0.0)),
    sort_var = getattr(s, BIN_SORT_VAR)
  ) for s in samples]

def load_cache(path):
  # print(f'Loading {path}')
  try:
    with open(path,'rb') as file:
      return pickle.load(file)
  except Exception as e:
    print(e)

def load(path, route=None, preprocess=False, dongleid=False, outpath=""):
  global MULTI_FILE
  ext = '.opfit'
  latpath = None
  allpath = None

  if not path and not route:
    exit(1)
  if path is not None:
    allpath = os.path.join(path, 'all.opfit')
  if route is not None:
    if path is not None:
      latpath = os.path.join(path, f'{route}{ext}')
    else:
      latpath = os.path.join(os.getcwd(), f'{route}{ext}')
  data = []
  old_num_points = 0
  if route:
    print(f'Loading from rlogs {route}')
    try:
      r = Route(route, data_dir=path)
      lr = MultiLogIterator(r.log_paths(), sort_by_time=True)
      data = collect(lr)

      if len(data):
        with open(latpath, 'wb') as f: # cache
          pickle.dump(data, f)
      data = filter(data)
    except Exception as e:
      print(f"Failed to load segment file {path}/{route}:\n{e}")
      
  # Only path
  else:
    if latpath and os.path.isfile(latpath):
      data = filter(load_cache(latpath))
    elif os.path.isfile(allpath):
      data = filter(load_cache(allpath))
    else:
      print(f'Loading many in {path}')
      data = []
      routes = set()
      latroutes = set()
      steer_offsets = []
      if not preprocess:
        errors={}
        for filename in (pbar := tqdm(os.listdir(path))):
          if filename.endswith(ext):
            latpath = os.path.join(path, filename)
            latroutes.add(filename.replace(ext,''))
            # commented code was used to correct existing .lat files
            # data1=load_cache(latpath)
            # for s in data1:
            #   s.torque_eps *= 3
            #   s.torque_driver *= 3
            # with open(latpath, 'wb') as f:
            #   pickle.dump(data1, f)
            if not PREPROCESS_ONLY:
              tmpdata = load_cache(latpath)
              try:
                data.extend(filter(tmpdata))
                old_num_points += len(tmpdata)
                if not PREPROCESS_ONLY:
                  steer_offsets.extend(s.steer_offset for s in tmpdata)
              except Exception as e:
                # print(e)
                if e in errors:
                  errors[e] += 1
                else:
                  errors[e] = 1
          numerr = sum(errors.values()) if len(errors) > 0 else 0
          pbar.set_description(f"Imported {len(data)} points (filtered out {(old_num_points-len(data))/max(1,old_num_points)*100.0:.1f}% of {old_num_points}) (skipped segments: {numerr})")
        if len(errors) > 0:
          nerr = sum(errors.values())
          print(f"{nerr} opfit files were not loaded due to errors:")
          for e,n in errors.items():
            print(f"({n}) {e}")
      if PREPROCESS_ONLY:
        if "realdata" in path or (preprocess and dongleid):
          MULTI_FILE = True
          # we're on device going through rlogs
          if preprocess and dongleid:
            dongle_id = dongleid
            rlog_path = "/Users/haiiro/Downloads/opfitfiles_batch"
            rlog_log_path = "/Users/haiiro/Downloads/opfitfiles.txt" # prevents rerunning rlogs
          else:
            with open("/data/params/d/DongleId","r") as df:
              dongle_id = df.read()
            rlog_path = "/data/media/0/opfitfiles"
            rlog_log_path = "/data/media/0/opfitfiles.txt" # prevents rerunning rlogs
          print(f"{dongle_id = }")
          if not os.path.exists(rlog_path):
            os.mkdir(rlog_path)
          latsegs = set([f for f in os.listdir(rlog_path) if ".opfit" in f])
          if os.path.exists(rlog_log_path): 
            # read in lat files saved by running `ls -1 /data/media/0/latfiles >> /data/media/0/latfiles.txt`
            with open(rlog_log_path, 'r') as rll:
              latsegs = latsegs | set(list(rll.read().split('\n')))
          with open(rlog_log_path, 'w') as rll:
            for ls in sorted(list(latsegs)):
              rll.write(f"\n{ls}")
          filenames = sorted([filename for filename in os.listdir(path) if len(filename.split('--')) == 3 and f"{dongle_id}|{filename}.opfit" not in latsegs])
          print(f"Preparing fit data from {len(filenames)} rlog segments")
          for filename in tqdm(filenames, desc="Preparing fit data from rlogs"):
            if len(filename.split('--')) == 3 and f"{dongle_id}|{filename}.opfit" not in latsegs:
              with tempfile.TemporaryDirectory() as d:
                if os.path.exists(os.path.join(path,filename,"rlog")):
                  shutil.copy(os.path.join(path,filename,"rlog"),os.path.join(d,f"{dongle_id}_{filename}--rlog"))
                elif os.path.exists(os.path.join(path,filename,"rlog.bz2")):
                  tmpbz2 = os.path.join(d,f"{dongle_id}_{filename}--rlog.bz2")
                  shutil.copy(os.path.join(path,filename,"rlog.bz2"),tmpbz2)
                
                seg_num = f"{dongle_id}|{filename}".split('--')[2]
                try:
                  route='--'.join(f"{dongle_id}|{filename}".split('--')[:2])
                  r = Route(route, data_dir=d)
                  lr = MultiLogIterator([lp for lp in r.log_paths() if lp])
                  data1 = collect(lr)
                  if len(data1):
                    with open(os.path.join(rlog_path, f"{route}--{seg_num}.opfit"), 'wb') as f:
                      pickle.dump(data1, f)
                except Exception as e:
                  print(f"Failed to load segment file {filename}: {e}")
                  continue
                finally:
                  with open(rlog_log_path, 'a') as rll:
                    rll.write(f"\n{route}--{seg_num}.opfit")
        else:
          # first make per-segment .lat files
          # get previously completed segments
          print("Batch processing rlogs into lat files")
          if outpath == "": outpath = path
          print(f"{outpath = }")
          latsegs = set()
          if not os.path.exists(outpath):
            os.mkdir(outpath)
          rlog_log_path = ""
          if preprocess:
            rlog_log_path = os.path.join(outpath, "/opfitfiles.txt") # prevents rerunning rlogs
            if os.path.exists(rlog_log_path): 
              # read in lat files saved by running `ls -1 /data/media/0/latfiles >> /data/media/0/latfiles.txt`
              with open(rlog_log_path, 'r') as rll:
                latsegs = latsegs | set(list(rll.read().split('\n')))
          for filename in os.listdir(outpath):
            if ext in filename.split("/")[-1]:
              latsegs.add(filename.replace(outpath, path).replace('.opfit','--rlog.bz2').replace('|','_'))
              latsegs.add(filename.replace(outpath, path).replace('.opfit','--rlog.bz2'))
          filenames = sorted([filename for filename in os.listdir(path) if filename.endswith("rlog.bz2") and filename not in latsegs])
          def process_file(filename):
            if len(filename.split('--')) == 4 and filename.endswith('rlog.bz2'):
              seg_num = filename.split('--')[2]
              route='--'.join(filename.split('--')[:2]).replace('_','|')
              latfile = os.path.join(outpath, f"{route}--{seg_num}.opfit")
              if filename not in latsegs:
                # print(f'loading rlog segment {fi} of {num_files} {filename}')
                with tempfile.TemporaryDirectory() as d:
                  try:
                    shutil.copy(os.path.join(path,filename),os.path.join(d,filename))
                    r = Route(route, data_dir=d)
                    lr = MultiLogIterator(r.log_paths(), sort_by_time=True)
                    data1 = collect(lr)
                    if len(data1):
                      with open(latfile, 'wb') as f:
                        pickle.dump(data1, f)
                    if outpath == path:
                      os.remove(os.path.join(path,filename))
                  except Exception as e:
                    print(f"Failed to load segment file {filename}:\n{e}")
              else:
                if outpath == path:
                  os.remove(os.path.join(path,filename))
          for filename in filenames:
            process_file(filename)
          # p_map(process_file, filenames, desc="Preparing fit data from rlogs")
          if preprocess:
            for filename in filenames:
                latsegs.add(filename.replace(outpath, path).replace('.opfit','--rlog.bz2').replace('|','_'))
                latsegs.add(filename.replace(outpath, path).replace('.opfit','--rlog.bz2'))
            with open(rlog_log_path, 'w') as rll:
              for ls in sorted(list(latsegs)):
                rll.write(f"\n{ls}")
          
          # for filename in tqdm(filenames, desc="Preparing fit data from rlogs"):
          #   if len(filename.split('--')) == 4 and filename.endswith('rlog.bz2'):
          #     seg_num = filename.split('--')[2]
          #     route='--'.join(filename.split('--')[:2]).replace('_','|')
          #     latfile = os.path.join(outpath, f"{route}--{seg_num}.lat")
          #     if filename not in latsegs:
          #       # print(f'loading rlog segment {fi} of {num_files} {filename}')
          #       with tempfile.TemporaryDirectory() as d:
          #         try:
          #           shutil.copy(os.path.join(path,filename),os.path.join(d,filename))
          #           r = Route(route, data_dir=d)
          #           lr = MultiLogIterator(r.log_paths(), sort_by_time=True)
          #           data1 = collect(lr)
          #           if len(data1):
          #             with open(latfile, 'wb') as f:
          #               pickle.dump(data1, f)
          #           if outpath == path:
          #             os.remove(os.path.join(path,filename))
          #         except Exception as e:
          #           print(f"Failed to load segment file {filename}:\n{e}")
          #     else:
          #       if outpath == path:
          #         os.remove(os.path.join(path,filename))
      else:
        print(f'max eps torque = {MAX_EPS_TORQUE:0.4f}')
        print(f"max driver torque = {MAX_GAS:0.4f}")
        print(f"max speed = {MAX_SPEED*2.24:0.4f} mph")
        print(f"min curvature rate = {MIN_GAS:0.4f}")
        print(f"max curvature rate = {MAX_GAS:0.4f}")
        print(f"min steer rate = {MIN_BRAKE:0.4f}")
        print(f"max steer rate = {MAX_BRAKE:0.4f}")
        print(f"{describe(steer_offsets) = }")
        for filename in os.listdir(path):
          if filename.endswith('rlog.bz2'):
            route='--'.join(filename.split('--')[:2]).replace('_','|')
            if route not in latroutes:
              routes.add(route)
        if len(routes) > 0:
          print(f'loading data from {len(routes)} routes')
        for ri,route in enumerate(routes):
          print(f'loading rlog {ri+1} of {len(routes)}: {route}')
          try:
            r = Route(route, data_dir=path)
            lr = MultiLogIterator(r.log_paths(), sort_by_time=True)
            data1 = collect(lr)
            if len(data1):
              with open(os.path.join(path, f"{route}.opfit"), 'wb') as f:
                pickle.dump(data1, f)
              data.extend(filter(data1))
              old_num_points += len(data1)
          except Exception as e:
            print(f"Failed to load segment file {path}{route}:\n{e}")
              
      # write all.dat
      # if len(dataraw):
      #   with open(allpath, 'wb') as f:
      #     pickle.dump(dataraw, f)
  if PREPROCESS_ONLY:
    exit(0)
  
  newlen = len(data)
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  with open('plots/out.txt','w') as f:
    if old_num_points > 0 and newlen > 0:
      f.write(f"{old_num_points} points filtered down to {newlen}\n")
    else:
      f.write(f"{newlen} filtered points\n")

  speed = np.array([sample.speed for sample in data])
  angle = np.array([sample.angle for sample in data])
  steer = np.array([sample.steer for sample in data])
  sort_var = np.array([sample.sort_var for sample in data])
  print(f'Samples: {len(speed) = }, {len(angle) = }, {len(steer) = }')
  return speed, angle, steer, sort_var


if __name__ == '__main__':
  global IS_ANGLE_PLOT
  parser = argparse.ArgumentParser()
  parser.add_argument('--path', type=str)
  parser.add_argument('--outpath', type=str)
  parser.add_argument('--route', type=str)
  parser.add_argument('--preprocess', action='store_true')
  parser.add_argument('--dongleid', type=str)
  args = parser.parse_args()
  

  # IS_ANGLE_PLOT = True
  # regfile = 'regularized'
  # if REGULARIZED and os.path.isfile(regfile):
  #   print("Opening regularized data")
  #   with open(regfile,'rb') as file:
  #     speed, angle, steer = pickle.load(file)
  # else:
  #   print("Loading new data")
  #   speed, angle, steer, sort_var = load(args.path, args.route, args.preprocess, args.dongleid, args.outpath)
  #   speed, angle, steer = regularize(speed, angle, steer, sort_var)
  #   with open(regfile, 'wb') as f:
  #     pickle.dump([speed, angle, steer], f)

  # fit(speed, angle, steer, IS_ANGLE_PLOT)
  # plot(speed, angle, steer)
  
  IS_ANGLE_PLOT = False
  regfile = 'regularized'
  if REGULARIZED and os.path.isfile(regfile):
    print("Opening regularized data")
    with open(regfile,'rb') as file:
      speed, angle, steer = pickle.load(file)
  else:
    print("Loading new data")
    speed, angle, steer, sort_var = load(args.path, args.route, args.preprocess, args.dongleid, args.outpath)
    speed, angle, steer = regularize(speed, angle, steer, sort_var)
    with open(regfile, 'wb') as f:
      pickle.dump([speed, angle, steer], f)

  fit(speed, angle, steer, IS_ANGLE_PLOT)
  plot(speed, angle, steer)
