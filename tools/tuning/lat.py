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
from pathlib import Path
from collections import deque
from common.numpy_fast import interp

from tools.tuning.lat_settings import *
if not PREPROCESS_ONLY:
  from scipy.stats import describe
  from scipy.signal import correlate, correlation_lags
  import matplotlib.pyplot as plt
  from tools.tuning.lat_plot import fit, plot
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

from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.config import Conversions as CV
from tools.lib.logreader import MultiLogIterator
from tools.lib.route import Route

MULTI_FILE = True

MAX_DRIVER_TORQUE = 0.0
MAX_EPS_TORQUE = 0.0
MAX_SPEED = 0.0
MIN_CURVATURE_RATE = 0.0
MAX_CURVATURE_RATE = 0.0
MIN_STEER_RATE = 0.0
MAX_STEER_RATE = 0.0

# Reduce samples using binning and outlier rejection
def regularize(speed, angle, steer, sort_var):
  print("Regularizing...")
  # Bin by rounding
  speed_bin = np.around(speed*2)/2
  angle_bin = np.around(angle*2, decimals=0 if IS_ANGLE_PLOT else 1)/2

  i = 0
  std = []
  count = []
  while i != len(speed):
      # Select bins by mask
      mask = (speed_bin == speed_bin[i]) & (angle_bin == angle_bin[i])

      # Exclude outliers
      sigma = np.std(steer[mask])
      mean = np.mean(steer[mask])
      inliers = mask & (np.fabs(steer - mean) <= BIN_SIGMA * sigma)
      
      c = inliers.sum()
      s = np.std(steer[inliers])
      # Use this bin
      if c > BIN_COUNT and s < BIN_STD:
        sort_var_tmp = np.abs(sort_var[inliers])
        sort_ids = np.argsort(sort_var_tmp)
        
        npts = max(BIN_COUNT, int(len(inliers) * 0.1))
        speed_tmp = speed[inliers][sort_ids][:npts]
        angle_tmp = angle[inliers][sort_ids][:npts]
        steer_tmp = steer[inliers][sort_ids][:npts]
        
        speed[i] = np.mean(speed_tmp)
        angle[i] = np.mean(angle_tmp)
        steer[i] = np.mean(steer_tmp)

        count.append(c)
        std.append(s)
        mask[i] = False
        i += 1

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
  lateral_jerk: float = np.nan
  roll: float = np.nan
  pitch: float = np.nan
  curvature_device: float = np.nan
  lateral_accel_device: float = np.nan
  steer_cmd: float = np.nan
  steer_cmd_out: float = np.nan
  desired_steer_angle: float = np.nan
  desired_steer_rate: float = np.nan
  desired_accel: float = np.nan
  gas_cmd: float = np.nan
  gas_cmd_out: float = np.nan
  gas: float = np.nan
  gas_pressed: bool = False
  brake_cmd: float = np.nan
  brake_cmd_out: float = np.nan
  brake: float = np.nan
  brake_pressed: bool = False
  car_make: str = ''
  car_fp: str = ''
  long_actuator_delay: float = np.nan
  t_cs: float = np.nan
  t_cc: float = np.nan
  t_llk: float = np.nan
  t_lp: float = np.nan
  t_lat_p: float = np.nan
  t: float = np.nan
  yaw_rate: float = np.nan
  stiffnessFactor: float = np.nan
  steerRatio: float = np.nan
  

class CleanSample(NamedTuple):
  angle: float = np.nan
  speed: float = np.nan
  steer: float = np.nan
  sort_var: float = np.nan

def collect(lr):
  s = Sample()
  sample_deque = deque(maxlen=16)
  deque_mid_idx = 8
  t_interp_field = "t_llk"
  max_t_delta = 0.1
  samples: list[Sample] = []
  section: list[Sample] = []

  section_start: int = 0
  section_end: int = 0
  last_msg_time: int = 0
  
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
        s.t_cs = msg.logMonoTime
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
        s.t_lp= msg.logMonoTime
        s.steer_offset = msg.liveParameters.angleOffsetDeg
        s.steer_offset_average = msg.liveParameters.angleOffsetAverageDeg  
        s.stiffnessFactor = msg.liveParameters.stiffnessFactor
        s.steerRatio = msg.liveParameters.steerRatio
        s.roll = msg.liveParameters.roll
        continue
      elif msg.which() == 'carControl':
        s.t_cc = msg.logMonoTime
        s.enabled = msg.carControl.enabled
        if hasattr(msg.carControl, "actuatorsOutput"):
          s.steer_cmd_out = msg.carControl.actuatorsOutput.steer
          s.gas_cmd_out = msg.carControl.actuatorsOutput.gas
          s.brake_cmd_out = msg.carControl.actuatorsOutput.brake
        s.steer_cmd = msg.carControl.actuators.steer
        s.gas_cmd = msg.carControl.actuators.gas
        s.brake_cmd = msg.carControl.actuators.brake
        s.desired_accel = msg.carControl.actuators.accel
        continue
      elif msg.which() == 'liveLocationKalman':
        s.t_llk = msg.logMonoTime
        s.yaw_rate = msg.liveLocationKalman.angularVelocityCalibrated.value[2]
        s.pitch = msg.liveLocationKalman.orientationNED.value[1]
      elif msg.which() == 'lateralPlan':
        s.t_lat_p = msg.logMonoTime
        # logs before 2021-05 don't have this field
        try:
          s.desired_curvature_rate = msg.lateralPlan.curvatureRates[0]
          s.desired_curvature = msg.lateralPlan.curvatures[0]
        except:
          s.desired_curvature_rate = 0
          s.desired_curvature = 0
        continue
      elif VM is None and msg.which() == 'carParams':
        CP = msg.carParams
        VM = VehicleModel(CP)
      else:
        continue

      # assert all messages have been received
      valid = not np.isnan(s.v_ego) and \
              not np.isnan(s.t_llk) and \
              not np.isnan(s.t_lat_p) and \
              not np.isnan(s.t_cc) and \
              VM is not None and \
              not np.isnan(s.t_lp)
      
      if valid:        
        # Check that the current sample time is not too far from the previous sample time.
        # If it is, reset the deque.
        if len(sample_deque) > 0:
          t_sample = getattr(s, t_interp_field)
          t_last = getattr(sample_deque[-1], t_interp_field)
          if (t_sample - t_last) * 1e-9 > max_t_delta:
            # print(f"Resetting deque at {t_sample} because t_sample - t_last = {(t_sample - t_last) * 1e-9} > {max_t_delta}")
            sample_deque.clear()
          else:
            # print(f"deque length = {len(sample_deque)}")
            sample_deque.append(s)
        else:
          # print(f"deque length = {len(sample_deque)}")
          sample_deque.append(s)
        
        # now interpolate a sample from the deque
        if len(sample_deque) == sample_deque.maxlen:
          # print(f"deque length = {len(sample_deque)}")
          cur_sample = sample_deque[deque_mid_idx]
          t_sample = getattr(cur_sample, t_interp_field)
          cur_sample.t = t_sample
          # need to do one interpolation for each field, using the field's
          # corresponsing time value.
          # This works by making a list of time values and then, for each field that shares the same
          # time value, interpolating the field value at that time.
          
          # we'll use a dict to organize this, where each key is the name of a time field, and the value
          # is a list of the fields that correspond to that time field.
          t_field_dict = {
            "t_cs": ["v_ego", "a_ego", "steer_angle", "steer_rate", "torque_eps", "torque_driver", "gas", "gas_pressed", "brake", "brake_pressed"],
            "t_lp": ["steer_offset", "steer_offset_average", "roll", "stiffnessFactor", "steerRatio"],
            "t_cc": ["enabled", "steer_cmd_out", "gas_cmd_out", "brake_cmd_out", "steer_cmd", "gas_cmd", "brake_cmd", "desired_accel"],
            "t_lat_p": ["desired_curvature_rate", "desired_curvature"],
            }
          for t_field, field_list in t_field_dict.items():
            t_field_list = [getattr(s, t_field) for s in sample_deque]
            for field in field_list:
              field_list = [getattr(s, field) for s in sample_deque]
              setattr(cur_sample, field, interp(t_sample, t_field_list, field_list))
          
          # now compute derived values
          cur_sample.car_make = CP.carName
          cur_sample.car_fp = CP.carFingerprint
          cur_sample.long_actuator_delay = (CP.longitudinalActuatorDelayUpperBound + CP.longitudinalActuatorDelayLowerBound) / 2
          VM.update_params(max(cur_sample.stiffnessFactor, 0.1), max(cur_sample.steerRatio, 0.1))
          current_curvature = -VM.calc_curvature(math.radians(s.steer_angle - s.steer_offset), s.v_ego, s.roll)
          current_curvature_rate = -VM.calc_curvature(math.radians(s.steer_rate), s.v_ego, 0.)
          cur_sample.lateral_accel = current_curvature * s.v_ego**2
          cur_sample.lateral_jerk = current_curvature_rate * s.v_ego**2
          cur_sample.curvature_device = (cur_sample.yaw_rate / s.v_ego) if s.v_ego > 0.01 else 0.
          cur_sample.lateral_accel_device = cur_sample.yaw_rate * s.v_ego  - (np.sin(s.roll) * ACCELERATION_DUE_TO_GRAVITY)
          cur_sample.desired_steer_angle = math.degrees(VM.get_steer_from_curvature(-s.desired_curvature, s.v_ego, s.roll))
          cur_sample.desired_steer_rate = math.degrees(VM.get_steer_from_curvature(-s.desired_curvature_rate, s.v_ego, 0))
            
          # then append the interpolated sample to the list of samples
          samples.append(deepcopy(cur_sample))
          
          # print out the interpolated sample as python dict
          # print(f"{cur_sample.__dict__}")
          
        s.v_ego = np.nan
          
          
    except:
      continue
  
  # print("message types found:\n" + ", ".join(list(type_set)))

  # # Terminated during valid section
  # if (section_end - section_start) * 1e-9 > MIN_SECTION_SECONDS:
  #   samples.extend(section)

  min_speed_reached = any([s.v_ego > 0.5 for s in samples])
  if len(samples) == 0 or not min_speed_reached:
    print(f'{len(samples)} samples found, {min_speed_reached = }')
    return np.array([])

  return np.array(samples)

def filter(samples):
  global MAX_DRIVER_TORQUE, MAX_EPS_TORQUE, MAX_SPEED, MIN_CURVATURE_RATE, MAX_CURVATURE_RATE, MAX_STEER_RATE, MIN_STEER_RATE
  # Order these to remove the most samples first
  
  
  # Some rlogs use [-300,300] for torque, others [-3,3]
  # Scale both from STEER_MAX to [-1,1]
  # steer_torque_key = "torque_eps" # gm
  # MAX_DRIVER_TORQUE = max(MAX_DRIVER_TORQUE, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MIN_CURVATURE_RATE = min(MIN_CURVATURE_RATE, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_CURVATURE_RATE = max(MAX_CURVATURE_RATE, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_STEER_RATE = min(MIN_STEER_RATE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_STEER_RATE = max(MAX_STEER_RATE, np.max(np.array([s.steer_rate for s in samples])))
  # for s in samples:
  #   if MAX_DRIVER_TORQUE > 40 or MAX_EPS_TORQUE > 40:
  #     s.torque_driver /= 300
  #     s.torque_eps /= 300
  #   else:
  #     s.torque_driver /= 3
  #     s.torque_eps /= 3
      
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # VW MQB cars, str cmd units 0.01Nm, 3.0Nm max
  # steer_torque_key = "steer_cmd" # vw
  # MAX_DRIVER_TORQUE = max(MAX_DRIVER_TORQUE, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  # MIN_CURVATURE_RATE = min(MIN_CURVATURE_RATE, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_CURVATURE_RATE = max(MAX_CURVATURE_RATE, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_STEER_RATE = min(MIN_STEER_RATE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_STEER_RATE = max(MAX_STEER_RATE, np.max(np.array([s.steer_rate for s in samples])))
  # for s in samples:
  #   s.torque_driver /= 300
    
  # VW PQ cars {CAR.PASSAT_NMS, CAR.SHARAN_MK2}, str cmd units 0.01Nm, 3.0Nm max
  # steer_torque_key = "steer_cmd" # vw
  # MAX_DRIVER_TORQUE = max(MAX_DRIVER_TORQUE, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MIN_CURVATURE_RATE = min(MIN_CURVATURE_RATE, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_CURVATURE_RATE = max(MAX_CURVATURE_RATE, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_STEER_RATE = min(MIN_STEER_RATE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_STEER_RATE = max(MAX_STEER_RATE, np.max(np.array([s.steer_rate for s in samples])))
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
  steer_torque_key = "torque_eps" # vw
  MAX_DRIVER_TORQUE = max(MAX_DRIVER_TORQUE, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  MIN_CURVATURE_RATE = min(MIN_CURVATURE_RATE, np.min(np.array([s.desired_curvature_rate for s in samples])))
  MAX_CURVATURE_RATE = max(MAX_CURVATURE_RATE, np.max(np.array([s.desired_curvature_rate for s in samples])))
  MIN_STEER_RATE = min(MIN_STEER_RATE, np.min(np.array([s.steer_rate for s in samples])))
  MAX_STEER_RATE = max(MAX_STEER_RATE, np.max(np.array([s.steer_rate for s in samples])))
  for s in samples:
    s.torque_driver /= 100 * 4
    s.torque_eps /= 5 * 4
  MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # MAX_DRIVER_TORQUE = max(MAX_DRIVER_TORQUE, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # all chrysler and ram 1500: steer scaled to 261. 361 for ram hd
  # steer_torque_key = "torque_eps" # vw
  # MAX_DRIVER_TORQUE = max(MAX_DRIVER_TORQUE, np.max(np.abs(np.array([s.torque_driver for s in samples]))))
  # MAX_EPS_TORQUE = max(MAX_EPS_TORQUE, np.max(np.abs(np.array([getattr(s, steer_torque_key) for s in samples]))))
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  # MIN_CURVATURE_RATE = min(MIN_CURVATURE_RATE, np.min(np.array([s.desired_curvature_rate for s in samples])))
  # MAX_CURVATURE_RATE = max(MAX_CURVATURE_RATE, np.max(np.array([s.desired_curvature_rate for s in samples])))
  # MIN_STEER_RATE = min(MIN_STEER_RATE, np.min(np.array([s.steer_rate for s in samples])))
  # MAX_STEER_RATE = max(MAX_STEER_RATE, np.max(np.array([s.steer_rate for s in samples])))
  # for s in samples:
  #   s.torque_driver /= 261
  #   s.torque_eps /= 261
  # MAX_SPEED = max(MAX_SPEED, np.max(np.array([s.v_ego for s in samples])))
  
  # print(f'max eps torque = {eps:0.4f}')
  # print(f"max driver torque = {driver:0.4f}")
  
  # Enabled and no steer pressed or not enabled and driver steer under threshold
  mask = np.array([(s.enabled and s.torque_driver <= STEER_PRESSED_MIN) or (not s.enabled and s.torque_driver <= STEER_PRESSED_MAX) for s in samples])
  samples = samples[mask]
  
  if not IS_ANGLE_PLOT:
    mask = np.array([abs(s.torque_driver + getattr(s, steer_torque_key)) > 0.25 * abs(s.lateral_accel) for s in samples])
    samples = samples[mask]
  
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
  ext = '.lat'
  latpath = None
  allpath = None

  if not path and not route:
    exit(1)
  if path is not None:
    allpath = os.path.join(path, 'all.lat')
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
        for filename in tqdm(os.listdir(path)):
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
                print(f"failed to load lat file: {latpath}\n{e}")
      if PREPROCESS_ONLY:
        if "realdata" in path or (preprocess and dongleid):
          MULTI_FILE = True
          # we're on device going through rlogs
          if preprocess and dongleid:
            dongle_id = dongleid
            rlog_path = "/Users/haiiro/Downloads/latfiles_batch"
            rlog_log_path = "/Users/haiiro/Downloads/latfiles.txt" # prevents rerunning rlogs
          else:
            with open("/data/params/d/DongleId","r") as df:
              dongle_id = df.read()
            rlog_path = "/data/media/0/latfiles"
            rlog_log_path = "/data/media/0/latfiles.txt" # prevents rerunning rlogs
          print(f"{dongle_id = }")
          if not os.path.exists(rlog_path):
            os.mkdir(rlog_path)
          latsegs = set([f for f in os.listdir(rlog_path) if ".lat" in f])
          if os.path.exists(rlog_log_path): 
            # read in lat files saved by running `ls -1 /data/media/0/latfiles >> /data/media/0/latfiles.txt`
            with open(rlog_log_path, 'r') as rll:
              latsegs = latsegs | set(list(rll.read().split('\n')))
          with open(rlog_log_path, 'w') as rll:
            for ls in sorted(list(latsegs)):
              rll.write(f"\n{ls}")
          filenames = sorted([filename for filename in os.listdir(path) if len(filename.split('--')) == 3 and f"{dongle_id}|{filename}.lat" not in latsegs])
          print(f"Preparing fit data from {len(filenames)} rlog segments")
          for filename in tqdm(filenames, desc="Preparing fit data from rlogs"):
            if len(filename.split('--')) == 3 and f"{dongle_id}|{filename}.lat" not in latsegs:
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
                    with open(os.path.join(rlog_path, f"{route}--{seg_num}.lat"), 'wb') as f:
                      pickle.dump(data1, f)
                except Exception as e:
                  print(f"Failed to load segment file {filename}: {e}")
                  continue
                finally:
                  with open(rlog_log_path, 'a') as rll:
                    rll.write(f"\n{route}--{seg_num}.lat")
        else:
          # first make per-segment .lat files
          # get previously completed segments
          print("Batch processing rlogs into lat files")
          if outpath == "": outpath = path
          print(f"{outpath = }")
          latsegs = set()
          if not os.path.exists(outpath):
            os.mkdir(outpath)
          rlog_log_path = os.path.join(outpath, "latfiles.txt") # prevents rerunning rlogs
          if os.path.exists(rlog_log_path): 
            print(f"Loading file blacklist: {rlog_log_path}")
            # read in lat files saved by running `ls -1 /data/media/0/latfiles >> /data/media/0/latfiles.txt`
            with open(rlog_log_path, 'r') as rll:
              latsegs = latsegs | set(list(rll.read().split('\n')))
          else:
            Path(rlog_log_path).touch()
          rlog_log_mdate = os.path.getmtime(rlog_log_path)
          with open(rlog_log_path, 'w') as rll:
            for ls in sorted(list(latsegs)):
              rll.write(f"\n{ls}")
          for filename in os.listdir(outpath):
            if ext in filename.split("/")[-1]:
              p1 = filename.replace(outpath, path).replace('.lat','--rlog.bz2').replace('|','_')
              p2 = filename.replace(outpath, path).replace('.lat','--rlog.bz2')
              if rlog_log_mdate and \
                (os.path.exists(p1) and os.path.getmtime(p1) > rlog_log_mdate) or \
                (os.path.exists(p2) and os.path.getmtime(p2) > rlog_log_mdate):
                for p in [p1,p2]: 
                  if p in latsegs: del(latsegs[p])
              else:
                latsegs.add(p1)
                latsegs.add(p2)
          print(f"{len(latsegs)//2} blacklisted files")
          filenames = sorted([filename for filename in os.listdir(path) if filename.endswith("rlog.bz2") and filename not in latsegs])
          def process_file(filename):
            if len(filename.split('--')) == 4 and filename.endswith('rlog.bz2'):
              seg_num = filename.split('--')[2]
              route='--'.join(filename.split('--')[:2]).replace('_','|')
              latfile = os.path.join(outpath, f"{route}--{seg_num}.lat")
              if filename not in latsegs:
                # print(f'loading rlog segment {fi} of {num_files} {filename}')
                with tempfile.TemporaryDirectory() as d:
                  try:
                    shutil.copy(os.path.join(path,filename),os.path.join(d,filename))
                    r = Route(route, data_dir=d)
                    lr = MultiLogIterator(r.log_paths(), sort_by_time=True)
                    data1 = collect(lr)
                    # print(f"{len(data1)} points in {filename}")
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
          p_map(process_file, filenames, desc="Preparing fit data from rlogs")
          if preprocess:
            for filename in filenames:
                latsegs.add(filename.replace(outpath, path).replace('.lat','--rlog.bz2').replace('|','_'))
                latsegs.add(filename.replace(outpath, path).replace('.lat','--rlog.bz2'))
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
        print(f"max driver torque = {MAX_DRIVER_TORQUE:0.4f}")
        print(f"max speed = {MAX_SPEED*2.24:0.4f} mph")
        print(f"min curvature rate = {MIN_CURVATURE_RATE:0.4f}")
        print(f"max curvature rate = {MAX_CURVATURE_RATE:0.4f}")
        print(f"min steer rate = {MIN_STEER_RATE:0.4f}")
        print(f"max steer rate = {MAX_STEER_RATE:0.4f}")
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
              with open(os.path.join(path, f"{route}.lat"), 'wb') as f:
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
  
  # define debug args using '      python3 /home/haiiro/openpilot-batch/openpilot/tools/tuning/lat.py --preprocess --path "$curdir/$dongle" --outpath "$outdir"'
  # debug_args = ['--preprocess', '--path', '/Volumes/video/scratch-video/rlogs/gm/CHEVROLET VOLT PREMIER 2017/2e983c9898739c34', ]
  
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

  if args.preprocess:
    exit(0)
  fit(speed, angle, steer, IS_ANGLE_PLOT)
  plot(speed, angle, steer)
