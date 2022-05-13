#!/usr/bin/env python3

# f(angle, velocity) = steer command
# Only use units of steer: [-1,1], m/s, and degrees

# remove points with poor distribution: std() threshold? P99?

import argparse
import os
import pickle
from copy import deepcopy
from typing import NamedTuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import correlate, correlation_lags
# import seaborn as sns
from tqdm import tqdm  # type: ignore
from scipy.stats import describe

from selfdrive.config import Conversions as CV
from tools.lib.logreader import MultiLogIterator
from tools.lib.route import Route
from tools.tuning.lat_plot import fit, plot
from tools.tuning.lat_settings import *

# Reduce samples using binning and outlier rejection
def regularize(speed, angle, steer):
  print("Regularizing...")
  # Bin by rounding
  speed_bin = np.around(speed*2)/2
  angle_bin = np.around(angle*2)/2

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
        speed[i] = np.mean(speed[inliers])
        angle[i] = np.mean(angle[inliers])
        steer[i] = np.mean(steer[inliers])

        count.append(c)
        std.append(s)
        mask[i] = False
        i += 1

      # Remove samples
      speed = speed[~mask]
      angle = angle[~mask]
      steer = steer[~mask]
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
  steer_angle: float = np.nan
  steer_rate: float = np.nan
  steer_offset: float = np.nan
  steer_offset_average: float = np.nan
  torque_eps: float = np.nan # -1,1
  torque_driver: float = np.nan # -1,1
  # curvature_plan: float = np.nan # lag
  # curvature_true: float = np.nan # lag
  curvature_rate: float = np.nan

class CleanSample(NamedTuple):
  angle: float = np.nan
  speed: float = np.nan
  steer: float = np.nan

def collect(lr):
  s = Sample()
  samples: list[Sample] = []
  section: list[Sample] = []

  section_start: int = 0
  section_end: int = 0
  last_msg_time: int = 0

  for msg in tqdm(sorted(lr, key=lambda msg: msg.logMonoTime)):
    # print(f'{msg.which() = }')
    if msg.which() == 'carState':
      s.v_ego  = msg.carState.vEgo
      s.steer_angle = msg.carState.steeringAngleDeg
      s.steer_rate = msg.carState.steeringRateDeg
      s.torque_eps = msg.carState.steeringTorqueEps
      s.torque_driver = msg.carState.steeringTorque
    elif msg.which() == 'liveParameters':
      s.steer_offset = msg.liveParameters.angleOffsetDeg
      s.steer_offset_average = msg.liveParameters.angleOffsetAverageDeg
      continue
    elif msg.which() == 'carControl':
      s.enabled = msg.carControl.enabled
      continue
    elif msg.which() == 'lateralPlan':
      # logs before 2021-05 don't have this field
      try:
        s.curvature_rate = msg.lateralPlan.curvatureRates[0]
      except:
        s.curvature_rate = 0
      continue
    else:
      continue

    # assert all messages have been received
    valid = not np.isnan(s.v_ego) and \
            not np.isnan(s.steer_offset) and \
            not np.isnan(s.curvature_rate)
    
    # if valid:
    #   print(f"{s.v_ego = :0.3f}\t{s.steer_angle = :0.3f}\t{s.steer_rate = :0.3f}\t{s.torque_driver = :0.3f}\t{s.torque_eps = :0.3f}")
    # else:
    #   print("invalid")
    #   pass

    # assert continuous section
    if last_msg_time:
      valid = valid and 0.1 > abs(msg.logMonoTime - last_msg_time) * 1e-9
    last_msg_time = msg.logMonoTime

    if valid:
      section.append(deepcopy(s))
      s.v_ego = np.nan
      section_end = msg.logMonoTime
      if not section_start:
        section_start = msg.logMonoTime
    elif section_start:
      # end of valid section
      if (section_end - section_start) * 1e-9 > MIN_SECTION_SECONDS:
        samples.extend(section)
      section = []
      section_start = section_end = 0

  # Terminated during valid section
  if (section_end - section_start) * 1e-9 > MIN_SECTION_SECONDS:
    samples.extend(section)

  # Some rlogs use [-300,300] for torque, others [-3,3]
  # Scale both from STEER_MAX to [-1,1]
  driver = np.max(np.abs(np.array([s.torque_driver for s in samples])))
  eps = np.max(np.abs(np.array([s.torque_eps for s in samples])))
  for s in samples:
    if driver > 10:
      s.torque_driver /= 300
    else:
      s.torque_driver /= 3
    if eps > 10:
      s.torque_eps /= 300
  if eps > 10:
    print('eps > 10')
  else:
    print(f'eps < 10 wtf:{eps = } !!!\n{len(samples) = }')
  if driver > 10:
    print('driver > 10')
  else:
    print(f'!!! driver < 10 wtf:{driver = } !!!')

  return np.array(samples)

def filter(samples):
  # Order these to remove the most samples first
  
  
  # No steer pressed
  data = np.array([s.torque_driver for s in samples])
  mask = np.abs(data) < STEER_PRESSED_MIN
  samples = samples[mask]

  # Enabled
  mask = np.array([s.enabled for s in samples])
  samples = samples[mask]

  # No steer rate: holding steady curve or straight
  # data = np.array([s.curvature_rate for s in samples])
  # mask = np.abs(data) < 0.003 # determined from plotjuggler
  # samples = samples[mask]

  # No steer rate: holding steady curve or straight
  data = np.array([s.steer_rate for s in samples])
  mask = np.abs(data) < STEER_RATE_MIN
  samples = samples[mask]

  # GM no steering below 7 mph
  data = np.array([s.v_ego for s in samples])
  mask = SPEED_MIN * CV.MPH_TO_MS <= data
  mask &= data <= SPEED_MAX * CV.MPH_TO_MS
  samples = samples[mask]

  # Not saturated
  # data = np.array([s.torque_eps for s in samples])
  # mask = np.abs(data) < 1.0
  # samples = samples[mask]

  return [CleanSample(
    speed = s.v_ego,
    angle = s.steer_angle - s.steer_offset,
    steer = (s.torque_driver + s.torque_eps)
  ) for s in samples]

def load_cache(path):
  print(f'Loading {path}')
  try:
    with open(path,'rb') as file:
      return pickle.load(file)
  except Exception as e:
    print(e)

def load(path, route=None):
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

  if route:
    print(f'Loading from rlogs {route}')
    r = Route(route, data_dir=path)
    lr = MultiLogIterator(r.log_paths(), sort_by_time=True)#, wraparound=False)
    data = collect(lr)

    if len(data):
      with open(latpath, 'wb') as f: # cache
        pickle.dump(data, f)
    data = filter(data)
    if PREPROCESS_ONLY:
      exit(0)
  # Only path
  else:
    if latpath and os.path.isfile(latpath):
      data = filter(load_cache(latpath))
    elif os.path.isfile(allpath):
      data = load_cache(allpath)
    else:
      print(f'Loading many cached in {path}')
      data = []
      for filename in os.listdir(path):
        if filename.endswith(ext):
          latpath = os.path.join(path, filename)
          data.extend(filter(load_cache(latpath)))
      # write all.dat
      if len(data):
        with open(allpath, 'wb') as f:
          pickle.dump(data, f)

  speed = np.array([sample.speed for sample in data])
  angle = np.array([sample.angle for sample in data])
  steer = np.array([sample.steer for sample in data])
  print(f'Samples: {len(speed)}')
  return speed, angle, steer


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--path')
  parser.add_argument('--route')
  args = parser.parse_args()

  regfile = 'regularized'
  if REGULARIZED and os.path.isfile(regfile):
    print("Opening regularized data")
    with open(regfile,'rb') as file:
      speed, angle, steer = pickle.load(file)
  else:
    print("Loading new data")
    speed, angle, steer = load(args.path, args.route)
    speed, angle, steer = regularize(speed, angle, steer)
    with open(regfile, 'wb') as f:
      pickle.dump([speed, angle, steer], f)

  fit(speed, angle, steer)
  plot(speed, angle, steer)
