#!/usr/bin/env python3

# f(angle, velocity) = steer command
# Only use units of steer: [-1,1], m/s, and degrees

# remove points with poor distribution: std() threshold? P99?

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

from tools.tuning.lat_settings import *
if not PREPROCESS_ONLY:
  from scipy.stats import describe
  from scipy.signal import correlate, correlation_lags
  import matplotlib.pyplot as plt
  from tools.tuning.lat_plot import fit, plot

from selfdrive.config import Conversions as CV
from tools.lib.logreader import MultiLogIterator
from tools.lib.route import Route



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

  lrd = dict()
  for msg in lr:
    msgid = f"{msg.logMonoTime}:{msg.which()}"
    if msgid in lrd:
      break
    lrd[msgid] = msg
  lr1 = list(lrd.values())
  print(f"{len(lr1)} messages")
  for msg in tqdm(sorted(lr1, key=lambda msg: msg.logMonoTime)):
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


  if len(samples) == 0:
    return np.array([])

  return np.array(samples)

def filter(samples):
  # Order these to remove the most samples first
  
  # Some rlogs use [-300,300] for torque, others [-3,3]
  # Scale both from STEER_MAX to [-1,1]
  driver = np.max(np.abs(np.array([s.torque_driver for s in samples])))
  eps = np.max(np.abs(np.array([s.torque_eps for s in samples])))
  one_over_three = 1. / 3.
  one_over_three_hundred = 1. / 300.
  for s in samples:
    if driver > 10 or eps > 10:
      s.torque_driver *= one_over_three_hundred
      s.torque_eps *= one_over_three_hundred
    else:
      s.torque_driver *= one_over_three
      s.torque_eps *= one_over_three
  print(f'max eps torque = {eps:0.4f}')
  print(f"max driver torque = {driver:0.4f}")
  
  # No steer pressed
  # data = np.array([s.torque_driver for s in samples])
  # mask = np.abs(data) < STEER_PRESSED_MIN
  # samples = samples[mask]

  # Enabled
  # mask = np.array([s.enabled for s in samples])
  # samples = samples[mask]

  # No steer rate: holding steady curve or straight
  # data = np.array([s.curvature_rate for s in samples])
  # mask = np.abs(data) < 0.003 # determined from plotjuggler
  # samples = samples[mask]

  # No steer rate: holding steady curve or straight
  # data = np.array([s.steer_rate for s in samples])
  # mask = np.abs(data) < STEER_RATE_MIN
  # samples = samples[mask]

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
  data = []
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
      print(f"Failed to load segment file {filename}:\n{e}")
      
  # Only path
  else:
    if latpath and os.path.isfile(latpath):
      data = filter(load_cache(latpath))
    elif os.path.isfile(allpath):
      data = filter(load_cache(allpath))
    else:
      print(f'Loading many in {path}')
      data = []
      dataraw = []
      routes = set()
      latroutes = set()
      for filename in os.listdir(path):
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
            data.extend(filter(load_cache(latpath)))
            dataraw.extend(load_cache(latpath))
      if PREPROCESS_ONLY:
        if "/data/media/0/realdata" in path:
          # we're on device going through rlogs
          with open("/data/params/d/DongleId","r") as df:
            dongle_id = df.read()
          print(f"{dongle_id = }")
          rlog_path = "/data/media/0/latfiles"
          if not os.path.exists(rlog_path):
            os.mkdir(rlog_path)
          latsegs = set([f for f in os.listdir(rlog_path) if ".lat" in f])
          num_files = len([None for filename in os.listdir(path) if len(filename.split('--')) == 3 and f"{dongle_id}|{filename}.lat" not in latsegs])
          fi = 0
          for filename in os.listdir(path):
            if len(filename.split('--')) == 3 and f"{dongle_id}|{filename}.lat" not in latsegs:
              fi += 1
              print(f"\nloading segment {fi} of {num_files}: {dongle_id}|{filename}")
              with tempfile.TemporaryDirectory() as d:
                if os.path.exists(os.path.join(path,filename,"rlog")):
                  print("found raw rlog")
                  shutil.copy(os.path.join(path,filename,"rlog"),os.path.join(d,f"{dongle_id}_{filename}--rlog"))
                elif os.path.exists(os.path.join(path,filename,"rlog.bz2")):
                  print("found bz2 rlog")
                  tmpbz2 = os.path.join(d,f"{dongle_id}_{filename}--rlog.bz2")
                  shutil.copy(os.path.join(path,filename,"rlog.bz2"),tmpbz2)
                  # os.system(f"bzip2 -d {tmpbz2}")
                else:
                  print("rlog not found")
                try:
                  route='--'.join(f"{dongle_id}|{filename}".split('--')[:2])
                  r = Route(route, data_dir=d)
                  print("route loaded")
                  lr = MultiLogIterator([lp for lp in r.log_paths() if lp])
                  print("log iterator loaded")
                  data1 = collect(lr)
                  print("log data preprocessed")
                  if len(data1):
                    seg_num = f"{dongle_id}|{filename}".split('--')[2]
                    with open(os.path.join(rlog_path, f"{route}--{seg_num}.lat"), 'wb') as f:
                      pickle.dump(data1, f)
                except Exception as e:
                    print(f"Failed to load segment file {filename}:\n{e}")
          else:
            # first make per-segment .lat files
            # get previously completed segments
            latsegs = set()
            for filename in os.listdir(path):
              if len(filename.split('--')) == 3 and filename.endswith('.lat'):
                latsegs.add(filename.replace('.lat','--rlog.bz2').replace('|','_'))
            num_files = len([ None for filename in os.listdir(path) if len(filename.split('--')) == 4 and filename.endswith('rlog.bz2') and filename not in latsegs ])
            fi=0
            for filename in os.listdir(path):
              if len(filename.split('--')) == 4 and filename.endswith('rlog.bz2'):
                if filename not in latsegs:
                  fi+=1
                  print(f'loading rlog segment {fi} of {num_files} {filename}')
                  with tempfile.TemporaryDirectory() as d:
                    try:
                      shutil.copy(os.path.join(path,filename),os.path.join(d,filename))
                      route='--'.join(filename.split('--')[:2]).replace('_','|')
                      r = Route(route, data_dir=d)
                      lr = MultiLogIterator(r.log_paths(), sort_by_time=True)
                      data1 = collect(lr)
                      if len(data1):
                        seg_num = filename.split('--')[2]
                        with open(os.path.join(path, f"{route}--{seg_num}.lat"), 'wb') as f:
                          pickle.dump(data1, f)
                      os.remove(os.path.join(path,filename))
                    except Exception as e:
                      print(f"Failed to load segment file {filename}:\n{e}")
                else:
                  os.remove(os.path.join(path,filename))
      else:
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
              dataraw.extend(data1)
          except Exception as e:
            print(f"Failed to load segment file {filename}:\n{e}")
              
      # write all.dat
      # if len(dataraw):
      #   with open(allpath, 'wb') as f:
      #     pickle.dump(dataraw, f)
  if PREPROCESS_ONLY:
    exit(0)
  
  oldlen = len(dataraw)
  newlen = len(data)
  if not os.path.isdir('plots'):
    os.mkdir('plots')
  with open('plots/out.txt','w') as f:
    if oldlen > 0 and newlen > 0:
      f.write(f"{oldlen} points filtered down to {newlen}\n")
    else:
      f.write(f"{newlen} filtered points\n")

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
