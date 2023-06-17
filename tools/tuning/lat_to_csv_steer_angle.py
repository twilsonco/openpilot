#!/usr/bin/env python3
import gc
import os
import re
import bisect
import pickle
import datetime
import pandas as pd
import copy
from tqdm import tqdm  # type: ignore
import numpy as np
from scipy.stats import describe
from sklearn.cluster import DBSCAN
from sklearn.mixture import GaussianMixture
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import LocalOutlierFactor
import math
import random
from collections import deque, defaultdict
from common.numpy_fast import interp
import pyarrow.feather as feather
import selfdrive.car.toyota.values as toyota
from matplotlib import pyplot as plt
from matplotlib.cm import ScalarMappable

def smooth_list(l, window_size):
    """
    Computes a smoothed copy of a list of floats using a moving average.
    
    Parameters:
        l (list): The list of floats to be smoothed.
        window_size (int): The size of the moving window to use for the moving average.
        
    Returns:
        A new list of floats that is a smoothed copy of the input list.
    """
    if not l:
        return []
    if window_size == 1:
        return l

    # Pad the list with zeros at the beginning and end to avoid edge effects
    pad = [0] * (window_size // 2)
    padded_list = pad + l + pad

    # Compute the moving average
    smoothed_list = []
    for i in range(window_size // 2, len(padded_list) - window_size // 2):
        window = padded_list[i - window_size // 2:i + window_size // 2 + 1]
        smoothed_list.append(sum(window) / float(window_size))

    return smoothed_list

def sign(x):
  if x < 0.0:
    return -1.0
  elif x > 0.0:
    return 1.0
  else:
    return 0.0

def lookahead_lookback_filter(samples, comp_func, n_forward = 0, n_back = 0):
  return [s for i,s in enumerate(samples) if \
    i > n_back and i < len(samples) - n_forward and \
    all([comp_func(s1) for s1 in samples[max(0,i-n_back):min(len(samples), i+n_forward+1)]])]

def human_readable(number):
    for unit in ['', 'K', 'M', 'B', 'T', 'P']:
        if abs(number) < 1000:
            return f"{number:.1f}{unit}" if int(number) != number else f"{int(number)}{unit}"
        number /= 1000
    return f"{number:.1f}E"

def is_file_modified_within_last_two_weeks(file_path):
    # get the modification time of the file
    if not os.path.exists(file_path):
      return False
    
    mod_time = os.path.getmtime(file_path)

    # convert the modification time to a datetime object
    mod_datetime = datetime.datetime.fromtimestamp(mod_time)

    # calculate the difference between the modification time and the current time
    delta = datetime.datetime.now() - mod_datetime

    # check if the difference is less than or equal to 14 days (2 weeks)
    if delta.days <= 14:
        return True
    else:
        return False


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
  lateral_jerk_w_roll: float = np.nan
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
  t: float = np.nan

class CleanLatSample():
  def __init__(self, s, route = ""):
    self.v_ego = float(s.v_ego)
    self.a_ego = float(s.a_ego)
    self.torque_eps = float(s.torque_eps)
    self.torque_driver = float(s.torque_driver)
    self.torque_adjusted: float = 0.0
    self.steer_cmd = float(s.steer_cmd)
    self.steer_angle = float(s.steer_angle)
    self.steer_rate = float(s.steer_rate)
    self.lateral_accel = float(s.lateral_accel)
    self.lateral_jerk = float(s.lateral_jerk)
    self.roll = float(s.roll)
    self.car_fp = s.car_fp
    self.car_make = s.car_make
    self.t = s.t
    self.enabled: bool = s.enabled
    self.route: str = route

def describe_to_string(describe_output):
    nobs, minmax, mean, var, skew, kurtosis = describe_output
    min_val, max_val = minmax
    return f"nobs={nobs:0.1f}, min_max=({min_val:0.1f}, {max_val:0.1f}), mean={mean:0.1f}, var={var:0.1f}, skew={skew:0.1f}, kurtosis={kurtosis:0.1f}"

def insert_and_merge(intervals, new_interval):
    index = bisect.bisect_left(intervals, new_interval)
    intervals.insert(index, new_interval)

    # Merge intervals with the previous interval if they overlap
    if index > 0 and intervals[index - 1][1] >= intervals[index][0]:
        intervals[index - 1] = (intervals[index - 1][0], max(intervals[index - 1][1], intervals[index][1]))
        intervals.pop(index)
        index -= 1

    # Merge intervals with the next interval if they overlap
    while index < len(intervals) - 1 and intervals[index][1] >= intervals[index + 1][0]:
        intervals[index] = (intervals[index][0], max(intervals[index][1], intervals[index + 1][1]))
        intervals.pop(index + 1)

def compute_adjusted_steer_torque(samples, eps_stats, driver_stats):
  blacklist_neighbor_secs = 0.5
  max_ratio = 30.0
  max_abs_long_accel = 0.8
  # check_func = lambda s: abs(s.torque_driver) < 0.15 and abs(s.a_ego) < max_abs_long_accel
  check_func = lambda s: not s.enabled and abs(s.torque_eps) == 0.0 and abs(s.a_ego) < max_abs_long_accel
  nlalb = 5
  torque_func = lambda s: s.torque_eps
  recip_eps = 1.0
  recip_driver = 1.0
  if len(samples) == 0:
    return [], 0.0
  if samples[0].car_make == 'gm':
    check_func = lambda s: abs(s.a_ego) < max_abs_long_accel and \
      ((s.enabled and abs(s.torque_driver) < 0.05 and \
        abs(s.steer_angle) <= 2.0 or \
        (s.v_ego > 12.0 or abs(s.steer_angle) / max(0.001, abs(s.torque_adjusted)) < 100)) \
       or \
      (not s.enabled and s.torque_eps == 0.0))
    recip_driver = 1.0 / 3.0
    recip_eps = 1.0 / 3.0
    torque_func = lambda s: s.torque_driver + s.torque_eps
  elif samples[0].car_make == 'volkswagen':
    torque_func = lambda s: s.torque_driver
    check_func = lambda s: not s.enabled and s.steer_cmd == 0.0 and abs(s.torque_eps) == 0.0 and abs(s.a_ego) < max_abs_long_accel
    recip_driver = 1.0 / 300.0
  elif samples[0].car_make == 'hyundai':
    check_func = lambda s: abs(s.a_ego) < max_abs_long_accel
    if any([k in samples[0].car_fp for k in ["KIA", "IONIQ", "GENESIS"]]):
      recip_driver = 1.0 / 600.0
    else:
      recip_driver = 1.0 / 400.0
    recip_eps = 1.0 / 15.0
    if any([k in samples[0].car_fp for k in ["SONATA"]]):
      check_func = lambda s: (abs(s.lateral_accel) / max(0.001, abs(s.torque_adjusted)) < 15) or (abs(s.steer_angle) <= 2.0) and abs(s.a_ego) < max_abs_long_accel
      nlalb = 0
  elif samples[0].car_make == 'chrysler':
    recip_driver = 1.0 / 361.0
    recip_eps = 1.0 / 300.0
    check_func = lambda s: abs(s.a_ego) < max_abs_long_accel
  elif samples[0].car_make == 'toyota':
    # check_func = lambda s: s.enabled and abs(s.a_ego) < 2.0 and (abs(s.lateral_accel) / max(0.001, abs(s.torque_adjusted)) < 25 or (abs(s.lateral_accel) <= 0.3))
    check_func = lambda s: abs(s.a_ego) < max_abs_long_accel and \
      (s.enabled and abs(s.torque_driver) < 0.05 and \
        abs(s.steer_angle) <= 2.0 or \
        (abs(s.steer_angle) / max(0.001, abs(s.torque_adjusted)) < 100))
    nlalb = 0
    if any([k in samples[0].car_fp for k in ["COROLLA HYBRID"]]):
      check_func = lambda s: not s.enabled and (abs(s.lateral_accel) / max(0.001, abs(s.torque_adjusted)) < 8) or (abs(s.steer_angle) <= 2.0) and abs(s.a_ego) < max_abs_long_accel
    recip_driver = 1 / 400.0
    recip_eps = 1 / 1500.0
  elif samples[0].car_make == 'honda':
    torque_func = lambda s: s.torque_driver
    check_func = lambda s: not s.enabled and abs(s.a_ego) < max_abs_long_accel
    recip_driver = 1.0 / 3000.0
  elif samples[0].car_make == 'subaru':
    check_func = lambda s: abs(s.a_ego) < max_abs_long_accel
    recip_driver = 1.0 / 300.0
    recip_eps = 1.0 / 1000.0
  elif samples[0].car_make == 'ford':
    recip_driver = 1.0 / 3.5
    torque_func = lambda s: s.torque_driver
  elif samples[0].car_make == 'mazda':
    check_func = lambda s: not s.enabled and abs(s.a_ego) < max_abs_long_accel
    torque_func = lambda s: s.torque_driver
    recip_driver = 1.0 / 20.0
    
  for s in samples:
    s.torque_driver *= recip_driver
    s.torque_eps *= recip_eps
    s.torque_adjusted = torque_func(s)
    
  mean_eps = eps_stats[2]
  std_eps = np.sqrt(eps_stats[3])
  mean_driver = driver_stats[2]
  std_driver = np.sqrt(driver_stats[3])
  
  mean_torque_adjusted = mean_driver
  std_torque_adjusted = std_driver

  # blacklist = []
  # blacklist_neighbor_secs = 0.5 * 1e9
  # blacklist_neighbor_secs_half = blacklist_neighbor_secs * 0.5

  # for s in samples:
  #     if (abs(s.torque_adjusted) > 0.01 and
  #         abs(s.lateral_accel) / abs(s.torque_adjusted) > max_ratio):
  #       interval = (s.t - blacklist_neighbor_secs_half, s.t + blacklist_neighbor_secs_half)
  #       insert_and_merge(blacklist, interval)

  # filtered_samples = []
  # for s in samples:
  #     if all(s.t < interval[0] or s.t > interval[1] for interval in blacklist):
  #         filtered_samples.append(s)
  # samples = filtered_samples
  
  # samples = [s for s in samples if (sign(s.torque_adjusted) == sign(s.lateral_accel)) or ((sign(s.torque_adjusted) != sign(s.lateral_accel)) and (abs(s.lateral_accel) <= 1.2))]
  # a_ego = smooth_list([s.a_ego for s in samples], 51)
    
  # # print  10 raw and smoothed a_ego values
  # print("a_ego raw: ", [s.a_ego for s in samples[100:110]])
  # print(f"a_ego smoothed: {a_ego[100:110]}")
  
  # for s in samples:
  #   s.route = ""
    # s.a_ego = a_ego.pop(0)
    
  
  
  samples = lookahead_lookback_filter(samples, check_func, nlalb, nlalb)
  
  return samples, recip_driver


torque_eps_key = defaultdict(lambda: "torque_eps", {'volkswagen': 'steer_cmd'})
    
def pickle_files_to_csv(input_dir, check_modified=True, print_stats=False, save_output=True):
    # List all pickle files in the input directory
    carname = os.path.basename(input_dir)
    pickle_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.lat')])
    
    data = []
    
    make = ""
    model = ""
    
    for pickle_file in pickle_files:
      with open(os.path.join(input_dir, pickle_file), 'rb') as f:
        try:
          pk = pickle.load(f)
        except:
          continue
        for s in pk:
          if make == "" and s.car_make != "":
            make = s.car_make
            model = s.car_fp
            break
        if make != "":
          break
    
    outfile = os.path.join(input_dir,f"{model}.feather")
    if check_modified and is_file_modified_within_last_two_weeks(outfile):
      return

    # Iterate through the pickle files and load the data
    i=0
    # random.shuffle(pickle_files)
    print("Loading pickle files...")
    columns = [
      'v_ego',
      # 'a_ego',
      'lateral_accel',
      'lateral_jerk',
      'steer_angle',
      'steer_rate',
      'roll', # actually lateral gravitational acceleration
      'steer_cmd',
      # "lateral_accel_1",
      # "lateral_jerk_1",
      # "roll_1"
      ]
    
    samples = []
    random.shuffle(pickle_files)
    for pickle_file in tqdm(pickle_files):
      with open(os.path.join(input_dir, pickle_file), 'rb') as f:
        try:
          pk = pickle.load(f)
        except:
          continue
        for s in pk:
          if s.v_ego < 0.1:
            continue
          samples.append(CleanLatSample(s, pickle_file))
          if print_stats:
            samples[-1].v_ego *= 2.24
        if i > 200:
          # break
          pass
        i += 1
    
    if len(samples) == 0:
      return
        
    eps = [s.torque_eps for s in samples]
    steer_angle = [s.steer_angle for s in samples]
    driver = [s.torque_driver for s in samples]
    
    eps_stats = describe(eps)
    driver_stats = describe(driver)
    steer_angle_stats = describe(steer_angle)
      
    print(f"Length of samples: {len(samples)}")
        
    if print_stats and len(samples) > 0:
      
      print("Preparing plots")
      
      # want to color points by dongle id, so get index of each point in unique list of dongle ids
      unique_dongle_ids = list(set([s.route[:16] for s in samples]))
      dongle_id_inds = [unique_dongle_ids.index(s.route[:16]) for s in samples]
      color_vals = dongle_id_inds
      

      v_ego_stats = describe([s.v_ego for s in samples])
      max_abs_eps = max([abs(v) for v in eps_stats[1]])
      mean_eps = eps_stats[2]
      std_eps = np.sqrt(eps_stats[3])
      
      eps = None
      driver = None
      
      max_abs_driver = max([abs(v) for v in driver_stats[1]])
      mean_driver = driver_stats[2]
      std_driver = np.sqrt(driver_stats[3])
      
      # print(f"{carname} data stats:")
      # print(f"  eps: {eps_stats[2]:.3f} +/- {std_eps:.3f} (max: {eps_stats[1][1]:.3f}) (min: {eps_stats[1][0]:.3f})")
      # print(f"  driver: {driver_stats[2]:.3f} +/- {std_driver:.3f} (max: {driver_stats[1][1]:.3f}) (min: {driver_stats[1][0]:.3f})")
      # print(f"  lat_accel: mean {steer_angle_stats[2]:.3f} (max: {steer_angle_stats[1][1]:.3f}) (min: {steer_angle_stats[1][0]:.3f})")
      # print(f"  v_ego: mean {v_ego_stats[2]:.3f} (max: {v_ego_stats[1][1]:.3f}) (min: {v_ego_stats[1][0]:.3f})")
      
      std_eps = max(std_eps, max_abs_eps * 0.1)
      std_driver = max(std_driver, max_abs_driver * 0.1)
      
      # print(f"eps and driver mean and std: {mean_eps:.3f}, {std_eps:.3f}, {mean_driver:.3f}, {std_driver:.3f}")
      
      speed_bins = range(0,91,18)
      batch_size = 50000

      # create the subplots
      plt.clf()
      fig, axs = plt.subplots(nrows=5, ncols=3, figsize=(12, 12))

      # loop through the speed bins and plot the data
      for i in range(len(speed_bins) - 1):
        try:
          # get the data for this speed bin
          v_ego_min = speed_bins[i]
          v_ego_max = speed_bins[i+1]
          
          # print(f"  generating plots for {v_ego_min}-{v_ego_max}")
          
          # plot the data in the left column with -torque_eps as y value
          data = [sample for sample in samples if v_ego_min <= sample.v_ego < v_ego_max \
                                                  and sample.enabled \
                                                  and abs(sample.torque_driver - mean_driver) < 0.25 * std_driver]
          
          dlen = human_readable(len(data))
          data = random.sample(data, min(batch_size, len(data)))
          dongle_id_inds = [unique_dongle_ids.index(s.route[:16]) for s in data]
          color_vals = dongle_id_inds
          # data = data[:min(batch_size, len(data))]
          # print(f"    {len(data)} eps samples")
          if len(data) > 0:
            ax = axs[i][0]
            y = [sample.torque_eps for sample in data]
            max_abs_y = max([abs(y) for y in y] + [1.0])
            max_abs_x = max([abs(sample.steer_angle) for sample in data] + [1.0])
            ax.scatter([sample.steer_angle for sample in data], y, s=1, alpha=0.1, c=color_vals, cmap='viridis')
            ax.set_title(("EPS steer torque @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
            if i == 4:
              ax.set_xlabel("steer angle [deg]", fontsize=10)
            ax.grid(True)
            ax.set_xlim(-max_abs_x, max_abs_x)
            ax.set_ylim(-max_abs_y, max_abs_y)
          
          # plot the data in the center column with torque_driver as y value
          data = [sample for sample in samples if v_ego_min <= sample.v_ego < v_ego_max \
                                                  and not sample.enabled]
                                                  #and (std_eps < 0.001 or (abs(sample.torque_eps - mean_eps) < 0.25 * std_eps))]
          dlen = human_readable(len(data))
          data = random.sample(data, min(batch_size, len(data)))
          dongle_id_inds = [unique_dongle_ids.index(s.route[:16]) for s in data]
          color_vals = dongle_id_inds
          
          # data = data[:min(batch_size, len(data))]
          # print(f"    {len(data)} driver samples")
          if len(data) > 0:
            ax = axs[i][1]
            y = [sample.torque_driver for sample in data]
            max_abs_y = max([abs(y) for y in y] + [1.0])
            max_abs_x = max([abs(sample.steer_angle) for sample in data] + [1.0])
            x = [sample.steer_angle for sample in data]
            
            # dataset = np.column_stack((x, y))
            
            # apply clustering to the data
            # Standardize the dataset
            # scaler = StandardScaler()
            # scaled_dataset = scaler.fit_transform(dataset)

            # Apply the DBSCAN algorithm
            # dbscan = DBSCAN(eps=0.5, min_samples=5)
            # dbscan.fit(scaled_dataset)
            
            # Apply the Gaussian Mixture Model
            # gmm = GaussianMixture(n_components=2)
            # gmm.fit(scaled_dataset)
            # labels = gmm.predict(scaled_dataset)
            
            # Apply the Local Outlier Factor algorithm
            # lof = LocalOutlierFactor(n_neighbors=20, contamination=0.1)
            # outlier_labels = lof.fit_predict(dataset)

            ax.scatter(x, y, s=1, alpha=0.1, c=color_vals, cmap='viridis')
            ax.set_title(("Driver steer torque @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
            if i == 4:
              ax.set_xlabel("steer angle [deg]", fontsize=10)
            ax.grid(True)
            ax.set_xlim(-max_abs_x, max_abs_x)
            ax.set_ylim(-max_abs_y, max_abs_y)
          
          # plot the data in the right column with adjusted torque value
          data = copy.deepcopy([sample for sample in samples if v_ego_min <= sample.v_ego < v_ego_max])
          if len(data) > 0:
            data, recip = compute_adjusted_steer_torque(data, eps_stats, driver_stats)
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            # data = data[:min(batch_size, len(data))]
            # print(f"    {len(data)} adjusted samples")
            ax = axs[i][2]
            y = [s.torque_adjusted for s in data]
            max_abs_y = max([abs(y) for y in y] + [1.0])
            max_abs_x = max([abs(sample.steer_angle) for sample in data] + [1.0])
            colors = [sample.v_ego for sample in data]
            if i == 4:
              ax.scatter([sample.steer_angle for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis', label="vEgo")
              sm = ScalarMappable(cmap='viridis')
              sm.set_array(colors)
              plt.colorbar(sm)
            else:
              ax.scatter([sample.steer_angle for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
            ax.set_title((f"Adj. torque ({recip:.2e}) @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
            if i == 4:
              ax.set_xlabel("steer angle [deg]", fontsize=10)
            ax.grid(True)
            ax.set_xlim(-max_abs_x, max_abs_x)
            ax.set_ylim(-max_abs_y, max_abs_y)
            # set y gridlines at 0.5 increments, passing through 0.0
            max_abs_y_rounded_ceiling = math.ceil(max_abs_y * 2.0) / 2.0
            ax.set_yticks(np.arange(-max_abs_y_rounded_ceiling, max_abs_y_rounded_ceiling + 0.5, 0.5))
            # thick hlines at -1 and 1
            ax.axhline(y=-1.0, color='r', linestyle='-', linewidth=0.5)
            ax.axhline(y=1.0, color='r', linestyle='-', linewidth=0.5)
        except Exception as e:
          print(f"  {e}")
          continue

      # adjust the spacing between subplots
      np.set_printoptions(precision=2)
      approx_logtime = len(pickle_files) / 60.0
      fig.suptitle(f"{carname} ({approx_logtime:0.0f} hrs of log data) | steer angle vs. eps/driver torque\nLeft two columns colored by user, right colored by speed (up to {human_readable(batch_size)} pts per plot)\neps: {describe_to_string(eps_stats)}\ndriver: {describe_to_string(driver_stats)}\nsteer angle: {describe_to_string(steer_angle_stats)}\nsteer rate {describe_to_string(describe([s.steer_rate for s in samples]))}\nv_ego {describe_to_string(v_ego_stats)}", fontsize=9)
      fig.subplots_adjust(top=0.85)
      plt.tight_layout()
      
      plt.savefig(os.path.join(input_dir, f"{carname} steer_angle_vs_torque.png"))
      plt.clf()
      plt.close("all")

      # show the plots
      # plt.show()
      
      # print(f"Done with {samples[0].car_fp}")
    
    if save_output:
      
      print("Processing data...")
      # for s in data:
      #     s.lateral_accel *= -1.0
      #     s.lateral_jerk *= -1.0
      #     s['steer_cmd'] = get_steer_torque(s)
      #     s.roll = -sin(s.roll) * 9.81
      #     if make == "" and s['car_make'] != "":
      #       make = s['car_make']
      #       model = s.car_fp
          
      # data = [s for s in data if ((s.enabled and s['torque_driver'] <= 0.5) or (not s.enabled and s['torque_driver'] <= 2.0))]
      # data = [vars(s) for s in data]
      
      # Need to determine values of what the lateral accel and jerk will be in the future at each point,
      # so it can be utilized by the model
      # This could be done using the model's predicted future conditions at each time point but those aren't 
      # accurate relative to what actually happened, and I'd have to regenerate lat files to use the model predictions.
      # On the road, the FF model will have access to up to 2.0s (2.5s minus a max assumed steer actuator delay of 0.5s) into the future of lateral accel and jerk, and
      # this comes over 10 or so data points. We'll sample at 0.25s for 7 points to get to 2.0s.
      # This also requires checking the times of each point.
      
      # take v_ego from mph back to m/s
      if print_stats:
        for s in samples:
          s.v_ego /= 2.24
          
      data = samples
      print(f"  {len(data)} samples")
      data, recip = compute_adjusted_steer_torque(data, eps_stats, driver_stats)
      print(f"  {len(data)} samples after adjusting steer torque")
      
      if False:
        outdata = []
        while len(data) > 0:
          try:
            s = data.pop()
            sout = vars(s)
            sout['lateral_accel'] *= -1.0
            sout['lateral_jerk'] *= -1.0
            sout['steer_cmd'] = sout['torque_adjusted']
            s['roll'] *= -1.0
            sout = {k: sout[k] for k in columns}
            outdata.append(sout)
          except Exception as e:
            print(f"  {e}")
            continue
      else:
        # desired_points = 15000000
        data.reverse() # so we can pop() from the end
        CTRL_RATE = 100
        record_times = [0.3, 0.6, 1.0, 1.5]
        steer_delay = 0.0
        steer_delay_frames = int(steer_delay * CTRL_RATE)
        record_times_strings = [f"{'m' if i < 0.0 else 'p'}{int(abs(round(i*10))):02d}" for i in record_times]
        record_times = np.array(record_times)
        columns = ['steer_cmd', 'lateral_accel', 'lateral_jerk', 'v_ego', 'steer_angle', 'steer_rate', 'roll'] \
                + [f"steer_angle_{i}" for i in record_times_strings] \
                + [f"roll_{i}" for i in record_times_strings]
        max_time = max(record_times) - (min(record_times+[0.0]) - steer_delay) + 0.04
        zero_time_ind = 0 if min(record_times) > 0.0 else int((-min(record_times+[0.0]) + 0.04 + steer_delay) * CTRL_RATE)
        print(f"Record times: {record_times}")
        max_len = int(max_time * CTRL_RATE)
        steer_angle_deque = deque(maxlen=max_len)
        roll_deque = deque(maxlen=max_len)
        sample_deque = deque(maxlen=max_len)
        outdata = []
        dt_max = 0.2
        with tqdm(total=len(data)) as pbar:
          while len(data) > 0:
            pbar.update(1)
            sample = data.pop()
            s = vars(sample)
            s['steer_cmd'] = s['torque_adjusted']
            s['lateral_accel'] *= -1.0
            s['lateral_jerk'] *= -1.0
            if len(sample_deque) > 0 and (s['t'] - sample_deque[-1]['t']) * 1e-9 > dt_max:
              steer_angle_deque = deque(maxlen=max_len)
              roll_deque = deque(maxlen=max_len)
              sample_deque = deque(maxlen=max_len)
            else:
              sample_deque.append(s)
              steer_angle_deque.append(s['steer_angle'])
              roll_deque.append(s['roll'])
            
            if len(steer_angle_deque) == max_len:
              sout = sample_deque[zero_time_ind]
              # fix steer delay, fetching the torque from steer_delay seconds ago so it corresponds to the conditions now.
              # sout['steer_cmd'] = sample_deque[zero_time_ind - steer_delay_frames]['steer_cmd']
              Ts = [(s['t'] - sout['t']) * 1e-9 for s in sample_deque]
              sout = {**sout, **{f"steer_angle_{ts}": interp(t, Ts, steer_angle_deque) for t,ts in zip(record_times, record_times_strings)}}
              sout = {**sout, **{f"roll_{ts}": interp(t, Ts, roll_deque) for t,ts in zip(record_times, record_times_strings)}}
              sout = {k: sout[k] for k in columns}
              outdata.append(sout)
              # if len(outdata) >= desired_points:
              #   break
          
      # outdata = [vars(s) for s in data]
      
      del data
      
      # pickle.dump(data, open(output_csv.replace(".csv", ".pkl"), "wb"))
      # pickle.dump(data, lzma.open(output_csv.replace(".csv", ".pkl.xz"), "wb"))
      print("creating dataframe")
      df = pd.DataFrame(outdata)
      
      # print 5 random rows
      print(df.sample(10))
      
      # Write the DataFrame to a CSV file
      print("writing file")
      df.sample(100).copy().to_csv(os.path.join(input_dir,f"{model}_sample.csv"), index=False)#, float_format='%.8g')
      # df.to_csv(os.path.join(input_dir,f"{model}.csv"), index=False)#, float_format='%.8g')
      # feather.write_dataframe(df, os.path.join(input_dir,f"{model}.feather"))
      # df.to_feather(os.path.join(input_dir,f"{model}.feather"))
      feather.write_feather(df, os.path.join(input_dir,f"{model}.feather"), version=1)

    return model

# Example usage:
input_dir = '/Users/haiiro/NoSync/latfiles'
# compile a regex pattern to match valid subdirectory names
pattern = re.compile(r'^[\-A-Z0-9a-z() ]+$')
def has_upper_word(text):
    words = text.split()
    for word in words:
        if word.isupper():
            return True
    return False

# iterate over all directories and subdirectories in the specified path
whitelist = ["toyota", "honda", "hyundai", "chrysler"]
whitelist = ["LACROSSE"]
blacklist = []
dirlist=[]
for root, dirs, files in os.walk(input_dir):
    for dir_name in dirs:
        # check if the directory name matches the regex pattern
        if pattern.match(dir_name) and has_upper_word(dir_name):
            d = os.path.join(root, dir_name)
            if (len(whitelist) > 0 and not any([w in d for w in whitelist])) or (any([b in d for b in blacklist])):
              continue
            print(f"Processing {d}...")
            # try:
            # dirlist.append(d)
            model = pickle_files_to_csv(d, check_modified=False, print_stats=False, save_output=True)
            blacklist.append(dir_name)
            # except Exception as e:
            #   print(f"Error processing {d}: {e}")
            #   continue

# def process_dir(d):
#   pickle_files_to_csv(d, check_modified=False, print_stats=True)

# for d in dirlist:
#   print(f"Processing {d}...")
#   process_dir(d)