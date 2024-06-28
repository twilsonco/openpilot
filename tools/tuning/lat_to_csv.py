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
from sklearn.linear_model import LinearRegression
import math
import random
import unicodedata
from collections import deque, defaultdict
from common.numpy_fast import interp
import pyarrow.feather as feather
import selfdrive.car.toyota.values as toyota
from matplotlib import pyplot as plt
from matplotlib.cm import ScalarMappable
from typing import List
import time

DEBUG=0 # number of segments to load. 0 for no debugging
MAX_LAT_FILES = 7000

DONGLE_ID_BLACKLIST = {}

STEER_AND_EPS_SOURCE_CARS = ["GENESIS", "MAZDA", "SUBARU"]

def sanitize(filename):
    """Return a fairly safe version of the filename.
    https://gitlab.com/jplusplus/sanitize-filename/-/blob/master/sanitize_filename/sanitize_filename.py

    We don't limit ourselves to ascii, because we want to keep municipality
    names, etc, but we do want to get rid of anything potentially harmful,
    and make sure we do not exceed Windows filename length limits.
    Hence a less safe blacklist, rather than a whitelist.
    """
    blacklist = ["\\", "/", ":", "*", "?", "\"", "<", ">", "|", "\0"]
    reserved = [
        "CON", "PRN", "AUX", "NUL", "COM1", "COM2", "COM3", "COM4", "COM5",
        "COM6", "COM7", "COM8", "COM9", "LPT1", "LPT2", "LPT3", "LPT4", "LPT5",
        "LPT6", "LPT7", "LPT8", "LPT9",
    ]  # Reserved words on Windows
    filename = "".join(c for c in filename if c not in blacklist)
    # Remove all charcters below code point 32
    filename = "".join(c for c in filename if 31 < ord(c))
    filename = unicodedata.normalize("NFKD", filename)
    filename = filename.rstrip(". ")  # Windows does not allow these at end
    filename = filename.strip()
    if all([x == "." for x in filename]):
        filename = "__" + filename
    if filename in reserved:
        filename = "__" + filename
    if len(filename) == 0:
        filename = "__"
    if len(filename) > 255:
        parts = re.split(r"/|\\", filename)[-1].split(".")
        if len(parts) > 1:
            ext = "." + parts.pop()
            filename = filename[:-len(ext)]
        else:
            ext = ""
        if filename == "":
            filename = "__"
        if len(ext) > 254:
            ext = ext[254:]
        maxl = 255 - len(ext)
        filename = filename[:maxl]
        filename = filename + ext
        # Re-check last character (if there was no extension)
        filename = filename.rstrip(". ")
        if len(filename) == 0:
            filename = "__"
    return filename

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

def lookahead_lookback_filter(samples, comp_func, n_forward = 0, n_back = 0, max_driver_torque = 1e9, max_eps_torque = 1e9):
  return [s for i,s in enumerate(samples) if \
    i > n_back and i < len(samples) - n_forward and \
    all([comp_func(s1, max_driver_torque, max_eps_torque) for s1 in samples[max(0,i-n_back):min(len(samples), i+n_forward+1)]])]

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
  lateral_accel: float = np.nan
  lateral_jerk: float = np.nan
  roll: float = np.nan
  pitch: float = np.nan
  curvature_device: float = np.nan
  lateral_accel_device: float = np.nan
  steer_cmd: float = np.nan
  steer_cmd_out: float = np.nan
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
  car_eps_fp: str = ''
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

class CleanLatSample():
  def __init__(self, s, route = ""):
    self.v_ego = float(s.v_ego)
    self.a_ego = float(s.a_ego)
    self.torque_eps = float(s.torque_eps)
    self.torque_driver = float(s.torque_driver)
    self.torque_adjusted: float = 0.0
    self.torque_adjusted_eps: float = 0.0
    self.torque_adjusted_driver: float = 0.0
    self.steer_cmd = float(s.steer_cmd)
    self.lateral_accel = float(s.lateral_accel)
    self.lateral_jerk = float(s.lateral_jerk)
    self.roll = float(s.roll)
    self.car_fp = s.car_fp
    self.car_eps_fp = s.car_eps_fp
    self.car_make = s.car_make
    self.t = s.t
    self.enabled: bool = s.enabled
    self.route: str = route
    self.steer_cmd_good: bool = False
    self.driver_torque_good: bool = False
    self.eps_torque_good: bool = False
    self.combined_torque_good: bool = False

def describe_to_string(describe_output):
    nobs, minmax, mean, var, skew, kurtosis = describe_output
    min_val, max_val = minmax
    return f"nobs={nobs:0.1f}, min_max=({min_val:0.1f}, {max_val:0.1f}), mean={mean:0.1f}, std={np.sqrt(var):0.1f}, skew={skew:0.1f}, kurtosis={kurtosis:0.1f}"

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
        
def find_scaling_factors(x_arrays: List[np.ndarray], 
                         y_arrays: List[np.ndarray], 
                         v_ego_arrays: List[np.ndarray],  # Add this argument
                         x_correct: np.ndarray, 
                         y_correct: np.ndarray, 
                         v_ego_correct: np.ndarray,  # Add this argument
                         n_bins: int, 
                         bin_size: int,
                         x_bound:float) -> List[List[float]]:  # Add this argument

    def bin_and_sample(x_array, y_array, v_ego_array, y_bound = None):  # Add v_ego_array as an argument
      # Apply X and V_Ego bounds
      x_mask = (np.abs(x_array) <= x_bound)
      v_ego_mask = (v_ego_array >= 5) & (v_ego_array <= 35)
      
      if y_bound is None:
        y_std = np.std(y_array)
        y_bounds = [- 1.5 * y_std, 1.5 * y_std]
        y_mask = (y_array >= y_bounds[0]) & (y_array <= y_bounds[1])
      else:
        y_mask = (np.abs(y_array) <= y_bound)
      
      mask = x_mask & y_mask & v_ego_mask
      x_array = x_array[mask]
      y_array = y_array[mask]
      v_ego_array = v_ego_array[mask]  # Mask v_ego_array too

      # Bin the data in 2D
      x_bins = np.linspace(-x_bound, x_bound, n_bins + 1)
      v_ego_bins = np.linspace(10, 25, int(15 / 3) + 1)  # Bins of 5 m/s width
      indices = np.digitize(x_array, x_bins) + n_bins * np.digitize(v_ego_array, v_ego_bins)

      print(f"Binned the data: {np.bincount(indices)} points in each bin")

      # Randomly sample from each bin
      x_binned_sampled = []
      y_binned_sampled = []
      for i_bin in range(1, n_bins**2 + 1):  # Bins are 1-indexed
          bin_indices = np.where(indices == i_bin)[0]
          sample_size = min(len(bin_indices), bin_size)
          
          # Calculate the bin ranges
          v_ego_bin_idx = (i_bin - 1) // n_bins
          x_bin_idx = (i_bin - 1) % n_bins

          # Ensure v_ego_bin_idx is within the valid range
          v_ego_bin_idx = min(v_ego_bin_idx, len(v_ego_bins) - 2)

          v_ego_bin_range = (v_ego_bins[v_ego_bin_idx], v_ego_bins[v_ego_bin_idx + 1])
          x_bin_range = (x_bins[x_bin_idx], x_bins[x_bin_idx + 1])
          
          # Combine the previous two print statements
          print(f"Will sample {sample_size} points from bin {i_bin}: lat_accel in {x_bin_range} and v_ego in {v_ego_bin_range}")
          
          sampled_indices = np.random.choice(bin_indices, sample_size, replace=False)
          x_binned_sampled.extend(x_array[sampled_indices])
          y_binned_sampled.extend(y_array[sampled_indices])

      return np.array(x_binned_sampled), np.array(y_binned_sampled)

    # Bin and sample the correct data
    print(f"Lat accel before binning: {describe_to_string(describe(x_correct))}")
    print(f"Steer cmd before binning: {describe_to_string(describe(y_correct))}")
    x_correct, y_correct = bin_and_sample(x_correct, y_correct, v_ego_correct, 1.0)  # Pass v_ego_correct here
    print(f"Lat accel after binning: {describe_to_string(describe(x_correct))}")
    print(f"Steer cmd after binning: {describe_to_string(describe(y_correct))}")

    # Fit a linear regression model to the correct data
    correct_model = LinearRegression()
    correct_model.fit(x_correct.reshape(-1, 1), y_correct)

    # Store the slope of the correct model
    correct_slope = correct_model.coef_[0]

    print(f"Fitted the correct model: slope = {-correct_slope}")

    # Array to store the scaling factors
    scaling_factors = []
    slopes = [correct_slope]

    # For each x and y array
    for i, (x_array, y_array, v_ego_arrays) in enumerate(zip(x_arrays, y_arrays, v_ego_arrays)):
        print(f"\nProcessing array {i + 1}")

        # Bin and sample the data
        print(f"Lat accel before binning: {describe_to_string(describe(x_array))}")
        print(f"torque before binning: {describe_to_string(describe(y_array))}")
        x_array, y_array = bin_and_sample(x_array, y_array, v_ego_arrays)
        print(f"Lat accel after binning: {describe_to_string(describe(x_array))}")
        print(f"torque after binning: {describe_to_string(describe(y_array))}")

        # Fit a linear regression model to the current data
        current_model = LinearRegression()
        current_model.fit(x_array.reshape(-1, 1), y_array)

        # Calculate the scaling factor as the ratio of the slopes
        scaling_factor = correct_slope / current_model.coef_[0]

        print(f"Fitted the model for array {i + 1}: slope = {-current_model.coef_[0]}, scaling factor = {scaling_factor}")

        # Append the scaling factor to the list
        scaling_factors.append(scaling_factor)
        slopes.append(current_model.coef_[0])

    return scaling_factors, slopes



def compute_adjusted_steer_torque(samples, eps_scale = 1.0, driver_scale = 1.0, eps_slope = 1.0, driver_slope = 1.0, inplace=True):
  blacklist_neighbor_secs = 0.5
  max_ratio = 30.0
  max_abs_long_accel = 2.5
  min_speed = 10.0
  max_speed = 25.0
  max_lat_accel = 4.0
  max_lat_jerk = 7.0
  min_slope = 0.15
  min_driver_slope = 0.3
  # eps_check_func = lambda s: abs(s.torque_driver) < 0.15 and abs(s.a_ego) < max_abs_long_accel
  combined_check_func = lambda s, me, md: ((s.enabled and abs(s.torque_driver) < md) or (not s.enabled and (me == 0.0 or abs(s.torque_eps) < me))) and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
  eps_check_func = lambda s, me, md: abs(s.torque_driver) < md and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
  driver_check_func = lambda s, me, md: (not s.enabled or s.steer_cmd == 0.0) and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
  steer_cmd_check_func = lambda s, me, md: eps_check_func(s, me, md)
  subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 2.0 and 2.0 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
  nlalb = 1 # number of points for look-ahead and look-back
  torque_func_eps = lambda s,e,d: e * s.torque_eps
  torque_func_driver = lambda s,e,d: d * s.torque_driver
  torque_func_combined = lambda s,e,d: e * s.torque_eps + d * s.torque_driver
  print(samples[0].car_make)
  if len(samples) == 0:
    return [], 0.0
  if samples[0].car_make == 'gm':
    eps_check_func = lambda s, me, md: abs(s.torque_driver) < md and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                                              and (abs(s.torque_eps) / max(0.001, abs(s.lateral_accel)) > 0.015)
    if samples[0].car_fp in ['CHEVROLET VOLT PREMIER 2018']:
      driver_check_func = lambda s, me, md: (not s.enabled and (me == 0.0 or abs(s.torque_eps) < me)) and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                                                and (abs(s.torque_driver) / max(0.001, abs(s.lateral_accel)) > 0.2)
    else:
      driver_check_func = lambda s, me, md: (not s.enabled and (me == 0.0 or abs(s.torque_eps) < me)) and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                                                and (abs(s.torque_driver) / max(0.001, abs(s.lateral_accel)) > 0.05)
    steer_cmd_check_func = lambda s, me, md: s.enabled and abs(s.torque_driver ) < 1.0 \
                                              and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.02)
    # torque_func_eps = lambda s,e,d: s.torque_driver * d + s.torque_eps * e
    # driver_scale = 1/3
    # eps_scale = 1/3
  elif samples[0].car_make == 'volkswagen':
    if samples[0].car_fp in ['VOLKSWAGEN PASSAT NMS']:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                            and (abs(s.torque_driver) / max(0.001, abs(s.lateral_accel)) > 10.0)
    else:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
    steer_cmd_check_func = lambda s, me, md: abs(s.torque_driver ) < 100 \
                                              and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.02)
    # torque_func_eps = lambda s,e,d: s.torque_driver * d
  elif samples[0].car_make == 'hyundai':
    steer_cmd_check_func = lambda s, me, md: abs(s.torque_driver ) < 50 \
                                              and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.02)
    if samples[0].car_fp in ['SONATA 2020']:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                                                and (abs(s.torque_driver) / max(0.001, abs(s.lateral_accel)) > 10.0)
    pass
  elif samples[0].car_make == 'chrysler':
    if samples[0].car_fp in ['CHRYSLER PACIFICA 2018', 'RAM 1500 5TH GEN']:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                            and (s.v_ego < 5.0 or abs(s.torque_driver) / max(0.001, abs(s.lateral_accel)) > 10.0)
      eps_check_func = lambda s, me, md: abs(s.torque_driver) < md and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                                          and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.1)
    else:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
      eps_check_func = lambda s, me, md: abs(s.torque_driver) < md and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                                          and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.02)
    steer_cmd_check_func = lambda s, me, md: eps_check_func(s, me, md)
    subset_outer_check = lambda s, _1, _2: s.v_ego >= 23.0 and abs(s.lateral_jerk) < 3.5 and 1.0 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
    pass
  elif samples[0].car_make == 'toyota':
    if samples[0].car_fp in ['TOYOTA CAMRY HYBRID 2021', 'TOYOTA PRIUS TSS2 2021']:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                            and (s.v_ego < 8.0 or abs(s.torque_driver) / max(0.001, abs(s.lateral_accel)) > 70.0)
    else:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
    steer_cmd_check_func = lambda s, me, md: abs(s.torque_driver ) < 50 \
                                              and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.02)
    subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 2.0 and 1.0 <= abs(s.lateral_accel) <= 2.0  \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
    pass
  elif samples[0].car_make == 'honda':
    driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
    steer_cmd_check_func = lambda s, me, md: abs(s.torque_driver ) < 50 \
                                              and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.02)
    pass
    # torque_func_eps = lambda s,e,d: s.torque_driver * d
  elif samples[0].car_make == 'subaru':
    steer_cmd_check_func = lambda s, me, md: abs(s.torque_driver ) < 50 \
                                              and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.02)
    pass
  elif samples[0].car_make == 'ford':
    if samples[0].car_fp in ['FORD MAVERICK 1ST GEN', 'FORD F-150 14TH GEN']:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk \
                            and (abs(s.torque_driver) / max(0.001, abs(s.lateral_accel)) > 0.15)
    else:
      driver_check_func = lambda s, me, md: s.steer_cmd == 0.0 and abs(s.a_ego) < max_abs_long_accel and abs(s.lateral_jerk) < max_lat_jerk
    pass
    # torque_func_eps = lambda s,e,d: s.torque_driver * d
  elif samples[0].car_make == 'mazda':
    steer_cmd_check_func = lambda s, me, md: abs(s.torque_driver ) < 50 \
                                              and (abs(s.steer_cmd) / max(0.001, abs(s.lateral_accel)) > 0.05)
    pass
    # torque_func_eps = lambda s,e,d: s.torque_driver * d
  
  max_driver_torque = max([abs(s.torque_driver) for s in samples]) * 0.25
  max_eps_torque = max([abs(s.torque_eps) for s in samples]) * 0.25
  steer_cmd_slope = 1.0
  if eps_scale == 1.0 and driver_scale == 1.0:
    # prepare arrays for finding scaling factor
    iter = -1
    subset_outer = []
    while True:
      iter += 1
      if iter == 1:
        print(f"{len(subset_outer)} is too few samples for scaling factor. trying again with relaxed (lat accel and lat jerk) constraints.")
        subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 2.0 and 1.75 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
      elif iter == 2:
        print(f"{len(subset_outer)} is too few samples for scaling factor. trying again with relaxed (lat jerk) constraints.")
        subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 3.5 and 1.5 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
      elif iter == 3:
        print(f"{len(subset_outer)} is too few samples for scaling factor. trying again with relaxed (lat accel) constraints.")
        subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 3.5 and 1.25 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
      elif iter == 4:
        print(f"{len(subset_outer)} is too few samples for scaling factor. trying again with relaxed (lat accel) constraints.")
        subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 3.5 and 1.0 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
      elif iter == 5:
        print(f"{len(subset_outer)} is too few samples for scaling factor. trying again with relaxed (lat accel) constraints.")
        subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 3.5 and 0.8 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
      elif iter == 6:
        print(f"{len(subset_outer)} is too few samples for scaling factor. trying again with relaxed (speed, lat accel) constraints.")
        min_speed -= 2.0
        subset_outer_check = lambda s, _1, _2: min_speed <= s.v_ego <= max_speed and abs(s.lateral_jerk) < 3.5 and 0.5 <= abs(s.lateral_accel) <= 2.5 \
                                                  and ((abs(s.steer_cmd) / abs(s.lateral_accel)) > min_slope or (abs(s.torque_driver) / abs(s.lateral_accel)) > min_driver_slope)
      elif iter > 1:
        break
        
      subset_outer = lookahead_lookback_filter(samples, subset_outer_check, nlalb, nlalb)
      print(f"subset_outer: {len(subset_outer)}")
      if len(subset_outer) < 100:
        continue
      
      
      if len(subset_outer) > 100:
        # calculate maximum absolute values of driver and eps torque
        
        subset = [s for s in subset_outer if (s.enabled or s.steer_cmd != 0.0) and (max_driver_torque == 0.0 or abs(s.torque_driver) < max_driver_torque) \
                                                and (abs(s.lateral_accel) / max(0.001, abs(s.steer_cmd)) < 50)]
        len_steer_cmd = len(subset)
        if len_steer_cmd > 100:
          x_steer_cmd = np.abs(np.array([s.lateral_accel for s in subset]))
          y_steer_cmd = np.abs(np.array([s.steer_cmd for s in subset]))
          print("steer cmd: ",describe_to_string(describe(y_steer_cmd)))
          print("steer cmd x: ",describe_to_string(describe(x_steer_cmd)))
          # then driver torque
        else:
          print(f"{len_steer_cmd} is too few steer cmd samples for scaling factor. trying again with fewer constraints.")
          continue
        subset = [s for s in subset_outer if (s.steer_cmd == 0.0) and abs(s.lateral_accel) / max(0.001, abs(s.torque_driver)) < 50]
        # subset = [s for s in subset_outer if driver_check_func(s, max_eps_torque, max_driver_torque)]
        len_driver = len(subset)
        # if len_driver < 100:
        #   print(f"{len_driver} is too few driver samples for scaling factor. trying again with fewer constraints.")
        #   subset = [s for s in subset_outer if (not s.enabled or s.steer_cmd == 0.0) and (abs(s.lateral_accel) / max(0.001, abs(s.torque_driver)) < 50)]
        #   len_driver = len(subset)
        if len_driver > 100:
          x_driver = np.abs(np.array([s.lateral_accel for s in subset]))
          y_driver = np.abs(np.array([s.torque_driver for s in subset]))
          print("driver: ",describe_to_string(describe(y_driver)))
          print("driver x: ",describe_to_string(describe(x_driver)))
        else:
          print(f"{len_driver} is too few driver samples for scaling factor. trying again with fewer constraints.")
          continue
        # then eps torque
        subset = [s for s in subset_outer if eps_check_func(s, max_eps_torque, max_driver_torque)]
        len_eps = len(subset)
        if len_eps < 100:
          print(f"{len_eps} is too few eps for scaling factor. trying again with fewer constraints.")
          subset = [s for s in subset_outer if (s.enabled or s.steer_cmd != 0.0) and (max_driver_torque == 0.0 or abs(s.torque_driver) < max_driver_torque) and (abs(s.lateral_accel) / max(0.001, abs(s.torque_eps)) < 50)]
          len_eps = len(subset)
        if len_eps > 100:
          x_eps = np.abs(np.array([s.lateral_accel for s in subset]))
          y_eps = np.abs(np.array([s.torque_eps for s in subset]))
          print("lka eps: ",describe_to_string(describe(y_eps)))
          print("lka eps x: ",describe_to_string(describe(x_eps)))
        else:
          print(f"{len_eps} is too few eps samples for scaling factor. trying again with fewer constraints.")
          continue
      
        # print lengths and proportions
        len_samples = len(samples)
        print('len_samples: ', len_samples)
        print('len_steer_cmd: ', len_steer_cmd, ' (', len_steer_cmd / len_samples, ')')
        print('len_driver: ', len_driver, ' (', len_driver / len_samples, ')')
        print('len_eps: ', len_eps, ' (', len_eps / len_samples, ')')
        
        # compute scaling factors for driver and eps torque
        if len_steer_cmd > 100:
          steer_cmd_slope = np.abs(np.mean(y_steer_cmd) / np.mean(x_steer_cmd))
          # model = LinearRegression()
          # model.fit(x_steer_cmd.reshape(-1, 1), y_steer_cmd)
          # steer_cmd_slope = -model.coef_[0]
          if len_eps > 100 :
            eps_slope = np.abs(np.mean(y_eps) / np.mean(x_eps))
            if eps_slope < 0.001:
              print(f"Invalid slope for eps: {eps_slope}. Setting to 1.0.")
              eps_slope = 1.0
            # model = LinearRegression()
            # model.fit(x_eps.reshape(-1, 1), y_eps)
            # steer_cmd_slope = model.coef_[0]
            eps_scale = steer_cmd_slope / eps_slope
          if len_driver > 100:
            driver_slope = np.abs(np.mean(y_driver) / np.mean(x_driver))
            if driver_slope < 0.001:
              print(f"Invalid slope for driver: {eps_slope}. Setting to 1.0.")
              driver_slope = 1.0
            # model = LinearRegression()
            # model.fit(x_driver.reshape(-1, 1), y_driver)
            # steer_cmd_slope = model.coef_[0]
            driver_scale = steer_cmd_slope / driver_slope
        
        break
  
  # print slopes
  print('steer_cmd_slope: ', steer_cmd_slope)
  print('eps_slope: ', eps_slope)
  print('driver_slope: ', driver_slope)
  
  # print scaling factors
  print('eps_scale: ', eps_scale)
  print('driver_scale: ', driver_scale)
  # print('num samples: ', len(samples))
    
  # scale data and compute adjusted torque
  out_samples = [] if not inplace else samples
  for s1 in tqdm(out_samples if inplace else samples):
    s = s1 if inplace else copy.deepcopy(s1)
    s.torque_adjusted_driver = torque_func_driver(s, float(eps_scale), float(driver_scale))
    s.torque_adjusted_eps = torque_func_eps(s, float(eps_scale), float(driver_scale))
    s.steer_cmd_good = steer_cmd_check_func(s, max_eps_torque, max_driver_torque)
    s.eps_torque_good = eps_check_func(s, max_eps_torque, max_driver_torque)
    s.driver_torque_good = driver_check_func(s, max_eps_torque, max_driver_torque)
    s.combined_torque_good = True
    if s.driver_torque_good and s.eps_torque_good:
      s.torque_adjusted = s.torque_adjusted_driver + s.torque_adjusted_eps
    elif s.driver_torque_good:
      s.torque_adjusted = s.torque_adjusted_driver
    elif s.eps_torque_good:
      s.torque_adjusted = s.torque_adjusted_eps
    else:
      s.torque_adjusted = 0.0
      s.combined_torque_good = False
    if not inplace:
      out_samples.append(s)
    
  out_samples = [s for s in out_samples if any([s.steer_cmd_good, s.eps_torque_good, s.driver_torque_good, s.combined_torque_good])]
  
  # # plot driver torque
  # x = np.array([s.lateral_accel for s in out_samples if s.driver_torque_good])
  # y = np.array([s.torque_driver for s in out_samples if s.driver_torque_good])
  # plt.scatter(x, y, s=1)
  # plt.show()
  
  return out_samples, [eps_scale, driver_scale], [steer_cmd_slope, eps_slope, driver_slope]

    
def pickle_files_to_csv(input_dir, check_modified=True, print_stats=False, save_output=True):
    # List all pickle files in the input directory
    carname = os.path.basename(input_dir)
    pickle_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.lat')])
    if len(pickle_files) > MAX_LAT_FILES:
      # sort by file modified time and take the most recent MAX_LAT_FILES
      pickle_files = sorted(pickle_files, key=lambda f: os.path.getmtime(os.path.join(input_dir, f)), reverse=True)[:MAX_LAT_FILES]
    
    data = []
    
    make = ""
    model = ""
    
    skip_by_eps_firmware = defaultdict(bool)
    
    print("Checking pickle files...")
    
    i=0
    for pickle_file in tqdm(pickle_files):
      with open(os.path.join(input_dir, pickle_file), 'rb') as f:
        try:
          pk = pickle.load(f)
        except:
          continue
        for s in pk:
          if make == "" and s.car_make != "":
            make = s.car_make
            model = s.car_fp
            eps_fp = s.car_eps_fp
            if eps_fp in skip_by_eps_firmware:
              break
            outfile = os.path.join(input_dir,f"{model}{eps_fp}.feather")
            make = ""
            model = ""
            skip_by_eps_firmware[eps_fp] = check_modified and is_file_modified_within_last_two_weeks(outfile)
            break
      if DEBUG and i > DEBUG:
        break
      i += 1

    # Iterate through the pickle files and load the data
    i=0
    # random.shuffle(pickle_files)
    print("Loading pickle files...")
    columns = [
      'v_ego',
      # 'a_ego',
      'lateral_accel',
      'lateral_jerk',
      'roll', # actually lateral gravitational acceleration
      'steer_cmd',
      # "lateral_accel_1",
      # "lateral_jerk_1",
      # "roll_1"
      ]
    
    samples = []
    
    tot_num_points = 0
    tot_num_points_by_eps_firmware = defaultdict(int)
    tot_num_pickle = 0
    tot_num_pickle_by_eps_firmware = defaultdict(int)
    
    if DEBUG:
      random.shuffle(pickle_files)
    for pickle_file in tqdm(pickle_files):
      filename = os.path.basename(pickle_file)
      if len(filename) > 16 and filename[:16] in DONGLE_ID_BLACKLIST:
        # print(f"Skipping {filename} because it is in the blacklist")
        continue
      with open(os.path.join(input_dir, pickle_file), 'rb') as f:
        try:
          pk = pickle.load(f)
        except:
          continue
        pickle_incremented = False
        for s in pk:
          eps_fp = s.car_eps_fp
          if not pickle_incremented:
            tot_num_pickle += 1
            tot_num_pickle_by_eps_firmware[eps_fp] += 1
            pickle_incremented = True
          if skip_by_eps_firmware[eps_fp]:
            continue
          if s.v_ego < 0.1 or (s.car_make in ["chrysler"] and abs(s.lateral_accel) > 0.1 and s.lateral_accel_device == 0.0):
            continue
          tot_num_points += 1
          tot_num_points_by_eps_firmware[s.car_eps_fp] += 1
          samples.append(CleanLatSample(s, pickle_file))
        if DEBUG and i > DEBUG:
          break
        i += 1
    
    if len(samples) == 0:
      return
    
    # get unique list of firmware versions
    eps_firmwares = list(set([s.car_eps_fp for s in samples if s.car_eps_fp != '']))
    if len(eps_firmwares) == 0:
      eps_firmwares = ['']
    else:
      eps_firmwares.sort()
      eps_firmwares.append('combined')
    print(f"EPS firmwares and number of points of each ({tot_num_points} total):")
    tot_num_points_full = copy.deepcopy(tot_num_points)
    print('\n'.join([f"{k}: {v}" for k,v in tot_num_points_by_eps_firmware.items()]))
    full_samples = samples
    
    for eps_firmware in eps_firmwares:
      if eps_firmware != '':
        print(f"Processing firmware: {eps_firmware}")
      else:
        eps_firmware = 'combined'
      samples = [s for s in full_samples if eps_firmware == 'combined' or s.car_eps_fp == eps_firmware]
        
      eps = [s.torque_eps for s in samples]
      lat_accel = [s.lateral_accel for s in samples]
      driver = [s.torque_driver for s in samples]
      
      eps_stats = describe(eps)
      driver_stats = describe(driver)
      lat_accel_stats = describe(lat_accel)
      roll_stats = describe([s.roll for s in samples])
      steer_cmd_stats = describe([s.steer_cmd for s in samples])
      long_accel_stats = describe([s.a_ego for s in samples])
          
      tot_num_points = tot_num_points_by_eps_firmware[eps_firmware] if eps_firmware != 'combined' else tot_num_points_full

      if print_stats and len(samples) > 0:
        
        print("Preparing plots")
        
        data1, scale_factor, slopes = compute_adjusted_steer_torque(samples, inplace=False)
        
        
        
        # convert to mph
        for s in samples:
          s.v_ego *= 2.24
        for s in data1:
          s.v_ego *= 2.24
        
        # want to color points by dongle id, so get index of each point in unique list of dongle ids
        unique_dongle_ids = list(set([s.route[:16] for s in samples]))
        dongle_id_inds = [unique_dongle_ids.index(s.route[:16]) for s in samples]
        color_vals = dongle_id_inds
        

        v_ego_stats = describe([s.v_ego for s in samples])
        max_abs_eps = max([abs(v) for v in eps_stats[1]])
        mean_eps = eps_stats[2]
        std_eps = np.sqrt(eps_stats[3])
        
        lat_accel = None
        eps = None
        driver = None
        
        gc.collect()
        
        max_abs_driver = max([abs(v) for v in driver_stats[1]])
        mean_driver = driver_stats[2]
        std_driver = np.sqrt(driver_stats[3])
        
        # print(f"{carname} data stats:")
        # print(f"  eps: {eps_stats[2]:.3f} +/- {std_eps:.3f} (max: {eps_stats[1][1]:.3f}) (min: {eps_stats[1][0]:.3f})")
        # print(f"  driver: {driver_stats[2]:.3f} +/- {std_driver:.3f} (max: {driver_stats[1][1]:.3f}) (min: {driver_stats[1][0]:.3f})")
        # print(f"  lat_accel: mean {lat_accel_stats[2]:.3f} (max: {lat_accel_stats[1][1]:.3f}) (min: {lat_accel_stats[1][0]:.3f})")
        # print(f"  v_ego: mean {v_ego_stats[2]:.3f} (max: {v_ego_stats[1][1]:.3f}) (min: {v_ego_stats[1][0]:.3f})")
        
        std_eps = max(std_eps, max_abs_eps * 0.1)
        std_driver = max(std_driver, max_abs_driver * 0.1)
        
        # print(f"eps and driver mean and std: {mean_eps:.3f}, {std_eps:.3f}, {mean_driver:.3f}, {std_driver:.3f}")
        
        
        speed_bins = range(0,91,18)
        batch_size = 50000

        # create the subplots
        plt.clf()
        fig, axs = plt.subplots(nrows=5, ncols=6, figsize=(24, 12))
        
        
        
        # adjusted data info
        print(f"adjusted data info: {len(data1)} samples, scale factor {scale_factor}")

        # loop through the speed bins and plot the data
        for i in range(len(speed_bins) - 1):
          try:
            # get the data for this speed bin
            v_ego_min = speed_bins[i]
            v_ego_max = speed_bins[i+1]
            
            # print(f"  generating plots for {v_ego_min}-{v_ego_max}")
            
            # plot the data in the left column with -steer cmd as y value
            data = [sample for sample in data1 if sample.steer_cmd_good and v_ego_min <= sample.v_ego < v_ego_max]
            
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            dongle_id_inds = [unique_dongle_ids.index(s.route[:16]) for s in data]
            color_vals = dongle_id_inds
            # data = data[:min(batch_size, len(data))]
            # print(f"    {len(data)} eps samples")
            col_num = 0
            if len(data) > 0:
              ax = axs[i][col_num]
              y = [sample.steer_cmd for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=color_vals, cmap='viridis')
              ax.set_title((f"Steer cmd (m={slopes[0]:.2f}) @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-1.0, 1.0)
            col_num += 1
              
              # plot the data in the left column with -torque_eps as y value
            data = [sample for sample in data1 if v_ego_min <= sample.v_ego < v_ego_max \
                                                    and sample.eps_torque_good]
            
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            dongle_id_inds = [unique_dongle_ids.index(s.route[:16]) for s in data]
            color_vals = dongle_id_inds
            # data = data[:min(batch_size, len(data))]
            # print(f"    {len(data)} eps samples")
            if len(data) > 0:
              ax = axs[i][col_num]
              y = [sample.torque_eps for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=color_vals, cmap='viridis')
              ax.set_title((f"LKA torque @ (m={slopes[1]:.2f}) " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-max_abs_y, max_abs_y)
            col_num += 1
            
            # plot the data in the center column with torque_driver as y value
            data = [sample for sample in data1 if v_ego_min <= sample.v_ego < v_ego_max \
                                                    and sample.driver_torque_good]
                                                    #and (std_eps < 0.001 or (abs(sample.torque_eps - mean_eps) < 0.25 * std_eps))]
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            dongle_id_inds = [unique_dongle_ids.index(s.route[:16]) for s in data]
            color_vals = dongle_id_inds
            
            # data = data[:min(batch_size, len(data))]
            # print(f"    {len(data)} driver samples")
            if len(data) > 0:
              ax = axs[i][col_num]
              y = [sample.torque_driver for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              x = [-sample.lateral_accel for sample in data]
              
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
              ax.set_title((f"Driver (m={slopes[2]:.2f}) @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-max_abs_y, max_abs_y)
            col_num += 1
              
              
            # plot the data in the left column with -torque_eps as y value
            data = [sample for sample in data1 if v_ego_min <= sample.v_ego < v_ego_max \
                                                    and sample.eps_torque_good]
            
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            colors = [sample.v_ego for sample in data]
            # data = data[:min(batch_size, len(data))]
            # print(f"    {len(data)} eps samples")
            if len(data) > 0:
              ax = axs[i][col_num]
              y = [sample.torque_adjusted_eps for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"LKA adj @ ({scale_factor[0]:.2e}) " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-max_abs_y, max_abs_y)
              ax.axhline(y=-1.0, color='r', linestyle='-', linewidth=0.5)
              ax.axhline(y=1.0, color='r', linestyle='-', linewidth=0.5)
            col_num += 1
              
              
            # plot the data in the left column with -torque_eps as y value
            data = [sample for sample in data1 if v_ego_min <= sample.v_ego < v_ego_max \
                                                    and sample.driver_torque_good]
            
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            colors = [sample.v_ego for sample in data]
            # data = data[:min(batch_size, len(data))]
            # print(f"    {len(data)} eps samples")
            if len(data) > 0:
              ax = axs[i][col_num]
              y = [sample.torque_adjusted_driver for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Driver adj @ ({scale_factor[1]:.2e}) " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-max_abs_y, max_abs_y)
              ax.axhline(y=-1.0, color='r', linestyle='-', linewidth=0.5)
              ax.axhline(y=1.0, color='r', linestyle='-', linewidth=0.5)
            col_num += 1
              
              
            
            # plot the data in the right column with adjusted torque value
            data = [sample for sample in data1 if v_ego_min <= sample.v_ego < v_ego_max and sample.combined_torque_good]
            if len(data) > 0:
              dlen = human_readable(len(data))
              data = random.sample(data, min(batch_size, len(data)))
              # data = data[:min(batch_size, len(data))]
              # print(f"    {len(data)} adjusted samples")
              ax = axs[i][col_num]
              y = [s.torque_adjusted for s in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              colors = [sample.v_ego for sample in data]
              if i == 4:
                ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis', label="vEgo")
                sm = ScalarMappable(cmap='viridis')
                sm.set_array(colors)
                plt.colorbar(sm)
              else:
                ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Combined adj @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("Lateral Acceleraion [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-max_abs_y, max_abs_y)
              # set y gridlines at 0.5 increments, passing through 0.0
              max_abs_y_rounded_ceiling = math.ceil(max_abs_y * 2.0) / 2.0
              ax.set_yticks(np.arange(-max_abs_y_rounded_ceiling, max_abs_y_rounded_ceiling + 0.5, 0.5))
              # thick hlines at -1 and 1
              ax.axhline(y=-1.0, color='r', linestyle='-', linewidth=0.5)
              ax.axhline(y=1.0, color='r', linestyle='-', linewidth=0.5)
            col_num += 1
          except Exception as e:
            print(f"  {e}")
            continue
          
        data = None
        data1 = None
        gc.collect()

        # adjust the spacing between subplots
        np.set_printoptions(precision=2)
        num_pickle = tot_num_pickle_by_eps_firmware[eps_firmware] if eps_firmware != 'combined' else tot_num_pickle
        approx_logtime = num_pickle / 60.0
        eps_firmware_str = "" if len(eps_firmwares) <= 1 else f" | eps_firmware: {eps_firmware}"
        suptitle=f"{carname}{eps_firmware_str} ({approx_logtime:0.0f} hrs of log data) | lat_accel (factor {1.0/slopes[0]:0.3f}) vs. steer cmd and eps/driver torque\nLeft three columns colored by user, right three by speed (up to {human_readable(batch_size)} pts per plot) | steer cmd: {describe_to_string(steer_cmd_stats)}\neps: {describe_to_string(eps_stats)} | driver: {describe_to_string(driver_stats)}\nlat accel: {describe_to_string(lat_accel_stats)} | lat jerk {describe_to_string(describe([s.lateral_jerk for s in samples]))}\nv_ego {describe_to_string(v_ego_stats)} | a_ego {describe_to_string(long_accel_stats)}\nroll {describe_to_string(roll_stats)}"
        if DEBUG:
          suptitle = f"<---DEBUG-{DEBUG}--->\n{suptitle}\n<---DEBUG-{DEBUG}--->"
        fig.suptitle(suptitle, fontsize=9)
        fig.subplots_adjust(top=0.85)
        plt.tight_layout()
        
        eps_firmware_str = "" if len(eps_firmwares) <= 1 else f"_{sanitize(eps_firmware)}"
        plt.savefig(os.path.join(input_dir, f"{carname}{eps_firmware_str} lat_accel_vs_torque.png"))
        plt.clf()
        plt.close("all")
        
        fig = None
        ax = None
        axs = None
        gc.collect()
        
        
        # return to m/s
        for s in samples:
          s.v_ego /= 2.24

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
        
        no_eps_torque_data = True
        pos_found = False
        neg_found = False
        for s in samples:
          if not pos_found and s.torque_eps > 0.5:
            pos_found = True
          elif not neg_found and s.torque_eps < -0.5:
            neg_found = True
          if pos_found and neg_found:
            print("  eps torque > 0.5 detected, can use eps torque data")
            no_eps_torque_data = False
            break
        
        approx_lat_jerk = True
        pos_found = False
        neg_found = False
        for s in samples:
          if not pos_found and s.lateral_jerk > 0.5:
            pos_found = True
          elif not neg_found and s.lateral_jerk < -0.5:
            neg_found = True
          if pos_found and neg_found:
            print("  lateral jerk > 0.5 detected, not approximating lateral jerk")
            approx_lat_jerk = False
            break
        
        print(f"{no_eps_torque_data=}, {approx_lat_jerk=}")
        
        data = samples
        print(f"  {len(data)} samples")
        data, scale_factor, slopes = compute_adjusted_steer_torque(data, inplace=True)
        if scale_factor[0] == 1.0:
          scale_string = f"{scale_factor[1]:0.2e}"
        elif scale_factor[1] == 1.0:
          scale_string = f"{scale_factor[0]:0.2e}"
        else:
          scale_string = f"eps:{scale_factor[0]:0.2e},driver:{scale_factor[1]:0.2e}"
        print(f"  {len(data)} samples after adjusting steer torque, scale factor {scale_string}")
        
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
          # data.reverse() # so we can pop() from the end
          CTRL_RATE = 100
          record_times = [-0.3, -0.2, -0.1, 0.3, 0.6, 1.0, 1.5]
          
          steer_delay = 0.18
          steer_delay_ind = int(steer_delay * CTRL_RATE)
          record_times_strings = [f"{'m' if i < 0.0 else 'p'}{int(abs(round(i*10))):02d}" for i in record_times]
          record_times = np.array(record_times)
          columns = ['steer_cmd', 'v_ego', 'lateral_accel', 'lateral_jerk', 'roll'] \
                  + [f"lateral_accel_{i}" for i in record_times_strings] \
                  + [f"roll_{i}" for i in record_times_strings]
          max_time = max(record_times) - (min(record_times+[0.0])) + 0.04
          zero_time_ind = 0 if min(record_times) > 0.0 else int((-min(record_times+[0.0]) + 0.04) * CTRL_RATE)
          print(f"Record times: {record_times}")
          max_len = int(max_time * CTRL_RATE)
          torque_sources = ['torque_adjusted', 'torque_adjusted_driver', 'torque_adjusted_eps', 'steer_cmd']
          check_sources = ['combined_torque_good', 'driver_torque_good', 'eps_torque_good', 'steer_cmd_good']
          torque_sources = ['torque_adjusted_eps']
          check_sources = ['eps_torque_good']
          # torque_sources = ['steer_cmd']
          # check_sources = ['steer_cmd_good']
          if no_eps_torque_data:
            torque_sources = ['steer_cmd']
            check_sources = ['steer_cmd_good']
          elif any([chk in model for chk in STEER_AND_EPS_SOURCE_CARS]):
            torque_sources = ['torque_adjusted_eps', 'steer_cmd']
            check_sources = ['eps_torque_good', 'steer_cmd_good']
          eps_firmware_str = "" if len(eps_firmwares) <= 1 else f"_{sanitize(eps_firmware)}"
          for torque_source, check_source in zip(torque_sources, check_sources):
            print(f"  Processing {torque_source}...")
            lat_accel_deque = deque(maxlen=max_len)
            roll_deque = deque(maxlen=max_len)
            sample_deque = deque(maxlen=max_len)
            outdata = []
            dt_max = min([j-i for i, j in zip(record_times[:-1], record_times[1:])])
            iter = 0
            # data.reverse()
            with tqdm(total=len(data)) as pbar:
              # while len(data) > 0:
              for sample in data:
                iter += 1
                pbar.update(1)
                # sample = data.pop()
                if iter > 10000000:
                  gc.collect()
                  iter = 0
                s = copy.deepcopy(vars(sample))
                if not s[check_source]:
                  continue
                s['lateral_accel'] *= -1.0
                s['lateral_jerk'] *= -1.0
                s['steer_cmd'] = s[torque_source]
                s['roll'] *= -1.0
                if len(sample_deque) > 0 and (s['t'] - sample_deque[-1]['t']) * 1e-9 > dt_max:
                  lat_accel_deque = deque(maxlen=max_len)
                  roll_deque = deque(maxlen=max_len)
                  sample_deque = deque(maxlen=max_len)
                else:
                  sample_deque.append(s)
                  lat_accel_deque.append(s['lateral_accel'])
                  roll_deque.append(s['roll'])
                
                if len(lat_accel_deque) == max_len:
                  sout = sample_deque[zero_time_ind]
                  # fix steer delay, fetching the torque (steer_cmd) from steer_delay seconds ago so it corresponds to the conditions now.
                  # sout['steer_cmd'] = sample_deque[zero_time_ind - steer_delay_ind]['steer_cmd']
                  Ts = [(s['t'] - sout['t']) * 1e-9 for s in sample_deque]
                  if approx_lat_jerk:
                    sout['lateral_jerk'] = (interp(0.15, Ts, lat_accel_deque) - interp(-0.15, Ts, lat_accel_deque)) / 0.3
                  sout = {**sout, **{f"lateral_accel_{ts}": interp(t, Ts, lat_accel_deque) for t,ts in zip(record_times, record_times_strings)}}
                  sout = {**sout, **{f"roll_{ts}": interp(t, Ts, roll_deque) for t,ts in zip(record_times, record_times_strings)}}
                  sout = {k: sout[k] for k in columns}
                  outdata.append(sout)
            
            if len(outdata) < 100:
              continue
            
            if torque_source == torque_sources[-1]:
              samples = None
              data = None
              gc.collect()
            
            # pickle.dump(data, open(output_csv.replace(".csv", ".pkl"), "wb"))
            # pickle.dump(data, lzma.open(output_csv.replace(".csv", ".pkl.xz"), "wb"))
            print("creating dataframe")
            df = pd.DataFrame(outdata)
            
            outdata = None
            gc.collect()
            
            # print 5 random rows
            print(df.sample(10))
            
            # Write the DataFrame to a CSV file
            print("writing file")
            df.sample(100).copy().to_csv(os.path.join(input_dir,f"{model}{eps_firmware_str}_{torque_source}_sample.csv"), index=False)#, float_format='%.8g')
            # df.to_csv(os.path.join(input_dir,f"{model}.csv"), index=False)#, float_format='%.8g')
            # feather.write_dataframe(df, os.path.join(input_dir,f"{model}.feather"))
            # df.to_feather(os.path.join(input_dir,f"{model}.feather"))
            feather.write_feather(df, os.path.join(input_dir,f"{model}{eps_firmware_str}_{torque_source}.feather"), version=1)

    return model

# Example usage:
input_dir = '/Volumes/video/scratch-video/latfiles'
# compile a regex pattern to match valid subdirectory names
pattern = re.compile(r'^[\-A-Z0-9a-z() ]+$')
def has_upper_word(text):
    words = text.split()
    for word in words:
        if word.isupper():
            return True
    return False

# iterate over all directories and subdirectories in the specified path
whitelist = [] + STEER_AND_EPS_SOURCE_CARS
blacklist = ["nissan", "ford", "mock"]
dirlist=[]
ignore_files_with_past_num_days = 0
check_time = time.time() - ignore_files_with_past_num_days * 86400
print_stats = True
save_output = True
for root, dirs, files in os.walk(input_dir):
    for dir_name in dirs:
        # check if the directory name matches the regex pattern
        if pattern.match(dir_name) and has_upper_word(dir_name):
            d = os.path.join(root, dir_name)
            if (len(whitelist) > 0 and not any([w in d for w in whitelist])) or (any([b in d for b in blacklist])):
              continue
            # get modified time of the "first" png and feather files in the directory.
            # if the modified time is within the cutoff and the corresponding flag is set, skip directory.
            # Get png file modifited time:
            if print_stats:
              png_files = sorted([f for f in os.listdir(d) if f.endswith('.png') if "_b'" not in f or "combined" in f])
              if len(png_files) > 0:
                png_file = os.path.join(d, png_files[0])
                png_file_mtime = os.path.getmtime(png_file)
                if png_file_mtime > 0 and png_file_mtime > check_time:
                  print(f"Skipping {d} because png file is too new")
                  continue
            if save_output:
              feather_files = sorted([f for f in os.listdir(d) if f.endswith('.feather') if "_b'" not in f or "combined" in f])
              if len(feather_files) > 0:
                feather_file = os.path.join(d, feather_files[0])
                feather_file_mtime = os.path.getmtime(feather_file)
                if feather_file_mtime > 0 and feather_file_mtime > check_time:
                  print(f"Skipping {d} because feather file is too new")
                  continue
            
            print(f"Processing {d}...")
            # try:
            # dirlist.append(d)
            model = pickle_files_to_csv(d, check_modified=False, print_stats=True, save_output=True)
            # blacklist.append(dir_name)
            # except Exception as e:
            #   print(f"Error processing {d}: {e}")
            #   continue

# def process_dir(d):
#   pickle_files_to_csv(d, check_modified=False, print_stats=True)

# for d in dirlist:
#   print(f"Processing {d}...")
#   process_dir(d)