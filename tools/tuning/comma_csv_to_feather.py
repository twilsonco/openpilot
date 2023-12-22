#!/usr/bin/env python3
import gc
import os
import re
import bisect
import datetime
import pandas as pd
import copy
import csv
import unicodedata
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
from collections import deque, defaultdict
from common.numpy_fast import interp
import pyarrow.feather as feather
import selfdrive.car.toyota.values as toyota
from matplotlib import pyplot as plt
from matplotlib.cm import ScalarMappable
from typing import List
import tempfile
import zipfile

DEBUG=0 # number of segments to load. 0 for no debugging

DONGLE_ID_BLACKLIST = {}

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


class CleanLatSample():
  def __init__(self, s):
    self.v_ego = float(s['vEgo'])
    self.a_ego = float(s['aEgo'])
    self.steer_cmd = float(s['steer'])
    self.steer_cmd_filtered = float(s['steerFiltered'])
    self.lateral_accel_steer_angle = float(s['latAccelSteeringAngle'])
    self.lateral_accel_desired = float(s['latAccelDesired'])
    self.lateral_accel_localizer = float(s['latAccelLocalizer'])
    self.steer_angle_deg = float(s['steeringAngleDeg'])
    self.eps_firmware: str = s['epsFwVersion']
    self.roll = float(s['roll'])
    self.t = float(s['t'])
    self.lat_active: bool = s['latActive'] == 'True'
    self.steer_pressed: bool = s['steeringPressed'] == 'True'

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

    
def csv_files_to_feather(input_dir, check_modified=True, print_stats=False, save_output=True, out_path_base=""):
    # List all csv files in the input directory
    carname = os.path.basename(input_dir)
    csv_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.csv')])
    
    data = []
    
    make = ""
    
    for csv_file in csv_files:
      with open(os.path.join(input_dir, csv_file), 'rb') as f:
        csv_reader = csv.reader(csv_file)
        
    # Iterate through the csv files and load the data
    i=0
    tot_num_points = 0
    # random.shuffle(csv_files)
    print("Loading csv files...")
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
    
    steer_pressed_buffer = 20 # data is at 10Hz, so this is 1 second
    middle_point = steer_pressed_buffer // 2
    tot_num_points_by_eps_firmware = defaultdict(int)
    if DEBUG:
      random.shuffle(csv_files)
    for csv_file in (pbar := tqdm(csv_files)):
      filename = os.path.basename(csv_file)
      in_deque = deque(maxlen=steer_pressed_buffer)
      with open(os.path.join(input_dir, csv_file), 'r') as f:
        csv_reader = csv.DictReader(f)
        for s in csv_reader:
          # print(s)
          # continue
          try:
            in_deque.append(CleanLatSample(s))
            tot_num_points_by_eps_firmware[in_deque[-1].eps_firmware] += 1
            tot_num_points += 1
            if len(in_deque) == steer_pressed_buffer \
                                and not any([s.steer_pressed or not s.lat_active for s in in_deque]) \
                                and all([s.lat_active and abs(s.a_ego) < 2.0 for s in in_deque]):
              samples.append(in_deque[middle_point])
              if DEBUG and i > DEBUG:
                break
              i += 1
          except Exception as e:
            print(f"Error processing {filename}: {e}, on {s} at line {csv_reader.line_num}")
      pbar.set_description(f"Imported {i} points (rejected {(tot_num_points-i)/max(1,tot_num_points)*100.0:.1f}% of {tot_num_points})")
      if DEBUG and i > DEBUG:
        break
    
    print(f"Imported {i} points (rejected {(tot_num_points-i)/max(1,tot_num_points)*100.0:.1f}% of {tot_num_points})")
    print(f"{len(samples) = }")
    
    if len(samples) == 0:
      return False
    
    # get unique list of firmware versions
    eps_firmwares = list(set([s.eps_firmware for s in samples if s.eps_firmware != '']))
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
      samples = [s for s in full_samples if eps_firmware == 'combined' or s.eps_firmware == eps_firmware]
      
      steer_cmd_stats = describe([s.steer_cmd for s in samples])
      steer_cmd_filtered_stats = describe([s.steer_cmd_filtered for s in samples])
      long_accel_stats = describe([s.a_ego for s in samples])
      v_ego_stats = describe([s.v_ego for s in samples])
      steering_angle_deg_stats = describe([s.steer_angle_deg for s in samples])
      roll_stats = describe([s.roll for s in samples])
      lat_accel_steer_angle_stats = describe([s.lateral_accel_steer_angle for s in samples])
      lat_accel_desired_stats = describe([s.lateral_accel_desired for s in samples])
      lat_accel_localizer_stats = describe([s.lateral_accel_localizer for s in samples])
      
      tot_num_points = tot_num_points_by_eps_firmware[eps_firmware] if eps_firmware != 'combined' else tot_num_points_full
      if print_stats and len(samples) > 0:
        
        print("Preparing plots")
        
        # convert to mph
        for s in samples:
          s.v_ego *= 2.24
        
        speed_bins = range(0,91,18)
        batch_size = 50000

        # create the subplots
        plt.clf()
        fig, axs = plt.subplots(nrows=5, ncols=6, figsize=(24, 12))
        
        # adjusted data info
        print(f"data info: {len(samples)} samples")

        # loop through the speed bins and plot the data
        for i in range(len(speed_bins) - 1):
          try:
            # get the data for this speed bin
            v_ego_min = speed_bins[i]
            v_ego_max = speed_bins[i+1]
            
            # print(f"  generating plots for {v_ego_min}-{v_ego_max}")
            
            # plot the data in the left column with -steer cmd as y value
            data = [sample for sample in samples if v_ego_min <= sample.v_ego < v_ego_max]
            
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            colors = [sample.v_ego for sample in data]
            # data = data[:min(batch_size, len(data))]
            # print(f"    {len(data)} eps samples")
            col_num = -1
            if len(data) > 0:
              col_num += 1
              ax = axs[i][col_num]
              y = [sample.steer_cmd for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel_steer_angle for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Steer cmd @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel steer angle [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-1.0, 1.0)
              
              col_num += 1
              ax = axs[i][col_num]
              y = [sample.steer_cmd_filtered for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel_steer_angle for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Steer cmd filtered @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel steer angle [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-1.0, 1.0)
              
              
              
              col_num += 1
              ax = axs[i][col_num]
              y = [sample.steer_cmd for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel_desired for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Steer cmd @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel desired [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-1.0, 1.0)
              
              col_num += 1
              ax = axs[i][col_num]
              y = [sample.steer_cmd_filtered for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel_desired for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Steer cmd filtered @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel desired [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-1.0, 1.0)
              
              
              
              col_num += 1
              ax = axs[i][col_num]
              y = [sample.steer_cmd for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              ax.scatter([-sample.lateral_accel_localizer for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Steer cmd @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel localizer [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-1.0, 1.0)
              
              col_num += 1
              ax = axs[i][col_num]
              y = [sample.steer_cmd_filtered for sample in data]
              max_abs_y = max([abs(y) for y in y] + [1.0])
              # ax.scatter([-sample.lateral_accel_localizer for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              if i == 4:
                ax.scatter([-sample.lateral_accel_localizer for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis', label="vEgo")
                sm = ScalarMappable(cmap='viridis')
                sm.set_array(colors)
                plt.colorbar(sm)
              else:
                ax.scatter([-sample.lateral_accel_localizer for sample in data], y, s=1, alpha=0.1, c=colors, cmap='viridis')
              ax.set_title((f"Steer cmd filtered @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
              if i == 4:
                ax.set_xlabel("lateral_accel localizer [m/s²]", fontsize=10)
              ax.grid(True)
              ax.set_xlim(-4, 4)
              ax.set_ylim(-1.0, 1.0)
            
            
          except Exception as e:
            print(f"  {e}")
            continue

        # adjust the spacing between subplots
        np.set_printoptions(precision=2)
        # samples are at 10Hz, so 1 minute of data is 600 samples
        approx_logtime = len(samples) / 600.0 / 60.0
        eps_firmware_str = "" if len(eps_firmwares) <= 1 else f" | eps_firmware: {eps_firmware}"
        suptitle=f"{carname} ({approx_logtime:0.0f} hrs of log data; {human_readable(len(samples))} or {len(samples)/max(1,tot_num_points)*100:.2f}% of {human_readable(tot_num_points)} samples retained){eps_firmware_str} | lat_accels vs. steer cmds \nColored by  speed (up to {human_readable(batch_size)} pts per plot) | steer cmd: {describe_to_string(steer_cmd_stats)}\nsteer cmd filtered: {describe_to_string(steer_cmd_filtered_stats)} | lat accel (steer angle): {describe_to_string(lat_accel_steer_angle_stats)}\nlat accel (localizer): {describe_to_string(lat_accel_localizer_stats)} | lat accel (desired) {describe_to_string(lat_accel_desired_stats)}\nv_ego {describe_to_string(v_ego_stats)} | a_ego {describe_to_string(long_accel_stats)}\nroll {describe_to_string(roll_stats)} | steer angle (deg) {describe_to_string(steering_angle_deg_stats)}"
        if DEBUG:
          suptitle = f"<---DEBUG-{DEBUG}--->\n{suptitle}\n<---DEBUG-{DEBUG}--->"
        fig.suptitle(suptitle, fontsize=9)
        fig.subplots_adjust(top=0.85)
        plt.tight_layout()
        
        if out_path_base == "":
          path_base = os.path.join(os.path.dirname(os.path.dirname(input_dir)),"lat_plots")
        else:
          path_base = os.path.join(out_path_base, "lat_plots")
        print(f"Saving to {path_base}...")
        os.makedirs(path_base, exist_ok=True)
        
        eps_firmware_str = "" if len(eps_firmwares) <= 1 else f"_{sanitize(eps_firmware)}"
        plt.savefig(os.path.join(path_base, f"{carname}{eps_firmware_str}_lat.png"))
        plt.clf()
        plt.close("all")
        
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
            
        # data = [s for s in data if ((s.lat_active and s['torque_driver'] <= 0.5) or (not s.lat_active and s['torque_driver'] <= 2.0))]
        # data = [vars(s) for s in data]
        
        # Need to determine values of what the lateral accel and jerk will be in the future at each point,
        # so it can be utilized by the model
        # This could be done using the model's predicted future conditions at each time point but those aren't 
        # accurate relative to what actually happened, and I'd have to regenerate lat files to use the model predictions.
        # On the road, the FF model will have access to up to 2.0s (2.5s minus a max assumed steer actuator delay of 0.5s) into the future of lateral accel and jerk, and
        # this comes over 10 or so data points. We'll sample at 0.25s for 7 points to get to 2.0s.
        # This also requires checking the times of each point.
            
        data = samples
        print(f"  {len(data)} samples")
        
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
          CTRL_RATE = 10
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
          # torque_sources = ['steer_cmd', 'steer_cmd_filtered']
          # lat_accel_sources = ['lateral_accel_steer_angle', 'lateral_accel_localizer', 'lateral_accel_desired']
          torque_sources = ['steer_cmd_filtered']
          # lat_accel_sources = ['lateral_accel_steer_angle']
          lat_accel_sources = ['steer_angle_deg']
          eps_firmware_str = "" if len(eps_firmwares) <= 1 else f"_{sanitize(eps_firmware)}"
          for torque_source in torque_sources:
            for lat_accel_source in lat_accel_sources:
              print(f"  Processing {torque_source} vs {lat_accel_source}...")
              lat_accel_deque = deque(maxlen=max_len)
              roll_deque = deque(maxlen=max_len)
              sample_deque = deque(maxlen=max_len)
              outdata = []
              dt_max = min([j-i for i, j in zip(record_times[:-1], record_times[1:])])
              with tqdm(total=len(data)) as pbar:
                # while len(data) > 0:
                for sample in data:
                  pbar.update(1)
                  # sample = data.pop()
                  s = copy.deepcopy(vars(sample))
                  s['lateral_accel'] = s[lat_accel_source]
                  s['steer_cmd'] = s[torque_source]
                  s['lateral_accel'] *= -1.0
                  s['roll'] *= -1.0
                  if len(sample_deque) > 0 and (s['t'] - sample_deque[-1]['t']) > dt_max:
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
                    Ts = [(s['t'] - sout['t']) for s in sample_deque]
                    sout['lateral_jerk'] = (interp(0.15, Ts, lat_accel_deque) - interp(-0.15, Ts, lat_accel_deque)) / 0.3
                    sout = {**sout, **{f"lateral_accel_{ts}": interp(t, Ts, lat_accel_deque) for t,ts in zip(record_times, record_times_strings)}}
                    sout = {**sout, **{f"roll_{ts}": interp(t, Ts, roll_deque) for t,ts in zip(record_times, record_times_strings)}}
                    sout = {k: sout[k] for k in columns}
                    outdata.append(sout)
              
              if len(outdata) < 100:
                continue
                
              
              # csv.dump(data, open(output_csv.replace(".csv", ".pkl"), "wb"))
              # csv.dump(data, lzma.open(output_csv.replace(".csv", ".pkl.xz"), "wb"))
              print("creating dataframe")
              df = pd.DataFrame(outdata)
              
              # print 5 random rows
              print(df.sample(10))
              
              # Write the DataFrame to a CSV file
              if out_path_base == "":
                path_base = os.path.join(os.path.dirname(os.path.dirname(input_dir.replace("_"," "))),"sample_csv_files")
              else:
                path_base = os.path.join(out_path_base, "sample_csv_files")
              print(f"writing file to directory {path_base}")
              os.makedirs(path_base, exist_ok=True)
              # df.sample(100).copy().to_csv(os.path.join(path_base,f"{carname}{eps_firmware_str}_{torque_source}-vs-{lat_accel_source}_sample.csv"), index=False)#, float_format='%.8g')
              df.sample(100).copy().to_csv(os.path.join(path_base,f"{carname}{eps_firmware_str}_sample.csv"), index=False)#, float_format='%.8g')
              # df.to_csv(os.path.join(input_dir,f"{model}.csv"), index=False)#, float_format='%.8g')
              # feather.write_dataframe(df, os.path.join(input_dir,f"{model}.feather"))
              # df.to_feather(os.path.join(input_dir,f"{model}.feather"))
              if out_path_base == "":
                path_base = os.path.join(os.path.dirname(os.path.dirname(input_dir.replace("_"," "))),"feather_files")
              else:
                path_base = os.path.join(out_path_base, "feather_files")
              os.makedirs(path_base, exist_ok=True)
              feather.write_feather(df, os.path.join(path_base,f"{carname}{eps_firmware_str}.feather"), version=1)

    return True

# point this to the folder containing all the zip files
input_dir = '/Users/haiiro/NoSync/comma-steer-data/zip_files'
# input_dir = '/Volumes/video/scratch-video/comma-data/commaSteeringControl/data'
def has_upper_word(text):
    words = text.split(' ')
    for word in words:
        if word.isupper():
            return True
    return False

# iterate over all directories and subdirectories in the specified path
whitelist = ["RAV4", "IMPREZA", "FORESTER", "ASCENT", "JEEP", "PACIFICA", "ACADIA", "VOLT", "NISSAN", "HONDA"]
whitelist = ["VOLT"]
blacklist = []
dirlist=[]
out_path_base = os.path.dirname(input_dir)
for root, dirs, files in os.walk(input_dir):
    for dir_name in dirs:
        # check if the directory name matches the regex pattern
        if has_upper_word(dir_name):
            d = os.path.join(root, dir_name)
            if (len(whitelist) > 0 and not any([w in d for w in whitelist])) or (any([b in d for b in blacklist])):
              continue
            print(f"Processing {d}...")
            # try:
            # dirlist.append(d)
            if csv_files_to_feather(d, check_modified=False, print_stats=True, save_output=True, out_path_base=out_path_base.replace("_"," ")):
              blacklist.append(dir_name)
            # except Exception as e:
            #   print(f"Error processing {d}: {e}")
            #   continue
    for file in files:
      if (len(whitelist) > 0 and not any([w in file for w in whitelist])) or (any([b in file for b in blacklist])):
        continue
      if file.endswith(".zip"):
        # unzip to a temporary directory and save the directory name to a variable
        with tempfile.TemporaryDirectory() as tmpdirname:
          print(f"Extracting {file} to temp folder: {tmpdirname}...")
          with zipfile.ZipFile(os.path.join(root, file), 'r') as zip_ref:
            zip_ref.extractall(tmpdirname)
          # process the directory
          for root1, dirs1, files1 in os.walk(tmpdirname):
            for dir_name in dirs1:
                # check if the directory name matches the regex pattern
                if has_upper_word(dir_name):
                    d = os.path.join(root1, dir_name)
                    if (len(whitelist) > 0 and not any([w in d for w in whitelist])) or (any([b in d for b in blacklist])):
                      continue
                    print(f"Processing {d}...")
                    # try:
                    # dirlist.append(d)
                    
                    if csv_files_to_feather(d, check_modified=False, print_stats=False, save_output=True, out_path_base=out_path_base):
                      blacklist.append(dir_name)

# def process_dir(d):
#   csv_files_to_feather(d, check_modified=False, print_stats=True)

# for d in dirlist:
#   print(f"Processing {d}...")
#   process_dir(d)