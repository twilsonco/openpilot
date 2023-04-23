#!/usr/bin/env python3

import os
import re
import pickle
import datetime
import pandas as pd
from tqdm import tqdm  # type: ignore
import numpy as np
from scipy.stats import describe
import math
import random
from collections import deque, defaultdict
from common.numpy_fast import interp
import pyarrow.feather as feather
import selfdrive.car.toyota.values as toyota
from matplotlib import pyplot as plt
from matplotlib.cm import ScalarMappable

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
  def __init__(self, s):
    self.v_ego = s.v_ego
    self.a_ego = s.a_ego
    self.torque_eps = s.torque_eps
    self.torque_driver = s.torque_driver
    self.torque_adjusted: float = 0.0
    self.steer_cmd = s.steer_cmd
    self.lateral_accel = s.lateral_accel
    self.lateral_jerk = s.lateral_jerk
    self.car_fp = s.car_fp
    self.car_make = s.car_make
    self.t = s.t
    self.enabled: bool = s.enabled

def describe_to_string(describe_output):
    nobs, minmax, mean, var, skew, kurtosis = describe_output
    min_val, max_val = minmax
    return f"nobs={nobs:0.1f}, min_max=({min_val:0.1f}, {max_val:0.1f}), mean={mean:0.1f}, var={var:0.1f}, skew={skew:0.1f}, kurtosis={kurtosis:0.1f}"



def compute_adjusted_steer_torque(samples, eps_stats, driver_stats):
  if len(samples) == 0:
    return [], 0.0
  # mean_eps = eps_stats[2]
  # std_eps = np.sqrt(eps_stats[3])
  # mean_driver = driver_stats[2]
  # std_driver = np.sqrt(driver_stats[3])
  if samples[0].car_make == 'gm':
    recip = 1.0 / 3.0
  elif samples[0].car_make == 'volkswagen':
    recip = 1.0 / 300.0
  elif samples[0].car_make == 'hyundai':
    if "KIA" in samples[0].car_fp:
      recip = 1.0 / 600.0
    elif "IONIQ" in samples[0].car_fp:
      recip = 1.0 / 600.0
    else:
      recip = 1.0 / 400.0
  elif samples[0].car_make == 'chrysler':
    recip = 1.0 / (261.0 if samples[0].car_fp != 'RAM HD 5TH GEN' else 361)
  elif samples[0].car_make == 'toyota':
    recip = 1 / 400.0
  elif samples[0].car_make == 'honda':
    recip = 1.0 / 2400.0
  else:
    recip = 0.0
    
  for s in samples:
    s.torque_adjusted = s.torque_driver  * recip
  
  
  samples = lookahead_lookback_filter(samples, lambda s:not s.enabled or s.steer_cmd == 0.0, 5, 5)
  # samples = [s for s in samples if not s.enabled and s.steer_cmd == 0.0 and (std_eps < 0.01 or abs(s.torque_eps - mean_eps) < 0.1 * std_eps)]
  
  return samples, recip

def get_adjusted_steer_torque(sample):
  if sample.car_make == 'gm':
    recip = 1.0 / 3.0
    return (sample.torque_driver + sample.torque_eps) * recip
      
  elif sample.car_make == 'volkswagen':
    recip = 1.0 / 300.0
    return sample.torque_driver * recip
    
  elif sample.car_make == 'hyundai':
    if "KIA" in sample.car_fp:
      recip = 1.0 / 800.0
    else:
      recip = 1.0 / 300.0
    return sample.torque_driver * recip
  
  elif sample.car_make == 'chrysler':
    recip = 1.0 / (261.0 if sample.car_fp != 'RAM HD 5TH GEN' else 361)
    return sample.torque_driver * recip 
  
  elif sample.car_make == 'toyota':
    return sample.torque_driver / (2 * toyota.EPS_SCALE[sample.car_fp])
  
  elif sample.car_make == 'honda':
    recip = 1.0 / 2400.0
    return sample.torque_driver * recip
  
  return 0.0

torque_eps_key = defaultdict(lambda: "torque_eps", {'volkswagen': 'steer_cmd'})
    
def pickle_files_to_csv(input_dir, check_modified=True, print_stats=False):
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
      'roll', # actually lateral gravitational acceleration
      'steer_cmd',
      # "lateral_accel_1",
      # "lateral_jerk_1",
      # "roll_1"
      ]
    
    driver_only_makes = ['toyota', 'honda', 'hyundai', 'volkswagen', 'chrysler']
    samples = []
    random.shuffle(pickle_files)
    for pickle_file in tqdm(pickle_files):
      with open(os.path.join(input_dir, pickle_file), 'rb') as f:
        try:
          pk = pickle.load(f)
        except:
          continue
        for s in pk:
          samples.append(CleanLatSample(s))
          if print_stats:
            samples[-1].v_ego *= 2.24
            # sout = {k:v for k,v in vars(s).items() if k in columns}
            # sout['torque_adjusted'] = get_adjusted_steer_torque(s)
            # sout['car_make'] = s.car_make
            # sout['torque_eps'] = s.torque_eps
            # # sout['t'] = s.t
            # data.append(sout)
        # if i > 100:
        #   break
        # i += 1
        
    eps = [s.torque_eps for s in samples]
    lat_accel = [s.lateral_accel for s in samples]
    driver = [s.torque_driver for s in samples]
    
    eps_stats = describe(eps)
    driver_stats = describe(driver)
    lat_accel_stats = describe(lat_accel)
        
        
    if print_stats and len(samples) > 0:
      
      print("Preparing plots")
      

      v_ego_stats = describe([s.v_ego for s in samples])
      max_abs_eps = max([abs(v) for v in eps_stats[1]])
      mean_eps = eps_stats[2]
      std_eps = np.sqrt(eps_stats[3])
      
      lat_accel = None
      eps = None
      driver = None
      
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
                                                  and abs(sample.a_ego) < 0.2 \
                                                  and sample.enabled \
                                                  and abs(sample.torque_driver - mean_driver) < 0.25 * std_driver]
          dlen = human_readable(len(data))
          data = random.sample(data, min(batch_size, len(data)))
          # print(f"    {len(data)} eps samples")
          if len(data) > 0:
            ax = axs[i][0]
            y = [sample.torque_eps for sample in data]
            max_abs_y = max([abs(y) for y in y] + [1.0])
            ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=[sample.v_ego for sample in data], cmap='viridis')
            ax.set_title(("EPS steer torque @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
            if i == 4:
              ax.set_xlabel("lateral_accel [m/s²]", fontsize=10)
            ax.grid(True)
            ax.set_xlim(-4, 4)
            ax.set_ylim(-max_abs_y, max_abs_y)
          
          # plot the data in the center column with torque_driver as y value
          data = [sample for sample in samples if v_ego_min <= sample.v_ego < v_ego_max \
                                                  and abs(sample.a_ego) < 0.2 \
                                                  and abs(sample.steer_cmd) == 0.0 \
                                                  and not sample.enabled]
                                                  #and (std_eps < 0.001 or (abs(sample.torque_eps - mean_eps) < 0.25 * std_eps))]
          dlen = human_readable(len(data))
          data = random.sample(data, min(batch_size, len(data)))
          # print(f"    {len(data)} driver samples")
          if len(data) > 0:
            ax = axs[i][1]
            y = [sample.torque_driver for sample in data]
            max_abs_y = max([abs(y) for y in y] + [1.0])
            ax.scatter([-sample.lateral_accel for sample in data], y, s=1, alpha=0.1, c=[sample.v_ego for sample in data], cmap='viridis')
            ax.set_title(("Driver steer torque @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
            if i == 4:
              ax.set_xlabel("lateral_accel [m/s²]", fontsize=10)
            ax.grid(True)
            ax.set_xlim(-4, 4)
            ax.set_ylim(-max_abs_y, max_abs_y)
          
          # plot the data in the right column with adjusted torque value
          data = [sample for sample in samples if v_ego_min <= sample.v_ego < v_ego_max \
                                                  and abs(sample.a_ego) < 0.2 \
                                                  and abs(sample.steer_cmd) == 0.0 \
                                                  and not sample.enabled]
          if len(data) > 0:
            data, recip = compute_adjusted_steer_torque(data, eps_stats, driver_stats)
            dlen = human_readable(len(data))
            data = random.sample(data, min(batch_size, len(data)))
            # print(f"    {len(data)} adjusted samples")
            ax = axs[i][2]
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
            ax.set_title((f"Adj. driver ({recip:.2e}) @ " if i == 0 else "") + f"{v_ego_min:0.0f}-{v_ego_max:0.0f}mph ({dlen})", fontsize=12)
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
        except Exception as e:
          print(f"  {e}")
          continue

      # adjust the spacing between subplots
      np.set_printoptions(precision=2)
      approx_logtime = len(pickle_files) / 60.0
      fig.suptitle(f"{carname} ({approx_logtime:0.0f} hrs of log data) | lat_accel vs. eps/driver torque, colored by speed (up to {human_readable(batch_size)} pts per plot)\neps: {describe_to_string(eps_stats)}\ndriver: {describe_to_string(driver_stats)}\nlat accel: {describe_to_string(lat_accel_stats)}\nv_ego {describe_to_string(v_ego_stats)}", fontsize=9)
      fig.subplots_adjust(top=0.85)
      plt.tight_layout()
      
      plt.savefig(os.path.join(input_dir, f"{carname} lat_accel_vs_torque.png"))
      plt.close(fig) 


      # show the plots
      # plt.show()
      
      # print(f"Done with {samples[0].car_fp}")
      return
    
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
    data, recip = compute_adjusted_steer_torque(data, eps_stats, driver_stats)
    
    eps = [s.torque_eps for s in samples]
    lat_accel = [s.lateral_accel for s in samples]
    driver = [s.torque_driver for s in samples]
    eps_stats = describe(eps)
    driver_stats = describe(driver)
    lat_accel_stats = describe(lat_accel)
    
    mean_eps = eps_stats[2]
    std_eps = np.sqrt(eps_stats[3])
    
    if True:
      outdata = []
      dt_max = 0.03
      for s in data:
        try:
          sout = {k: s[k] for k in columns}
          if s['car_make'] in driver_only_makes and not (abs(s['steer_cmd']) == 0.0 and (std_eps < 0.001 or (abs(s['torque_eps'] - mean_eps) < 0.25 * std_eps))):
            continue
          s['lateral_accel'] *= -1.0
          s['lateral_jerk'] *= -1.0
          s['steer_cmd'] = s['torque_adjusted']
          # s['roll'] *= -1.0
          outdata.append(sout)
        except:
          continue
    else:
      # desired_points = 15000000
      CTRL_RATE = 100
      record_times = np.array([-0.3, 0.3, 0.6, 1.1, 2.0])
      columns = [
        'v_ego',
        # 'a_ego',
        'lateral_accel',
        'lateral_jerk',
        'roll',
        'steer_cmd'] + [f"lateral_accel_{i}" for i in range(len(record_times))] + [f"lateral_jerk_{i}" for i in range(len(record_times))] + [f"roll_{i}" for i in range(len(record_times))]
      max_time = max(record_times) - min(record_times) + 0.04
      zero_time_ind = int(-min(record_times) * 100 + 1)
      print(f"Record times: {record_times}")
      max_len = int(max_time * CTRL_RATE)
      i = int(max_len / 2)
      lat_accel_deque = deque(maxlen=max_len)
      lat_jerk_deque = deque(maxlen=max_len)
      roll_deque = deque(maxlen=max_len)
      sample_deque = deque(maxlen=max_len)
      outdata = []
      dt_max = 0.03
      for s in tqdm(data):
        sout = vars(s)
        sout['lateral_accel'] *= -1.0
        sout['lateral_jerk'] *= -1.0
        # s['roll'] *= -1.0
        if len(sample_deque) > 0 and (s['t'] - sample_deque[-1]['t']) * 1e-9 > dt_max:
          lat_accel_deque = deque(maxlen=max_len)
          lat_jerk_deque = deque(maxlen=max_len)
          roll_deque = deque(maxlen=max_len)
          sample_deque = deque(maxlen=max_len)
        else:
          sample_deque.append(sout)
          lat_accel_deque.append(sout['lateral_accel'])
          lat_jerk_deque.append(sout['lateral_jerk'])
          roll_deque.append(sout['roll'])
        
        if len(lat_accel_deque) == max_len:
          sout = sample_deque[zero_time_ind]
          Ts = [(s['t'] - sout['t']) * 1e-9 for s in sample_deque]
          sout = {**sout, **{f"lateral_accel_{i}": interp(t, Ts, lat_accel_deque) - sout['lateral_accel'] for i, t in enumerate(record_times)}}
          sout = {**sout, **{f"lateral_jerk_{i}": interp(t, Ts, lat_jerk_deque) - sout['lateral_jerk'] for i, t in enumerate(record_times)}}
          sout = {**sout, **{f"roll_{i}": interp(t, Ts, roll_deque) - sout['roll'] for i, t in enumerate(record_times)}}
          sout = {k: sout[k] for k in columns}
          outdata.append(sout)
          # if len(outdata) >= desired_points:
          #   break
        
    # outdata = [vars(s) for s in data]
    
    
    
    
    # pickle.dump(data, open(output_csv.replace(".csv", ".pkl"), "wb"))
    # pickle.dump(data, lzma.open(output_csv.replace(".csv", ".pkl.xz"), "wb"))
    print("creating dataframe")
    df = pd.DataFrame(outdata)
    
    # Write the DataFrame to a CSV file
    print("writing file")
    # df.to_csv(os.path.join(input_dir,f"{model}.csv"), index=False)#, float_format='%.8g')
    # feather.write_dataframe(df, os.path.join(input_dir,f"{model}.feather"))
    # df.to_feather(os.path.join(input_dir,f"{model}.feather"))
    feather.write_feather(df, os.path.join(input_dir,f"{model}.feather"), version=1)

# Example usage:
input_dir = '/Users/haiiro/NoSync/latfiles'
# compile a regex pattern to match valid subdirectory names
pattern = re.compile(r'^[A-Z0-9 ]+$')

# iterate over all directories and subdirectories in the specified path
whitelist = ["toyota", "honda", "hyundai", "chrysler"]
whitelist = ["toyota", "hyundai", "gm", "chrysler"]
dirlist=[]
for root, dirs, files in os.walk(input_dir):
    for dir_name in dirs:
        # check if the directory name matches the regex pattern
        if pattern.match(dir_name):
            d = os.path.join(root, dir_name)
            # if not any([w in d.lower() for w in whitelist]):
            #   continue
            # print(f"Processing {d}...")
            # try:
            dirlist.append(d)
            # pickle_files_to_csv(d, check_modified=False, print_stats=True)
            # except Exception as e:
            #   print(f"Error processing {d}: {e}")
            #   continue

def process_dir(d):
  pickle_files_to_csv(d, check_modified=False, print_stats=True)

for d in dirlist:
  print(f"Processing {d}...")
  process_dir(d)