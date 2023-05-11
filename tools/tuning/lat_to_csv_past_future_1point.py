#!/usr/bin/env python3

import os
import pickle
import pandas as pd
from tqdm import tqdm  # type: ignore
import numpy as np
import lzma
from math import sin
import random
from collections import deque
from common.numpy_fast import interp
import pyarrow.feather as feather

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
  

def get_steer_torque(s):
  if s.car_make == 'gm':
    s.torque_driver /= 3
    s.torque_eps /= 3
    return s.torque_driver + s.torque_eps
  elif s.car_make == 'volkswagen':
    s.torque_driver /= 300
    s.steer_cmd *= 3.0
    return s.torque_driver + (s.steer_cmd if not np.isnan(s.steer_cmd) else 0.0)
  elif s.car_make == 'hyundai':
    s.torque_driver /= 100 * 4
    s.torque_eps /= 5 * 4
    return s.torque_driver + s.torque_eps
  elif s.car_make == 'chrysler':
    s.torque_driver /= 261
    s.torque_eps /= 261
    return s.torque_driver + s.torque_eps
  elif s.car_make == 'mock':
    return 0.0
  else:
    raise ValueError("Unsupported car make")
    
def pickle_files_to_csv(input_dir):
    # List all pickle files in the input directory
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

    # Iterate through the pickle files and load the data
    i=0
    # random.shuffle(pickle_files)
    print("Loading pickle files...")
    record_times = np.array([-0.3, 0.3, 0.6, 1.1, 2.0])
    columns = [
      'v_ego',
      # 'a_ego',
      'lateral_accel',
      'lateral_jerk',
      'roll',
      'steer_cmd'] + [f"lateral_accel_{i}" for i in range(len(record_times))] + [f"lateral_jerk_{i}" for i in range(len(record_times))] + [f"roll_{i}" for i in range(len(record_times))]
    for pickle_file in tqdm(pickle_files):
      with open(os.path.join(input_dir, pickle_file), 'rb') as f:
        try:
          pk = pickle.load(f)
        except:
          continue
        for s in pk:
          sout = {k:v for k,v in vars(s).items() if k in columns}
          sout['steer_cmd'] = get_steer_torque(s)
          sout['t'] = s.t
          data.append(sout)
        # if i > 50:
        #   break
        # i += 1
    
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
    
    # desired_points = 15000000
    CTRL_RATE = 100
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
      s['lateral_accel'] *= -1.0
      s['lateral_jerk'] *= -1.0
      # s['roll'] *= -1.0
      # sout = {k: s[k] for k in columns}
      # outdata.append(sout)
      if len(sample_deque) > 0 and (s['t'] - sample_deque[-1]['t']) * 1e-9 > dt_max:
        lat_accel_deque = deque(maxlen=max_len)
        lat_jerk_deque = deque(maxlen=max_len)
        roll_deque = deque(maxlen=max_len)
        sample_deque = deque(maxlen=max_len)
      else:
        sample_deque.append(s)
        lat_accel_deque.append(s['lateral_accel'])
        lat_jerk_deque.append(s['lateral_jerk'])
        roll_deque.append(s['roll'])
      
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
    df.head(10).copy().to_csv(os.path.join(input_dir,f"{model}_torque-model-input_sample.csv"), index=False)#, float_format='%.8g')
    # feather.write_dataframe(df, os.path.join(input_dir,f"{model}.feather"))
    # df.to_feather(os.path.join(input_dir,f"{model}.feather"))
    feather.write_feather(df, os.path.join(input_dir,f"{model}_torque-model-input.feather"), version=1)

# Example usage:
input_dir = '/Users/haiiro/NoSync/latfiles/gm/CHEVROLET VOLT PREMIER 2018'
pickle_files_to_csv(input_dir)
