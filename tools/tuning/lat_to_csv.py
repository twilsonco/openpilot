#!/usr/bin/env python3

import os
import pickle
import pandas as pd
from tqdm import tqdm  # type: ignore
import numpy as np
import lzma
from math import sin
import random

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

def pickle_files_to_csv(input_dir, output_csv):
    # List all pickle files in the input directory
    pickle_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.lat')])
    
    data = []

    # Iterate through the pickle files and load the data
    i=0
    # random.shuffle(pickle_files)
    for pickle_file in tqdm(pickle_files):
        with open(os.path.join(input_dir, pickle_file), 'rb') as f:
            data.extend([s for s in pickle.load(f) if s.v_ego > 0.2 \
              and ((s.enabled and s.torque_driver/3 <= 0.5) or (not s.enabled and s.torque_driver/3 <= 1.5))])
            # print(f"{data[-1].torque_driver = }")
            # i+=1
            # if i > 1500:
            #     break
              
    for s in data:
        s.torque_driver /= 3
        s.torque_eps /= 3
        s.roll = sin(s.roll) * 9.81
        s.lateral_accel *= -1.0
        s.lateral_jerk *= -1.0
        s.steer_cmd = s.torque_driver + (s.torque_eps if not np.isnan(s.torque_eps) else 0.0)
    
    
    # pickle.dump(data, open(output_csv.replace(".csv", ".pkl"), "wb"))
    # pickle.dump(data, lzma.open(output_csv.replace(".csv", ".pkl.xz"), "wb"))
    print("creating dataframe")
    data = pd.DataFrame([vars(s) for s in data], columns=[
        'v_ego',
        'a_ego',
        'lateral_accel',
        'lateral_jerk',
        'roll',
        'steer_cmd'])
    
    # Write the DataFrame to a CSV file
    print("writing csv")
    data.to_csv(output_csv, index=False, float_format='%.6g')

# Example usage:
input_dir = '/Users/haiiro/NoSync/latfiles/gm/CHEVROLET VOLT PREMIER 2018'
output_csv = '/Users/haiiro/NoSync/voltlat_large.csv'
pickle_files_to_csv(input_dir, output_csv)
