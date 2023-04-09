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

    # Iterate through the pickle files and load the data
    i=0
    # random.shuffle(pickle_files)
    for pickle_file in tqdm(pickle_files):
        with open(os.path.join(input_dir, pickle_file), 'rb') as f:
            pk = pickle.load(f)
            data.extend([s for s in pk if s.v_ego > 0.01])
            # print(f"{data[-1].torque_driver = }")
            # i+=1
            # if i > 1500:
            #     break
              
    for s in data:
        s.lateral_accel *= -1.0
        s.lateral_jerk *= -1.0
        s.steer_cmd = get_steer_torque(s)
        s.roll = -sin(s.roll) * 9.81
        if make == "" and s.car_make != "":
          make = s.car_make
          model = s.car_fp
        
    # data = [s for s in data if ((s.enabled and s.torque_driver <= 0.5) or (not s.enabled and s.torque_driver <= 2.0))]
    # data = [vars(s) for s in data]
    
    # Need to determine values of what the lateral accel and jerk will be in the future at each point,
    # so it can be utilized by the model
    # This could be done using the model's predicted future conditions at each time point but those aren't 
    # accurate relative to what actually happened, and I'd have to regenerate lat files to use the model predictions.
    # On the road, the FF model will have access to up to 2.5s into the future of lateral accel and jerk, and
    # this comes over 10 or so data points. That's probably too many. A value of around 0.4s is the baseline based
    # on steer actuator delays, so we'll include 
    
    
    # pickle.dump(data, open(output_csv.replace(".csv", ".pkl"), "wb"))
    # pickle.dump(data, lzma.open(output_csv.replace(".csv", ".pkl.xz"), "wb"))
    print("creating dataframe")
    df = pd.DataFrame([vars(s) for s in data], columns=[
        'v_ego',
        # 'a_ego',
        'lateral_accel',
        'lateral_jerk',
        'roll', # actually lateral gravitational acceleration
        'steer_cmd'])
    
    # Write the DataFrame to a CSV file
    print("writing csv")
    df.to_csv(os.path.join(input_dir,f"{model}.csv"), index=False, float_format='%.8g')

# Example usage:
input_dir = '/Users/haiiro/NoSync/latfiles/gm/CHEVROLET VOLT PREMIER 2018'
pickle_files_to_csv(input_dir)
