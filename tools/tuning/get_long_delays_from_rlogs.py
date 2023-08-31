import os
from collections import defaultdict

import numpy as np
import pandas as pd
from tqdm import tqdm

from tools.lib.logreader import LogReader, MultiLogIterator

def moving_average_filter(data, window_size):
    """
    Applies a moving average filter to the input data.

    Args:
        data (list): A list of floats to be smoothed.
        window_size (int): The size of the moving window.

    Returns:
        list: A list of smoothed floats.
    """
    window = np.ones(window_size) / window_size
    smoothed_data = np.convolve(data, window, mode='same')
    return smoothed_data.tolist()

def lr_to_df(route_name: str, lr: LogReader | MultiLogIterator):
  data = defaultdict(list)
  
  pitch = 0.
  vEgo = 0.
  aEgo = 0.
  user_gas = 0.
  user_brake = 0.
  
  enabled = False
  gas_out = 0.
  brake_out = 0.
  accel = 0.
  
  t_cs = None
  t_llk = None
  t_cc = None

  try:
    for msg in lr:
      if msg.which() == 'carControl':
        t_cc = msg.logMonoTime
        m = msg.carControl
        enabled = m.enabled
        gas_out = m.actuatorsOutput.gas
        brake_out = m.actuatorsOutput.brake
        accel = m.actuators.accel
      elif msg.which() == 'liveLocationKalman':
        t_llk = msg.logMonoTime
        m = msg.liveLocationKalman
        pitch = m.orientationNED.value[1]
      elif msg.which() == 'carState':
        t_cs = msg.logMonoTime
        m = msg.carState
        vEgo = m.vEgo
        aEgo = m.aEgo
        user_gas = m.gas
        user_brake = m.brake
      
      if None not in [pitch, vEgo, enabled]:
        # all data collected
        data['pitch'].append((t_llk, pitch))
        data['v_ego'].append((t_cs, vEgo))
        data['a_ego'].append((t_cs, aEgo))
        data['user_brake'].append((t_cs, user_brake))
        data['user_gas'].append((t_cs, user_gas))
        data['enabled'].append((t_cc, enabled))
        data['gas_out'].append((t_cc, gas_out))
        data['brake_out'].append((t_cc, brake_out))
        data['accel'].append((t_cc, accel))
        
        # set values back to None
        pitch = None
        vEgo = None
        aEgo = None
        user_gas = None
        user_brake = None
        enabled = None
        gas_out = None
        brake_out = None
        accel = None
        t_cs = None
        t_llk = None
        t_cc = None
  except IndexError:
    pass
  
  # smooth data
  for key in ['pitch', 'v_ego', 'a_ego', 'user_brake', 'user_gas', 'enabled', 'gas_out', 'brake_out', 'accel']:
    data[key] = moving_average_filter([v for _, v in data[key]], 5)

  df = pd.DataFrame(data)
  df['route'] = route_name

  df = df[df['v_ego'] > 1e-2]
  return df


def get_routes(base_path='/data/media/0/realdata'):
  all_files = os.listdir(base_path)
  routes = [s[:-3] for s in all_files if s.endswith('--0')]
  return {r: [f for f in all_files if f.startswith(r)] for r in routes}


def get_processed_routes(parquet_file):
  df = pd.read_parquet(parquet_file, columns=['route'])
  return df['route'].unique().tolist()


def process_all_files(base_path='/data/media/0/realdata', outfile=None):
  if outfile is None:
    outfile = os.path.join(base_path, 'processed.parquet')
  routes = get_routes(base_path)
  if os.path.isfile(outfile):
    processed_routes = get_processed_routes(outfile)
  else:
    processed_routes = []
  routes = {k: v for k, v in routes.items() if k not in processed_routes}

  with tqdm(total=sum(len(v) for v in routes.values()), desc='Converting rlogs to parquet') as pbar:
    for route, files in routes.items():
      rlogs = [os.path.join(base_path, f, 'rlog') for f in files]

      batch_size = 5
      for i in range(0, len(rlogs), batch_size):
        logs = rlogs[i:i+batch_size]
        lr = MultiLogIterator(logs, sort_by_time=True)
        df = lr_to_df(route, lr)
        df.to_parquet(outfile, append=os.path.isfile(outfile))
        pbar.update(len(logs))


if __name__ == '__main__':
  process_all_files()
