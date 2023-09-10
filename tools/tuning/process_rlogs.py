import os
import sys
from collections import defaultdict

import pandas as pd
from tqdm import tqdm

from tools.lib.logreader import LogReader, MultiLogIterator


def lr_to_df(route_name: str, lr):
  data = defaultdict(list)
  
  pitch = 0.
  vEgo = 0.
  aEgo = 0.
  user_gas = 0.
  user_brake = 0.

  try:
    for msg in lr:
      try:
        if msg.which() == 'carControl':
          m = msg.carControl
          data['enabled'].append(m.enabled)

          data['gas_out'].append(m.actuatorsOutput.gas)
          data['user_gas'].append(user_gas)

          data['brake_out'].append(m.actuatorsOutput.brake)
          data['user_brake'].append(user_brake)

          data['accel'].append(m.actuators.accel)
          data['pitch'].append(pitch)
          data['v_ego'].append(vEgo)
          data['a_ego'].append(aEgo)
        elif msg.which() == 'liveLocationKalman':
          m = msg.liveLocationKalman
          pitch = m.orientationNED.value[1]
        elif msg.which() == 'carState':
          m = msg.carState
          vEgo = m.vEgo
          aEgo = m.aEgo
          user_gas = m.gas
          user_brake = m.brake
      except Exception as e:
        # print(e)
        pass
  except IndexError:
    pass

  df = pd.DataFrame(data)
  df['route'] = route_name

  df = df[df['v_ego'] > 1e-2]
  return df


def get_routes(base_path='/data/media/0/realdata'):
  all_files = os.listdir(base_path)
  routes = [s[:-3] for s in all_files if s.endswith('--0--rlog.bz2')]
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

  with tqdm(total=sum(len(v) for v in routes.values()), desc=f'Converting {base_path}') as pbar:
    for route, files in routes.items():
      rlogs = [os.path.join(base_path, f) for f in files]

      batch_size = 5
      for i in range(0, len(rlogs), batch_size):
        try:
          logs = rlogs[i:i+batch_size]
          lr = MultiLogIterator(logs, sort_by_time=True)
          df = lr_to_df(route, lr)
          df.to_parquet(outfile, append=os.path.isfile(outfile))
          pbar.update(len(logs))
        except Exception as e:
          pass


if __name__ == '__main__':
  if len(sys.argv) > 1:
    base_path = sys.argv[1]
    outfile = sys.argv[2] if len(sys.argv) > 2 else None
    process_all_files(base_path, outfile)
  else:
    process_all_files()
