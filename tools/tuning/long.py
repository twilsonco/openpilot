#!/usr/bin/env python3

# This is modified from Shane Smiskol's auto-ff-new
# https://github.com/sshane/openpilot/tree/auto-ff-new

# TODO
# Autoload DBC

import os
import sys
import matplotlib.pyplot as plt
import numpy as np

from tqdm import tqdm  # type: ignore
from selfdrive.config import Conversions as CV
from tools.lib.logreader import MultiLogIterator
from tools.lib.route import Route
from cereal import car
from opendbc.can.parser import CANParser

DT_CTRL = 0.01

def collect(lr):
  all_msgs = sorted(lr, key=lambda msg: msg.logMonoTime)
  data = [[]]
  prev_t = 0
  can1_updated = can2_updated = False

  vars = ['t', 'a', 'v', 'traction_force', 'traction_force_active']
  t = a = v = traction_force = traction_force_active = None

  # Select CAN signals
  # Volt has electric traction motor for accel / regen, and friction brakes.
  signals = [
      ("TractionForce", "ASCMTractionForce", 0),
      ("TractionForceActive", "ASCMTractionForce", 0),
  ]
  cp1 = CANParser("gm_global_a_powertrain", signals, enforce_checks=False)
  signals = [
      ("FrictionBrakeCmd", "EBCMFrictionBrakeCmd", 0),
  ]
  cp2 = CANParser("gm_global_a_chassis", signals, enforce_checks=False)

  for msg in tqdm(all_msgs):

    if msg.which() == 'carState':
      v = msg.carState.vEgo
      a = msg.carState.aEgo
    elif msg.which == 'sendcan':
      bytes = msg.as_builder().to_bytes()
      for u in cp1.update_string(bytes):
        if u == 715:  # ASCMTractionForce
          can1_updated = True
      for u in cp2.update_string(bytes):
        if u == 789:  # EBCMFrictionBrakeCmd
          can2_updated = True
          print('brake')
    else:
      continue

    if can1_updated and can2_updated:
      can1_updated = can2_updated = False
    else:
      continue

    traction_force = int(cp1.vl['ASCMTractionForce']['TractionForce'])
    traction_force_active = bool(
        cp1.vl['ASCMTractionForce']['TractionForceActive'])
    no_friction_brake = bool(
        0 == cp2.vl['EBCMFrictionBrakeCmd']['FrictionBrakeCmd'])

    # Continuous sections
    t = msg.logMonoTime * 1e-9
    valid = abs(t - prev_t) < 1 / 10
    prev_t = t

    # All variables are set
    for var in vars:
      valid = valid and eval(var) != None
    valid = valid and traction_force_active
    valid = valid and no_friction_brake

    if valid:
      d = {}
      for var in vars:
        d.update({var: eval(var)})
      data[-1].append(d)
    elif len(data[-1]):  # if last list has items in it, append new section
      print(
          f'Sampling frequency: {len(data[-1]) / (data[-1][-1]["t"] - data[-1][0]["t"])} Hz'
      )
      print(f'Found len(section): {len(data[-1])}')

      # Determine lag of this section
      f = np.array([sample['traction_force'] for sample in data[-1]])
      a = np.array([sample['a'] for sample in data[-1]])
      lag(f, a)

      # TODO: select median lag, then remove it.
      # For now, hardcode 570 ms cmd -> accel
      # Slide acceleration -570 ms
      DELAY_SAMPLES = 57
      for i in range(len(data[-1]) - DELAY_SAMPLES):
        data[-1][i]['a'] = data[-1][i + DELAY_SAMPLES]['a']
      data[-1] = data[-1][:-DELAY_SAMPLES]  # delete last samples

      data.append([])
  del all_msgs

  data = [sec for sec in data
          if len(sec) > MIN_SECTION]  # long enough sections
  data = [i for j in data for i in j]  # flatten

  return data


def load(lr, route_name):
  filename = f'{route_name}--long'
  if os.path.exists(filename):
    print(f'Loading from {filename}')
    with open(filename, 'rb') as f:
      data = pickle.load(f)
  else:
    print('Loading from rlogs')
    data = collect(lr)
    with open(filename, 'wb') as f:  # save processed data
      pickle.dump(data, f)
  print(f'Samples: {len(data)}')
  return data


def _fit(x_input, kf, kv, c):
  v, a = x_input.copy()
  return kf * a + kv * v + c


def plot(data):
  # analyzes how torque needed changes based on speed
  if SPEED_DATA_ANALYSIS := True:
    if PLOT_ANGLE_DIST := False:
      sns.distplot([
          line['steer_angle'] for line in data if abs(line['steer_angle']) < 30
      ],
                   bins=200)
      raise Exception

    res = 100
    color = 'blue'

    _angles = [
        [5, 10],
        [10, 20],
        [10, 15],
        [15, 20],
        [20, 25],
        [20, 30],
        [30, 45],
    ]

    for idx, angle_range in enumerate(_angles):
      angle_range_str = '{} deg'.format('-'.join(map(str, angle_range)))
      temp_data = [
          line for line in data
          if angle_range[0] <= abs(line['steer_angle']) <= angle_range[1]
      ]
      if not len(temp_data):
        continue
      print(f'{angle_range} samples: {len(temp_data)}')
      plt.figure()
      speeds, torque = zip(*([line['v_ego'], line['torque']]
                             for line in temp_data))
      plt.scatter(np.array(speeds) * CV.MS_TO_MPH,
                  torque,
                  label=angle_range_str,
                  color=color,
                  s=0.05)

      _x_ff = np.linspace(0, max(speeds), res)
      _y_ff = [
          old_feedforward(_i, np.mean(angle_range), 0, old_kf) * MAX_TORQUE
          for _i in _x_ff
      ]
      plt.plot(_x_ff * CV.MS_TO_MPH,
               _y_ff,
               color='orange',
               label='standard ff model at {} deg'.format(
                   np.mean(angle_range)))

      _y_ff = [
          feedforward(_i, np.mean(angle_range), offset, fit_kf) * MAX_TORQUE
          for _i in _x_ff
      ]
      plt.plot(_x_ff * CV.MS_TO_MPH,
               _y_ff,
               color='purple',
               label='new fitted ff function')

      plt.legend()
      plt.xlabel('speed (mph)')
      plt.ylabel('torque')
      plt.savefig(f'plots/{angle_range_str}.png')

  # analyzes how angle changes need of torque (RESULT: seems to be relatively linear, can be tuned by k_f)
  if ANGLE_DATA_ANALYSIS := True:
    if PLOT_ANGLE_DIST := False:
      sns.distplot([
          line['steer_angle'] for line in data if abs(line['steer_angle']) < 30
      ],
                   bins=200)
      raise Exception

    res = 100

    _speeds = np.r_[[
        [0, 10],
        [10, 20],
        [20, 30],
        [30, 40],
        [40, 50],
        [50, 60],
        [54, 56],
        [64, 66],
        [60, 70],
    ]] * CV.MPH_TO_MS
    color = 'blue'

    for idx, speed_range in enumerate(_speeds):
      speed_range_str = '{} mph'.format('-'.join(
          [str(round(i * CV.MS_TO_MPH, 1)) for i in speed_range]))
      temp_data = [
          line for line in data
          if speed_range[0] <= line['v_ego'] <= speed_range[1]
      ]
      if not len(temp_data):
        continue
      print(f'{speed_range_str} samples: {len(temp_data)}')

      data_speeds = np.array([line['v_ego'] for line in temp_data])
      data_angles = np.array([line['steer_angle'] for line in temp_data])
      data_torque = np.array([line['torque'] for line in temp_data])

      params, covs = curve_fit(
          _fit_kf,
          np.array([data_speeds, data_angles]),
          np.array(data_torque) / MAX_TORQUE,
          # maxfev=800
      )
      fit_kf = params[0]
      offset = params[1]
      print(params[0])
      print(params[1])

      plt.figure()
      angles, torque, speeds = zip(
          *([line['steer_angle'], line['torque'], line['v_ego']]
            for line in temp_data))
      plt.scatter(angles, torque, label=speed_range_str, color=color, s=0.05)

      _x_ff = np.linspace(0, max(angles), res)
      _y_ff = [
          old_feedforward(np.mean(speed_range), _i, 0, old_kf) * MAX_TORQUE
          for _i in _x_ff
      ]
      plt.plot(_x_ff,
               _y_ff,
               color='orange',
               label='standard ff model at {} mph'.format(
                   np.round(np.mean(speed_range) * CV.MS_TO_MPH, 1)))

      _y_ff = [
          feedforward(np.mean(speed_range), _i, offset, fit_kf) * MAX_TORQUE
          for _i in _x_ff
      ]
      plt.plot(_x_ff, _y_ff, color='purple', label='new fitted ff function')

      plt.legend()
      plt.xlabel('angle (deg)')
      plt.ylabel('torque')
      plt.savefig(f'plots/{speed_range_str}.png')


def filter(data):
  # Filter
  data = [sample for sample in data if sample['v'] > 1]
  #data = [sample for sample in data if sample['traction_force'] > 6400] # only acceleration

  print(f'Samples after filtering:  {len(data)}\n')
  assert len(
      data
  ) > MIN_SAMPLES, f'too few filtered samples: {len(data)} > {MIN_SAMPLES}'

  # Format
  v = np.array([line['v'] for line in data])
  a = np.array([line['a'] for line in data])
  f = np.array([line['traction_force'] for line in data])
  del data

  # Statistics
  for var in ['v', 'a', 'f']:
    print(f'{var}: ' + str(describe(eval(var))))

  # Fit
  params, covs = curve_fit(
      _fit,
      np.array([v, a]),
      np.array(f),
      # maxfev=800
  )

  print(params)
  exit()

  # Old vs. New
  std_func = []
  fitted_func = []
  for line in data:
    std_func.append(
        abs(
            old_feedforward(line['v_ego'], line['steer_angle'], 0, old_kf) *
            MAX_TORQUE - line['torque']))
    fitted_func.append(
        abs(
            feedforward(line['v_ego'], line['steer_angle'], offset, fit_kf) *
            MAX_TORQUE - line['torque']))
  print('MAE stock {}, fitted {}'.format(round(np.mean(std_func), 3),
                                         round(np.mean(fitted_func), 3)))
  print('STD, current vs. fitted: {}, {}'.format(round(np.std(std_func), 3),
                                                 round(np.std(fitted_func),
                                                       3)))

  # plot(data)


if __name__ == '__main__':
  if len(sys.argv) == 3:
    r = Route(sys.argv[1], data_dir=sys.argv[2])
  else:
    r = Route(sys.argv[1])
  lr = MultiLogIterator(r.log_paths(), wraparound=False)

  data = load(lr, r.route_name)
  filter(data)
