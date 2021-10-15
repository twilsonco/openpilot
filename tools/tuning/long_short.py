#!/usr/bin/env python3

# This is modified from Shane Smiskol's auto-ff-new
# https://github.com/sshane/openpilot/tree/auto-ff-new

# Universal longitudinal map from acceleration to gasbrake.

import argparse
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

def collect(logreader):
    return

def load(path, route=None):
  print(f'Loading from rlogs {route}')
  r = Route(route, data_dir=path)
  lr = MultiLogIterator(r.log_paths(), wraparound=False)
  data = collect(lr)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--path')
  parser.add_argument('--route')
  args = parser.parse_args()

  load(args.path, args.route)
