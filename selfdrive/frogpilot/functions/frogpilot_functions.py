import numpy as np

from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params

params = Params()
params_memory = Params("/dev/shm/params")

CRUISING_SPEED = 5  # Roughly the speed cars go when not touching the gas while in drive

class FrogPilotFunctions:
