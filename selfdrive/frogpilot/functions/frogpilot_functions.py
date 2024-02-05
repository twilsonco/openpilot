import numpy as np

from openpilot.common.numpy_fast import interp
from openpilot.common.params import Params

params = Params()
params_memory = Params("/dev/shm/params")

class FrogPilotFunctions:
