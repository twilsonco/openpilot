import numpy as np

PREPROCESS_ONLY = True
REGULARIZED = False
SPEED_PLOTS = True # For all speeds at a fixed angle, plot steer
ANGLE_PLOTS = True # For all angles at a fixed speed, plot steer
FIT_EACH_PLOT = False # Print curve fit for each plot
PLOT_ANGLE_DIST = False # TBD

# Set to override
ANGLE = None
SIGMOID_SPEED = None
SIGMOID = None
SPEED = None

BIN_COUNT = 50
BIN_SIGMA = np.inf
BIN_STD = np.inf

DT_CTRL = 0.01
MAX_COMMAND = 300 # TODO: autodetect from fingerprint

# Section == continuous valid samples of interest, engaged without steering
SECTION_DELAY = 5. # TODO
MIN_SECTION_SECONDS = 10.

SPEED_MIN = 8
SPEED_MAX = 80