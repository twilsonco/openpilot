import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.stats import describe
from math import erf, fabs, atan, cos

from selfdrive.config import Conversions as CV

from tools.tuning.lat_settings import *

PI = 3.14159
  
def clip(v,l,h):
  return min(h, max(v, l))

def get_sigmoid_coef(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT):
  return np.vectorize(SIGMOID_COEF_RIGHT if angle > 0. else SIGMOID_COEF_LEFT)


def get_sigmoid_coef_torque(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT):
  return np.vectorize(SIGMOID_COEF_RIGHT if angle < 0. else SIGMOID_COEF_LEFT)

def get_slope_torque(angle, ANGLE_COEF2, ANGLE_OFFSET):
# ANGLE_COEF2 is right
  return np.vectorize(ANGLE_COEF2 if angle < 0. else ANGLE_OFFSET)

def get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED):
  x = ANGLE * (desired_angle + ANGLE_OFFSET)
  sigmoid = x / (1 + fabs(x))
  return (SIGMOID_SPEED * sigmoid * v_ego) + (SIGMOID * sigmoid) + (SPEED * v_ego)
 

def get_steer_feedforward_sigmoid1(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (angle) / max(0.01,speed)
  sigmoid = x / (1. + fabs(x))
  return ((SIGMOID_COEF_RIGHT if (angle) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * (0.01 + speed + SPEED_OFFSET) ** ANGLE_COEF2 + ANGLE_OFFSET * ((angle) * SPEED_COEF - atan((angle) * SPEED_COEF))

def get_steer_feedforward_erf_old(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = erf(x)
  sigmoid *= (SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
  linear = ANGLE_COEF2 * (angle + ANGLE_OFFSET)
  return sigmoid + linear

def get_steer_feedforward_torque_sigmoid(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = x / (1. + fabs(x))
  sigmoid *= (SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
  linear = ANGLE_COEF2 * (angle + ANGLE_OFFSET)
  return sigmoid + linear

def get_steer_feedforward_torque_sigmoid1(lateral_accel_value, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = ANGLE_COEF * (lateral_accel_value + ANGLE_OFFSET) * (40.23 / (max(1.0,v_ego + SPEED_OFFSET))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (lateral_accel_value + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = x / (1. + fabs(x))
  sigmoid *= sigmoid_factor * sigmoid_factor
  sigmoid *= max(0.2, 40.23 / (max(1.0,v_ego + SPEED_OFFSET2))**SPEED_COEF2)
  linear = ANGLE_COEF2 * (lateral_accel_value + ANGLE_OFFSET)
  return sigmoid + linear

def get_steer_feedforward_torque_sigmoid_to_linear(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = x / (1. + fabs(x))
  sigmoid *= sigmoid_factor
  linear = ANGLE_COEF2 * (angle + ANGLE_OFFSET)
  sigmoid += linear
  
  linear_solo = angle * SPEED_COEF2
  max_speed = 30.
  speed_norm = 0.5 * cos(clip(speed / max_speed, 0., 1.) * PI) + 0.5
  
  out = (1-speed_norm) * linear_solo + (speed_norm) * sigmoid
  
  return out

def get_steer_feedforward_erf1(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid_factor = (SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT)
  sigmoid = erf(x)
  sigmoid *= sigmoid_factor * sigmoid_factor
  sigmoid *= (40.23 / (max(0.05,speed + SPEED_OFFSET2))**SPEED_COEF2)
  linear = ANGLE_COEF2 * (angle + ANGLE_OFFSET)
  return sigmoid + linear


def get_steer_feedforward_bolt_euv_old(desired_angle, v_ego):
  ANGLE = 0.0758345580739845
  ANGLE_OFFSET = 0.#31396926577596984
  SIGMOID_SPEED = 0.04367532050459129
  SIGMOID = 0.43144116109994846
  SPEED = -0.002654134623368279
  return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED)

def get_steer_feedforward_bolt_euv(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2)))

def get_steer_feedforward_bolt_euv_torque(desired_lateral_accel, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (desired_lateral_accel + ANGLE_OFFSET) * (40.23 / (max(1.0,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * (desired_lateral_accel + ANGLE_OFFSET)


def get_steer_feedforward_lacrosse(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2)))

def get_steer_feedforward_lacrosse_torque(desired_lateral_accel, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (desired_lateral_accel + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * (desired_lateral_accel + ANGLE_OFFSET)
  

def get_steer_feedforward_suburban_old(desired_angle, v_ego):
  ANGLE = 0.06562376600261893
  ANGLE_OFFSET = 0.#-2.656819831714162
  SIGMOID_SPEED = 0.04648878299738527
  SIGMOID = 0.21826990273744493
  SPEED = -0.001355528078762762
  return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED)

def get_steer_feedforward_suburban(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2)))

def get_steer_feedforward_suburban_torque_old(angle, speed):
    ANGLE_COEF = 0.66897758
    ANGLE_COEF2 = 0.01000000
    ANGLE_OFFSET = -0.44029828
    SPEED_OFFSET = 2.31755298
    SIGMOID_COEF_RIGHT = 0.35709901
    SIGMOID_COEF_LEFT = 0.36136769
    SPEED_COEF = 0.13870482
    
    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + fabs(x))
    
    linear = angle
    max_speed = 26.
    speed_norm = 0.5 * cos(clip(speed / max_speed, 0., 1.) * PI) + 0.5
    
    return (speed_norm) * linear + (1-speed_norm) * (((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2))))
  
def get_steer_feedforward_palisade_torque_old(angle, speed):
    linear = angle
    linear2 = angle * 0.39
    max_speed = 26.
    speed_norm = 0.5 * cos(clip(speed / max_speed, 0., 1.) * PI) + 0.5
    
    return (speed_norm) * linear + (1-speed_norm) * linear2


def get_steer_feedforward_suburban_torque(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + np.fabs(x))
    
    return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2)))

def get_steer_feedforward_bolt_old(desired_angle, v_ego):
  ANGLE = 0.06370624896135679
  ANGLE_OFFSET = 0.#32536345911579184
  SIGMOID_SPEED = 0.06479105208670367
  SIGMOID = 0.34485246691603205
  SPEED = -0.0010645479469461995
  return get_steer_feedforward_sigmoid(desired_angle, v_ego, ANGLE, ANGLE_OFFSET, SIGMOID_SPEED, SIGMOID, SPEED)

def get_steer_feedforward_bolt_torque_old(desired_lateral_accel, speed):
  ANGLE_COEF = 0.79289935
  ANGLE_COEF2 = 0.24485508
  SPEED_OFFSET = 1.00000000
  SIGMOID_COEF_RIGHT = 0.30436939
  SIGMOID_COEF_LEFT = 0.22542412
  SPEED_COEF = 0.77320476
  ANGLE_OFFSET=0.0
  x = ANGLE_COEF * (desired_lateral_accel + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * (desired_lateral_accel + ANGLE_OFFSET)
  
def get_steer_feedforward_bolt(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
    x = ANGLE_COEF * (angle + ANGLE_OFFSET)
    sigmoid = x / (1. + fabs(x))
    return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) > 0. else SIGMOID_COEF_LEFT) * sigmoid) * ((speed + SPEED_OFFSET) * SPEED_COEF) * ((fabs(angle + ANGLE_OFFSET) ** fabs(ANGLE_COEF2)))

def get_steer_feedforward_bolt_torque(desired_lateral_accel, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (desired_lateral_accel + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (desired_lateral_accel + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * (desired_lateral_accel + ANGLE_OFFSET)

def get_steer_feedforward_volt_old(desired_angle, v_ego):
  ANGLE_COEF = 1.23514093
  ANGLE_COEF2 = 2.00000000
  ANGLE_OFFSET = 0.03891270
  SPEED_OFFSET = 8.58272983
  SIGMOID_COEF_RIGHT = 0.00154548
  SIGMOID_COEF_LEFT = 0.00168327
  SPEED_COEF = 0.16283995
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
# def get_steer_feedforward_volt_torque_old(desired_lateral_accel, v_ego):
#   ANGLE_COEF = 0.08617848
#   ANGLE_COEF2 = 0.14
#   ANGLE_OFFSET = 0.00205026
#   SPEED_OFFSET = -3.48009247
#   SIGMOID_COEF_RIGHT = 0.56664089
#   SIGMOID_COEF_LEFT = 0.50360594
#   SPEED_COEF = 0.55322718
#   return get_steer_feedforward_erf_old(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_volt_torque_old(desired_lateral_accel, v_ego):#, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  ANGLE_COEF = 0.15461558
  ANGLE_COEF2 = 0.22491234
  ANGLE_OFFSET = -0.01173257
  SPEED_OFFSET = 2.37065180
  SIGMOID_COEF_RIGHT = 0.14917052
  SIGMOID_COEF_LEFT = 0.13559770
  SPEED_COEF = 0.49912791
  SPEED_COEF2 = 0.37766423
  SPEED_OFFSET2 = -0.36618369
  return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)

def get_steer_feedforward_volt(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  # ANGLE_COEF = 1.23514093
  # ANGLE_COEF2 = 2.00000000
  # ANGLE_OFFSET = 0.03891270
  # SPEED_OFFSET = 8.58272983
  # SIGMOID_COEF_RIGHT = 0.00154548
  # SIGMOID_COEF_LEFT = 0.00168327
  # SPEED_COEF = 0.16283995
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_volt_torque(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  # ANGLE_COEF = 0.09096546
  # ANGLE_COEF2 = 0.12402084
  # ANGLE_OFFSET = 0.
  # SPEED_OFFSET = -3.35899817
  # SIGMOID_COEF_RIGHT = 0.48819415
  # SIGMOID_COEF_LEFT = 0.55110842
  # SPEED_COEF = 0.57397696
  # return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
  # return get_steer_feedforward_torque_sigmoid1(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
  return get_steer_feedforward_torque_sigmoid1(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_acadia_torque(desired_lateral_accel, v_ego):#, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  ANGLE_COEF = 0.32675089
  ANGLE_COEF2 = 0.22085755
  ANGLE_OFFSET = 0.
  SPEED_OFFSET = -3.17614605
  SIGMOID_COEF_RIGHT = 0.42425039
  SIGMOID_COEF_LEFT = 0.44546354
  SPEED_COEF = 0.78390078
  return get_steer_feedforward_erf_old(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_escalade(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_escalade_torque(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_silverado(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_silverado_torque(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_erf1(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# For comparison with previous best
def old_feedforward(speed, angle):
  
  # sierra silverado combined
  # ANGLE_COEF = 0.06539361463056717
  # SPEED_OFFSET = -0.8390269362439537
  # SIGMOID_COEF_LEFT = 0.023681877712247515
  # SIGMOID_COEF_RIGHT = 0.5709779025308087
  # SPEED_COEF = -0.0016656455765509301
  
  #sierra only
  # ANGLE_COEF = 0.07375408334531243
  # SPEED_OFFSET = -0.43842460609320844
  # SIGMOID_COEF_LEFT = 0.015039986300916987
  # SIGMOID_COEF_RIGHT = 0.6154522080649616
  # SPEED_COEF = -0.00238195057681674
  
  # silverado only
  # ANGLE_COEF = 0.07017408594582242
  # SPEED_OFFSET = -0.7108582322213549
  # SIGMOID_COEF_LEFT = 0.02534582973830592
  # SIGMOID_COEF_RIGHT = 0.5901819029949994
  # SPEED_COEF = -0.0026961086215487357
  # return feedforward(speed, angle, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  # return 0.0002 * (speed ** 2) * angle # old bolt and bolteuv
  # return 0.00004 * (speed ** 2) * angle # old silverado/sierra
  # return 0.000195 * (speed ** 2) * angle # old suburban


  
  if IS_ANGLE_PLOT:
    # old volt sigmoid
    # x = angle * 0.02904609
    # sigmoid = x / (1 + np.fabs(x))
    # return 0.10006696 * sigmoid * (speed + 3.12485927)
    #old acadia sigmoid
    # desired_angle = 0.09760208 * angle
    # sigmoid = desired_angle / (1 + np.fabs(desired_angle))
    # return 0.04689655 * sigmoid * (speed + 10.028217)
    # current acadia sigmoid
    ANGLE_COEF = 5.00000000
    ANGLE_COEF2 = 1.90844451
    ANGLE_COEF3 = 0.03401073
    SPEED_OFFSET = 13.72019138
    SIGMOID_COEF_RIGHT = 0.00100000
    SIGMOID_COEF_LEFT = 0.00101873
    SPEED_COEF = 0.36844505
    return np.vectorize(get_steer_feedforward_sigmoid1)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

    # return np.vectorize(get_steer_feedforward_suburban_old)(angle, speed)
    # return np.vectorize(get_steer_feedforward_bolt_old)(angle, speed)
    # return 0.000195 * (speed ** 2) * angle # old suburban
    return 0.0002 * (speed ** 2) * angle # old bolt and bolteuv
    return 0.00004 * (speed ** 2) * angle # old silverado/sierra, gm default
    # return 0.000045 * (speed ** 2) * angle # old escalade
    # return 0.00005 * (speed ** 2) * angle
  else:
    # return np.vectorize(get_steer_feedforward_bolt_torque_old)(angle, speed)
    # kf = .33 # old volt torque, lacrosse
    # kf = 0.4 # old tahoe torque
    # kf = 0.35 # sonata 
    # return np.vectorize(get_steer_feedforward_palisade_torque_old)(angle,speed)
    # kf = 0.393 # palisade 
    # kf = 1/1.4 # vw (golf, passat)
    kf = 1/2.9638737459977467 # sonata 2020
    # kf = 1/2.544642494803999 # palisade 2020
    # kf = 1/2 # ram 1500
    # kf = 1/1.593387270257916 #pacifica 2018
    # return np.vectorize(get_steer_feedforward_volt_torque_old)(angle, speed)
    # return np.vectorize(get_steer_feedforward_acadia_torque)(angle, speed)
    return angle * kf
    
    # return np.vectorize(get_steer_feedforward_suburban_torque_old)(angle, speed)
    # return np.vectorize(get_steer_feedforward_bolt_torque_old)(angle, speed)
    
    # current acadia
    # ANGLE_COEF = 0.32675089
    # ANGLE_COEF2 = 0.22085755
    # ANGLE_OFFSET = 0.
    # SPEED_OFFSET = -3.17614605
    # SIGMOID_COEF_RIGHT = 0.42425039
    # SIGMOID_COEF_LEFT = 0.44546354
    # SPEED_COEF = 0.78390078
    # return np.vectorize(get_steer_feedforward_erf1)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  



def new_feedforward(speed, angle):
  return feedforward(speed, angle, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)

def feedforward(speed, angle, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  if IS_ANGLE_PLOT:
    # return ANGLE_COEF * angle * speed ** SPEED_COEF # tahoe angle fit

    
    # great for volt
    return np.vectorize(get_steer_feedforward_sigmoid1)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    # x = ANGLE_COEF * (angle) / np.fmax(0.01,speed)
    # sigmoid = x / (1. + np.fabs(x))
    # return (np.vectorize(get_sigmoid_coef)(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT) * sigmoid) * (0.01 + speed + SPEED_OFFSET) ** ANGLE_COEF2 + ANGLE_OFFSET * (angle * SPEED_COEF - np.arctan(angle * SPEED_COEF))
    
    # return np.vectorize(get_steer_feedforward_bolt_euv)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    
    return np.vectorize(get_steer_feedforward_lacrosse)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    return np.vectorize(get_steer_feedforward_suburban)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  else:
    # return ANGLE_COEF * (angle + ANGLE_OFFSET) / np.log(np.fmax(1.0, speed)) / (np.log(SPEED_COEF)) 
    
    # bolt
    # x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (np.fmax(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
    # sigmoid = np.vectorize(erf)(x)
    # return (np.vectorize(get_sigmoid_coef)(angle + ANGLE_OFFSET) * sigmoid) + ANGLE_COEF2 * (angle + ANGLE_OFFSET)
    
    # return np.vectorize(get_steer_feedforward_bolt_euv_torque)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    # return np.vectorize(get_steer_feedforward_lacrosse_torque)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    
    # suburban
    return np.vectorize(get_steer_feedforward_volt_torque)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)
    
    # great for volt
    # return np.vectorize(get_steer_feedforward_erf1)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
    # x = ANGLE_COEF * (angle) * (40.23 / (np.fmax(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
    # sigmoid = np.vectorize(erf)(x)
    # return (np.vectorize(get_sigmoid_coef)(angle, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * angle
    
    # great for silverado/sierra
    # x = ANGLE_COEF * (angle) * (40.23 / (np.fmax(0.2,speed)))
    # sigmoid = np.vectorize(erf)(x)
    # return (np.vectorize(get_sigmoid_coef)(angle) * sigmoid) + np.vectorize(get_slope)(angle) * angle
    
    # x = ANGLE_COEF * (angle)
    # # sigmoid = x / (1. + np.fabs(x))
    # sigmoid = np.vectorize(erf)(x)
    # # sigmoid = np.arcsinh(x)
    # return (np.vectorize(get_sigmoid_coef)(angle) * sigmoid) + 0.1 * angle

# def feedforward(speed, angle, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):

#   return angle * ANGLE_COEF / (np.maximum(speed - SPEED_OFFSET, 0.1) * SPEED_COEF)**SIGMOID_COEF_LEFT

def _fit_kf(x_input, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2):
  speed, angle = x_input.copy()
  return feedforward(speed, angle, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2)

def fit(speed, angle, steer, angle_plot=True):
  global IS_ANGLE_PLOT
  IS_ANGLE_PLOT = angle_plot
  
  print(f'speed: {describe(speed)}')
  print(f'angle: {describe(angle)}')
  print(f'steer: {describe(steer)}')
  
  print("Performing fit...")
  
  global ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2, BOUNDS
  BOUNDS = ([0.001, 0.01, 0.4, 0., 0.1, 0.1, 0.001, 0., 0.],
            [100., 2.0, 1.0, 20., 5., 5.0, 10., 1., 1.]) if IS_ANGLE_PLOT else \
          ([0.001, 0.22, -3., 15.0, 0.1, 0.1, 0.1, 0.1, 30.0],
          [5., 0.23, 3., 40., 0.5, 0.5, 2.0, 1.0, 40.0])
  params, _ = curve_fit(  # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
    _fit_kf,
    np.array([speed, angle]),
    np.array(steer),
    maxfev=900000,
    bounds=BOUNDS
  )
  ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF, SPEED_COEF2, SPEED_OFFSET2 = params
  print(f'Fit: {params}')
  i = 0
  print(f"{ANGLE_COEF = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{ANGLE_COEF2 = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{ANGLE_OFFSET = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{SPEED_OFFSET = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{SIGMOID_COEF_RIGHT = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{SIGMOID_COEF_LEFT = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{SPEED_COEF = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{SPEED_COEF2 = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  i += 1
  print(f"{SPEED_OFFSET2 = :.8f} in [{BOUNDS[0][i]}, {BOUNDS[1][i]}]")
  print(f"{BOUNDS = }")

  old_residual = np.fabs(old_feedforward(speed, angle) - steer)
  new_residual = np.fabs(new_feedforward(speed, angle) - steer)
  old_mae = np.mean(old_residual)
  new_mae = np.mean(new_residual)
  print('MAE old {}, new {}'.format(round(old_mae, 4), round(new_mae, 4)))
  old_std = np.std(old_residual)
  new_std = np.std(new_residual)
  print('STD old {}, new {}'.format(round(old_std, 4), round(new_std, 4)))
  
  if not os.path.exists("plots"):
    os.mkdir("plots")
  with open("plots/out.txt","a") as f:
    f.write(f"    {ANGLE_COEF = :.8f}\n")
    f.write(f"    {ANGLE_COEF2 = :.8f}\n")
    f.write(f"    {ANGLE_OFFSET = :.8f}\n")
    f.write(f"    {SPEED_OFFSET = :.8f}\n")
    f.write(f"    {SIGMOID_COEF_RIGHT = :.8f}\n")
    f.write(f"    {SIGMOID_COEF_LEFT = :.8f}\n")
    f.write(f"    {SPEED_COEF = :.8f}\n")
    f.write(f"    {SPEED_COEF2 = :.8f}\n")
    f.write(f"    {SPEED_OFFSET2 = :.8f}\n")
    f.write('mean absolute error: old {}, new {}\n'.format(round(old_mae, 4), round(new_mae, 4)))
    f.write('standard deviation: old {}, new {}\n'.format(round(old_std, 4), round(new_std, 4)))
    f.write(f"fit computed using {len(speed)} points")

def plot(speed, angle, steer):
  if SPEED_PLOTS:
    os.system('rm plots/deg*')
    abs_angle = np.fabs(angle)
    abs_steer = np.fabs(steer)

    # if PLOT_ANGLE_DIST:
    #   sns.distplot([
    #       line['angle'] for line in data if abs(line['angle']) < 30
    #   ],
    #                bins=200)
    #   raise Exception

    res = 1000

    if IS_ANGLE_PLOT:
      _angles = []
      STEP = 3 # degrees
      for a in range(0, 45, STEP):
        _angles.append([a, a + STEP])
      _angles = np.r_[_angles]
    else: # lateral acceleration plot
      _angles = []
      STEP = 0.2 # degrees
      astart = 0.
      aend = 5.
      for a in np.linspace(astart, aend, num=int((aend-astart)/STEP)).tolist():
        _angles.append([a, a + STEP])
      _angles = np.r_[_angles]

    angle_points_dict = {}
    
    for angle_range in _angles:
      if IS_ANGLE_PLOT:
        start = round(angle_range[0])
        end = round(angle_range[1])
        angle_range_str = f'deg {start:02d}-{end:02d}'
      else:
        start = angle_range[0]
        end = angle_range[1]
        angle_range_str = f'lat_accel {start:.2f}-{end:.2f}'
      mask = (angle_range[0] <= abs_angle) & (abs_angle <= angle_range[1])

      plot_speed = speed[mask]
      plot_angle = abs_angle[mask]
      plot_steer = abs_steer[mask]

      params = None
      if FIT_EACH_PLOT and sum(mask) > 4:
        try:
          global ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF
          params, _ = curve_fit(  # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
            _fit_kf,
            np.array([plot_speed, plot_angle]),
            np.array(plot_steer),
            maxfev=9000,
          )
          ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF = params
        except RuntimeError as e:
          print(e)
          continue
      
      angle_points_dict[angle_range_str] = len(plot_speed)
      # print(f'{angle_range_str} ({len(plot_speed)}): {params}')
      plt.figure(figsize=(12,8))
      plt.scatter(plot_speed * CV.MS_TO_MPH,
                  plot_steer,
                  label=angle_range_str,
                  color='black',
                  s=1.)

      _x_ff = np.linspace(0, 80, res)
      _y_ff = [
          old_feedforward(_i, np.mean(angle_range))
          for _i in _x_ff
      ]
      plt.plot(_x_ff * CV.MS_TO_MPH,
               _y_ff,
               color='red',
               label='old')

      _y_ff = [
          new_feedforward(_i, np.mean(angle_range))
          for _i in _x_ff
      ]
      plt.plot(_x_ff * CV.MS_TO_MPH,
               _y_ff,
               color='blue',
               label='new')

      plt.title(angle_range_str)
      plt.legend(loc='upper left')
      plt.xlabel('speed (mph)')
      plt.ylabel('steer')
      plt.ylim(0., 1.5)
      plt.xlim(SPEED_MIN, SPEED_MAX)
      plt.grid(axis='x', color='0.95')
      plt.grid(axis='y', color='0.95')
      if not os.path.isdir('plots'):
        os.mkdir('plots')
      plt.savefig(f'plots/{angle_range_str}.png')
      plt.close()
  
    print(''.join([ "{}:{}{}".format(k,v,'\n' if (ki+1) % 5 == 0 else ', ') for ki,(k,v) in enumerate(sorted(list(angle_points_dict.items()), key=lambda x: x[0]))]))
    

  if ANGLE_PLOTS:
    os.system('rm plots/mph*')
    # if PLOT_ANGLE_DIST:
    #   sns.displot([
    #       line['angle'] for line in data if abs(line['angle']) < 30
    #   ],
    #               bins=200)
    #   raise Exception

    res = 1000

    _speeds = []
    STEP = 10 # mph
    for s in range(SPEED_MIN, 90, STEP):
      _speeds.append([s, s + STEP])
    _speeds = np.r_[_speeds]

    angle_points_dict = {}
    for speed_range in _speeds:
      start = round(speed_range[0])
      end = round(speed_range[1])
      speed_range_str = f'mph {start:02d}-{end:02d}'
      mask = (speed_range[0] <= speed * CV.MS_TO_MPH) & (speed * CV.MS_TO_MPH <= speed_range[1])

      plot_speed = speed[mask]
      plot_angle = angle[mask]
      plot_steer = steer[mask]

      params = None
      if FIT_EACH_PLOT and sum(mask) > 4:
        try:
          params, _ = curve_fit( # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
            _fit_kf,
            np.array([plot_speed, plot_angle]),
            np.array(plot_steer),
            maxfev=9000,
          )
          ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF = params
        except RuntimeError as e:
          print(e)
          continue

      # print(f'{speed_range_str} ({len(plot_speed)}): {params}')
      angle_points_dict[speed_range_str] = len(plot_speed)
      plt.figure(figsize=(12,8))
      plt.scatter(plot_angle, plot_steer, label=speed_range_str, color='black', s=1.)

      if IS_ANGLE_PLOT:
        _x_ff = np.linspace(-60, 60, res)
      else:
        _x_ff = np.linspace(-3.5, 3.5, res)
      _y_ff = [
          old_feedforward(np.mean(speed_range) * CV.MPH_TO_MS, _i) for _i in _x_ff
      ]
      plt.plot(
          _x_ff,
          _y_ff,
          color='red',
          label='old'
      )
      _y_ff = [
          new_feedforward(np.mean(speed_range) * CV.MPH_TO_MS, _i)
          for _i in _x_ff
      ]
      plt.plot(_x_ff, _y_ff, color='blue', label='new')

      plt.title(speed_range_str)
      plt.legend(loc='lower right')
      if IS_ANGLE_PLOT:
        plt.xlabel('angle (deg)')
        plt.xlim(-55.,55.)
      else:
        plt.xlabel('lateral acceleration (m/s^2)')
        plt.xlim(-3.5,3.5)
      plt.ylabel('steer')
      plt.ylim(-1.5, 1.5)
      plt.grid(axis='x', color='0.95')
      plt.grid(axis='y', color='0.95')
      # plt.xlim(-max(abs(plot_angle)), max(abs(plot_angle)))
      plt.savefig(f'plots/{speed_range_str}.png')
      plt.close()
    
    print(''.join([ "{}:{}{}".format(k,v,'\n' if (ki+1) % 5 == 0 else ', ') for ki,(k,v) in enumerate(sorted(list(angle_points_dict.items()), key=lambda x: x[0]))]))

  if SPEED_PLOTS or ANGLE_PLOTS:
    # Create animations
    cmds = [
      f'rm -rf ~/Downloads/plots',
      # 'convert -delay 40 plots/deg*.png deg-up.gif',
      # 'convert -delay 40 plots/lat*.png deg-up.gif',
      # 'convert -reverse deg-up.gif deg-down.gif',
      # 'convert -loop -1 deg-up.gif deg-down.gif deg.gif',
      'convert -delay 40 plots/mph*.png mph-up.gif',
      'convert -reverse mph-up.gif mph-down.gif',
      'convert -loop -1 mph-up.gif mph-down.gif mph.gif',
      # 'convert -loop -1 deg.gif mph.gif solution.gif',
      'mv *.gif plots/',
      'mv plots ~/Downloads/',
      'rm -f ~/Downloads/plots/deg*.png',
      'rm -f ~/Downloads/plots/lat*.png',
      # 'rm -f ~/Downloads/plots/mph*.png',
      # 'rm -f regularized',
      f'mv ~/Downloads/plots ~/Downloads/plots_{"angle" if IS_ANGLE_PLOT else "torque"}',
      f'cp regularized ~/Downloads/plots_{"angle" if IS_ANGLE_PLOT else "torque"}/regularized'
    ]
    for cmd in cmds:
      print(cmd)
      os.system(cmd)
