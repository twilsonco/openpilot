import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
from scipy.stats import describe
from math import erf, fabs, atan, cos

from common.numpy_fast import interp
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

def get_steer_feedforward_erf(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  x = ANGLE_COEF * (angle + ANGLE_OFFSET) * (40.23 / (max(0.05,speed + SPEED_OFFSET))**SPEED_COEF)
  sigmoid = erf(x)
  return ((SIGMOID_COEF_RIGHT if (angle + ANGLE_OFFSET) < 0. else SIGMOID_COEF_LEFT) * sigmoid) + ANGLE_COEF2 * (angle + ANGLE_OFFSET)


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
  ANGLE_COEF = 0.18708832
  ANGLE_COEF2 = 0.28818528
  ANGLE_OFFSET = 0.#21370785
  SPEED_OFFSET = 20.00000000
  SIGMOID_COEF_RIGHT = 0.36997215
  SIGMOID_COEF_LEFT = 0.43181054
  SPEED_COEF = 0.34143006
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
def get_steer_feedforward_volt_torque_old(desired_lateral_accel, v_ego):
  ANGLE_COEF = 0.08617848
  ANGLE_COEF2 = 0.14
  ANGLE_OFFSET = 0.00205026
  SPEED_OFFSET = -3.48009247
  SIGMOID_COEF_RIGHT = 0.56664089
  SIGMOID_COEF_LEFT = 0.50360594
  SPEED_COEF = 0.55322718
  return get_steer_feedforward_erf(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

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
def get_steer_feedforward_volt_torque(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  # ANGLE_COEF = 0.09096546
  # ANGLE_COEF2 = 0.12402084
  # ANGLE_OFFSET = 0.
  # SPEED_OFFSET = -3.35899817
  # SIGMOID_COEF_RIGHT = 0.48819415
  # SIGMOID_COEF_LEFT = 0.55110842
  # SPEED_COEF = 0.57397696
  return get_steer_feedforward_erf(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_acadia_torque(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  # ANGLE_COEF = 0.09096546
  # ANGLE_COEF2 = 0.12402084
  # ANGLE_OFFSET = 0.
  # SPEED_OFFSET = -3.35899817
  # SIGMOID_COEF_RIGHT = 0.48819415
  # SIGMOID_COEF_LEFT = 0.55110842
  # SPEED_COEF = 0.57397696
  return get_steer_feedforward_erf(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_escalade(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_escalade_torque(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_erf(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

def get_steer_feedforward_silverado(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_sigmoid1(desired_angle, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

# Volt determined by iteratively plotting and minimizing error for f(angle, speed) = steer.
def get_steer_feedforward_silverado_torque(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF):
  return get_steer_feedforward_erf(desired_lateral_accel, v_ego, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

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
    # ANGLE_COEF = 5.00000000
    # ANGLE_COEF2 = 1.90844451
    # ANGLE_OFFSET = 0.03401073
    # SPEED_OFFSET = 13.72019138
    # SIGMOID_COEF_RIGHT = 0.00100000
    # SIGMOID_COEF_LEFT = 0.00101873
    # SPEED_COEF = 0.36844505
    # return np.vectorize(get_steer_feedforward_sigmoid1)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)

    # return np.vectorize(get_steer_feedforward_suburban_old)(angle, speed)
    # return np.vectorize(get_steer_feedforward_bolt_old)(angle, speed)
    # return 0.000195 * (speed ** 2) * angle # old suburban
    # return 0.0002 * (speed ** 2) * angle # old bolt and bolteuv
    # return 0.00004 * (speed ** 2) * angle # old silverado/sierra
    # return 0.000045 * (speed ** 2) * angle # old escalade
    # return 0.00005 * (speed ** 2) * angle
    return 0
    return get_steer_feedforward_volt_old(angle, speed)
  else:
    return 0
    return get_steer_feedforward_volt_torque_old(angle, speed)
    # return np.vectorize(get_steer_feedforward_bolt_torque_old)(angle, speed)
    # kf = .33 # old volt torque
    # kf = 0.4 # old tahoe torque
    # kf = 0.35 # sonata 
    return np.vectorize(get_steer_feedforward_palisade_torque_old)(angle,speed)
    kf = 0.393 # palisade 
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
    # return np.vectorize(get_steer_feedforward_erf)(angle, speed, ANGLE_COEF, ANGLE_COEF2, ANGLE_OFFSET, SPEED_OFFSET, SIGMOID_COEF_RIGHT, SIGMOID_COEF_LEFT, SPEED_COEF)
  



def new_feedforward(speed, angle):
  ds,dv = build_steer_correction_lists(DS4L, DS3L, DS2L, DS1L, DS1R, DS2R, DS3R, DS4R, DV4, DV3, DV2, DV1)
  # LATACCELFACTOR = 2.0
  # steer_lataccel_factors = [1.5, 1.15, 1.02, 1.0, 1.0, 1.0, 1.02, 1.15, 1.5]
  # vego_lataccel_factors = [1.5, 1.5, 1.25, 1.0, 1.0]
  # ds = steer_lataccel_factors
  # dv = vego_lataccel_factors
  return feedforward_generic(angle, speed, LATACCELFACTOR, ds,dv)

steer_break_pts = [-1.0, -0.9, -0.75, -0.5, 0.0, 0.5, 0.75, 0.9, 1.0]
vego_break_pts = [0.0, 10.0, 15.0, 20.0, 100.0]
def feedforward_generic(lateral_accel_value, vego, latAccelFactor, steer_lataccel_factors, vego_lataccel_factors):
  if type(lateral_accel_value) in [np.float64,float]:
    steer_torque = lateral_accel_value / latAccelFactor

    steer_correction_factor = np.interp(
      steer_torque,
      steer_break_pts,
      steer_lataccel_factors
    )

    vego_correction_factor = np.interp(
      vego,
      vego_break_pts,
      vego_lataccel_factors,
    )
    
    combined_factors = min(latAccelFactor, steer_correction_factor * vego_correction_factor)
    out = (steer_torque) / combined_factors
    # print(f"{lateral_accel_value = :0.2f} {vego = :.1f} {out = :.2f}")
    return out
  else:
    steer_torque = lateral_accel_value / latAccelFactor

    steer_correction_factor = np.interp(
      steer_torque,
      steer_break_pts,
      steer_lataccel_factors
    )

    vego_correction_factor = np.interp(
      vego,
      vego_break_pts,
      vego_lataccel_factors,
    )
    combined_factors = np.minimum([latAccelFactor] * len(lateral_accel_value), steer_correction_factor * vego_correction_factor)
    out = (steer_torque) / (steer_correction_factor * vego_correction_factor)
    # print(lateral_accel_value, vego, out)
    return out
  
# def feedforward_generic(lateral_accel_value, vego, latAccelFactor, steer_lataccel_factors, vego_lataccel_factors):
#   steer_torque = lateral_accel_value / latAccelFactor

#   steer_correction_factor = np.interp(
#     steer_torque,
#     steer_break_pts,
#     steer_lataccel_factors
#   )

#   vego_correction_factor = np.interp(
#     vego,
#     vego_break_pts,
#     vego_lataccel_factors,
#   )
#   out = (steer_torque) / (steer_correction_factor * vego_correction_factor)
#   # print(lateral_accel_value, vego, out)
#   return out

def build_steer_correction_lists(DS4L, DS3L, DS2L, DS1L, DS1R, DS2R, DS3R, DS4R, DV4, DV3, DV2, DV1):
  ds = [1.0] * 9
  DSL = [DS4L, DS3L, DS2L, DS1L]
  DSL.reverse()
  DSR = [DS1R, DS2R, DS3R, DS4R]
  dv = [1.0] * 5
  DV = [DV4, DV3, DV2, DV1]
  DV.reverse()
  for i in range(1,5):
    ds[4-(i)] += sum(DSL[:i])
    ds[4+(i)] += sum(DSR[:i])
    dv[-(i+1)] += sum(DV[:i])
  return ds,dv

def _fit_kf(x_input, LATACCELFACTOR, DS4L, DS3L, DS2L, DS1L, DS1R, DS2R, DS3R, DS4R, DV4, DV3, DV2, DV1):
  speed, angle = x_input.copy()
  ds,dv = build_steer_correction_lists(DS4L, DS3L, DS2L, DS1L, DS1R, DS2R, DS3R, DS4R, DV4, DV3, DV2, DV1)
  return feedforward_generic(angle, speed, LATACCELFACTOR, ds,dv)

def fit(speed, angle, steer, angle_plot=True):
  global IS_ANGLE_PLOT
  IS_ANGLE_PLOT = angle_plot
  
  print(f'speed: {describe(speed)}')
  print(f'angle: {describe(angle)}')
  print(f'steer: {describe(steer)}')
  
  print("Performing fit...")
  
  global LATACCELFACTOR, DS4L, DS3L, DS2L, DS1L, DS1R, DS2R, DS3R, DS4R, DV4, DV3, DV2, DV1
  
  params, _ = curve_fit(  # lgtm[py/mismatched-multiple-assignment] pylint: disable=unbalanced-tuple-unpacking
    _fit_kf,
    np.array([speed, angle]),
    np.array(steer),
    maxfev=900000,
    bounds=([0.2] + [-1.0] * 8 + [-1.0] * 4,
            [4.0] + [1.0] * 8 + [1.0] * 4)
  )
  LATACCELFACTOR, DS4L, DS3L, DS2L, DS1L, DS1R, DS2R, DS3R, DS4R, DV4, DV3, DV2, DV1 = params
  print(f'Fit: {params}')
  steer_lataccel_factors, vego_lataccel_factors = build_steer_correction_lists(DS4L, DS3L, DS2L, DS1L, DS1R, DS2R, DS3R, DS4R, DV4, DV3, DV2, DV1)
  # LATACCELFACTOR = 2.0
  # steer_lataccel_factors = [1.5, 1.15, 1.02, 1.0, 1.0, 1.0, 1.02, 1.15, 1.5]
  # vego_lataccel_factors = [1.5, 1.5, 1.25, 1.0, 1.0]
  print(f"{LATACCELFACTOR = :.8f}")
  print(f"{steer_lataccel_factors = }")
  print(f"{vego_lataccel_factors = }")

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
    f.write(f"    {LATACCELFACTOR = :.8f}\n")
    f.write(f"    {steer_lataccel_factors = }\n")
    f.write(f"    {vego_lataccel_factors = }\n")
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
      aend = 3.
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
    STEP = 5 # mph
    for s in range(SPEED_MIN, SPEED_MAX, STEP):
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
      # 'convert -delay 20 plots/deg*.png deg-up.gif',
      # 'convert -delay 20 plots/lat*.png deg-up.gif',
      # 'convert -reverse deg-up.gif deg-down.gif',
      # 'convert -loop -1 deg-up.gif deg-down.gif deg.gif',
      'convert -delay 20 plots/mph*.png mph-up.gif',
      'convert -reverse mph-up.gif mph-down.gif',
      'convert -loop -1 mph-up.gif mph-down.gif mph.gif',
      # 'convert -loop -1 deg.gif mph.gif solution.gif',
      'mv *.gif plots/',
      'mv plots ~/Downloads/',
      'rm -f ~/Downloads/plots/deg*.png',
      'rm -f ~/Downloads/plots/lat*.png',
      'rm -f ~/Downloads/plots/mph*.png',
      'rm -f regularized',
      f'mv ~/Downloads/plots ~/Downloads/plots_{"angle" if IS_ANGLE_PLOT else "torque"}'
    ]
    for cmd in cmds:
      print(cmd)
      os.system(cmd)
