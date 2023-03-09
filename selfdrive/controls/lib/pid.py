import numpy as np
from numbers import Number
from collections import deque
from common.differentiator import Differentiator
from common.integrator import Integrator
from common.op_params import opParams
from common.numpy_fast import clip, interp
from math import log2


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


class PIDController:
  def __init__(self, k_p=0., k_i=0., k_d=0., k_f=1., k_11=0., k_12=0., k_13=0., k_period=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, derivative_period=1., integral_period=2.5):
    self._op_params = opParams(calling_function="pid.py")
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_d = k_d  # derivative gain
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    # "autotuned" PID implementation from https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7150979
    self._k_11 = k_11  # proportional gain factor
    self._k_12 = k_12  # integral gain factor
    self._k_13 = k_13  # derivative gain factor
    if isinstance(self._k_11, Number):
      self._k_11 = [[0], [self._k_11]]
    if isinstance(self._k_12, Number):
      self._k_12 = [[0], [self._k_12]]
    if isinstance(self._k_13, Number):
      self._k_13 = [[0], [self._k_13]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.sat_limit = sat_limit
    
    self._gain_update_factor = 0.0
    self.error_integral = Integrator(integral_period, rate, passive=not any([k > 0.0 for k in self._k_i[1]]))
    self.error_rate = Differentiator(derivative_period, rate, 
                                     passive=not any([k > 0.0 for k in self._k_d[1]]),
                                     bounds=[neg_limit, pos_limit])
    
    if any([k > 0.0 for kk in [self._k_11[1], self._k_12[1], self._k_13[1]] for k in kk]):
      self._k_period = max(2,round(k_period * rate))  # period of time for autotune calculation (seconds converted to frames)
      self.error_norms = deque(maxlen=self._k_period)
    else:
      self.error_norms = None

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])
  
  @property
  def k_11(self):
    return interp(self.speed, self._k_11[0], self._k_11[1])

  @property
  def k_12(self):
    return interp(self.speed, self._k_12[0], self._k_12[1])

  @property
  def k_13(self):
    return interp(self.speed, self._k_13[0], self._k_13[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.kp = 0.0
    self.ki = 0.0
    self.kd = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.error_integral.reset()
    self.error_rate.reset()
    if self.error_norms is not None:
      self.error_norms = deque(maxlen=self._k_period)
  
  def update_i_period(self, integral_period):
    self.error_integral.update_period(integral_period)
  
  def update_d_period(self, derivative_period):
    self.error_rate.update_period(derivative_period)

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))

    self.kp = self.k_p
    self.ki = self.k_i
    self.kd = self.k_d
    
    if self.error_norms is not None:
      self.error_norms.append(error / max(speed*0.1, 1.0))
      if len(self.error_norms) == self.error_norms.maxlen:
        delta_error_norm = self.error_norms[-1] - self.error_norms[0]
        self._gain_update_factor = self.error_norms[-1] * delta_error_norm
        if self._gain_update_factor != 0.:
          abs_guf = abs(self._gain_update_factor)
          self.kp *= 1. + min(5., self.k_11 * max(0.0, self._gain_update_factor))
          self.ki *= 1. + clip(self.k_12 * self._gain_update_factor, -1., 5.)
          self.kd *= 1. + min(5., self.k_13 * abs_guf)
    
    self.p = error * self.kp
    self.i = self.error_integral.update(error) * self.ki
    self.d = self.error_rate.update(error) * self.kd
    self.f = feedforward * self.k_f
        
    control = self.p + self.f + self.i + self.d
    self.saturated = self._check_saturation(control, check_saturation, error)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
