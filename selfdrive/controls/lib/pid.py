import numpy as np
from numbers import Number
from collections import deque
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
    self._k_i_period = self._op_params.get("TUNE_PID_ki_period_default_s")
    self._k_i_scale = self._k_i_period / max(0.01, integral_period)
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
    
    self._rate = rate
    self._i_period_s = integral_period

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self._i_raw = 0.0 # raw integral of error
    self.sat_limit = sat_limit
    
    self._gain_update_factor = 0.0
    
    if any([k > 0.0 for k in self._k_i[1]]):
      self._i_period = max(2,round(integral_period * rate))  # period of time for integral calculation (seconds converted to frames)
      self._i_dt = 0.5 / rate # multiplied to get trapezoidal area at each step, hence the 1/2
      self.errors_i = deque(maxlen=self._i_period)
    else:
      self.errors_i = None

    if any([k > 0.0 for k in self._k_d[1]]):
      self._d_period = max(2,round(derivative_period * rate))  # period of time for derivative calculation (seconds converted to frames)
      self._d_period_recip = 1. / self._d_period
      self.errors_d = deque(maxlen=self._d_period)
    else:
      self.errors_d = None
    
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
    if self.errors_i is not None:
      self.errors_i = deque(maxlen=self._i_period)
      self._i_raw = 0.0
    if self.errors_d is not None:
      self.errors_d = deque(maxlen=self._d_period)
    if self.error_norms is not None:
      self.error_norms = deque(maxlen=self._k_period)
  
  def update_i_period(self, integral_period):
    if integral_period != self._i_period_s:
      self._i_period_s = integral_period
      self._i_period = max(2,round(integral_period * self._rate))  # period of time for integral calculation (seconds converted to frames)
      self._i_dt = 0.5 / self._rate # multiplied to get trapezoidal area at each step, hence the 1/2
      self.errors_i = deque(maxlen=self._i_period)
      self._i_raw = 0.0
      self._k_i_period = self._op_params.get("TUNE_PID_ki_period_default_s")
      self._k_i_scale = self._k_i_period / max(0.01, self._i_period_s)
      
  def _update_i_period(self):
    k_i_period = self._op_params.get("TUNE_PID_ki_period_default_s")
    if k_i_period != self._k_i_period:
      self._k_i_period = k_i_period
      self._i_period_s += 1.0
      self.update_i_period(self._i_period_s - 1.0)

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))

    self.kp = self.k_p
    self.ki = self.k_i
    self.kd = self.k_d
    
    if self.errors_d is not None:
      self.errors_d.append(error)
    
    if self.error_norms is not None and self.errors_d is not None and len(self.errors_d) > 0:
      self.error_norms.append(self.errors_d[-1] / max(speed*0.1, 1.0))
      if len(self.error_norms) == self.error_norms.maxlen:
        delta_error_norm = self.error_norms[-1] - self.error_norms[0]
        self._gain_update_factor = self.error_norms[-1] * delta_error_norm
        if self._gain_update_factor != 0.:
          abs_guf = abs(self._gain_update_factor)
          self.kp *= 1. + min(5., self.k_11 * max(0.0, self._gain_update_factor))
          self.ki *= 1. + clip(self.k_12 * self._gain_update_factor, -1., 5.)
          self.kd *= 1. + min(5., self.k_13 * abs_guf)

    if self.errors_i is not None:
      if len(self.errors_i) == self.errors_i.maxlen: # subtract off oldest trapezoid
        self._i_raw -= self._i_dt * (self.errors_i[0] + self.errors_i[1])
      if override or freeze_integrator:
        self.errors_i.append(0.0)
      else:
        self.errors_i.append(error)
      if len(self.errors_i) > 1: # add in new trapezoid
        self._i_raw += self._i_dt * (self.errors_i[-2] + self.errors_i[-1])
    
    self.p = error * self.kp
    self.f = feedforward * self.k_f
    
    if self.errors_d is not None and len(self.errors_d) == int(self.errors_d.maxlen):  # makes sure we have enough history for period
      self.d = clip((self.errors_d[-1] - self.errors_d[0]) * self._d_period_recip * self.kd, self.neg_limit, self.pos_limit)
    else:
      self.d = 0.

    if self.errors_i is not None:
      self.i = self._i_raw * self.ki * self._k_i_scale
    else:
      self.i = 0.0
        
    control = self.p + self.f + self.i + self.d
    self.saturated = self._check_saturation(control, check_saturation, error)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
