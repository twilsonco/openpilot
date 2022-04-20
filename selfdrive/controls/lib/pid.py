import numpy as np
from numbers import Number
from collections import deque
from common.numpy_fast import clip, interp


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


class PIDController:
  def __init__(self, k_p=0., k_i=0., k_d=0., k_f=1., k_11=0., k_12=0., k_13=0., k_period=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, derivative_period=1.):
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

    self._k_11 = k_11  # proportional gain factor
    self._k_12 = k_12  # integral gain factor
    self._k_13 = k_13  # derivative gain factor
    if isinstance(self._k_11, Number):
      self._k_11 = [[0], [self._k_11]]
    if isinstance(self._k_12, Number):
      self._k_12 = [[0], [self._k_12]]
    if isinstance(self._k_13, Number):
      self._k_13 = [[0], [self._k_13]]
    self.do_auto_tune = any(v > 0. for kv in [self._k_11, self._k_12, self._k_13] for v in kv[-1])
    self._k_period = round(k_period * rate)  # period of time for autotune calculation (seconds converted to frames)

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = sat_limit
    self._d_period = round(derivative_period * rate)  # period of time for derivative calculation (seconds converted to frames)
    self._d_period_recip = 1. / self._d_period

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
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.errors = deque()
    self.error_norms = deque()

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    
    if self.do_auto_tune:
      abs_sp = setpoint if setpoint > 0. else -setpoint
      self.error_norms.append(float(error) / (abs_sp + 1.))
      while len(self.error_norms) > self._k_period:
        self.error_norms.popleft()

    kp = self.k_p
    ki = self.k_i
    kd = self.k_d
    
    if self.do_auto_tune and len(self.error_norms) >= self._k_period:
      delta_error_norm = self.error_norms[-1] - self.error_norms[-self._k_period]
      gain_update_factor = self.error_norms[-1] * delta_error_norm
      if gain_update_factor != 0.:
        abs_guf = abs(gain_update_factor)
        kp *= 1. + min(2., self.k_11 * abs_guf)
        ki *= 1. + clip(self.k_12 * gain_update_factor, -1., 2.)
        kd *= 1. + min(2., self.k_13 * abs_guf)
    
    if len(self.errors) >= self._d_period and kd > 0.:  # makes sure we have enough history for period
      d = (error - self.errors[-self._d_period]) * self._d_period_recip  # get deriv in terms of 100hz (tune scale doesn't change)
      d *= kd
    else:
      d = 0.
    
    self.p = error * kp
    self.f = feedforward * self.k_f

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * ki * self.i_rate
      control = self.p + self.f + i + d

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i + d
    self.saturated = self._check_saturation(control, check_saturation, error)

    self.errors.append(float(error))
    while len(self.errors) > self._d_period:
      self.errors.popleft()

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
