import numpy as np
from numbers import Number
from collections import deque
from common.numpy_fast import clip, interp


class PIDController():
  def __init__(self, k_p, k_i, k_f=0., k_d=0., pos_limit=1e308, neg_limit=-1e308, rate=100, derivative_period=1.):
    self._k_p = k_p
    self._k_i = k_i
    self._k_d = k_d
    self.k_f = k_f   # feedforward gain
    if isinstance(self._k_p, Number):
      self._k_p = [[0], [self._k_p]]
    if isinstance(self._k_i, Number):
      self._k_i = [[0], [self._k_i]]
    if isinstance(self._k_d, Number):
      self._k_d = [[0], [self._k_d]]

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.speed = 0.0
    
    if any([k > 0.0 for k in self._k_d[1]]):
      self._d_period = round(derivative_period * rate)  # period of time for derivative calculation (seconds converted to frames)
      self._d_period_recip = 1. / self._d_period
      self.outputs = deque(maxlen=self._d_period)
    else:
      self.outputs = None

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
  def error_integral(self):
    return self.i/self.k_i

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.d = 0.0
    self.f = 0.0
    self.control = 0
    if self.outputs is not None:
      self.outputs = deque(maxlen=self._d_period)

  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    if self.outputs is not None and len(self.outputs) == int(self._d_period):  # makes sure we have enough history for period
      self.d = (self.outputs[-1] - self.outputs[0]) * self._d_period_recip * self.k_d
      self.d = clip(self.d, -abs(self.p), abs(self.p))
    else:
      self.d = error_rate * self.k_d

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + i + self.d + self.f

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.i + self.d + self.f

    self.control = clip(control, self.neg_limit, self.pos_limit)
    if self.outputs is not None:
      self.outputs.append(self.control)
    return self.control
