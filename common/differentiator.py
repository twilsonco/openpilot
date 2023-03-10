from collections import deque
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import clip

class Differentiator:
  def __init__(self, derivative_period, rate, alpha=0.0, passive=False, bounds=[-1e150, 1e150]):
    self.dt = 1.0/rate
    self._D = FirstOrderFilter(0.0, alpha, self.dt)
    self.passive = passive
    if not self.passive:
      if not isinstance(bounds, list) or not all([type(v) == float for v in bounds]):
        self.bounds = [-1e150, 1e150]
      else:
        self.bounds = bounds
      self._d_period_s = 0.0
      self._rate = rate
      self.update_period(derivative_period)
    
  def reset(self):
    if self.passive:
      return
    self._vals = deque(maxlen=self._d_period)
    self._D.x = 0.0
  
  @property
  def x(self):
    return self._D.x
  
  def update_period(self, derivative_period):
    if not self.passive and derivative_period != self._d_period_s:
      self._d_period_s = max(0.02, derivative_period)
      self._d_period = max(2,round(self._d_period_s * self._rate))  # period of time for derivative calculation (seconds converted to frames)
      self._d_period_recip = 1 / self._d_period_s
      self._vals = deque(maxlen=self._d_period)
  
  def update(self, val):
    if self.passive:
      return 0.0
    self._vals.append(val)
    if len(self._vals) == int(self._vals.maxlen):  # makes sure we have enough history for period
      self._D.update((self._vals[-1] - self._vals[0]) * self._d_period_recip)
      self._D.x = clip(self._D.x, self.bounds[0], self.bounds[1])
    return self._D.x