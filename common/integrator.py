from collections import deque
from common.numpy_fast import sign
from common.op_params import opParams

class Integrator:
  def __init__(self, integral_period, rate, passive=False):
    self.passive = passive
    if not self.passive:
      self._op_params = opParams(calling_function="common/integrator.py")
      self._i_period_s = 0.0
      self._rate = rate
      self._unwind_rate = 0.3 / rate
      self.update_period(integral_period, force_update=True)
  
  @property
  def x(self):
    if self.passive:
      return 0.0
    else:
      return self._x * self._k_i_scale
    
  def reset(self):
    if self.passive:
      return
    self.vals = deque(maxlen=self._i_period)
    self._x = 0.0
    
  def update_period(self, integral_period, force_update=False):
    if not self.passive and integral_period != self._i_period_s:
      self._i_period_s = max(0.03, integral_period)
      self._i_period = max(3,round(self._i_period_s * self._rate))  # period of time for integral calculation (seconds converted to frames)
      self._i_dt = 0.5 / self._rate # multiplied to get trapezoidal area at each step, hence the 1/2
      self.vals = deque(maxlen=self._i_period)
      self._x = 0.0
      self._k_i_period = self._op_params.get("TUNE_PID_ki_period_default_s", force_update=force_update)
      self._k_i_scale = self._k_i_period / max(0.01, self._i_period_s)
  
  def update(self, val, override = False):
    if self.passive:
      return 0.0
    if len(self.vals) == self.vals.maxlen: # subtract off oldest trapezoid
      self._x -= self._i_dt * (self.vals[0] + self.vals[1])
    if override:
      self.vals.append(0.0)
      self._x -= self._unwind_rate * float(sign(self._x))
    else:
      self.vals.append(val)
      if len(self.vals) > 1: # add in new trapezoid
        self._x += self._i_dt * (self.vals[-2] + self.vals[-1])
    return self._x * self._k_i_scale