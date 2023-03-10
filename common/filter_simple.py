from common.numpy_fast import clip

class FirstOrderFilter:
  # first order filter
  def __init__(self, x0, rc, dt, initialized=True, rate_up=None, rate_down=None, min_val=None, max_val=None):
    self.x = x0
    self.dt = dt
    self.rate_up = abs(rate_up) if rate_up is not None else None
    self.rate_down = abs(rate_down) if rate_down is not None else None
    self.min_val = min_val
    self.max_val = max_val
    self.update_alpha(rc)
    self.initialized = initialized

  def update_alpha(self, rc):
    self.alpha = self.dt / (rc + self.dt)

  def update(self, xi):
    if self.min_val is not None and xi < self.min_val:
      x = self.min_val
    elif self.max_val is not None and xi > self.max_val:
      x = self.max_val
    else:
      x = xi
    if self.initialized:
      x1 = (1. - self.alpha) * self.x + self.alpha * x
      if self.rate_up is not None and abs(x1) > abs(self.x):
        x1 = clip(x1, self.x - self.rate_up, self.x + self.rate_up)
      elif self.rate_down is not None and abs(x1) < abs(self.x):
        x1 = clip(x1, self.x - self.rate_down, self.x + self.rate_down)
      self.x = x1
    else:
      self.initialized = True
      self.x = x
    return self.x
