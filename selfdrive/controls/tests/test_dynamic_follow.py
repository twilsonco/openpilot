#!/opt/homebrew/bin/python3

from random import random
import numpy as np

def interp(x, xp, fp):
  N = len(xp)

  def get_interp(xv):
    hi = 0
    while hi < N and xv > xp[hi]:
      hi += 1
    low = hi - 1
    return fp[-1] if hi == N and xv > xp[low] else (
      fp[0] if hi == 0 else
      (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low])

  return [get_interp(v) for v in x] if hasattr(x, '__iter__') else get_interp(x)

class DynamicFollow():
  self.t_last = 0
  
  self.speed_fp_limit_bp = [i * CV.MPH_TO_MS for i in [55.,80.]]
  self.speed_fp_limit_v = [1.,2.] # [follow profile number 0-based] restricted to close/med follow until 55mph, smoothly increase to far follow by 80mph
  
  self.fp_point_rate_bp = [0.5, 1.0, 1.5, 2.0]
  self.fp_point_rate_v = [1./30., 1./60., 1./120., 1./500.] # [follow profile per second] ~30s to go from close to medium, then ~10 minutes to go from medium to far
  
  self.points_cur = 0. # [follow profile number 0-based]
  self.points_last = 0.
  self.points_bounds = [-10., 2.] # [follow profile number 0-based] min and max follow levels (at the 1/30 fp_point_rate, -10 means after the max number of cutins it takes ~5 minutes to get back to medium follow)
  
  self.cutin_dist_penalty_bp = [i * 0.3 for i in [40., 150.]] # [ft converted to m]
  self.cutin_dist_penalty_v = [2., 0.] # [follow profile] cutin of 40ft or less, drop to close follow, with less penalty up to 150ft, at which point you don't care
  
  self.cutin_vel_penalty_bp = [i * CV.MPH_TO_MS for i in [-15., 15.]] # [mph] velocity of new lead
  self.cutin_vel_penalty_v = [2., -2.] # [follow profile] additionally go to close follow for new lead approaching too quickly, or *offset* the penalty for a close cutin if they're moving away (to keep from darting after a new cutin)

  self.cutin_penalty_last = 0. # penalty for most recent cutin, so that it can be rescinded if the cutin really just cut *over* in front of you (i.e. they quickly disappear)
  
  self.lead_d_last = 0.
  
  def __init__(self,fpi = 1):
    self.points_cur = fpi
    self.points_last = fpi
    self.t_last = sec_since_boot()
  
  def update(self, has_lead, lead_d, lead_v_rel, v_ego, t):
    dur = t - self.t_last
    self.t_last = t
    if has_lead:
      new_lead = abs(lead_d - self.lead_d_last) > 2.5
      self.lead_d_last = lead_d
      if new_lead:
        penalty_dist = interp(lead_d, self.cutin_dist_penalty_bp, self.cutin_dist_penalty_v)
        penalty_vel = interp(lead_v_rel, self.cutin_vel_penalty_bp, self.cutin_vel_penalty_v)
        penalty = max(0., penalty_dist + penalty_vel)
        self.points_cur = max(self.points_bounds[0], self.points_cur - penalty)
        return self.points_cur
    else:
      self.lead_d_last = 1000.

    rate = interp(self.points_cur, self.fp_point_rate_bp, self.fp_point_rate_v)
    step = rate * dur
    
    self.points_cur = min(self.points_bounds[-1], self.points_cur + step)

    self.t_last = t
    
    return self.points_cur


def main():
  df = DynamicFollow()
  
  t = 0
  lead_d = 0.
  lead_v_rel = 0.
  v_ego = 40.
  has_lead = True
  n = 3000
  s = 5
  nx = n / s
  x = np.zeros(nx)
  y = np.zeros(nx)
  for i in range(nx):
    t = i * s
    x[i] = t
    rn = random()
    new_lead = rn > 0.96
    if new_lead:
      has_lead = new_lead
      lead_v_rel = interp(random(),[0.,1.],[-9.,9.])
      lead_d = interp(random(), [0.,1.], [1.5,50.])
    elif rn < 0.01:
      has_lead = False
    
    y[i] = df.update(has_lead, lead_d, lead_v_rel, v_ego, t)
  
  
  
  return

