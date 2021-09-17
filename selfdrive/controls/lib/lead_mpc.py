import math
import numpy as np
from common.numpy_fast import interp
from common.realtime import sec_since_boot
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.lead_mpc_lib import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG, CONTROL_N
from selfdrive.swaglog import cloudlog

MPC_T = list(np.arange(0,1.,.2)) + list(np.arange(1.,10.6,.6))

CITY_HWY_TRANSITION_SPEEDS = [20.12, 33.53] # [m/s], [20.12, 33.53] = [45mph, 75mph]

TR_DEFAULT = 1.8

FOLLOW_PROFILES = [
  [ # one-bar
    [-1.0, 2.0], # bp0 and bp1; lead car relative velocities [m/s] (set both to 0.0 to disable dynamic brakepoints)
    [0.5, 1.6], # follow distances corresponding to bp0 and bp1 [s]
    0.2, # additional follow distance above CITY_HWY_TRANSITION_SPEEDS[1] [s]
    1.1, # stopping distance behind stopped lead car [m]
    [MPC_COST_LONG.DISTANCE * 15.0, MPC_COST_LONG.DISTANCE * 4.0], # mpc distance costs to use at close/far follow distance behind lead (higher value means harder accel/braking to make up distance) (recommended to use factors of MPC_COST_LONG.DISTANCE) (It's ok to only have one value, ie static distance cost )
    [1.0, 3.0], # distances behind lead car [s] corresponding to the distance costs above
    2.0 # [units of MPC_COST_LONG.DISTANCE] lead car pull-away distance cost shift factor "d" (when lead car is pulling away, ie v_lead < 0, distance cost will be increased by |v_lead| * d)
  ],
  [ # two-bar
    [-2.0, 0.0],
    [0.5, 1.8],
    0.3,
    1.5,
    [MPC_COST_LONG.DISTANCE * 2, MPC_COST_LONG.DISTANCE],
    [0.8, 1.5],
    2.0
  ],
  [ # three-bar
    [-2.0, 0.0],
    [0.5, 2.4],
    0.5,
    1.5,
    [MPC_COST_LONG.DISTANCE * 1.1, MPC_COST_LONG.DISTANCE * 0.1],
    [0.6, 4.0],
    2.0
  ]
]

D = -0.5 # [m/s] lead car pull-away distance cost shift factor cutoff! Lead car has to be pulling away faster than this before it starts increasing the mpc distance cost (must be negative)
D = min(D, 0.0)

# In order to have a dynamic distance cost, so that far following can be more relaxed, but not at the expense of braking response when close, we'll maintain three mpcs with the low, stock, and high distance costs, then interpolate the solution each iteration from the solutions of those three
DIST_COSTS = [f([f(p[4] + [MPC_COST_LONG.DISTANCE]) for p in FOLLOW_PROFILES]) for f in [min,max]]
DIST_COSTS.insert(1, MPC_COST_LONG.DISTANCE)

# calculate the desired follow distance and mcp distance cost from current state
def calc_follow_profile(v_ego, v_lead, x_lead, fp, do_log = False):
  v_rel = v_ego - v_lead   # calculate relative velocity vs lead car
  hwy_shift = interp(v_ego, CITY_HWY_TRANSITION_SPEEDS, [0.0, fp[2]]) # calculate variable shift of objective follow distance based on city/highway speed
  od = interp(v_rel, fp[0], [fp[1][0], fp[1][1]]) + hwy_shift # calculate objective distance in seconds(ish)
  d_lead = (x_lead / v_ego) if v_ego > 0.1 else 0.0 # distance to lead car in seconds
  dist_cost = interp(d_lead, fp[5], fp[4])
  if v_rel < D:
    dist_cost += MPC_COST_LONG.DISTANCE * (-(v_rel - D)) * fp[6]
  if do_log:
    cloudlog.debug("lead_mpc follow distance and distance cost; {}; {}; {}; {}; {}; {}; {}; {}".format(fp, v_ego, v_lead, x_lead, hwy_shift, d_lead, od, dist_cost))
  return od, dist_cost

# like interp, but for the self.mpc_solution ie the log_t struct of ./lead_mpc_lib/longitudinal_mpc.c, so y_vals is 
# Probably not the fastest way to do this, but isn't Python awesome...
def interp_mpc_solution(x, x_vals, y_vals):
  out = y_vals[0]
  for prop in ['x_ego','v_ego','a_ego','j_ego','x_l','v_l','a_l','t']:
    y_prop = [interp(x, x_vals, [getattr(y, prop)[i] for y in y_vals]) for i in range(len(getattr(y_vals[0], prop)))]
    setattr(out, prop, y_prop)
  out.cost = interp(x, x_vals, [y.cost for y in y_vals])
  return out

def log_mpcsolution(mpcsolution, ind):
  cloudlog.debug("lead_mpc solution {}".format(ind))
  for prop in ['x_ego','v_ego','a_ego','j_ego','x_l','v_l','a_l','t']:
    cloudlog.debug("   {}: {}".format(prop, ",".join(["{0:4.2f}".format(i) for i in getattr(mpcsolution, prop)])))
  cloudlog.debug("   cost: {0:7.4f}".format(mpcsolution.cost))

def log_curstate(state, ind):
  logstr = ";".join(["{0}: {1:7.4f}".format(prop, getattr(state, prop)) for prop in ['x_ego', 'v_ego', 'a_ego', 'x_l', 'v_l', 'a_l']])
  cloudlog.debug("lead_mpc state {};{}".format(ind, logstr))
  

class LeadMpc():
  def __init__(self, mpc_id):
    self.lead_id = mpc_id

    self.reset_mpc()
    self.prev_lead_status = False
    self.prev_lead_x = 0.0
    self.new_lead = False

    self.last_cloudlog_t = 0.0
    self.n_its = 0
    self.duration = 0
    self.status = False
    
    self.last_follow_log_t = 0.0

    self.v_solution = np.zeros(CONTROL_N)
    self.a_solution = np.zeros(CONTROL_N)
    self.j_solution = np.zeros(CONTROL_N)

  def reset_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.lead_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU

    self.libmpcs = [None] * 3
    self.mpc_solutions = [None] * 3
    self.cur_states = [None] * 3
    mpci = 1
    for i in range(3):
      if i == 1:
        self.libmpcs[i] = self.libmpc # since DIST_COSTS[1] == MPC_COST_LONG.DISTANCE
        self.mpc_solutions[i] = self.mpc_solution
        self.cur_states[i] = self.cur_state
      else:
        ffi, self.libmpcs[i] = libmpc_py.get_libmpc(self.lead_id + mpci)
        self.libmpcs[i].init(MPC_COST_LONG.TTC, DIST_COSTS[i],
                         MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        mpci += 1
        self.mpc_solutions[i] = ffi.new("log_t *")
        self.cur_states[i] = ffi.new("state_t *")
        self.cur_states[i][0].v_ego = 0
        self.cur_states[i][0].a_ego = 0

  def set_cur_state(self, v, a):
    v_safe = max(v, 1e-3)
    a_safe = a
    for i in range(3):
      self.cur_states[i][0].v_ego = v_safe
      self.cur_states[i][0].a_ego = a_safe
    self.cur_state = self.cur_states[1]

  def update(self, CS, radarstate, v_cruise, a_target, active):
    v_ego = CS.vEgo
    if self.lead_id == 0:
      lead = radarstate.leadOne
    else:
      lead = radarstate.leadTwo
    self.status = lead.status

    # Setup current mpc state
    for i in range(3):
      self.cur_states[i][0].x_ego = 0.0

    follow_level = int(CS.readdistancelines) - 1 # use base 0
    if follow_level < 0 or follow_level > 2:
      follow_level = 1
    fp = FOLLOW_PROFILES[follow_level]
    stopping_distance = fp[3]
    
    t_log = sec_since_boot()
    do_log = False and lead is not None and lead.status and t_log - self.last_follow_log_t > 0.5

    if lead is not None and lead.status:
      x_lead = max(0, lead.dRel - stopping_distance)  # increase stopping distance to car by X [m]
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0

      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        for i in range(3):
          self.libmpcs[i].init_with_simulation(v_ego, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      for i in range(3):
        self.cur_states[i][0].x_l = x_lead
        self.cur_states[i][0].v_l = v_lead
      
      # Setup mpc
      tr, dist_cost = calc_follow_profile(v_ego, v_lead, lead.dRel, fp, do_log)
    else:
      self.prev_lead_status = False
      # Fake a fast lead car, so mpc keeps running
      for i in range(3):
        self.cur_states[i][0].x_l = 50.0
        self.cur_states[i][0].v_l = v_ego + 10.0
      a_lead = 0.0
      self.a_lead_tau = _LEAD_ACCEL_TAU
      tr = TR_DEFAULT
      dist_cost = MPC_COST_LONG.DISTANCE
      
    t = sec_since_boot()
    self.n_its = 0
    for i in range(3):
      self.n_its += self.libmpcs[i].run_mpc(self.cur_states[i], self.mpc_solutions[i], self.a_lead_tau, a_lead, tr)
    self.libmpc = self.libmpcs[1]
    self.cur_state = self.cur_states[1]
    self.mpc_solution = interp_mpc_solution(dist_cost, DIST_COSTS, self.mpc_solutions)
    
    if do_log:
      self.last_follow_log_t = t_log
      for i in range(3):
        log_mpcsolution(self.mpc_solutions[i], i)
      log_mpcsolution(self.mpc_solution, -1)
      for i in range(3):
        log_curstate(self.cur_states[i], i)
    
    self.v_solution = interp(T_IDXS[:CONTROL_N], MPC_T, self.mpc_solution.v_ego)
    self.a_solution = interp(T_IDXS[:CONTROL_N], MPC_T, self.mpc_solution.a_ego)
    self.j_solution = interp(T_IDXS[:CONTROL_N], MPC_T[:-1], self.mpc_solution.j_ego)
    self.duration = int((sec_since_boot() - t) * 1e9)
        
    # Reset if NaN or goes through lead car
    crashing = any(lead - ego < -50 for (lead, ego) in zip(self.mpc_solution[0].x_l, self.mpc_solution[0].x_ego))
    nans = any(math.isnan(x) for x in self.mpc_solution[0].v_ego)
    backwards = min(self.mpc_solution[0].v_ego) < -0.15

    if ((backwards or crashing) and self.prev_lead_status) or nans:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Longitudinal mpc %d reset - backwards: %s crashing: %s nan: %s" % (
                          self.lead_id, backwards, crashing, nans))
      self.a_mpc = CS.aEgo
      self.prev_lead_status = False
      for i in range(3):
        self.cur_states[i][0].v_ego = v_ego
        self.cur_states[i][0].a_ego = 0.0
        self.libmpcs[i].init(MPC_COST_LONG.TTC, DIST_COSTS[i],
                         MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.libmpc = self.libmpcs[1]
      self.cur_state = self.cur_states[1]
