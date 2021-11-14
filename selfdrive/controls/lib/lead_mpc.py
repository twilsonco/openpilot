import math
import numpy as np
from common.numpy_fast import interp, clip
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.lead_mpc_lib import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG, CONTROL_N
from selfdrive.swaglog import cloudlog

MPC_T = list(np.arange(0,1.,.2)) + list(np.arange(1.,10.6,.6))

TR_DEFAULT = 1.8
SNG_SPEED = 18 * CV.MPH_TO_MS
SNG_DIST_COST = MPC_COST_LONG.DISTANCE
SNG_ACCEL_COST = MPC_COST_LONG.ACCELERATION

FOLLOW_PROFILES = [
  [ # one-bar
    [-1.0, 2.3], # bp0 and bp1; lead car relative velocities [m/s] (set both to 0.0 to disable dynamic brakepoints)
    [0.5, 2.0], # follow distances corresponding to bp0 and bp1 [s]
    [0.0, 1.892, 3.7432, 5.8632, 8.0727, 10.7301, 14.343, 17.6275, 22.4049, 28.6752, 34.8858, 40.35], # lookup table of speeds for additional follow distances [m/s] (stolen from shane)
    [0.0, 0.00099, -0.0324, -0.0647, -0.0636, -0.0601, -0.0296, -0.1211, -0.2341, -0.3991, -0.432, -0.4625], # additional follow distances based on speed [s]
    1.2, # stopping distance behind stopped lead car [m]
    # now variable distance cost. Defined in two ways; one according to abs follow distance [m] and one in relative follow distance [s]. Larger distance cost wins. First the time-based:
    [1.0, 1.5, 2.3], # seconds behind lead car
    [MPC_COST_LONG.DISTANCE * 10., MPC_COST_LONG.DISTANCE * 7., MPC_COST_LONG.DISTANCE], # mpc distance costs lookup table based on follow distance behind lead (higher value means harder accel/braking to make up distance) (recommended to use factors of MPC_COST_LONG.DISTANCE) (It's ok to only have one value, ie static distance cost )
    [15., 33., 100.], # meters behind lead car
    [MPC_COST_LONG.DISTANCE * 10., MPC_COST_LONG.DISTANCE * 7., MPC_COST_LONG.DISTANCE], # distance costs for the absolute follow distances
    0., # [units of MPC_COST_LONG.DISTANCE / (m/s)] lead car pull-away speed cost shift factor "d" (when lead car is pulling away, ie v_lead < 0, distance cost will be increased by |v_lead| * d)
    0., # [units of MPC_COST_LONG.DISTANCE / (m/s)] lead car pull-away speed cost shift factor "d" (when lead car is pulling away, ie v_lead < 0, distance cost will be increased by |v_lead| * d)
    [1.5, 1.8, 2.8], # seconds behind lead car
    [MPC_COST_LONG.ACCELERATION * 1., MPC_COST_LONG.ACCELERATION * 1.2, MPC_COST_LONG.ACCELERATION * 1.4], # values of accel cost 
    [15., 20., 40.], # meters behind lead car
    [MPC_COST_LONG.ACCELERATION * 1., MPC_COST_LONG.ACCELERATION * 1.2, MPC_COST_LONG.ACCELERATION * 1.4], # values of accel cost
    0.1 * CV.MPH_TO_MS # amount to decrease accel cost by per mph over the max approaching lead velocity
  ],
  [ # two-bar
    [-2.0, -0.15],
    [0.5, 1.5],
    [0.0, 1.8627, 3.7253, 5.588, 7.4507, 9.3133, 11.5598, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336],
    [0.0, 0.0034975, 0.008495, 0.015, 0.025, 0.03945, 0.06195, 0.0745, 0.08895, 0.1005, 0.10495, 0.11045, 0.11845],
    1.2,
    [0.8, 2.1],
    [MPC_COST_LONG.DISTANCE, MPC_COST_LONG.DISTANCE * 0.7],
    [20., 30.],
    [MPC_COST_LONG.DISTANCE, MPC_COST_LONG.DISTANCE * 0.7],
    0.5 * CV.MPH_TO_MS,
    0.05 * CV.MPH_TO_MS, 
    [1.5, 1.8, 2.8], 
    [MPC_COST_LONG.ACCELERATION * 1., MPC_COST_LONG.ACCELERATION * 1.2, MPC_COST_LONG.ACCELERATION * 1.4], 
    [15., 20., 40.],
    [MPC_COST_LONG.ACCELERATION * 1., MPC_COST_LONG.ACCELERATION * 1.2, MPC_COST_LONG.ACCELERATION * 1.4], 
    0.1 * CV.MPH_TO_MS
  ],
  [ # three-bar
    [-3.0, -0.2],
    [0.5, 2.1],
    [0.0, 1.8627, 3.7253, 5.588, 7.4507, 9.3133, 11.5598, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336],
    [0.0, 0.006995, 0.01699, 0.03, 0.05, 0.0789, 0.1239, 0.149, 0.1779, 0.201, 0.2099, 0.2209, 0.2369],
    1.2,
    [0.8, 1.4, 1.8, 3.5],
    [MPC_COST_LONG.DISTANCE * 1.5, MPC_COST_LONG.DISTANCE * 0.7, MPC_COST_LONG.DISTANCE * 0.25, MPC_COST_LONG.DISTANCE * 0.1],
    [15., 25., 35., 45.], # meters behind lead car
    [MPC_COST_LONG.DISTANCE * 1.5, MPC_COST_LONG.DISTANCE * 0.8, MPC_COST_LONG.DISTANCE * 0.25, MPC_COST_LONG.DISTANCE * 0.1],
    0.5 * CV.MPH_TO_MS,
    0.1 * CV.MPH_TO_MS,
    [1.5, 1.8, 2.8], 
    [MPC_COST_LONG.ACCELERATION * 1., MPC_COST_LONG.ACCELERATION * 2.0, MPC_COST_LONG.ACCELERATION * 3.], 
    [10., 20., 40.],
    [MPC_COST_LONG.ACCELERATION * 1., MPC_COST_LONG.ACCELERATION * 2.0, MPC_COST_LONG.ACCELERATION * 3.], 
    0.2 * CV.MPH_TO_MS
  ]
]

FP_MIN_MAX_DIST_COSTS = [[f(f(fp[6]),f(fp[8])) for f in [min,max]] for fp in FOLLOW_PROFILES]
FP_MIN_MAX_ACCEL_COSTS = [[f(f(fp[12]),f(fp[14])) for f in [min,max]] for fp in FOLLOW_PROFILES]

LEAD_PULLAWAY_V_REL = -0.5 * CV.MPH_TO_MS # [m/s] lead car pull-away speed cost shift factor cutoff! Lead car has to be pulling away faster than this before it starts increasing the mpc distance cost (must be negative)
LEAD_APPROACHING_V_REL = 8. * CV.MPH_TO_MS # [m/s] lead car approaching cost shift factor cutoff! Lead car has to be approaching faster than this before it starts increasing/decreasing the mpc distance/acceleration costs (must be positive)

# calculate the desired follow distance and mcp distance cost from current state
def calc_follow_profile(v_ego, v_lead, x_lead, fpi):
  fp = FOLLOW_PROFILES[fpi]
  # adjust based on speed for sng smooth stopping
  sng_factor = interp(v_ego, [SNG_SPEED * 0.8, SNG_SPEED], [1., 0.])
  v_rel = v_ego - v_lead   # calculate relative velocity vs lead car
  d_lead = (x_lead / v_ego) if v_ego > 0.1 else 0.0 # distance to lead car in seconds
  
  # first target distance
  hwy_shift = interp(v_ego, fp[2], fp[3]) # calculate variable shift of objective follow distance based on city/highway speed
  tr = interp(v_rel, fp[0], fp[1]) + hwy_shift # calculate objective distance in seconds(ish)
  tr = TR_DEFAULT * sng_factor + tr * (1.0 - sng_factor)
  
  # then distance cost
  dist_cost = max(interp(d_lead, fp[5], fp[6]), interp(x_lead, fp[7], fp[8]))
  dist_cost = SNG_DIST_COST * sng_factor + dist_cost * (1.0 - sng_factor)
  if v_rel < LEAD_PULLAWAY_V_REL:
    dist_cost += MPC_COST_LONG.DISTANCE * (LEAD_PULLAWAY_V_REL - v_rel) * fp[9]
  elif v_rel > LEAD_APPROACHING_V_REL:
    dist_cost += MPC_COST_LONG.DISTANCE * (v_rel - LEAD_APPROACHING_V_REL) * fp[10]
  dist_cost = clip(dist_cost, FP_MIN_MAX_DIST_COSTS[fpi][0], FP_MIN_MAX_DIST_COSTS[fpi][1])
  
  # then accel cost
  accel_cost = min(interp(d_lead, fp[11], fp[12]), interp(x_lead, fp[13], fp[14]))
  accel_cost = SNG_ACCEL_COST * sng_factor + accel_cost * (1.0 - sng_factor)
  if v_rel > LEAD_APPROACHING_V_REL:
    accel_cost -= MPC_COST_LONG.ACCELERATION * (v_rel - LEAD_APPROACHING_V_REL) * fp[15]
  accel_cost = clip(accel_cost, FP_MIN_MAX_ACCEL_COSTS[fpi][0], FP_MIN_MAX_ACCEL_COSTS[fpi][1])
    
  return tr, dist_cost, accel_cost
  

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
    
    self.tr = 1.8
    self.dist_cost = 1. # this is normalized and displayed to the driver in a UI metric
    self.accel_cost = 1. # this is normalized and displayed to the driver in a UI metric
    self.dist_cost_last = MPC_COST_LONG.DISTANCE
    self.accel_cost_last = MPC_COST_LONG.ACCELERATION
    self.stopping_distance = 0.

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

  def set_cur_state(self, v, a):
    v_safe = max(v, 1e-3)
    a_safe = a
    self.cur_state.v_ego = v_safe
    self.cur_state.a_ego = a_safe

  def update(self, CS, radarstate, v_cruise, a_target, active):
    v_ego = CS.vEgo
    if self.lead_id == 0:
      lead = radarstate.leadOne
    else:
      lead = radarstate.leadTwo
    self.status = lead.status

    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0

    follow_level = int(CS.readdistancelines) - 1 # use base 0
    if follow_level < 0 or follow_level > 2:
      follow_level = 1
    fp = FOLLOW_PROFILES[follow_level]
    stopping_distance = fp[4]

    if lead is not None and lead.status:
      x_lead = max(0, lead.dRel - stopping_distance)  # increase stopping distance to car by X [m]
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0

      self.a_lead_tau = max(lead.aLeadTau, (a_lead ** 2 * math.pi) / (2 * (v_lead + 0.01) ** 2))
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        self.libmpc.init_with_simulation(v_ego, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
      
      # Setup mpc
      tr, dist_cost, accel_cost = calc_follow_profile(v_ego, v_lead, lead.dRel, follow_level)
    else:
      self.prev_lead_status = False
      # Fake a fast lead car, so mpc keeps running
      self.cur_state[0].x_l = max(v_ego * 8.0, 50.0) # put lead 8 seconds ahead
      self.cur_state[0].v_l = v_ego + 20.0
      a_lead = 0.0
      self.a_lead_tau = _LEAD_ACCEL_TAU
      tr = TR_DEFAULT
      dist_cost = MPC_COST_LONG.DISTANCE
      accel_cost = MPC_COST_LONG.ACCELERATION
    
    if dist_cost != self.dist_cost_last or accel_cost != self.accel_cost_last:
      self.libmpc.change_costs(MPC_COST_LONG.TTC, dist_cost, accel_cost, MPC_COST_LONG.JERK)
      self.dist_cost_last = dist_cost
      self.accel_cost_last = self.accel_cost
    
    self.tr = tr
    self.dist_cost = dist_cost / MPC_COST_LONG.DISTANCE
    self.accel_cost = accel_cost / MPC_COST_LONG.ACCELERATION
    self.stopping_distance = stopping_distance
      
    t = sec_since_boot()
    self.n_its = 0
    self.n_its += self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, tr)
    
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
      self.cur_state[0].v_ego = v_ego
      self.cur_state[0].a_ego = 0.0
      self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                         MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
