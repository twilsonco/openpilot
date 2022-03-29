import math
import numpy as np
from common.numpy_fast import interp, clip
from common.realtime import sec_since_boot
from common.params import Params
from selfdrive.config import Conversions as CV
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.lead_mpc_lib import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG, CONTROL_N
from selfdrive.swaglog import cloudlog

MPC_T = list(np.arange(0,1.,.2)) + list(np.arange(1.,10.6,.6))

TR_DEFAULT = 1.8
SNG_SPEED = 20. * CV.MPH_TO_MS
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
    0., # [units of MPC_COST_LONG.DISTANCE / (m/s)] lead car pull-away speed cost shift factor "d" (when lead car is pulling away, ie v_lead < 0, distance cost will be increased by |v_lead| * d if it's above the cutoff defined below)
    0., # [units of MPC_COST_LONG.DISTANCE / (m/s)] lead car approaching speed cost shift factor "d" (when lead car is approaching, ie v_lead > 0, distance cost will be increased by |v_lead| * d if it's above the cutoff defined below)
  ],
  [ # two-bar
    [-2.0, -0.15],
    [0.5, 1.5],
    [0.0, 1.8627, 3.7253, 5.588, 7.4507, 9.3133, 11.5598, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336],
    [0.0, 0.0034975, 0.008495, 0.015, 0.025, 0.03945, 0.06195, 0.0745, 0.08895, 0.1005, 0.10495, 0.11045, 0.11845],
    1.0,
    [0.8, 2.1],
    [MPC_COST_LONG.DISTANCE, MPC_COST_LONG.DISTANCE * 0.7],
    [20., 30.],
    [MPC_COST_LONG.DISTANCE, MPC_COST_LONG.DISTANCE * 0.7],
    0.5 * CV.MPH_TO_MS,
    0.05 * CV.MPH_TO_MS, 
  ],
  [ # three-bar
    [-3.0, -0.2],
    [0.5, 2.1],
    [0.0, 1.8627, 3.7253, 5.588, 7.4507, 9.3133, 11.5598, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336],
    [0.0, 0.0020985, 0.005097, 0.009, 0.015, 0.02367, 0.037167, 0.0447, 0.05337, 0.0603, 0.06297, 0.06627, 0.07107],
    0.8,
    [0.8, 1.4, 1.8, 3.5],
    [MPC_COST_LONG.DISTANCE * 1.5, MPC_COST_LONG.DISTANCE * 0.7, MPC_COST_LONG.DISTANCE * 0.25, MPC_COST_LONG.DISTANCE * 0.1],
    [15., 25., 35., 45.], # meters behind lead car
    [MPC_COST_LONG.DISTANCE * 1.5, MPC_COST_LONG.DISTANCE * 0.8, MPC_COST_LONG.DISTANCE * 0.25, MPC_COST_LONG.DISTANCE * 0.1],
    0.5 * CV.MPH_TO_MS,
    0.05 * CV.MPH_TO_MS,
  ]
]

FP_MIN_MAX_DIST_COSTS = [[f(f(fp[6]),f(fp[8])) for f in [min,max]] for fp in FOLLOW_PROFILES]

LEAD_PULLAWAY_V_REL = -0.5 * CV.MPH_TO_MS # [m/s] lead car pull-away speed cost shift factor cutoff! Lead car has to be pulling away faster than this before it starts increasing the mpc distance cost (must be negative)
LEAD_APPROACHING_V_REL = 10. * CV.MPH_TO_MS # [m/s] lead car approaching cost shift factor cutoff! Lead car has to be approaching faster than this before it starts increasing/decreasing the mpc distance/acceleration costs (must be positive)

# calculate the desired follow distance and mcp distance cost from current state
def calc_follow_profile(v_ego, v_lead, x_lead, fpi):
  fp = FOLLOW_PROFILES[fpi]
  # adjust based on speed for sng smooth stopping
  sng_factor = interp(v_ego, [SNG_SPEED * 0.6, SNG_SPEED], [1., 0.])
  v_rel = v_ego - v_lead   # calculate relative velocity vs lead car
  d_lead = (x_lead / v_ego) if v_ego > 0.1 else 0.0 # distance to lead car in seconds
  
  # first target distance
  hwy_shift = interp(v_ego, fp[2], fp[3]) # calculate variable shift of objective follow distance based on city/highway speed
  tr_equil = interp(0., fp[0], fp[1]) # distance when speed is matched
  lead_sng_factor = interp(v_lead, [0., SNG_SPEED], [1., 0.])
  tr = interp(v_rel, fp[0], fp[1]) + hwy_shift # calculate objective distance in seconds(ish)
  tr = tr_equil * lead_sng_factor + tr * (1.0 - lead_sng_factor)
  tr = min(tr_equil,TR_DEFAULT) * sng_factor + tr * (1.0 - sng_factor)
  
  # then distance cost
  dist_cost = max(interp(d_lead, fp[5], fp[6]), interp(x_lead, fp[7], fp[8]))
  dist_cost = SNG_DIST_COST * sng_factor + dist_cost * (1.0 - sng_factor)
  if v_rel < LEAD_PULLAWAY_V_REL:
    dist_cost += MPC_COST_LONG.DISTANCE * (LEAD_PULLAWAY_V_REL - v_rel) * fp[9]
  elif v_rel > LEAD_APPROACHING_V_REL:
    dist_cost += MPC_COST_LONG.DISTANCE * (v_rel - LEAD_APPROACHING_V_REL) * fp[10]
  dist_cost = clip(dist_cost, FP_MIN_MAX_DIST_COSTS[fpi][0], FP_MIN_MAX_DIST_COSTS[fpi][1])
    
  return tr, dist_cost, MPC_COST_LONG.ACCELERATION

def interp_follow_profile(v_ego, v_lead, x_lead, fp_float):
  if fp_float <= 0.:
    fp = calc_follow_profile(v_ego, v_lead, x_lead, 0)
  elif fp_float < 1.:
    fp1,fp2 = [calc_follow_profile(v_ego, v_lead, x_lead, i) for i in [0,1]]
    fp = [(1. - fp_float) * i + fp_float * j for i,j in zip(fp1,fp2)]
  elif fp_float < 2.:
    fptmp = fp_float - 1
    fp1,fp2 = [calc_follow_profile(v_ego, v_lead, x_lead, i) for i in [1,2]]
    fp = [(1. - fptmp) * i + fptmp * j for i,j in zip(fp1,fp2)]
  else:
    fp = calc_follow_profile(v_ego, v_lead, x_lead, 2)
  return fp
  
class DynamicFollow():
  user_timeout_t = 300. # amount of time df waits after the user sets the follow level
  
  ####################################
  #    ACCRUING FOLLOW POINTS
  ####################################
  
  # limit follow distance (profile) used based on speed
  speed_fp_limit_bp = [i * CV.MPH_TO_MS for i in [55., 80.]]
  speed_fp_limit_v = [1., 2.]  # [follow profile number 0-based] restricted to close/med follow until 55mph, smoothly increase to far follow by 80mph
  
  # acrue follow points at different rates (per time) depending the current follow level
  # it will take significantly more time to change from medium to far than from close to medium (if allowed by your speed)
  fp_point_rate_bp = [0.0, 1.0, 1.5, 2.0]
  fp_point_rate_v = [1. / 30., 1. / 240., 1. / 900., 1. / 3000.]  # [follow profile per second] ~30s to go from close to medium, then ~10 minutes to go from medium to far
  
  # follow point accrual rate is also scaled by speed, so you don't earn points for being stopped behind a lead
  speed_rate_factor_bp = [i * CV.MPH_TO_MS for i in [1., 30.]] # [mph]
  speed_rate_factor_v = [0.,1.] # slow "time" at low speed to you have to be actually moving behind a lead in order to earn points
  
  points_bounds = [-10., 2.]  # [follow profile number 0-based] min and max follow levels (at the 1/30 fp_point_rate, -10 means after the max number of cutins it takes ~5 minutes to get back to medium follow)
  
  
  #############################################
  #    PENALIZING FOLLOW POINTS FOR CUT-INS:
  #    MORE PENALTY MEANS DRIVE DEFENSIVELY
  #    FOR MORE TIME AFTER A CUT-IN
  #############################################
  
  # penalize more for close cut-ins than for far 
  # (greatest of the time/length based distance penalty will be used)
  # (at low speeds, the distance penalty will dominate)
  cutin_time_dist_penalty_bp = [.2, 1.0, 2.0, 2.5]	# [distance from cut-in in seconds]
  cutin_time_dist_penalty_v = [2.5, 1.0, 0.5, 0.0]  # [follow profile change]

  cutin_dist_penalty_bp = [i * 0.3 for i in [15, 30., 60.]]	# [distance from cut-in in ft]
  cutin_dist_penalty_v = [2.5, 1.0, 0.0]  # [follow profile change]
  
  # penalize more for approaching cut-ins, and *offset* the distance penalty for cut-ins pulling away
  cutin_vel_penalty_bp = [i * CV.MPH_TO_MS for i in [-15., 0., 15.]]  # [mph] relative velocity of new lead
  cutin_vel_penalty_v = [-3.,0., 2.5]  # [follow profile] additionally go to close follow for new lead approaching too quickly, or *offset* the penalty for a close cutin if they're moving away (to keep from darting after a new cutin)
  
  # bonus penalty factor for repeat cut-ins
  cutin_last_t_factor_bp = [0., 15.] # [s] time since last cutin
  cutin_last_t_factor_v = [3., 1.] # [unitless] factor of cutin penalty
  
  # rescind penalty for cut-overs (cut-in and then left lane quickly)
  cutin_rescind_t_bp = [2.,6.] # [s] time since last cut-in
  cutin_rescind_t_v = [1., 0.] # [unitless] factor of cut-in penalty that is rescinded
  
  def __init__(self,fpi = 1):
    self.points_cur = fpi # [follow profile number 0-based] number of current points. corresponds to follow level
    self.t_last = 0. # sec_since_boot() on last iteration
    self.cutin_penalty_last = 0.  # penalty for most recent cutin, so that it can be rescinded if the cutin really just cut *over* in front of you (i.e. they quickly disappear)
    self.lead_d_last = 0.
    self.has_lead_last = False
    self.user_timeout_last_t = -self.user_timeout_t # (i.e. it's already been 300 seconds)
    self.cutin_t_last = 0. # sec_since_boot() of last remembered cut-in
  
  def update(self, has_lead, lead_d, lead_v, v_ego):
    t = sec_since_boot()
    dur = t - self.t_last
    self.t_last = t
    lead_gone = (self.has_lead_last and not has_lead) or self.lead_d_last - lead_d < -2.5
    new_lead = has_lead and (not self.has_lead_last or self.lead_d_last - lead_d > 2.5)
    if new_lead:
      if v_ego > 0.:
        time_dist = lead_d / v_ego
        penalty_time = interp(time_dist, self.cutin_time_dist_penalty_bp, self.cutin_time_dist_penalty_v)
      else:
        penalty_time = 0.
      penalty_dist = interp(lead_d, self.cutin_dist_penalty_bp, self.cutin_dist_penalty_v)
      penalty_dist = max(penalty_dist, penalty_time)
      lead_v_rel = v_ego - lead_v
      penalty_vel = interp(lead_v_rel, self.cutin_vel_penalty_bp, self.cutin_vel_penalty_v)
      penalty = max(0., penalty_dist + penalty_vel)
      penalty *= interp(t - self.cutin_t_last, self.cutin_last_t_factor_bp, self.cutin_last_t_factor_v)
      points_old = self.points_cur
      if t - self.user_timeout_last_t > self.user_timeout_t:
        self.points_cur = max(self.points_bounds[0], self.points_cur - penalty)
      self.cutin_t_last = t
      self.cutin_penalty_last = points_old - self.points_cur
      self.has_lead_last = has_lead
      return self.points_cur
    elif lead_gone and t - self.user_timeout_last_t > self.user_timeout_t and t - self.cutin_t_last < self.cutin_rescind_t_bp[-1]:
      rescinded_penalty = self.cutin_penalty_last * interp(t - self.cutin_t_last, self.cutin_rescind_t_bp, self.cutin_rescind_t_v)
      self.points_cur += max(0,rescinded_penalty)
      self.cutin_t_last = t - self.cutin_rescind_t_bp[-1] - 1.

    rate = interp(self.points_cur, self.fp_point_rate_bp, self.fp_point_rate_v)
    speed_factor = interp(v_ego, self.speed_rate_factor_bp, self.speed_rate_factor_v)
    step = rate * dur * speed_factor
    if t - self.user_timeout_last_t > self.user_timeout_t:
      self.points_cur = min(interp(v_ego, self.speed_fp_limit_bp, self.speed_fp_limit_v), self.points_cur + step)

    self.has_lead_last = has_lead

    return self.points_cur
  
  def set_fp(self,fpi):
    self.points_cur = fpi
    self.user_timeout_last_t = self.t_last
    self.cutin_t_last = -self.cutin_rescind_t_bp[-1]
  
  def reset(self, fpi = 1):
    self.points_cur = fpi
    self.user_timeout_last_t = -self.user_timeout_t
    self.cutin_t_last = -self.cutin_rescind_t_bp[-1]

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
    
    self.df = DynamicFollow()
    self.follow_level_last = 1
    self.follow_level_df = 0.
    self.dynamic_follow_active = False
    
    self.one_pedal_mode_active_last = False
    self.coast_one_pedal_mode_active_last = False
    
    self.params_check_last_t = 0.
    self.params_check_freq = 0.5 # check params at 2Hz
    self._params = Params()

  def reset_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.lead_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU
    self.df = DynamicFollow()

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
    
    t = sec_since_boot()
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      dynamic_follow_active = self._params.get_bool("DynamicFollow")
      if dynamic_follow_active and dynamic_follow_active != self.dynamic_follow_active:
        self.df.reset(1)
      self.dynamic_follow_active = dynamic_follow_active
    
    if (not CS.onePedalModeActive and self.one_pedal_mode_active_last) or (not CS.coastOnePedalModeActive and self.coast_one_pedal_mode_active_last):
      self.df.reset(1)
    self.one_pedal_mode_active_last = CS.onePedalModeActive
    self.coast_one_pedal_mode_active_last = CS.coastOnePedalModeActive
    
    if follow_level != self.follow_level_last:
      self.df.set_fp(follow_level)
      
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
      # dynamic follow 
      if self.dynamic_follow_active:
        self.follow_level_df = self.df.update(True, lead.dRel, v_lead, v_ego)
        tr, dist_cost, accel_cost = interp_follow_profile(v_ego, v_lead, lead.dRel, self.follow_level_df)
      else:
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
      self.accel_cost_last = accel_cost
    
    self.tr = tr
    self.dist_cost = dist_cost / MPC_COST_LONG.DISTANCE
    self.accel_cost = accel_cost / MPC_COST_LONG.ACCELERATION
    self.stopping_distance = stopping_distance
    self.follow_level_last = follow_level
      
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
