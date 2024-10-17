import math
import numpy as np
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, clip
from common.realtime import sec_since_boot, DT_MDL
from common.op_params import opParams
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

CLOSE_FOLLOW_MIN_FOLLOW_DIST_S = 0.2
MEDIUM_FOLLOW_MIN_FOLLOW_DIST_S = 0.5
FOLLOW_PROFILES = [
  [ # one-bar
    [-1.4, 2.8], # bp0 and bp1; lead car relative velocities [m/s] (set both to 0.0 to disable dynamic brakepoints)
    [0.6, 2.0], # follow distances corresponding to bp0 and bp1 [s]
    [0.0, 1.892, 3.7432, 5.8632, 8.0727, 10.7301, 14.343, 17.6275, 22.4049, 28.6752, 34.8858, 40.35], # lookup table of speeds for additional follow distances [m/s] (stolen from shane)
    [0.0, 0.00099, -0.0324, -0.0647, -0.0636, -0.0601, -0.0296, -0.1211, -0.2141, -0.3691, -0.38, -0.40], # additional follow distances based on speed [s]
    0., # stopping distance behind stopped lead car [m]
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
    [0.8, 1.5],
    [0.0, 1.8627, 3.7253, 5.588, 7.4507, 9.3133, 11.5598, 13.645, 22.352, 31.2928, 33.528, 35.7632, 40.2336],
    [0.0, 0.0034975, 0.008495, 0.015, 0.025, 0.03945, 0.06195, 0.0745, 0.08895, 0.1005, 0.10495, 0.11045, 0.11845],
    0.,
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
    0.,
    [0.8, 1.4, 1.8, 3.5],
    [MPC_COST_LONG.DISTANCE * 1.5, MPC_COST_LONG.DISTANCE * 0.7, MPC_COST_LONG.DISTANCE * 0.15, MPC_COST_LONG.DISTANCE * 0.1],
    [15., 25., 35., 45.], # meters behind lead car
    [MPC_COST_LONG.DISTANCE * 1.5, MPC_COST_LONG.DISTANCE * 0.8, MPC_COST_LONG.DISTANCE * 0.25, MPC_COST_LONG.DISTANCE * 0.1],
    0.5 * CV.MPH_TO_MS,
    0.05 * CV.MPH_TO_MS,
  ]
]

cp = FOLLOW_PROFILES[0] # close follow profile
# these are used for opParams adjustment of close-follow responsiveness
CLOSE_FOLLOW_EQUIL_FOLLOW_DISTANCE = interp(0.0, cp[0], cp[1])
close_max_delta_follow = abs(CLOSE_FOLLOW_EQUIL_FOLLOW_DISTANCE - CLOSE_FOLLOW_MIN_FOLLOW_DIST_S)
CLOSE_TOWARDS_BP = [0.0, 2.0] # corresponds to the bounds of the FP_close_gas_factor opParam
CLOSE_TOWARDS_V = [0.0, close_max_delta_follow]
CLOSE_AWAY_RANGE = abs(cp[1][-1] - CLOSE_FOLLOW_EQUIL_FOLLOW_DISTANCE)

mp = FOLLOW_PROFILES[1] # close follow profile
# these are used for opParams adjustment of close-follow responsiveness
MEDIUM_FOLLOW_EQUIL_FOLLOW_DISTANCE = interp(0.0, mp[0], mp[1])
medium_max_delta_follow = abs(MEDIUM_FOLLOW_EQUIL_FOLLOW_DISTANCE - MEDIUM_FOLLOW_MIN_FOLLOW_DIST_S)
MEDIUM_TOWARDS_BP = [0.0, 2.0] # corresponds to the bounds of the FP_close_gas_factor opParam
MEDIUM_TOWARDS_V = [0.0, medium_max_delta_follow]
MEDIUM_AWAY_RANGE = abs(mp[1][-1] - MEDIUM_FOLLOW_EQUIL_FOLLOW_DISTANCE)

FP_MIN_MAX_DIST_COSTS = [[f(f(fp[6]),f(fp[8])) for f in [min,max]] for fp in FOLLOW_PROFILES]

LEAD_PULLAWAY_V_REL = -0.5 * CV.MPH_TO_MS # [m/s] lead car pull-away speed cost shift factor cutoff! Lead car has to be pulling away faster than this before it starts increasing the mpc distance cost (must be negative)
LEAD_APPROACHING_V_REL = 10. * CV.MPH_TO_MS # [m/s] lead car approaching cost shift factor cutoff! Lead car has to be approaching faster than this before it starts increasing/decreasing the mpc distance/acceleration costs (must be positive)

# calculate the desired follow distance and mcp distance cost from current state
def calc_follow_profile(v_ego, v_lead, x_lead, fpi, follow_profiles, follow_distance_offsets):
  fp = follow_profiles[fpi]
  # adjust based on speed for sng smooth stopping
  sng_factor = interp(v_ego, [SNG_SPEED * 0.6, SNG_SPEED], [1., 0.])
  v_rel = v_ego - v_lead   # calculate relative velocity vs lead car
  d_lead = (x_lead / v_ego) if v_ego > 0.1 else 0.0 # distance to lead car in seconds
  
  # first target distance
  hwy_shift = interp(v_ego, fp[2], fp[3]) # calculate variable shift of objective follow distance based on city/highway speed
  tr_equil = interp(0., fp[0], fp[1]) # distance when speed is matched
  lead_sng_factor = interp(v_lead, [0., SNG_SPEED], [1., 0.])
  tr = interp(v_rel, fp[0], fp[1]) + hwy_shift # calculate objective distance in seconds(ish)
  tr += follow_distance_offsets[fpi]
  tr = tr_equil * lead_sng_factor + tr * (1.0 - lead_sng_factor)
  tr = max(tr_equil,TR_DEFAULT) * sng_factor + tr * (1.0 - sng_factor)
  
  # then distance cost
  dist_cost = max(interp(d_lead, fp[5], fp[6]), interp(x_lead, fp[7], fp[8]))
  dist_cost = SNG_DIST_COST * sng_factor + dist_cost * (1.0 - sng_factor)
  if v_rel < LEAD_PULLAWAY_V_REL:
    dist_cost += MPC_COST_LONG.DISTANCE * (LEAD_PULLAWAY_V_REL - v_rel) * fp[9]
  elif v_rel > LEAD_APPROACHING_V_REL:
    dist_cost += MPC_COST_LONG.DISTANCE * (v_rel - LEAD_APPROACHING_V_REL) * fp[10]
  dist_cost = clip(dist_cost, FP_MIN_MAX_DIST_COSTS[fpi][0], FP_MIN_MAX_DIST_COSTS[fpi][1])
    
  return tr, dist_cost, MPC_COST_LONG.ACCELERATION

def interp_follow_profile(v_ego, v_lead, x_lead, fp_float, follow_profiles, follow_distance_offsets):
  if fp_float <= 0.:
    fp = calc_follow_profile(v_ego, v_lead, x_lead, 0, follow_profiles, follow_distance_offsets)
  elif fp_float < 1.:
    fp1,fp2 = [calc_follow_profile(v_ego, v_lead, x_lead, i, follow_profiles, follow_distance_offsets) for i in [0,1]]
    fp = [(1. - fp_float) * i + fp_float * j for i,j in zip(fp1,fp2)]
  elif fp_float < 2.:
    fptmp = fp_float - 1
    fp1,fp2 = [calc_follow_profile(v_ego, v_lead, x_lead, i, follow_profiles, follow_distance_offsets) for i in [1,2]]
    fp = [(1. - fptmp) * i + fptmp * j for i,j in zip(fp1,fp2)]
  else:
    fp = calc_follow_profile(v_ego, v_lead, x_lead, 2, follow_profiles, follow_distance_offsets)
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
  cutin_time_dist_penalty_bp = [.2, 1.0, 1.5]	# [distance from cut-in in seconds]
  cutin_time_dist_penalty_v = [1.5, 0.5, 0.0]  # [follow profile change]

  cutin_dist_penalty_bp = [i * 0.3 for i in [15, 30., 60.]]	# [distance from cut-in in ft]
  cutin_dist_penalty_v = [1.5, 0.75, 0.0]  # [follow profile change]
  
  cutin_dist_max = 50.0 # [m] leads past this distance aren't used
  
  # penalize more for approaching cut-ins, and *offset* the distance penalty for cut-ins pulling away
  cutin_vel_penalty_bp = [i * CV.MPH_TO_MS for i in [-15., 0., 15.]]  # [mph] relative velocity of new lead
  cutin_vel_penalty_v = [-1.5,0., 2.5]  # [follow profile] additionally go to close follow for new lead approaching too quickly, or *offset* the penalty for a close cutin if they're moving away (to keep from darting after a new cutin)
  
  # bonus penalty factor for repeat cut-ins
  cutin_last_t_factor_bp = [0., 15.] # [s] time since last cutin
  cutin_last_t_factor_v = [3., 1.] # [unitless] factor of cutin penalty
  
  # rescind penalty for cut-overs (cut-in and then left lane quickly)
  cutin_rescind_t_bp = [2.,6.] # [s] time since last cut-in
  cutin_rescind_t_v = [1., 0.] # [unitless] factor of cut-in penalty that is rescinded
  
  ############################################
  #    PENALIZING BASED ON ADJACENT TRAFFIC
  ############################################
  
  traffic_penalty_factor = 0.05 * DT_MDL # amount of penalty per second. 0.05 means it takes 20 seconds for it to go from medium to close follow (this is scaled by the number of adjacent cars)
  
  def __init__(self,fpi = 1):
    self._op_params = opParams(calling_function="lead mpc DynamicFollow")
    self.points_cur = fpi # [follow profile number 0-based] number of current points. corresponds to follow level
    self.t_last = 0. # sec_since_boot() on last iteration
    self.cutin_penalty_last = 0.  # penalty for most recent cutin, so that it can be rescinded if the cutin really just cut *over* in front of you (i.e. they quickly disappear)
    self.lead_d_last = 0.
    self.has_lead_last = False
    self.user_timeout_last_t = -self.user_timeout_t # (i.e. it's already been 300 seconds)
    self.cutin_t_last = 0. # sec_since_boot() of last remembered cut-in
    self.new_lead = False
    self.lead_gone = False
    self.penalty_dist = 0.
    self.penalty_vel = 0.
    self.penalty_time = 0.
    self.penalty = 0.
    self.last_cutin_factor = 0.
    self.rescinded_penalty = 0.
    self.traffic_penalty = 0.
    
    self.lateralPlan = None

    self.update_op_params()
  
  def update_op_params(self):
    self.distance_gain_factor = self._op_params.get('FP_DF_distance_gain_factor')
    self.distance_penalty_factor = self._op_params.get('FP_DF_distance_penalty_factor')
    self.velocity_penalty_factor = self._op_params.get('FP_DF_velocity_penalty_factor')
    self.traffic_penalty_factor = self._op_params.get('FP_DF_traffic_penalty_factor')
  
  def update_init(self):
    self.new_lead = False
    self.lead_gone = False
    self.penalty_dist = 0.
    self.penalty_vel = 0.
    self.penalty_time = 0.
    self.penalty = 0.
    self.last_cutin_factor = 0.
    self.rescinded_penalty = 0.
  
  def update(self, has_lead, lead_d, lead_v, v_ego, tr):
    self.update_init()
    
    self.traffic_penalty = 0.0
    if self.lateralPlan is not None:
      if self.lateralPlan.trafficCountLeft > 1:
        error = max(0.0, tr - self.lateralPlan.trafficMinSeperationLeft)
        error *= (self.lateralPlan.trafficCountLeft - 1)
        self.traffic_penalty = self.traffic_penalty_factor * error
      if self.lateralPlan.trafficCountRight > 1:
        error = max(0.0, tr - self.lateralPlan.trafficMinSeperationRight)
        error *= (self.lateralPlan.trafficCountRight - 1)
        self.traffic_penalty = max(self.traffic_penalty_factor * error, self.traffic_penalty)
      self.traffic_penalty *= self.traffic_penalty_factor
    
    t = sec_since_boot()
    dur = t - self.t_last
    self.t_last = t
    self.lead_gone = (self.has_lead_last and not has_lead) \
                or (has_lead and self.lead_d_last - lead_d < -5.0 \
                  and lead_d <= self.cutin_dist_max)
    self.new_lead = has_lead and lead_d <= self.cutin_dist_max \
                and (not self.has_lead_last or self.lead_d_last - lead_d > 5.0)
    self.lead_d_last = lead_d if has_lead else 1000.0
    self.has_lead_last = has_lead
    if self.new_lead:
      if v_ego > 0.:
        time_dist = lead_d / v_ego
        self.penalty_time = interp(time_dist, self.cutin_time_dist_penalty_bp, self.cutin_time_dist_penalty_v)
      else:
        self.penalty_time = 0.
        time_dist = 100.
      self.penalty_dist = interp(lead_d, self.cutin_dist_penalty_bp, self.cutin_dist_penalty_v)
      self.penalty_dist = max(self.penalty_dist, self.penalty_time)
      self.penalty_dist *= self.distance_penalty_factor
      lead_v_rel = v_ego - lead_v
      self.penalty_vel = interp(lead_v_rel, self.cutin_vel_penalty_bp, self.cutin_vel_penalty_v)
      self.penalty_vel *= self.velocity_penalty_factor
      self.penalty = max(0., self.penalty_dist + self.penalty_vel)
      self.last_cutin_factor = interp(t - self.cutin_t_last, self.cutin_last_t_factor_bp, self.cutin_last_t_factor_v)
      self.penalty *= self.last_cutin_factor
      points_old = self.points_cur
      if t - self.user_timeout_last_t > self.user_timeout_t:
        self.points_cur = max(self.points_bounds[0], self.points_cur - self.penalty - self.traffic_penalty)
      self.cutin_t_last = t
      self.cutin_penalty_last = points_old - self.points_cur
      return self.points_cur
    elif self.lead_gone and t - self.user_timeout_last_t > self.user_timeout_t and t - self.cutin_t_last < self.cutin_rescind_t_bp[-1]:
      self.rescinded_penalty = self.cutin_penalty_last * interp(t - self.cutin_t_last, self.cutin_rescind_t_bp, self.cutin_rescind_t_v)
      self.points_cur += max(0,self.rescinded_penalty)
      self.cutin_t_last = t - self.cutin_rescind_t_bp[-1] - 1.
  
    rate = interp(self.points_cur, self.fp_point_rate_bp, self.fp_point_rate_v)
    speed_factor = interp(v_ego, self.speed_rate_factor_bp, self.speed_rate_factor_v)
    step = rate * dur * speed_factor * self.distance_gain_factor
    max_points = interp(v_ego, self.speed_fp_limit_bp, self.speed_fp_limit_v)
    if t - self.user_timeout_last_t > self.user_timeout_t:
      if self.traffic_penalty > 0.0:
        self.traffic_penalty *= speed_factor
        self.points_cur = min(max_points, self.points_cur - self.traffic_penalty)
      else:
        self.points_cur = min(max_points, self.points_cur + step)

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

    self.df = DynamicFollow()
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

    self.tr_override = False
    
    self.v_solution = np.zeros(CONTROL_N)
    self.a_solution = np.zeros(CONTROL_N)
    self.j_solution = np.zeros(CONTROL_N)
    
    self.follow_level_last = 1
    self.follow_level_df = 0.
    self.dynamic_follow_active = False
    
    self.MADS_lead_braking_enabled = False
    self.long_control_active = False
    
    self.params_check_last_t = 0.
    self.params_check_freq = 0.5 # check params at 2Hz
    self._params = Params()
    self._op_params = opParams(calling_function="lead mpc LeadMpc")
    self.stopping_distance_offset = self._op_params.get('FP_stop_distance_offset_m', force_update=True)
    self.jerk_cost = self._op_params.get('FP_jerk_cost', force_update=True)
    self.jerk_cost_last = self.jerk_cost
    
    self._follow_profiles = FOLLOW_PROFILES[:]
    self.update_op_params()

  def reset_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.lead_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU
    self.df.reset()
    
  def set_cur_state(self, v, a):
    v_safe = max(v, 1e-3)
    a_safe = a
    self.cur_state.v_ego = v_safe
    self.cur_state.a_ego = a_safe
  
  def update_op_params(self):
    self._close_gas_factor = self._op_params.get('FP_close_gas_factor')
    self._medium_gas_factor = self._op_params.get('FP_medium_gas_factor')
    self._follow_distance_offsets = [self._op_params.get(f"FP_{i}_distance_offset_s") for i in ['close','medium','far']]
    # update close follow gas factor
    close_follow_towards_delta = interp(self._close_gas_factor, CLOSE_TOWARDS_BP, CLOSE_TOWARDS_V)
    new_close_follow_towards = max(CLOSE_FOLLOW_MIN_FOLLOW_DIST_S,
                                   CLOSE_FOLLOW_EQUIL_FOLLOW_DISTANCE - close_follow_towards_delta)
    close_follow_away_delta = CLOSE_AWAY_RANGE * self._close_gas_factor
    new_close_follow_away = CLOSE_FOLLOW_EQUIL_FOLLOW_DISTANCE + close_follow_away_delta
    self._follow_profiles[0][1] = [new_close_follow_towards, new_close_follow_away]
    # now medium follow gas factor
    medium_follow_towards_delta = interp(self._medium_gas_factor, MEDIUM_TOWARDS_BP, MEDIUM_TOWARDS_V)
    new_medium_follow_towards = max(MEDIUM_FOLLOW_MIN_FOLLOW_DIST_S,
                                    MEDIUM_FOLLOW_EQUIL_FOLLOW_DISTANCE - medium_follow_towards_delta)
    medium_follow_away_delta = MEDIUM_AWAY_RANGE * self._medium_gas_factor
    new_medium_follow_away = MEDIUM_FOLLOW_EQUIL_FOLLOW_DISTANCE + medium_follow_away_delta
    self._follow_profiles[1][1] = [new_medium_follow_towards, new_medium_follow_away]
    
    self.stopping_distance_offset = self._op_params.get('FP_stop_distance_offset_m')
    
    self.jerk_cost = self._op_params.get('FP_jerk_cost')

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
    if self.MADS_lead_braking_enabled and not self.long_control_active:
      follow_level = 1
    
    t = sec_since_boot()
    if t - self.params_check_last_t >= self.params_check_freq:
      self.params_check_last_t = t
      self.update_op_params()
      dynamic_follow_active = self._params.get_bool("DynamicFollow")
      if dynamic_follow_active and dynamic_follow_active != self.dynamic_follow_active:
        self.df.reset(1)
      self.dynamic_follow_active = dynamic_follow_active
    
    if not self.long_control_active:
      self.df.reset(1)
      self.dynamic_follow_active = False
    
    if follow_level != self.follow_level_last:
      self.df.set_fp(follow_level)
      
    if follow_level < 0 or follow_level > 2:
      follow_level = 1
    fp = self._follow_profiles[follow_level]
    stopping_distance = fp[4] + self.stopping_distance_offset
    
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
        self.follow_level_df = self.df.update(lead.status, lead.dRel, v_lead, v_ego, self.tr)
        tr, dist_cost, accel_cost = interp_follow_profile(v_ego, v_lead, lead.dRel, self.follow_level_df, self._follow_profiles, self._follow_distance_offsets)
      else:
        tr, dist_cost, accel_cost = calc_follow_profile(v_ego, v_lead, lead.dRel, follow_level, self._follow_profiles, self._follow_distance_offsets)
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
    
    if dist_cost != self.dist_cost_last or accel_cost != self.accel_cost_last or self.jerk_cost != self.jerk_cost_last:
      self.libmpc.change_costs(MPC_COST_LONG.TTC, dist_cost, accel_cost, self.jerk_cost)
      self.dist_cost_last = dist_cost
      self.accel_cost_last = accel_cost
      self.jerk_cost_last = self.jerk_cost
    
    
    if not self.tr_override:
      self.tr = tr
      
    self.dist_cost = dist_cost / MPC_COST_LONG.DISTANCE
    self.accel_cost = accel_cost / MPC_COST_LONG.ACCELERATION
    self.stopping_distance = stopping_distance
    self.follow_level_last = follow_level
      
    t = sec_since_boot()
    self.n_its = 0
    self.n_its += self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, self.tr)
    
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
