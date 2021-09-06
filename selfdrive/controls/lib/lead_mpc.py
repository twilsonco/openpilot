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

# One, two and three bar distances (in s)
ONE_BAR_DISTANCE = 0.8  # in seconds
TWO_BAR_DISTANCE = 1.5  # in seconds
THREE_BAR_DISTANCE = 3.0  # in seconds
FOUR_BAR_DISTANCE = 3.0   # in seconds
STOPPING_DISTANCE = 1.8  # distance between you and lead car when you come to stop

TR = TWO_BAR_DISTANCE  # default interval

 # Variables that change braking profiles
CITY_SPEED = 19.44  # braking profile changes when below this speed based on following dynamics below [m/s]

# City braking profile changes (makes the car brake harder because it wants to be farther from the lead car - increase to brake harder)
ONE_BAR_PROFILE = [ONE_BAR_DISTANCE, 2.5]
ONE_BAR_PROFILE_BP = [0, 2.75]

TWO_BAR_PROFILE = [TWO_BAR_DISTANCE, 2.5]
TWO_BAR_PROFILE_BP = [0, 3.0]

THREE_BAR_PROFILE = [THREE_BAR_DISTANCE, 2.5]
THREE_BAR_PROFILE_BP = [0.0, 4.0]

# Highway braking profiles
H_ONE_BAR_PROFILE = [ONE_BAR_DISTANCE, ONE_BAR_DISTANCE+0.5]
H_ONE_BAR_PROFILE_BP = [0.0, 2.5]

H_TWO_BAR_PROFILE = [TWO_BAR_DISTANCE, TWO_BAR_DISTANCE+0.3]
H_TWO_BAR_PROFILE_BP = [0.0, 3.0]

H_THREE_BAR_PROFILE = [THREE_BAR_DISTANCE, THREE_BAR_DISTANCE+0.3]
H_THREE_BAR_PROFILE_BP = [0.0, 4.0]

class LeadMpc():
  def __init__(self, mpc_id):
    self.lead_id = mpc_id

    self.reset_mpc()
    self.v_mpc = 0.0
    self.v_mpc_future = 0.0
    self.a_mpc = 0.0
    self.v_cruise = 0.0
    self.prev_lead_status = False
    self.prev_lead_x = 0.0
    self.new_lead = False
    self.v_rel = 10
    self.lastTR = 2

    self.last_cloudlog_t = 0.0
    self.n_its = 0
    self.duration = 0
    self.status = False

    self.v_solution = np.zeros(CONTROL_N)
    self.a_solution = np.zeros(CONTROL_N)
    self.j_solution = np.zeros(CONTROL_N)
    
    self.oneBarBP = [-0.1, 2.25]
    self.twoBarBP = [-0.1, 2.5]
    self.threeBarBP = [0.5, 3.0]
    self.oneBarProfile = [ONE_BAR_DISTANCE, 2.1]
    self.twoBarProfile = [TWO_BAR_DISTANCE, 2.1]
    self.threeBarProfile = [THREE_BAR_DISTANCE, 2.4]
    self.oneBarHwy = [ONE_BAR_DISTANCE, ONE_BAR_DISTANCE+0.5]
    self.twoBarHwy = [TWO_BAR_DISTANCE, TWO_BAR_DISTANCE+0.3]
    self.threeBarHwy = [THREE_BAR_DISTANCE, THREE_BAR_DISTANCE+0.3]

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
    self.cur_state[0].v_ego = v_safe
    self.cur_state[0].a_ego = a_safe

  def update(self, CS, radarstate, v_cruise, a_target, active):
    v_ego = CS.vEgo
    if self.lead_id == 0:
      lead = radarstate.leadOne
    else:
      lead = radarstate.leadTwo
    self.status = lead.status

    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0

    if lead is not None and lead.status:
      x_lead = max(0, lead.dRel - STOPPING_DISTANCE)  # increase stopping distance to car by X [m]
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0

      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        self.libmpc.init_with_simulation(v_ego, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
    else:
      self.prev_lead_status = False
      # Fake a fast lead car, so mpc keeps running
      self.cur_state[0].x_l = 50.0
      self.cur_state[0].v_l = v_ego + 10.0
      a_lead = 0.0
      v_lead = 0.0
      self.a_lead_tau = _LEAD_ACCEL_TAU
    
    # Calculate conditions
    self.v_rel = v_lead - v_ego   # calculate relative velocity vs lead car
   
    # Is the car running surface street speeds?
    if v_ego < CITY_SPEED:
      self.street_speed = 1
    else:
      self.street_speed = 0

    # Calculate mpc
    # Adjust distance from lead car when distance button pressed 
    if CS.readdistancelines == 1:
      #if self.street_speed and (self.lead_car_gap_shrinking or self.tailgating):
      if self.street_speed:
        TR = interp(-self.v_rel, self.oneBarBP, self.oneBarProfile)  
      else:
        TR = interp(-self.v_rel, H_ONE_BAR_PROFILE_BP, self.oneBarHwy) 
      if CS.readdistancelines != self.lastTR:
        self.libmpc.init(MPC_COST_LONG.TTC, 1.0, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = CS.readdistancelines  

    elif CS.readdistancelines == 2:
      #if self.street_speed and (self.lead_car_gap_shrinking or self.tailgating):
      if self.street_speed:
        TR = interp(-self.v_rel, self.twoBarBP, self.twoBarProfile)
      else:
        TR = interp(-self.v_rel, H_TWO_BAR_PROFILE_BP, self.twoBarHwy)
      if CS.readdistancelines != self.lastTR:
        self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = CS.readdistancelines  

    elif CS.readdistancelines == 3:
      if self.street_speed:
      #if self.street_speed and (self.lead_car_gap_shrinking or self.tailgating):
        TR = interp(-self.v_rel, self.threeBarBP, self.threeBarProfile)
      else:
        TR = interp(-self.v_rel, H_THREE_BAR_PROFILE_BP, self.threeBarHwy)
      if CS.readdistancelines != self.lastTR:
        self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
        self.lastTR = CS.readdistancelines   

    elif CS.readdistancelines == 4:
      TR = FOUR_BAR_DISTANCE
      if CS.readdistancelines != self.lastTR:
        self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK) 
        self.lastTR = CS.readdistancelines      

    else:
     TR = TWO_BAR_DISTANCE # if readdistancelines != 1,2,3,4
     self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
     
    t = sec_since_boot()
    self.n_its = self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, TR)
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

      self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                       MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.cur_state[0].v_ego = v_ego
      self.cur_state[0].a_ego = 0.0
      self.a_mpc = CS.aEgo
      self.prev_lead_status = False
