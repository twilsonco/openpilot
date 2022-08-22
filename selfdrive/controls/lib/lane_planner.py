import numpy as np
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp, clip
from common.realtime import DT_MDL
from common.realtime import sec_since_boot
from selfdrive.hardware import EON, TICI
from selfdrive.swaglog import cloudlog
from enum import Enum

TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
if EON:
  CAMERA_OFFSET = 0.06
  PATH_OFFSET = 0.0
elif TICI:
  CAMERA_OFFSET = -0.04
  PATH_OFFSET = -0.04
else:
  CAMERA_OFFSET = 0.0
  PATH_OFFSET = 0.0

LANE_WIDTH_DEFAULT = 3.7

class LaneOffset: 
  OFFSET = 0.11 # [unitless] offset of the left/right positions as factor of current lane width
  AUTO_OFFSET_FACTOR = 0.8 # 20% less offset when auto mode
  OFFSET_MAX = 0.8 # [m]
  DUR = 2.0 # [s] time it takes to switch lane positions
  STEP = OFFSET / DUR * DT_MDL * LANE_WIDTH_DEFAULT
  
  DUR_SLOW = 8.0 # [s] same, but slower for when position change is caused by auto lane offset
  STEP_SLOW = OFFSET / DUR_SLOW * DT_MDL * LANE_WIDTH_DEFAULT
  
  AUTO_MIN_SHOULDER_WIDTH_FACTOR = 0.00 # [unitless] shoulder width must be as this this factor of lane width
  AUTO_MAX_LANE_WIDTH_FACTOR = 0.6 # [unitless] adjacent lanes whose width is narrower than this factor of lane width are considered a shoulder
  
  AUTO_MAX_PRED_LAT_ACCEL = 0.9 # [m/s^2] more than this predicted amount of lateral accel causes instant resuming of center position
  AUTO_MAX_CUR_LAT_ACCEL = 0.7 # same but for instantaneous vehicle lateral accel
  
  AUTO_MIN_LANELINE_PROB = 0.9
  AUTO_MIN_ADJACENT_LANELINE_PROB = 0.3
  
  AUTO_LANE_STATE_MIN_TIME = 3.0 # [s] amount of time the lane state must stay the same before it can be acted upon
  
  AUTO_ENABLE_ROAD_TYPES = {0, 10, 20, 30} # freeway and state highways (see highway ranks in /Users/haiiro/NoSync/optw/openpilot/selfdrive/mapd/lib/WayRelation.py)
  
  def __init__(self, mass=0.):
    self.offset = 0.
    self.lane_pos = 0.
    self.offset_scale = interp(float(mass), [1607., 2729.], [1., 0.7]) # scales down offset based on vehicle width (assumed to go as mass)
    self._auto_is_active = False
    self._auto_was_enabled_by_road_type = False
    self._road_type = 100 # unknown
    self._lane_pos_auto = 0. # 0., 1., -1. = center, left, right
    self._lane_probs = np.zeros(4) # [left adjacent, left, right, right adjacent]
    self._road_edge_probs = np.zeros(2) # [left, right]
    self._lane_width_mean_left_adjacent = 0.
    self._lane_width_mean_right_adjacent = 0.
    self._shoulder_width_mean_left = 0.
    self._shoulder_width_mean_right = 0.
    self._cs = None
    self._lp = None
    self._lat_accel_cur = 0.
    self._lat_accel_pred = 0.
    self._lane_state_changed_last_t = 0.
  
  def update_lane_info(self, md, v_ego, lane_width):
    self._lane_width_mean_left_adjacent = 0.
    self._lane_width_mean_right_adjacent = 0.
    self._shoulder_width_mean_left = 0.
    self._shoulder_width_mean_right = 0.
    if len(md.laneLines) == 4 and len(md.laneLines[0].t) == TRAJECTORY_SIZE \
        and len(md.roadEdges) >= 2 and len(md.roadEdges[0].t) == TRAJECTORY_SIZE:
      
      # use mean lane widths so that it can start recentering in response to
      # upcoming bad conditions.
      
      # get road edge "probabilities"
      self._road_edge_probs = [ interp(md.roadEdgeStds[i], [.3, 1.], [1.0, 0.0]) for i in range(2) ]
            
      # get laneline probabilities
      for i in range(3): 
        l_prob, r_prob = md.laneLineProbs[i], md.laneLineProbs[i+1]
        if ((i == 0 and l_prob > self.AUTO_MIN_ADJACENT_LANELINE_PROB) or l_prob > self.AUTO_MIN_LANELINE_PROB) \
            and ((i == 2 and r_prob > self.AUTO_MIN_ADJACENT_LANELINE_PROB) or r_prob > self.AUTO_MIN_LANELINE_PROB):
          # Reduce reliance on uncertain lanelines (less stringent for adjacent lanelines)
          rll_y, lll_y = np.array(md.laneLines[i+1].y), np.array(md.laneLines[i].y)
          l_std_mod = interp(md.laneLineStds[i], [.15, .3 if i > 0 else .6], [1.0, 0.0])
          r_std_mod = interp(md.laneLineStds[i+1], [.15, .3 if i < 2 else .6], [1.0, 0.0])
          l_prob *= l_std_mod
          r_prob *= r_std_mod
          
          if i == 0: # set left adjacent 
            self._lane_probs[0] = l_prob
            self._lane_width_mean_left_adjacent = np.mean(rll_y - lll_y)
            if self._road_edge_probs[0] > 0.:
              self._shoulder_width_mean_left = np.mean(lll_y - np.array(md.roadEdges[0].y))
          elif i == 1: # set center
            self._lane_probs[1] = l_prob
            self._lane_probs[2] = r_prob
          else: # set right 
            self._lane_probs[3] = r_prob
            self._lane_width_mean_right_adjacent = np.mean(rll_y - lll_y)
            if self._road_edge_probs[1] > 0.:
              self._shoulder_width_mean_right = np.mean(np.array(md.roadEdges[1].y) - rll_y)
        else:
          l_std_mod = interp(md.laneLineStds[i], [.15, .3 if i > 0 else .6], [1.0, 0.0])
          r_std_mod = interp(md.laneLineStds[i+1], [.15, .3 if i < 2 else .6], [1.0, 0.0])
          l_prob *= l_std_mod
          r_prob *= r_std_mod
          if i == 0: # set left adjacent 
            self._lane_probs[0] = l_prob
          elif i == 1: # set center
            self._lane_probs[1] = l_prob
            self._lane_probs[2] = r_prob
          else: # set right 
            self._lane_probs[3] = r_prob

      
      # see if adjacent lane can be treated as shoulder because it's too narrow
      if self._lane_width_mean_left_adjacent < self.AUTO_MAX_LANE_WIDTH_FACTOR * lane_width:
        self._shoulder_width_mean_left += self._lane_width_mean_left_adjacent
        self._lane_width_mean_left_adjacent = 0.
      if self._lane_width_mean_right_adjacent < self.AUTO_MAX_LANE_WIDTH_FACTOR * lane_width:
        self._shoulder_width_mean_right += self._lane_width_mean_right_adjacent
        self._lane_width_mean_right_adjacent = 0.
    return
  
  def update_lane_pos_auto(self, lane_width):
    lane_pos_auto = 0.
    if self._lane_probs[1] > self.AUTO_MIN_LANELINE_PROB \
        and self._lane_probs[2] > self.AUTO_MIN_LANELINE_PROB \
        and self._lat_accel_cur < self.AUTO_MAX_CUR_LAT_ACCEL \
        and self._lat_accel_pred < self.AUTO_MAX_PRED_LAT_ACCEL:
      if (self._lane_width_mean_left_adjacent > 0. and self._lane_width_mean_right_adjacent > 0.) \
          or (self._lane_width_mean_left_adjacent == 0. and self._lane_width_mean_right_adjacent == 0.):
        lane_pos_auto = 0.
      elif self._lane_width_mean_left_adjacent > 0. and self._shoulder_width_mean_right >= self.AUTO_MIN_SHOULDER_WIDTH_FACTOR * lane_width:
        lane_pos_auto = -1.
      elif self._lane_width_mean_right_adjacent > 0. and self._shoulder_width_mean_left >= self.AUTO_MIN_SHOULDER_WIDTH_FACTOR * lane_width:
        lane_pos_auto = 1.
    
    if lane_pos_auto != self._lane_pos_auto:
      self._lane_state_changed_last_t = sec_since_boot()
    
    self._lane_pos_auto = lane_pos_auto
    return lane_pos_auto
    
  def update(self, lane_pos=0., lane_width=LANE_WIDTH_DEFAULT, auto_active=False, md=None, sm=None): # 0., 1., -1. = center, left, right
    t = sec_since_boot()
    
    if md is not None and sm is not None:
      if sm.valid.get('carState', False):
        self._cs = sm['carState']
      if sm.valid.get('longitudinalPlan', False):
        self._lp = sm['longitudinalPlan']
      if self._cs is not None:
        self.update_lane_info(md, self._cs.vEgo, lane_width)
        self.update_lane_pos_auto(lane_width)
      if self._lp is not None:
        self._lat_accel_cur = self._lp.visionCurrentLateralAcceleration
        self._lat_accel_pred = self._lp.visionMaxPredictedLateralAcceleration
      self._auto_is_active = auto_active
    else:
      self._auto_is_active = False
      
    do_slow = self._auto_is_active # and self._lane_pos_auto != 0.
    if self._auto_is_active:
      if (t - self._lane_state_changed_last_t > self.AUTO_LANE_STATE_MIN_TIME):
        self.lane_pos = self._lane_pos_auto
      else:
        self.lane_pos = 0.
    else:
      self.lane_pos = lane_pos
    offset = self.OFFSET * self.offset_scale * self.lane_pos * lane_width * (self.AUTO_OFFSET_FACTOR if self._auto_is_active else 1.)
    if offset > self.offset:
      self.offset = min(offset, self.offset + (self.STEP_SLOW if do_slow else self.STEP))
    elif offset < self.offset:
      self.offset = max(offset, self.offset - (self.STEP_SLOW if do_slow else self.STEP))
    return clip(self.offset, -self.OFFSET_MAX, self.OFFSET_MAX)

class LanePlanner:
  def __init__(self, wide_camera=False, mass=0.):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(LANE_WIDTH_DEFAULT, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = LANE_WIDTH_DEFAULT
    self.lane_offset = LaneOffset(mass)

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

  def parse_model(self, md, lane_pos = 0., sm=None, auto_lane_pos_active=False):
    offset = self.lane_offset.update(lane_pos, self.lane_width, auto_active=auto_lane_pos_active, md=md, sm=sm)
    if len(md.laneLines) == 4 and len(md.laneLines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(md.laneLines[1].t) + np.array(md.laneLines[2].t))/2
      # left and right ll x is the same
      self.ll_x = md.laneLines[1].x
      # only offset left and right lane lines; offsetting path does not make sense
      self.lll_y = np.array(md.laneLines[1].y) - self.camera_offset - offset
      self.rll_y = np.array(md.laneLines[2].y) - self.camera_offset - offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(md.meta.desireState):
      self.l_lane_change_prob = md.meta.desireState[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = md.meta.desireState[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] -= self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in [0.0, 1.5, 3.0]:
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0., 31.], [2.5, 3.5])
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(path_t, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    else:
      cloudlog.warning("Lateral mpc - NaNs in laneline times, ignoring")
    return path_xyz
