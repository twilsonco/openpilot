import numpy as np
import math
from cereal import log
from collections import defaultdict
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.op_params import opParams
from common.params import Params
from common.realtime import sec_since_boot, DT_MDL
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.lane_planner import TRAJECTORY_SIZE
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.swaglog import cloudlog


_MIN_V = 5.6  # Do not operate under 20km/h

_EVAL_STEP = 5.  # mts. Resolution of the curvature evaluation.
_EVAL_START = 20.  # mts. Distance ahead where to start evaluating vision curvature.
_EVAL_LENGHT = 150.  # mts. Distance ahead where to stop evaluating vision curvature.
_EVAL_RANGE = np.arange(_EVAL_START, _EVAL_LENGHT, _EVAL_STEP)

_MIN_LANE_PROB = 0.6  # Minimum lanes probability to allow curvature prediction based on lanes.

# scale up current measured lateral acceleration if the lateral controller is saturated
_LAT_SAT_DECEL_V = [1.0, 1.5] # unitless. scales current lateral acceleration
_LAT_SAT_DECEL_BP = [0.2, 1.2] # seconds. time since lateral controller saturated

_DEBUG = False

def _debug(msg):
  if not _DEBUG:
    return
  print(msg)

VisionTurnControllerState = log.LongitudinalPlan.VisionTurnControllerState

def eval_lat_acc(v_ego, x_curv):
  """
  This function returns a vector with the lateral acceleration based
  for the provided speed `v_ego` evaluated over curvature vector `x_curv`
  """

  def lat_acc(curv):
    a = v_ego**2 * curv
    return a

  return np.vectorize(lat_acc)(x_curv)


def _description_for_state(turn_controller_state):
  if turn_controller_state == VisionTurnControllerState.disabled:
    return 'DISABLED'
  if turn_controller_state == VisionTurnControllerState.entering:
    return 'ENTERING'
  if turn_controller_state == VisionTurnControllerState.turning:
    return 'TURNING'
  if turn_controller_state == VisionTurnControllerState.leaving:
    return 'LEAVING'


class VisionTurnController():
  def __init__(self, CP):
    self._params = Params()
    self._op_params = opParams(calling_function="vision turn controller")
    self._CP = CP
    self._VM = VehicleModel(CP)
    self._op_enabled = False
    self._gas_pressed = False
    self._is_enabled = self._params.get_bool("TurnVisionControl")
    self._last_params_update = 0.
    self._v_cruise_setpoint = 0.
    self._v_ego = 0.
    self._a_ego = 0.
    self._a_target = FirstOrderFilter(0., self._op_params.get('VTSC_smoothing_factor', force_update=True), DT_MDL)
    self._v_overshoot = 0.
    self._state = VisionTurnControllerState.disabled
    self._CS = None
    self._controls_state = None
    self._liveparams = None
    self._vf = 1.
    self._ema_k = 1/10 # exponential moving average factor for smoothing predicted values
    # scale velocity used to determine curvature in order to provide more braking at low speed
    # where the LKA torque is less capable despite low lateral acceleration.
    # This will be a default dict based on road type, allowing for easy adjustment of vision braking based on road type
    # See the list of "highway" types here https://wiki.openstreetmap.org/wiki/Key:highway
    # Also see selfdrive/mapd/lib/WayRelation.py for a list of ranks
    self._SPEED_SCALE_DEFAULT_BP = [i*CV.MPH_TO_MS for i in [18.0, 55.0]] # [mph] speeds corresponding to scaling values, so you can alter low/high speed behavior for each road type
    self._SPEED_SCALE_BP = [i*CV.MPH_TO_MS for i in [35.0, 55.0]]
    
    self._reset(True)
    
  def get_speed_scale(self, road_rank):
    if road_rank == 0:
      return [self._SPEED_SCALE_BP, [self._op_params.get('VTSC_low_speed_scale_freeway'), 1.0]]
    elif road_rank in [10, 20, 30]:
      return [self._SPEED_SCALE_BP, [self._op_params.get('VTSC_low_speed_scale_state_highway'), 1.0]]
    elif road_rank in [1,11,21,31]:
      return [self._SPEED_SCALE_BP, [self._op_params.get('VTSC_low_speed_scale_interchange'), 1.0]]
    else:
      return [self._SPEED_SCALE_DEFAULT_BP, [self._op_params.get('VTSC_low_speed_scale_default'), 1.0]]

  @property
  def state(self):
    return self._state

  @state.setter
  def state(self, value):
    if value != self._state:
      _debug(f'TVC: TurnVisionController state: {_description_for_state(value)}')
      if value == VisionTurnControllerState.disabled:
        self._reset()
    self._state = value

  @property
  def a_target(self):
    return self._a_target.x if self.is_active else self._a_ego

  @property
  def v_turn(self):
    return self._v_overshoot if self.is_active and self._lat_acc_overshoot_ahead else self._v_ego

  @property
  def is_active(self):
    return self._state != VisionTurnControllerState.disabled

  def _reset(self, full_reset = False):
    if full_reset:
      self._pred_curvatures = np.array([])
      self._max_pred_lat_acc = 0.
      self._max_pred_curvature = 0.
      self._max_pred_roll_compensation = 0.
      self._current_lat_acc = 0.
      self._current_lat_acc_no_roll = 0.
      self._max_v_for_current_curvature = 0.
      self._max_pred_lat_acc_dist = 0.
    self._v_overshoot_distance = 200.
    self._lat_acc_overshoot_ahead = False
    self._predicted_path_source = 'none'
    self._lat_sat_last = False
    self._lat_sat_t = 0.
    self._speed_scale_bp_v = self.get_speed_scale(-1)
  
  def eval_curvature(self, poly, x_vals, path_roll_poly, max_x):
    """
    This function returns a vector with the curvature based on path defined by `poly`
    evaluated on distance vector `x_vals`
    """
    max_lat_accel = 0.
    max_curvature = 0.
    max_roll_compensation = 0.
    max_lat_accel_dist = 0.
    # https://en.wikipedia.org/wiki/Curvature#  Local_expressions
    def curvature(x):
      nonlocal max_lat_accel, max_curvature, max_roll_compensation, max_lat_accel_dist
      a = (2 * poly[1] + 6 * poly[0] * x) / (1 + (3 * poly[0] * x**2 + 2 * poly[1] * x + poly[2])**2)**(1.5)
      xx = min(x,max_x) # use farthest predicted roll/velocity instead of extrapolating
      v = self._v_ego * self._vf
      rc = self._VM.roll_compensation(np.polyval(path_roll_poly, xx), v) if self._VM is not None else 0.
      if abs(rc) > abs(a): # don't want to brake for roll absent curvature
        rc = abs(a) * np.sign(rc)
      c = abs(a + rc)
      la = c * v**2
      if la > max_lat_accel:
        max_lat_accel = la
        max_curvature = a + rc
        max_roll_compensation = rc
        max_lat_accel_dist = xx
      return c
    
    return np.array([curvature(x) for x in x_vals]), max_lat_accel, max_curvature, max_roll_compensation, max_lat_accel_dist

  def _update_op_params(self):
    self._a_target.update_alpha(self._op_params.get('VTSC_smoothing_factor'))
    lat_accel_factor = self._op_params.get('VTSC_lat_accel_factor')
    long_accel_factor = self._op_params.get('VTSC_long_accel_factor')
    self._ENTERING_PRED_LAT_ACC_TH = 1.3 * lat_accel_factor  # Predicted Lat Acc threshold to trigger entering turn state.
    self._ABORT_ENTERING_PRED_LAT_ACC_TH = 0.9 * lat_accel_factor  # Predicted Lat Acc threshold to abort entering state if speed drops.
    self._TURNING_LAT_ACC_TH = 0.8 * lat_accel_factor  # Lat Acc threshold to trigger turning turn state.
    self._LEAVING_LAT_ACC_TH = 0.7 * lat_accel_factor  # Lat Acc threshold to trigger leaving turn state.
    self._FINISH_LAT_ACC_TH = 0.4 * lat_accel_factor  # Lat Acc threshold to trigger end of turn cycle.
    self._A_LAT_REG_MAX = 1.8 * lat_accel_factor  # Maximum lateral acceleration
    # Lookup table for the minimum smooth deceleration during the ENTERING state
    # depending on the actual maximum absolute lateral acceleration predicted on the turn ahead.
    self._ENTERING_SMOOTH_DECEL_V = [i * long_accel_factor for i in [0.0, -0.2, -1.0]]  # min decel value allowed on ENTERING state
    self._ENTERING_SMOOTH_DECEL_BP = [i * lat_accel_factor for i in [1.25, 1.3, 2.4]]  # absolute value of lat acc ahead
    # Lookup table for the acceleration for the TURNING state
    # depending on the current lateral acceleration of the vehicle.
    self._TURNING_ACC_V = [i * long_accel_factor if i < 0.0 else i for i in [0.5, 0.0, -0.8]]  # acc value
    self._TURNING_ACC_BP = [i * lat_accel_factor for i in [1.4, 1.9, 2.4]]  # absolute value of current lat acc
    self._LEAVING_ACC = 0.7 * lat_accel_factor  # Confortable acceleration to regain speed while leaving a turn.
    
    
  
  def _update_params(self):
    time = sec_since_boot()
    if time > self._last_params_update + 0.5:
      self._is_enabled = self._params.get_bool("TurnVisionControl")
      self._update_op_params()
      self._last_params_update = time

  def _update_calculations(self, sm):
    # Get path polynomial aproximation for curvature estimation from model data.
    path_poly = None
    model_data = sm['modelV2'] if sm.valid.get('modelV2', False) else None
    lat_planner_data = sm['lateralPlan'] if sm.valid.get('lateralPlan', False) else None
    
    # For updating VehicleModel
    if sm.valid.get('liveParameters', False):
      self._liveparams = sm['liveParameters']
    if sm.valid.get('carState', False):
      self._CS = sm['carState']
    
    if self._CS is not None:
      steering_angle = self._CS.steeringAngleDeg
    else:
      return
    
    # scale velocity used to determine curvature in order to provide more braking at low speed
    self._speed_scale_bp_v = self.get_speed_scale(int(sm['liveMapData'].currentRoadType))
      
    self._vf = interp(self._v_ego, self._speed_scale_bp_v[0], self._speed_scale_bp_v[1])

    # 1. When the probability of lanes is good enough, compute polynomial from lanes as they are way more stable
    # on current mode than driving path.
    if model_data is not None and len(model_data.laneLines) == 4 and len(model_data.laneLines[0].t) == TRAJECTORY_SIZE:
      ll_x = model_data.laneLines[1].x  # left and right ll x is the same
      lll_y = np.array(model_data.laneLines[1].y)
      rll_y = np.array(model_data.laneLines[2].y)
      l_prob = model_data.laneLineProbs[1]
      r_prob = model_data.laneLineProbs[2]
      lll_std = model_data.laneLineStds[1]
      rll_std = model_data.laneLineStds[2]

      # Reduce reliance on lanelines that are too far apart or will be in a few seconds
      width_pts = rll_y - lll_y
      prob_mods = []
      for t_check in [0.0, 1.5, 3.0]:
        width_at_t = interp(t_check * (self._v_ego + 7), ll_x, width_pts)
        prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
      mod = min(prob_mods)
      l_prob *= mod
      r_prob *= mod

      # Reduce reliance on uncertain lanelines
      l_std_mod = interp(lll_std, [.15, .3], [1.0, 0.0])
      r_std_mod = interp(rll_std, [.15, .3], [1.0, 0.0])
      l_prob *= l_std_mod
      r_prob *= r_std_mod

      # Find path from lanes as the average center lane only if min probability on both lanes is above threshold.
      if l_prob > _MIN_LANE_PROB and r_prob > _MIN_LANE_PROB:
        c_y = width_pts / 2 + lll_y
        path_poly = np.polyfit(ll_x, c_y, 3)
        self._predicted_path_source = 'lanelines'

    # 2. If not polynomial derived from lanes, then derive it from compensated driving path with lanes as
    # provided by `lateralPlanner`.
    if path_poly is None and lat_planner_data is not None and len(lat_planner_data.dPathWLinesX) > 0 \
       and lat_planner_data.dPathWLinesX[0] > 0:
      path_poly = np.polyfit(lat_planner_data.dPathWLinesX, lat_planner_data.dPathWLinesY, 3)
      self._predicted_path_source = 'pathWithLanes'
      
    
    # 3. Use path curvature otherwise,
    if path_poly is None and model_data is not None and len(model_data.position.y) >= 16: #16 based on Harald's use of predicted orientations
      path_poly = np.polyfit(np.array(model_data.position.x)[:16], np.array(model_data.position.y)[:16], 3)
      self._predicted_path_source = 'modelPosition'

    # 4. If no polynomial derived from lanes or driving path, then provide a straight line poly.
    if path_poly is None:
      path_poly = np.array([0., 0., 0., 0.])
      self._predicted_path_source = 'none'
      
    if self._liveparams is not None:
      x = max(self._liveparams.stiffnessFactor, 0.1)
      sr = max(self._liveparams.steerRatio, 0.1)
      self._VM.update_params(x, sr)
      roll_compensation = self._VM.roll_compensation(self._liveparams.roll, self._vf * self._v_ego)
      angle_offset = self._liveparams.angleOffsetDeg
    else:
      roll_compensation = 0.
      angle_offset = 0.
    
    current_curvature_no_roll = self._VM.calc_curvature(-math.radians(steering_angle - angle_offset), self._vf * self._v_ego, 0.)
    if abs(roll_compensation) > abs(current_curvature_no_roll):
      roll_compensation = abs(current_curvature_no_roll) * np.sign(roll_compensation)
    current_curvature = abs(current_curvature_no_roll + roll_compensation)
      
    self._current_lat_acc = current_curvature * (self._vf * self._v_ego)**2
    self._current_lat_acc_no_roll = abs(current_curvature_no_roll) * (self._vf * self._v_ego)**2
    
    lat_sat = False
    if sm.valid.get('controlsState', False):
      self._controls_state = sm['controlsState']
    if self._controls_state is not None and self._lat_sat_t >= 0:
      lat_type = self._controls_state.lateralControlState.which()
      if lat_type == 'indiState':
        lat_sat = (self._controls_state.lateralControlState.indiState.output >= 0.95)
      elif lat_type == 'pidState':
        lat_sat = (self._controls_state.lateralControlState.pidState.output >= 0.95)
      elif lat_type == 'lqrState':
        lat_sat = (self._controls_state.lateralControlState.lqrState.output >= 0.95)
      elif lat_type == 'angleState':
        lat_sat = (self._controls_state.lateralControlState.angleState.output >= 0.95)
      elif lat_type == 'torqueState':
        lat_sat = (self._controls_state.lateralControlState.torqueState.output >= 0.95)
      elif lat_type == 'torqueIndiState':
        lat_sat = (self._controls_state.lateralControlState.torqueIndiState.output >= 0.95)
      elif lat_type == 'torqueLqrState':
        lat_sat = (self._controls_state.lateralControlState.torqueLqrState.output >= 0.95)
      else: # unknown type
        cloudlog.info(f"Vision controller: unknown lateralControlState: {lat_type}")
        self._lat_sat_t = -1
      if lat_sat:
        if not self._lat_sat_last:
          self._lat_sat_t = sec_since_boot()
        if self._lat_sat_last and self._lat_sat_t > 0.:
          lat_sat_decel_factor = interp(sec_since_boot() - self._lat_sat_t, _LAT_SAT_DECEL_BP, _LAT_SAT_DECEL_V)
        else:
          lat_sat_decel_factor = 1.0
        self._current_lat_acc *= lat_sat_decel_factor
    
    self._lat_sat_last = lat_sat
    
    self._max_v_for_current_curvature = math.sqrt(self._A_LAT_REG_MAX / current_curvature) if current_curvature > 0 \
      else V_CRUISE_MAX * CV.KPH_TO_MS
    
    # get model-predicted relative road roll, with current roll
    if model_data is not None and self._liveparams is not None and len(model_data.position.x) >= 16:
      path_x = np.array(model_data.position.x)[:16]
      path_rolls = np.array(model_data.orientation.x)[:16] + self._liveparams.roll
      path_roll_poly = np.polyfit(path_x, path_rolls, 3)
    else:
      path_roll_poly = np.array([0., 0., 0., 0.])
      path_x = [0.]
      
    
    
    pred_curvatures, max_pred_lat_acc, max_pred_curvature, max_pred_roll_compensation, max_pred_lat_acc_dist = self.eval_curvature(path_poly, _EVAL_RANGE, path_roll_poly, path_x[-1])

    self._max_pred_lat_acc = self._ema_k * max_pred_lat_acc + (1. - self._ema_k) * self._max_pred_lat_acc
    self._max_pred_curvature = self._ema_k * max_pred_curvature + (1. - self._ema_k) * self._max_pred_curvature
    self._max_pred_roll_compensation = self._ema_k * max_pred_roll_compensation + (1. - self._ema_k) * self._max_pred_roll_compensation
    self._max_pred_lat_acc_dist = self._ema_k * max_pred_lat_acc_dist + (1. - self._ema_k) * self._max_pred_lat_acc_dist
    if self._pred_curvatures.shape != pred_curvatures.shape:
      self._pred_curvatures = pred_curvatures
    else:
      self._pred_curvatures = pred_curvatures * self._ema_k + self._pred_curvatures * (1. - self._ema_k) 

    max_curvature_for_vego = self._A_LAT_REG_MAX / max(self._vf * self._v_ego, 0.1)**2
    lat_acc_overshoot_idxs = np.nonzero(pred_curvatures >= max_curvature_for_vego)[0]
    self._lat_acc_overshoot_ahead = len(lat_acc_overshoot_idxs) > 0

    if self._lat_acc_overshoot_ahead:
      self._v_overshoot = min(math.sqrt(self._A_LAT_REG_MAX / abs(self._max_pred_curvature)), self._v_cruise_setpoint)
      self._v_overshoot_distance = max(lat_acc_overshoot_idxs[0] * _EVAL_STEP + _EVAL_START, _EVAL_STEP)
      _debug(f'TVC: High LatAcc. Dist: {self._v_overshoot_distance:.2f}, v: {self._v_overshoot * CV.MS_TO_KPH:.2f}')

  def _state_transition(self):
    # In any case, if system is disabled or the feature is disabeld or gas is pressed, disable.
    if not self._op_enabled or not self._is_enabled or self._gas_pressed:
      self.state = VisionTurnControllerState.disabled
      return

    # DISABLED
    if self.state == VisionTurnControllerState.disabled:
      # Do not enter a turn control cycle if speed is low.
      if self._v_ego <= _MIN_V:
        pass
      # If substantial lateral acceleration is predicted ahead, then move to Entering turn state.
      elif self._max_pred_lat_acc >= self._ENTERING_PRED_LAT_ACC_TH:
        self.state = VisionTurnControllerState.entering
      elif self._current_lat_acc >= self._TURNING_LAT_ACC_TH:
        self.state = VisionTurnControllerState.turning
    # ENTERING
    elif self.state == VisionTurnControllerState.entering:
      # Transition to Turning if current lateral acceleration is over the threshold.
      if self._current_lat_acc >= self._TURNING_LAT_ACC_TH:
        self.state = VisionTurnControllerState.turning
      # Abort if the predicted lateral acceleration drops
      elif self._max_pred_lat_acc < self._ABORT_ENTERING_PRED_LAT_ACC_TH:
        self.state = VisionTurnControllerState.disabled
    # TURNING
    elif self.state == VisionTurnControllerState.turning:
      # Transition to Leaving if current lateral acceleration drops drops below threshold.
      if self._current_lat_acc <= self._LEAVING_LAT_ACC_TH:
        self.state = VisionTurnControllerState.leaving
    # LEAVING
    elif self.state == VisionTurnControllerState.leaving:
      # Transition back to Turning if current lateral acceleration goes back over the threshold.
      if self._current_lat_acc >= self._TURNING_LAT_ACC_TH:
        self.state = VisionTurnControllerState.turning
      # Finish if current lateral acceleration goes below threshold.
      elif self._current_lat_acc < self._FINISH_LAT_ACC_TH:
        self.state = VisionTurnControllerState.disabled

  def _update_solution(self):
    # DISABLED
    if self.state == VisionTurnControllerState.disabled:
      # when not overshooting, calculate v_turn as the speed at the prediction horizon when following
        # the smooth deceleration.
      a_target = self._a_ego
    # ENTERING
    elif self.state == VisionTurnControllerState.entering:
      # when not overshooting, target a smooth deceleration in preparation for a sharp turn to come.
      a_target = interp(self._max_pred_lat_acc, self._ENTERING_SMOOTH_DECEL_BP, self._ENTERING_SMOOTH_DECEL_V)
      if self._lat_acc_overshoot_ahead:
        # when overshooting, target the acceleration needed to achieve the overshoot speed at
        # the required distance
        a_target_overshoot = min((self._v_overshoot**2 - (self._vf * self._v_ego)**2) / (2 * self._v_overshoot_distance), a_target)
        if self.state == VisionTurnControllerState.entering:
          a_target = a_target_overshoot
        _debug(f'TVC Entering: Overshooting: {self._lat_acc_overshoot_ahead}')
        _debug(f'    Decel: {a_target:.2f}, target v: {self.v_turn * CV.MS_TO_KPH}')
      else:
        a_target_overshoot = 3.0 #big value
    # TURNING
    elif self.state == VisionTurnControllerState.turning:
      # When turning we provide a target acceleration that is confortable for the lateral accelearation felt.
      a_target = interp(self._current_lat_acc, self._TURNING_ACC_BP, self._TURNING_ACC_V)
    # LEAVING
    elif self.state == VisionTurnControllerState.leaving:
      # When leaving we provide a confortable acceleration to regain speed.
      a_target = self._LEAVING_ACC

    if self._soften_decel:
      a_target = max(a_target, 0.4)
    # update solution values.
    self._a_target.update(a_target)

  def update(self, enabled, v_ego, a_ego, v_cruise_setpoint, sm):
    self._op_enabled = enabled
    self._gas_pressed = sm['carState'].gas > 1e-5
    self._v_ego = v_ego
    self._a_ego = a_ego
    self._v_cruise_setpoint = v_cruise_setpoint
    self._soften_decel = sm['carState'].lowVisibilityActive or sm['carState'].slipperyRoadsActive

    self._update_params()
    self._update_calculations(sm)
    self._state_transition()
    self._update_solution()
