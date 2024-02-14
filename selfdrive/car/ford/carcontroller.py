from cereal import car
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.ford import fordcan
from openpilot.selfdrive.car.ford.values import CANFD_CAR, CarControllerParams
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from openpilot.selfdrive.modeld.constants import ModelConstants

LongCtrlState = car.CarControl.Actuators.LongControlState
VisualAlert = car.CarControl.HUDControl.VisualAlert


def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                           current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, CarControllerParams)

  return clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)

# For calculating other lat control inputs
LAT_PLAN_MIN_IDX = 5

def get_time_derivatives(values, t_diffs):
  # compute finite difference time-derivatives between subsequent values
  value_diffs = np.diff(values)
  rate_values = value_diffs / t_diffs
  return rate_values

def sign(x):
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)

def get_lookahead_value(future_vals, current_val):
  if len(future_vals) == 0:
    return current_val

  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]

  # if any future val has opposite sign of current val, return 0
  if len(same_sign_vals) < len(future_vals):
    return 0.0

  # otherwise return the value with minimum absolute value
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.CAN = fordcan.CanBus(CP)
    self.frame = 0

    self.apply_curvature_last = 0
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    
    # for computing other lat control inputs
    self.look_ahead_v = [0.8, 1.8] # how many seconds in the future to look ahead in [0, ~2.1] in 0.1 increments
    self.look_ahead_bp = [9.0, 35.0] # corresponding speeds in m/s in [0, ~40] in 1.0 increments
    # precompute time differences between ModelConstants.T_IDXS
    self.t_diffs = np.diff(ModelConstants.T_IDXS)
    self.desired_curvature_rate_scale = -0.003 # determined in plotjuggler to best match with `LatCtlCrv_NoRate2_Actl`
    self.future_lookup_time_diff = 0.3
    self.future_lookup_time = (CP.steerActuatorDelay, CP.steerActuatorDelay + self.future_lookup_time_diff)
    self.dist_from_lane_center_scale = -1.0
    self.dist_from_lane_center_rate_scale = -0.07 # determined in plotjuggler to best match with `LatCtlPath_An_Actl`
    # scale down dist_from_lane_center_rate input when under high curvature (current or predicted)
    self.max_curvature_for_dist_from_lane_center_rate_bp = [0.003, 0.005]
    self.model_orientation_rate_z_scale = 0.04 # determined in plotjuggler to best match with `controlsState/desiredCurvature`
    # scale down both dist_from_lane_center and _rate when lanelines are unclear
    self.min_laneline_confidence_bp = [0.4, 0.6]


  def update(self, CC, CS, now_nanos, model_data=None):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        # apply rate limits, curvature error limit, and clip to signal range
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
        apply_curvature = apply_ford_curvature_limits(actuators.curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw)
      else:
        apply_curvature = 0.

      self.apply_curvature_last = apply_curvature
      
      if model_data is not None and len(model_data.orientation.x) >= CONTROL_N:
        # "lookahead" is used to check whether current curvature rate is "deliberate" i.e. will persist over a long enough time.
        # Don't want to send short-lived curvature rate-based commands.
        lookahead = interp(CS.out.vEgo, self.look_ahead_bp, self.look_ahead_v)
        lookahead_upper_idx = next((i for i, val in enumerate(ModelConstants.T_IDXS) if val > lookahead), 16)
        
        # Computing three values: desired curvature rate, distance from lane center, and distance from lane center rate
        # First get desired curvature rate
        predicted_curvature_rate = get_time_derivatives(model_data.orientationRate.z, self.t_diffs)
        desired_curvature_rate = (interp(self.future_lookup_time[1], ModelConstants.T_IDXS, model_data.orientationRate.z) \
          - interp(self.future_lookup_time[0], ModelConstants.T_IDXS, model_data.orientationRate.z)) / self.future_lookup_time_diff
        # Lookahead curvature rate is what will be sent to the car
        lookahead_curvature_rate = get_lookahead_value(predicted_curvature_rate[LAT_PLAN_MIN_IDX:lookahead_upper_idx], desired_curvature_rate)
        
        # Now get distance from lane center
        dist_from_lane_center_full = (np.array(model_data.laneLines[1].y) + np.array(model_data.laneLines[2].y)) / 2
        dist_from_lane_center = interp(self.future_lookup_time[0], ModelConstants.T_IDXS, dist_from_lane_center_full)
        
        # Now get distance from lane center rate
        dist_from_lane_center_rate = (interp(self.future_lookup_time[1], ModelConstants.T_IDXS, dist_from_lane_center_full) \
          - interp(self.future_lookup_time[0], ModelConstants.T_IDXS, dist_from_lane_center_full)) / self.future_lookup_time_diff
        # Downscale dist_from_lane_center_rate when under or approaching high curvature
        max_abs_desired_curvature = max([abs(i) * self.model_orientation_rate_z_scale for i in list(model_data.orientationRate.z)[0:lookahead_upper_idx]]+[abs(actuators.curvature)])
        dist_from_lane_center_rate *= interp(max_abs_desired_curvature, self.max_curvature_for_dist_from_lane_center_rate_bp, [1.0, 0.0])
        
        # Downscale both dist_from_lane_center and _rate when lanelines are unclear
        laneline_confidence = (model_data.laneLineProbs[1] + model_data.laneLineProbs[2]) / 2
        laneline_confidence_scale = interp(laneline_confidence, self.min_laneline_confidence_bp, [0.0, 1.0])
        dist_from_lane_center *= laneline_confidence_scale
        dist_from_lane_center_rate *= laneline_confidence_scale
      else:
        lookahead_curvature_rate = 0.0
        dist_from_lane_center = 0.0
        dist_from_lane_center_rate = 0.0


      if self.CP.carFingerprint in CANFD_CAR:
        # TODO: extended mode
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0xF
        can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 
                                                     dist_from_lane_center * self.dist_from_lane_center_scale, 
                                                     dist_from_lane_center_rate * self.dist_from_lane_center_rate_scale, 
                                                     -apply_curvature, 
                                                     lookahead_curvature_rate * self.desired_curvature_rate_scale, 
                                                     counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 
                                                    dist_from_lane_center * self.dist_from_lane_center_scale, 
                                                    dist_from_lane_center_rate * self.dist_from_lane_center_rate_scale, 
                                                    -apply_curvature, 
                                                    lookahead_curvature_rate * self.desired_curvature_rate_scale))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # Both gas and accel are in m/s^2, accel is used solely for braking
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      gas = accel
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, v_ego_kph=V_CRUISE_MAX))

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))
    # send acc ui msg at 5Hz or if ui state changes
    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                         fcw_alert, CS.out.cruiseState.standstill, hud_control,
                                         CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert

    new_actuators = actuators.copy()
    new_actuators.curvature = self.apply_curvature_last

    self.frame += 1
    return new_actuators, can_sends
