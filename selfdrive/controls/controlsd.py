#!/usr/bin/env python3
import os
import math
from numbers import Number

from cereal import car, log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import clip, interp, mean
from common.op_params import opParams
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV
from selfdrive.swaglog import cloudlog
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise, V_CRUISE_MIN
from selfdrive.controls.lib.drive_helpers import get_lag_adjusted_curvature
from selfdrive.controls.lib.drive_helpers import apply_deadzone
from selfdrive.controls.lib.longcontrol import LongControl, STARTING_TARGET_SPEED
from selfdrive.controls.lib.lane_planner import TRAJECTORY_SIZE
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from selfdrive.controls.lib.latcontrol_torque_indi import LatControlTorqueINDI
from selfdrive.controls.lib.latcontrol_torque_lqr import LatControlTorqueLQR
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.locationd.calibrationd import Calibration
from selfdrive.modeld.constants import T_IDXS
from selfdrive.hardware import HARDWARE, TICI, EON
from selfdrive.manager.process_config import managed_processes

LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
STEER_ANGLE_SATURATION_TIMEOUT = 1.0 / DT_CTRL
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

MAX_ABS_PITCH = 0.314 # 20% grade = 18 degrees = pi/10 radians
MAX_ABS_PRED_PITCH_DELTA = MAX_ABS_PITCH * 0.5 * DT_CTRL # 10% grade per second

SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
IGNORE_PROCESSES = {"rtshield", "uploader", "deleter", "loggerd", "logmessaged", "tombstoned", "gpsd",
                    "logcatd", "proclogd", "clocksd", "updated", "timezoned", "manage_athenad"} | \
                    {k for k, v in managed_processes.items() if not v.enabled}

ACTUATOR_FIELDS = set(car.CarControl.Actuators.schema.fields.keys())

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.LateralPlan.Desire
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName

SpeedLimitControlState = log.LongitudinalPlan.SpeedLimitControlState

class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None):
    config_realtime_process(4 if TICI else 3, Priority.CTRL_HIGH)

    self.accel_pressed = False
    self.decel_pressed = False
    self.accel_pressed_last = 0.
    self.decel_pressed_last = 0.
    self.fastMode = False
    self.lk_mode_last = False
    self.oplongcontrol_last = False
    self.network_strength_last = log.DeviceState.NetworkStrength.unknown
    self.network_last_change_t = -60
    
    self._op_params = opParams("controlsd", check_for_reset=True)
    self.use_sensors = False
    
    self.gpsWasOK = False

    # Setup sockets
    self.pm = pm
    if self.pm is None:
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])

    self.camera_packets = ["roadCameraState", "driverCameraState"]
    if TICI:
      self.camera_packets.append("wideRoadCameraState")

    params = Params()
    self.joystick_mode = params.get_bool("JoystickDebugMode")
    joystick_packet = ['testJoystick'] if self.joystick_mode else []
    
    self.gray_panda_support_enabled = params.get_bool("GrayPandaSupport")

    self.sm = sm
    if self.sm is None:
      ignore = ['driverCameraState', 'managerState'] if SIMULATION else ['liveWeatherData','carState','controlsState']
      if self.gray_panda_support_enabled:
        ignore += ['gpsLocationExternal']
      self.sm = messaging.SubMaster(['deviceState', 'pandaState', 'modelV2', 'liveCalibration',
                                     'driverMonitoringState', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman',
                                     'managerState', 'liveParameters', 'radarState', 'gpsLocationExternal', 'liveWeatherData',
                                     'carState','controlsState'] + self.camera_packets + joystick_packet,
                                     ignore_alive=ignore, ignore_avg_freq=['radarState', 'longitudinalPlan', 'gpsLocationExternal', 'liveWeatherData','controlsState'])

    self.can_sock = can_sock
    if can_sock is None:
      can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

    if TICI:
      self.log_sock = messaging.sub_sock('androidLog')

    # wait for one pandaState and one CAN packet
    print("Waiting for CAN messages...")
    get_one_can(self.can_sock)
    
    self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'])

    # read params
    self.is_metric = params.get_bool("IsMetric")
    self.is_ldw_enabled = params.get_bool("IsLdwEnabled")
    community_feature_toggle = True
    self.openpilot_enabled_toggle = params.get_bool("OpenpilotEnabledToggle")
    passive = params.get_bool("Passive") or not self.openpilot_enabled_toggle

    # detect sound card presence and ensure successful init
    sounds_available = HARDWARE.get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'

    controller_available = self.CI.CC is not None and not passive and not self.CP.dashcamOnly
    community_feature = self.CP.communityFeature or \
                        self.CP.fingerprintSource == car.CarParams.FingerprintSource.can
    community_feature_disallowed = community_feature and (not community_feature_toggle)
    self.read_only = not car_recognized or not controller_available or \
                       self.CP.dashcamOnly or community_feature_disallowed
    if self.read_only:
      self.CP.safetyModel = car.CarParams.SafetyModel.noOutput

    # Write CarParams for radard
    cp_bytes = self.CP.to_bytes()
    params.put("CarParams", cp_bytes)
    put_nonblocking("CarParamsCache", cp_bytes)

    self.CC = car.CarControl.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)

    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'indi':
      self.LaC = LatControlINDI(self.CP)
    elif self.CP.lateralTuning.which() == 'lqr':
      self.LaC = LatControlLQR(self.CP)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torqueIndi':
      self.LaC = LatControlTorqueINDI(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torqueLqr':
      self.LaC = LatControlTorqueLQR(self.CP, self.CI)

    self.initialized = False
    self.state = State.disabled
    self.enabled = False
    self.enabled_last = False
    self.active = False
    self.lat_active = False
    self.can_rcv_error = False
    self.soft_disable_timer = 0
    self.v_cruise_kph = 255
    self.v_cruise_kph_last = 0
    self.v_cruise_last_changed = 0.
    self.stock_speed_adjust = not params.get_bool("ReverseSpeedAdjust")
    self.MADS_enabled = Params().get_bool("MADSEnabled")
    self.MADS_lead_braking_enabled = self.MADS_enabled and params.get_bool("MADSLeadBraking")
    self.accel_modes_enabled = params.get_bool("AccelModeButton")
    self.mismatch_counter = 0
    self.can_error_counter = 0
    self.last_blinker_frame = 0
    self.saturated_count = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]
    self.logged_comm_issue = False
    self.v_target = 0.0
    self.a_target = 0.0
    self.pitch = 0.0
    self.pitch_accel_deadzone = 0.01 # [radians] ≈ 1% grade
    self.k_mean = FirstOrderFilter(0., 20, DT_CTRL)
    
    self.slippery_roads_activated = False
    self.slippery_roads = False
    self.low_visibility = False
    self.low_visibility_activated = False
    self.weather_safety_enabled = params.get_bool("WeatherSafetyEnabled")
    self.weather_valid = False
    
    self.reset_metrics = params.get_bool("MetricResetSwitch")
    if self.reset_metrics:
      params.put("TripDistance", "0.0")
      params.put("NumberOfDisengagements", "0")
      params.put("NumberOfInterventions", "0")
      params.put("NumberOfInteractions", "0")
      params.put("NumberOfDistractions", "0")
      params.put("CarSecondsRunning", "0.0")
      params.put("EngagedDistance", "0.0")
      params.put("OpenPilotSecondsEngaged", "0.0")
    
    self.parked_timer = 0.0
    self.distance_traveled_total = float(params.get("TripDistance", encoding="utf8"))
    self.car_running_timer_total = float(params.get("CarSecondsRunning", encoding="utf8"))
    self.car_running_timer_session = 0.0
    self.openpilot_long_control_timer_total = float(params.get("OpenPilotSecondsEngaged", encoding="utf8"))
    self.openpilot_long_control_timer_session = 0.0
    self.disengagement_timer = 0.0
    self.interaction_timer = 0.0 # [s] time since any interaction
    self.intervention_timer = 0.0 # [s] time since screen steering/gas/brake interaction
    self.distraction_timer = 0.0 # [s] time since driver distracted
    self.disengagement_count_total = int(params.get("NumberOfDisengagements", encoding="utf8"))
    self.disengagement_count_session = 0
    self.interaction_count_total = int(params.get("NumberOfInteractions", encoding="utf8"))
    self.interaction_count_session = 0
    self.intervention_count_total = int(params.get("NumberOfInterventions", encoding="utf8"))
    self.intervention_count_session = 0
    self.distraction_count_total = int(params.get("NumberOfDistractions", encoding="utf8"))
    self.distraction_count_session = 0
    self.distance_last = 0.0
    self.engaged_dist = 0.0
    self.engaged_dist_session = 0.0
    self.engaged_dist_total = float(params.get("EngagedDistance", encoding="utf8"))
    self.interaction_dist = 0.0
    self.intervention_dist = 0.0
    self.distraction_dist = 0.0
    self.interaction_last_t = sec_since_boot()
    self.intervention_last_t = sec_since_boot()
    self.distraction_last_t = sec_since_boot()
    self.params_check_last_t = 0.0
    self.params_check_freq = 0.3
    self._params = params
    self.params_write_freq = 30.0
    self.params_write_last_t = sec_since_boot()
    self.op_params_override_lateral = self._params.get_bool('OPParamsLateralOverride')
    self.op_params_override_long = self._params.get_bool('OPParamsLongitudinalOverride')

    # TODO: no longer necessary, aside from process replay
    self.sm['liveParameters'].valid = True

    self.startup_event = get_startup_event(car_recognized, controller_available, self.CP.fuzzyFingerprint,
                                           len(self.CP.carFw) > 0)

    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
    if community_feature_disallowed and car_recognized and not self.CP.dashcamOnly:
      self.events.add(EventName.communityFeatureDisallowed, static=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
    elif self.read_only:
      self.events.add(EventName.dashcamMode, static=True)
    elif self.joystick_mode:
      self.events.add(EventName.joystickDebug, static=True)
      self.startup_event = None

    # controlsd is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.prof = Profiler(False)  # off by default

  def update_events(self, CS):
    """Compute carEvents from carState"""

    self.events.clear()
    self.events.add_from_msg(CS.events)
    self.events.add_from_msg(self.sm['driverMonitoringState'].events)
    self.events.add_from_msg(self.sm['longitudinalPlan'].eventsDEPRECATED)

    # Handle startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Don't add any more events if not initialized
    if not self.initialized:
      self.events.add(EventName.controlsInitializing)
      return
    
    # Alert when network drops, but only if map braking or speed limit control is enabled
    t = sec_since_boot()
    
    if t - self.params_check_last_t > self.params_check_freq:
      if self.op_params_override_lateral:
        self.LaC.update_op_params()
      if self.op_params_override_long:
        self.LoC.update_op_params()
      screen_tapped = self._params.get_bool("ScreenTapped")
      if screen_tapped:
        self.CI.screen_tapped = True
        put_nonblocking("ScreenTapped", "0")
      self.distance_last = CS.vEgo * (t - self.params_check_last_t)
      self.distance_traveled_total += self.distance_last
      if CS.gearShifter in ['drive', 'low', 'reverse']:
        self.car_running_timer_session += self.params_check_freq
        self.car_running_timer_total += self.params_check_freq
      if CS.gearShifter == 'park':
        self.parked_timer += self.params_check_freq
      else:
        self.parked_timer = 0.0
      self.CI.CS.parked_timer = self.parked_timer
      if not self.enabled and self.enabled_last and CS.vEgo > 0.5:
        self.disengagement_count_session += 1
        self.disengagement_count_total += 1
        self.disengagement_last_t = t
      elif self.enabled and self.enabled_last:
        self.openpilot_long_control_timer_session += self.params_check_freq
        self.openpilot_long_control_timer_total += self.params_check_freq
        self.disengagement_timer += self.params_check_freq
        self.engaged_dist += self.distance_last
        self.engaged_dist_session += self.distance_last
        self.engaged_dist_total += self.distance_last
        self.interaction_dist += self.distance_last
        self.intervention_dist += self.distance_last
      if not self.enabled:
        self.interaction_last_t = t
        self.intervention_last_t = t
        self.disengagement_timer = 0.0
        self.engaged_dist = 0.0
      if CS.gearShifter not in ['drive','low']:
        self.interaction_last_t = t
        self.intervention_last_t = t
        self.distraction_last_t = t
      else:
        car_interaction = CS.brakePressed or CS.gas > 1e-5 or CS.steeringPressed
        if screen_tapped or car_interaction or self.CI.driver_interacted:
          if self.interaction_timer > 2.0:
            self.interaction_count_session += 1
            self.interaction_count_total += 1
          self.interaction_last_t = t
          self.interaction_dist = 0.0
          self.CI.driver_interacted = False
        if car_interaction:
          if self.intervention_timer > 2.0:
            self.intervention_count_session += 1
            self.intervention_count_total += 1
          self.intervention_last_t = t
          self.intervention_dist = 0.0
        if not car_interaction and self.sm['driverMonitoringState'].isDistracted or CS.vEgo < 0.1:
          if CS.vEgo > 0.1 and self.distraction_timer > 2.0:
            self.distraction_count_session += 1
            self.distraction_count_total += 1
          self.distraction_last_t = t
          self.distraction_dist = 0.0
        else:
          self.distraction_dist += self.distance_last
      self.interaction_timer = t - self.interaction_last_t
      self.intervention_timer = t - self.intervention_last_t
      self.distraction_timer = t - self.distraction_last_t
      self.MADS_lead_braking_enabled = self.MADS_enabled and self._params.get_bool("MADSLeadBraking")
      self.enabled_last = self.enabled
      self.params_check_last_t = t
      if t - self.params_write_last_t > self.params_write_freq:
        put_nonblocking("NumberOfDisengagements", str(self.disengagement_count_total))
        put_nonblocking("NumberOfInteractions", str(self.interaction_count_total))
        put_nonblocking("NumberOfInterventions", str(self.intervention_count_total))
        put_nonblocking("NumberOfDistractions", str(self.distraction_count_total))
        put_nonblocking("CarSecondsRunning", str(self.car_running_timer_total))
        put_nonblocking("OpenPilotSecondsEngaged", str(self.openpilot_long_control_timer_total))
        put_nonblocking("TripDistance", str(self.distance_traveled_total))
        put_nonblocking("EngagedDistance", str(self.engaged_dist_total))
        if self.reset_metrics:
          self.reset_metrics = False
          put_nonblocking("MetricResetSwitch", "0")
        self.params_write_last_t = t
    
    network_strength = self.sm['deviceState'].networkStrength
    if network_strength != self.network_strength_last:
      if network_strength == log.DeviceState.NetworkStrength.unknown:
        self.network_last_change_t = t
      elif self.network_strength_last == log.DeviceState.NetworkStrength.unknown:
        self.network_last_change_t = t
        if t - self.network_last_change_t > 60 and t - self.network_last_change_t < 61:
          self.events.add(EventName.signalRestored)
    self.network_strength_last = network_strength
    
    if network_strength == log.DeviceState.NetworkStrength.unknown \
      and t - self.network_last_change_t > 60 and t - self.network_last_change_t < 61 \
      and (self.sm['longitudinalPlan'].speedLimitControlState != SpeedLimitControlState.inactive \
        or self.sm['longitudinalPlan'].turnSpeedControlState != SpeedLimitControlState.inactive \
          ):
      self.events.add(EventName.signalLost)
    
    if self.slippery_roads_activated:
      self.slippery_roads_activated = False
      self.events.add(EventName.slipperyRoadsActivated)
    if self.low_visibility_activated:
      self.low_visibility_activated = False
      self.events.add(EventName.lowVisibilityActivated)
      
    if self._op_params.get("op_edit_param_changed"):
      self._op_params.put("op_edit_param_changed", False, do_log=False)
      self.events.add(EventName.opParamsParamChanged)

    # Create events for battery, temperature, disk space, and memory
    if EON and self.sm['deviceState'].batteryPercent < 1 and self.sm['deviceState'].chargingError:
      # at zero percent battery, while discharging, OP should not allowed
      self.events.add(EventName.lowBattery)
    if self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
    # TODO: make tici threshold the same
    if self.sm['deviceState'].memoryUsagePercent > (90 if TICI else 65) and not SIMULATION:
      self.events.add(EventName.lowMemory)
    cpus = list(self.sm['deviceState'].cpuUsagePercent)[:(-1 if EON else None)]
    if max(cpus, default=0) > 95 and not SIMULATION:
      self.events.add(EventName.highCpuUsage)

    # Alert if fan isn't spinning for 5 seconds
    if self.sm['pandaState'].pandaType in [PandaType.uno, PandaType.dos]:
      if self.sm['pandaState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
        # allow enough time for the fan controller in the panda to recover from stalls
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = self.sm.frame

    # Handle calibration status
    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != Calibration.CALIBRATED:
      if cal_status == Calibration.UNCALIBRATED:
        self.events.add(EventName.calibrationIncomplete)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Handle lane change
    if self.sm['lateralPlan'].laneChangeState == LaneChangeState.preLaneChange:
      direction = self.sm['lateralPlan'].laneChangeDirection
      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)
      else:
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
        else:
          self.events.add(EventName.preLaneChangeRight)
    elif self.sm['lateralPlan'].laneChangeState in [LaneChangeState.laneChangeStarting,
                                                 LaneChangeState.laneChangeFinishing]:
      self.events.add(EventName.laneChange)

    if self.can_rcv_error or not CS.canValid:
      self.events.add(EventName.canError)

    safety_mismatch = self.sm['pandaState'].safetyModel != self.CP.safetyModel or self.sm['pandaState'].safetyParam != self.CP.safetyParam
    if safety_mismatch or self.mismatch_counter >= 200:
      self.events.add(EventName.controlsMismatch)

    if not self.sm['liveParameters'].valid:
      self.events.add(EventName.vehicleModelInvalid)

    if len(self.sm['radarState'].radarErrors):
      self.events.add(EventName.radarFault)
    elif not self.sm.valid["pandaState"]:
      self.events.add(EventName.usbError)
    elif not self.sm.all_alive_and_valid():
      self.events.add(EventName.commIssue)
      if not self.logged_comm_issue:
        invalid = [s for s, valid in self.sm.valid.items() if not valid]
        not_alive = [s for s, alive in self.sm.alive.items() if not alive]
        cloudlog.event("commIssue", invalid=invalid, not_alive=not_alive)
        self.logged_comm_issue = True
    else:
      self.logged_comm_issue = False

    if not self.sm['lateralPlan'].mpcSolutionValid:
      self.events.add(EventName.plannerError)
    if not self.sm['liveLocationKalman'].sensorsOK and not NOSENSOR:
      if self.sm.frame > 5 / DT_CTRL:  # Give locationd some time to receive all the inputs
        self.events.add(EventName.sensorDataInvalid)
    if not self.sm['liveLocationKalman'].posenetOK:
      self.events.add(EventName.posenetInvalid)
    if not self.sm['liveLocationKalman'].deviceStable:
      self.events.add(EventName.deviceFalling)
    if log.PandaState.FaultType.relayMalfunction in self.sm['pandaState'].faults:
      self.events.add(EventName.relayMalfunction)
    if self.sm['longitudinalPlan'].fcw or (self.enabled and self.sm['modelV2'].meta.hardBrakePredicted):
      self.events.add(EventName.fcw)

    if TICI:
      logs = messaging.drain_sock(self.log_sock, wait_for_one=False)
      messages = []
      for m in logs:
        try:
          messages.append(m.androidLog.message)
        except UnicodeDecodeError:
          pass

      for err in ["ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED"]:
        for m in messages:
          if err not in m:
            continue

          csid = m.split("CSID:")[-1].split(" ")[0]
          evt = {"0": EventName.roadCameraError, "1": EventName.wideRoadCameraError,
                 "2": EventName.driverCameraError}.get(csid, None)
          if evt is not None:
            self.events.add(evt)

    # TODO: fix simulator
    if not SIMULATION:
      if not NOSENSOR:
        self.gpsWasOK = self.gpsWasOK or self.sm['liveLocationKalman'].gpsOK
        if self.gpsWasOK and not self.sm['liveLocationKalman'].gpsOK and (self.distance_traveled > 1000):
          # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
          self.events.add(EventName.noGps)
      if not self.sm.all_alive(self.camera_packets):
        self.events.add(EventName.cameraMalfunction)
      if self.sm['modelV2'].frameDropPerc > 30:
        self.events.add(EventName.modeldLagging)
      if self.sm['liveLocationKalman'].excessiveResets:
        self.events.add(EventName.localizerMalfunction)

      # Check if all manager processes are running
      not_running = set(p.name for p in self.sm['managerState'].processes if not p.running)
      if self.sm.rcv_frame['managerState'] and (not_running - IGNORE_PROCESSES):
        self.events.add(EventName.processNotRunning)

    # Only allow engagement with brake pressed when stopped behind another stopped car
    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds) > 1:
      v_future = speeds[-1]
    else:
      v_future = 100.0
    if CS.brakePressed and v_future >= STARTING_TARGET_SPEED \
      and self.CP.openpilotLongitudinalControl and CS.vEgo < 0.3:
      self.events.add(EventName.noTarget)

  def data_sample(self):
    """Receive data from sockets and update carState"""

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS = self.CI.update(self.CC, can_strs)

    self.sm.update(0)

    all_valid = CS.canValid and self.sm.all_alive_and_valid()
    if not self.initialized and (all_valid or self.sm.frame * DT_CTRL > 3.5 or SIMULATION):
      self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])
      self.initialized = True
      Params().put_bool("ControlsReady", True)

    # Check for CAN timeout
    if not can_strs:
      self.can_error_counter += 1
      self.can_rcv_error = True
    else:
      self.can_rcv_error = False

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    if not self.sm['pandaState'].controlsAllowed and self.enabled:
      self.mismatch_counter += 1

    self.distance_traveled += CS.vEgo * DT_CTRL

    return CS

  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    self.v_cruise_kph_last = self.v_cruise_kph
    
    cur_time = sec_since_boot()
    
    self.use_sensors = cur_time > self._op_params.get("TUNE_sensor_lockout_time_s")

    # if stock cruise is completely disabled, then we can use our own set speed logic
    if not self.CP.pcmCruise:
      for b in CS.buttonEvents:
        if b.pressed:
          if b.type == car.CarState.ButtonEvent.Type.accelCruise:
            self.accel_pressed = True
            self.accel_pressed_last = cur_time
          elif b.type == car.CarState.ButtonEvent.Type.decelCruise:
            self.decel_pressed = True
            self.decel_pressed_last = cur_time
        else:
          if b.type == car.CarState.ButtonEvent.Type.accelCruise:
            self.accel_pressed = False
          elif b.type == car.CarState.ButtonEvent.Type.decelCruise:
            self.decel_pressed = False
            
      v_cruise = self.v_cruise_kph if self.is_metric else int(round((float(self.v_cruise_kph) * 0.6233 + 0.0995)))
      vEgo = getattr(CS, "vEgo", None)
      vEgo = int(round((float(vEgo) * 3.6 if self.is_metric else int(round((float(vEgo) * 3.6 * 0.6233 + 0.0995)))))) if vEgo else v_cruise
      
      if self.weather_safety_enabled and self.sm.updated['liveWeatherData']:
        if self.sm['liveWeatherData'].valid:
          precip_1h = self.sm['liveWeatherData'].rain1Hour * 4 + self.sm['liveWeatherData'].snow1Hour
          precip_3h = self.sm['liveWeatherData'].rain3Hour * 4 + self.sm['liveWeatherData'].snow3Hour
          precip = max(precip_1h * 1.5, precip_3h)
          slippery_roads = self.sm['liveWeatherData'].temperature < -1.0 \
            and precip > 15
          low_visibility = self.sm['liveWeatherData'].visibility <= 1500 \
            or precip > 10
          if slippery_roads and (not self.weather_valid or not self.slippery_roads):
            self.CI.CS.slippery_roads_active = True
            self.slippery_roads_activated = True
            self.CI.CS.slippery_roads_activated_t = cur_time
            cloudlog.info(f"Weather safety: Activating slippery road mode for {precip}mm @ {self.sm['liveWeatherData'].temperature}C")
          elif low_visibility and (not self.weather_valid or not self.low_visibility):
            self.CI.CS.low_visibility_active = True
            self.CI.CS.low_visibility_activated_t = cur_time
            self.low_visibility_activated = True
            cloudlog.info(f"Weather safety: Activating low visibility mode. Visibility {self.sm['liveWeatherData'].visibility}m, precipitation {precip}mm")
          
          self.slippery_roads = slippery_roads
          self.low_visibility = low_visibility
        self.weather_valid = self.sm['liveWeatherData'].valid
      
      if not self.slippery_roads and self.CI.CS.slippery_roads_active:
        self.CI.CS.slippery_roads_active = False
        if not self.accel_modes_enabled:
          self.CI.CS.accel_mode = 1
          put_nonblocking("AccelMode", str(int(self.CI.CS.accel_mode)))
      if not self.low_visibility and self.CI.CS.low_visibility_active:
        self.CI.CS.low_visibility_active = False
      if self.sm.updated['gpsLocationExternal']:
        self.CI.CS.altitude = self.sm['gpsLocationExternal'].altitude
      if self.sm.updated['lateralPlan'] and len(self.sm['lateralPlan'].curvatures) > 0:
        k_mean = mean(self.sm['lateralPlan'].curvatures)
        self.k_mean.update(k_mean)
        self.CI.CC.params.future_curvature = self.k_mean.x
      
      self.CI.CS.speed_limit_active = (self.sm['longitudinalPlan'].speedLimitControlState == log.LongitudinalPlan.SpeedLimitControlState.active)
      if self.CI.CS.speed_limit_active:
        self.CI.CS.speed_limit = (self.sm['longitudinalPlan'].speedLimit + self.sm['longitudinalPlan'].speedLimitOffset) * 3.6 # convert to kph
      
      self.v_cruise_kph = update_v_cruise(v_cruise, CS.buttonEvents, self.enabled and CS.cruiseState.enabled, cur_time, self.accel_pressed,self.decel_pressed, self.accel_pressed_last, self.decel_pressed_last, self.fastMode, self.stock_speed_adjust, vEgo, CS.gas > 1e-5)
    
      self.v_cruise_kph = self.v_cruise_kph if self.is_metric else int(round((float(round(self.v_cruise_kph))-0.0995)/0.6233))
      
      if self.v_cruise_kph != self.v_cruise_kph_last:
        self.v_cruise_last_changed = cur_time

      if(self.accel_pressed or self.decel_pressed):
        if self.v_cruise_kph_last != self.v_cruise_kph:
          self.accel_pressed_last = cur_time
          self.decel_pressed_last = cur_time
          self.fastMode = True
      else:
        self.fastMode = False  
    elif self.CP.pcmCruise and CS.cruiseState.enabled:
      self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
    
    

    if self.events.any(ET.RESET_V_CRUISE):
      self.v_cruise_kph = 0
      
    self.CI.CS.v_cruise_kph = self.v_cruise_kph

    # decrease the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types = [ET.PERMANENT]

    # ENABLED, PRE ENABLING, SOFT DISABLING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.events.any(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif self.events.any(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if self.events.any(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = 300   # 3s
            self.current_alert_types.append(ET.SOFT_DISABLE)

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not self.events.any(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if not self.events.any(ET.PRE_ENABLE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

    # DISABLED
    elif self.state == State.disabled:
      if self.events.any(ET.ENABLE):
        if self.events.any(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if self.events.any(ET.PRE_ENABLE):
            self.state = State.preEnabled
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)
          self.v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, self.v_cruise_kph_last)
          

    # Check if actuators are enabled
    self.active = self.state == State.enabled or self.state == State.softDisabling
    if self.active:
      self.current_alert_types.append(ET.WARNING)
      
    self.lat_active = self.CI.CS.lkaEnabled and vEgo > self.CP.minSteerSpeed \
                    and (self.active or ((self.CI.CS.cruiseMain and self.MADS_enabled) and self.sm['liveCalibration'].calStatus == Calibration.CALIBRATED))

    # Check if openpilot is engaged
    self.enabled = self.active or self.state == State.preEnabled
    
    if not self.openpilot_enabled_toggle:
      self.current_alert_types = []

  def state_control(self, CS):
    """Given the state, this function returns an actuators packet"""

    # Update VehicleModel
    params = self.sm['liveParameters']
    x = max(params.stiffnessFactor, 0.1)
    sr = max(params.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    lat_plan = self.sm['lateralPlan']
    long_plan = self.sm['longitudinalPlan']
    
    if self.sm.updated['liveParameters'] and len(self.sm['modelV2'].orientation.y) == TRAJECTORY_SIZE:
      future_pitch_diff = clip(interp(self.CI.CS.pitch_future_time, T_IDXS, self.sm['modelV2'].orientation.y), -MAX_ABS_PRED_PITCH_DELTA, MAX_ABS_PRED_PITCH_DELTA)
      self.CI.CS.pitch_raw = self.sm['liveParameters'].pitch + future_pitch_diff
    
    self.CI.CS.coasting_long_plan = long_plan.longitudinalPlanSource
    self.CI.CS.coasting_lead_d = long_plan.leadDist
    self.CI.CS.coasting_lead_v = long_plan.leadV
    self.CI.CS.tr = long_plan.desiredFollowDistance

    actuators = car.CarControl.Actuators.new_message()
    actuators.longControlState = self.LoC.long_control_state

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    # State specific actions

    if not self.active:
      if not self.lat_active:
        self.LaC.reset()
      if not self.MADS_lead_braking_enabled or not self.CI.CS.cruiseMain:
        self.LoC.reset(v_pid=CS.vEgo)
    else:
      if not self.CI.CS.lkaEnabled:
        self.LaC.reset()

    if not self.joystick_mode:
      # accel PID loop
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_kph * CV.KPH_TO_MS, self.CI)
      t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL
      actuators.accel = self.LoC.update(self.active, CS, self.CP, long_plan, pid_accel_limits, t_since_plan, MADS_lead_braking_enabled=self.MADS_lead_braking_enabled)
      
      # compute pitch-compensated accel
      if self.sm.updated['liveParameters']:
        self.pitch = apply_deadzone(self.sm['liveParameters'].pitchFutureLong, self.pitch_accel_deadzone)
      actuators.accelPitchCompensated = actuators.accel + ((ACCELERATION_DUE_TO_GRAVITY * math.sin(self.pitch)) if self.use_sensors else 0.0)

      # Steering PID loop and lateral MPC
      desired_curvature, desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,
                                                                             lat_plan.psis,
                                                                             lat_plan.curvatures,
                                                                             lat_plan.curvatureRates)
      actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(self.lat_active, 
                                                                             CS, self.CP, self.VM, params, 
                                                                             desired_curvature, desired_curvature_rate, self.sm['liveLocationKalman'],
                                                                             use_roll=self.use_sensors, lat_plan=lat_plan,
                                                                             model_data=self.sm['modelV2'])
    else:
      lac_log = log.ControlsState.LateralDebugState.new_message()
      if self.sm.rcv_frame['testJoystick'] > 0 and self.active:
        actuators.accel = 4.0*clip(self.sm['testJoystick'].axes[0], -1, 1)

        steer = clip(self.sm['testJoystick'].axes[1], -1, 1)
        # max angle is 45 for angle-based cars
        actuators.steer, actuators.steeringAngleDeg = steer, steer * 45.

        lac_log.active = True
        lac_log.steeringAngleDeg = CS.steeringAngleDeg
        lac_log.output = steer
        lac_log.saturated = abs(steer) >= 0.9

    # Check for difference between desired angle and angle for angle based control
    angle_control_saturated = self.CP.steerControlType == car.CarParams.SteerControlType.angle and \
      abs(actuators.steeringAngleDeg - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD

    if angle_control_saturated and not CS.steeringPressed and self.lat_active:
      self.saturated_count += 1
    else:
      self.saturated_count = 0

    # Send a "steering required alert" if saturation count has reached the limit
    if (lac_log.saturated and not CS.steeringPressed) or \
       (self.saturated_count > STEER_ANGLE_SATURATION_TIMEOUT):

      if len(lat_plan.dPathPoints):
        # Check if we deviated from the path
        left_deviation = actuators.steer > 0 and lat_plan.dPathPoints[0] < -0.1
        right_deviation = actuators.steer < 0 and lat_plan.dPathPoints[0] > 0.1

        if left_deviation or right_deviation and CS.lkaEnabled:
          self.events.add(EventName.steerSaturated)

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, Number):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return actuators, lac_log

  def publish_logs(self, CS, start_time, actuators, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled
    CC.actuators = actuators
    
    if len(self.sm['liveLocationKalman'].orientationNED.value) > 2:
      CC.roll = self.sm['liveLocationKalman'].orientationNED.value[0]
      CC.pitch = self.sm['liveLocationKalman'].orientationNED.value[1]

    CC.cruiseControl.override = True
    CC.cruiseControl.cancel = not self.CP.pcmCruise or (not self.enabled and CS.cruiseState.enabled)
    if self.joystick_mode and self.sm.rcv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
      CC.cruiseControl.cancel = True

    # TODO remove car specific stuff in controls
    # Some override values for Honda
    # brake discount removes a sharp nonlinearity
    brake_discount = (1.0 - clip(-actuators.accel * (3.0/4.0), 0.0, 1.0))
    speed_override = max(0.0, (self.LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount)
    CC.cruiseControl.speedOverride = float(speed_override if self.CP.pcmCruise else 0.0)
    CC.cruiseControl.accelOverride = float(self.CI.calc_accel_override(CS.aEgo, self.a_target,
                                                                       CS.vEgo, self.v_target))
    
    CC.hudControl.setSpeed = float(self.v_cruise_kph) * CV.KPH_TO_MS
    CC.hudControl.speedVisible = self.enabled
    CC.hudControl.lanesVisible = self.enabled
    CC.hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead

    right_lane_visible = self.sm['lateralPlan'].rProb > 0.5
    left_lane_visible = self.sm['lateralPlan'].lProb > 0.5
    CC.hudControl.rightLaneVisible = bool(right_lane_visible)
    CC.hudControl.leftLaneVisible = bool(left_lane_visible)

    recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                    and not self.active and self.sm['liveCalibration'].calStatus == Calibration.CALIBRATED

    meta = self.sm['modelV2'].meta
    if len(meta.desirePrediction) and ldw_allowed:
      l_lane_change_prob = meta.desirePrediction[Desire.laneChangeLeft - 1]
      r_lane_change_prob = meta.desirePrediction[Desire.laneChangeRight - 1]
      l_lane_close = left_lane_visible and (self.sm['modelV2'].laneLines[1].y[0] > -(1.08 + CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (self.sm['modelV2'].laneLines[2].y[0] < (1.08 - CAMERA_OFFSET))

      CC.hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      CC.hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

    if CC.hudControl.rightLaneDepart or CC.hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)

    clear_event = ET.WARNING if ET.WARNING not in self.current_alert_types else None
    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, self.sm, self.is_metric])
    self.AM.add_many(self.sm.frame, alerts, self.enabled)
    self.AM.process_alerts(self.sm.frame, clear_event)
    CC.hudControl.visualAlert = self.AM.visual_alert

    if not self.read_only and self.initialized:
      # send car controls over can
      can_sends = self.CI.apply(CC)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
    
    
    CC.onePedalAccelOutput = float(self.CI.CC.one_pedal_decel)
    CC.onePedalAccelInput = float(self.CI.CC.one_pedal_decel_in)
    CC.onePedalP = float(self.CI.CC.one_pedal_pid.p)
    CC.onePedalI = float(self.CI.CC.one_pedal_pid.i)
    CC.onePedalD = float(self.CI.CC.one_pedal_pid.d)
    CC.onePedalF = float(self.CI.CC.one_pedal_pid.f)

    force_decel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                  (self.state == State.softDisabling)

    # Curvature & Steering angle
    params = self.sm['liveParameters']

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetDeg)
    curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, params.roll)

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    controlsState = dat.controlsState
    controlsState.alertText1 = self.AM.alert_text_1
    controlsState.alertText2 = self.AM.alert_text_2
    controlsState.alertSize = self.AM.alert_size
    controlsState.alertStatus = self.AM.alert_status
    controlsState.alertBlinkingRate = self.AM.alert_rate
    controlsState.alertType = self.AM.alert_type
    controlsState.alertSound = self.AM.audible_alert
    controlsState.canMonoTimes = list(CS.canMonoTimes)
    controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    controlsState.lateralPlanMonoTime = self.sm.logMonoTime['lateralPlan']
    controlsState.enabled = self.enabled
    controlsState.active = self.active
    controlsState.latActive = self.lat_active
    controlsState.madsEnabled = self.MADS_enabled and not self.active and self.CI.CS.cruiseMain
    controlsState.curvature = curvature
    controlsState.state = self.state
    controlsState.engageable = not self.events.any(ET.NO_ENTRY)
    controlsState.longControlState = self.LoC.long_control_state
    controlsState.vPid = float(self.LoC.v_pid)
    controlsState.vCruise = float(self.v_cruise_kph)
    controlsState.aTarget = float(self.LoC.a_target)
    controlsState.upAccelCmd = float(self.LoC.pid.p)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    controlsState.udAccelCmd = float(self.LoC.pid.d)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    controlsState.startMonoTime = int(start_time * 1e9)
    controlsState.forceDecel = bool(force_decel)
    controlsState.canErrorCounter = self.can_error_counter
    controlsState.interactionTimer = int(self.interaction_timer)
    controlsState.interventionTimer = int(self.intervention_timer)
    controlsState.distractionTimer = int(self.distraction_timer)
    controlsState.parkedTimer = int(self.parked_timer)
    
    controlsState.applyGas = int(self.CI.CC.apply_gas)
    controlsState.applyBrakeOut = int(-self.CI.CC.apply_brake_out)
    controlsState.applyBrakeIn = int(-self.CI.CC.apply_brake_in)
    controlsState.applySteer = int(self.CI.CC.apply_steer)
    controlsState.brakesAllowed = bool(self.CI.CC.brakes_allowed)
    controlsState.gasBrakeThresholdAccel = float(self.CI.CC.threshold_accel)
    
    controlsState.distanceTraveledSession = float(self.distance_traveled)
    controlsState.distanceTraveledTotal = float(self.distance_traveled_total)
    controlsState.carRunningTimerTotal = int(self.car_running_timer_total)
    controlsState.carRunningTimerSession = int(self.car_running_timer_session)
    controlsState.openpilotLongControlTimerTotal = int(self.openpilot_long_control_timer_total)
    controlsState.openpilotLongControlTimerSession = int(self.openpilot_long_control_timer_session)
    controlsState.disengagementTimer = int(self.disengagement_timer)
    controlsState.disengagementCountTotal = int(self.disengagement_count_total)
    controlsState.disengagementCountSession = int(self.disengagement_count_session)
    controlsState.interactionCountTotal = int(self.interaction_count_total)
    controlsState.interactionCountSession = int(self.interaction_count_session)
    controlsState.interventionCountTotal = int(self.intervention_count_total)
    controlsState.interventionCountSession = int(self.intervention_count_session)
    controlsState.distractionCountTotal = int(self.distraction_count_total)
    controlsState.distractionCountSession = int(self.distraction_count_session)
    controlsState.interactionDistance = float(self.interaction_dist)
    controlsState.interventionDistance = float(self.intervention_dist)
    controlsState.engagedDistance = float(self.engaged_dist)
    controlsState.engagedDistanceTotal = float(self.engaged_dist_total)
    controlsState.engagedDistanceSession = float(self.engaged_dist_session)
    controlsState.distractionDistance = int(self.distraction_dist)
    controlsState.percentEngagedTimeSession = float(self.openpilot_long_control_timer_session / max(self.car_running_timer_session, 1.0) * 100.0)
    controlsState.percentEngagedTimeTotal = float(self.openpilot_long_control_timer_total / max(self.car_running_timer_total, 1.0) * 100.0)
    controlsState.percentEngagedDistanceSession = float(self.engaged_dist_session / max(1.0, self.distance_traveled) * 100.0)
    controlsState.percentEngagedDistanceTotal = float(self.engaged_dist_total / max(1.0, self.distance_traveled_total) * 100.0)

    lat_tuning = self.CP.lateralTuning.which()
    if self.joystick_mode:
      controlsState.lateralControlState.debugState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      controlsState.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      controlsState.lateralControlState.pidState = lac_log
    elif lat_tuning == 'lqr':
      controlsState.lateralControlState.lqrState = lac_log
    elif lat_tuning == 'indi':
      controlsState.lateralControlState.indiState = lac_log
    elif lat_tuning == 'torque':
      controlsState.lateralControlState.torqueState = lac_log
    elif lat_tuning == 'torqueLqr':
      controlsState.lateralControlState.torqueLqrState = lac_log
    elif lat_tuning == 'torqueIndi':
      controlsState.lateralControlState.torqueIndiState = lac_log
    self.pm.send('controlsState', dat)

    # carState
    car_events = self.events.to_msg()
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.events = car_events
    self.pm.send('carState', cs_send)

    # carEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('carEvents', len(self.events))
      ce_send.carEvents = car_events
      self.pm.send('carEvents', ce_send)
    self.events_prev = self.events.names.copy()

    # carParams - logged every 50 seconds (> 1 per segment)
    if (self.sm.frame % int(50. / DT_CTRL) == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

    # copy CarControl to pass to CarInterface on the next iteration
    self.CC = CC

  def step(self):
    start_time = sec_since_boot()
    self.prof.checkpoint("Ratekeeper", ignore=True)

    # Sample data from sockets and get a carState
    CS = self.data_sample()
    self.prof.checkpoint("Sample")

    self.update_events(CS)

    if not self.read_only and self.initialized:
      # Update control state
      self.state_transition(CS)
      self.prof.checkpoint("State transition")

    # Compute actuators (runs PID loops and lateral MPC)
    actuators, lac_log = self.state_control(CS)

    self.prof.checkpoint("State Control")

    # Publish data
    self.publish_logs(CS, start_time, actuators, lac_log)
    self.prof.checkpoint("Sent")

  def controlsd_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()

def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
