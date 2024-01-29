import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV


class FrogPilotPlanner:
  def __init__(self, params, params_memory):
    self.v_cruise = 0

    self.update_frogpilot_params(params, params_memory)

    self.update_frogpilot_params(params, params_memory)

  def update(self, carState, controlsState, modelData, mpc, sm, v_cruise, v_ego):
    enabled = controlsState.enabled

    self.v_cruise = self.update_v_cruise(carState, controlsState, modelData, enabled, v_cruise, v_ego)

  def update_v_cruise(self, carState, controlsState, modelData, enabled, v_cruise, v_ego):
    v_ego_diff = max(carState.vEgoRaw - carState.vEgoCluster, 0)
    return v_cruise - v_ego_diff

  def publish(self, sm, pm, mpc):
    frogpilot_plan_send = messaging.new_message('frogpilotPlan')
    frogpilot_plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])
    frogpilotPlan = frogpilot_plan_send.frogpilotPlan

    pm.send('frogpilotPlan', frogpilot_plan_send)

  def update_frogpilot_params(self, params, params_memory):
    self.is_metric = params.get_bool("IsMetric")

    custom_ui = params.get_bool("CustomUI")

    lateral_tune = params.get_bool("LateralTune")

    longitudinal_tune = params.get_bool("LongitudinalTune")
