"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource as MpcLongitudinalPlanSource
from openpilot.sunnypilot import get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelController, AccelControllerState, AccelProfile
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import (
  MPC_DECEL_JERK_COST_MULTIPLIER, MPC_DECEL_JERK_MAX_REQUIRED_DECEL, MPC_DECEL_JERK_MAX_TARGET_REDUCTION,
)
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc, dt: float = DT_MDL):
    self.params = Params()
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()
    self.accel_controller = AccelController(CP, dt=dt)
    self.accel_controller_result = None
    self._accel_jerk_smoothing_blocked = False
    self._radar_log_mono_time = None
    self._radar_fresh_this_cycle = True

    self._param_read_frames = max(1, int(round(0.25 / dt)))
    self._param_frame = 0
    self.accel_personality_enabled = False
    self.accel_personality = int(AccelProfile.normal)

    self.output_v_target = 0.
    self.output_a_target = 0.

  def _read_accel_controller_params(self) -> None:
    if self._param_frame % self._param_read_frames == 0:
      self.accel_personality_enabled = self.params.get_bool("AccelPersonalityEnabled")
      self.accel_personality = get_sanitize_int_param(
        "AccelPersonality", int(AccelProfile.eco), int(AccelProfile.sport), self.params,
      )

    self._param_frame += 1

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def _update_radar_freshness(self, sm: messaging.SubMaster) -> bool:
    try:
      radar_log_mono_time = int(sm.logMonoTime['radarState'])
      radar_healthy = bool(sm.valid['radarState'] and sm.alive['radarState'])
    except (AttributeError, KeyError, TypeError, ValueError):
      return True

    previous_log_mono_time = getattr(self, '_radar_log_mono_time', None)
    radar_advanced = previous_log_mono_time is None or radar_log_mono_time > previous_log_mono_time
    if radar_advanced:
      self._radar_log_mono_time = radar_log_mono_time
    return radar_healthy and radar_advanced

  def update_accel_controller(self, sm: messaging.SubMaster, base_speed: float, engaged: bool, cruise_initialized: bool,
                              acc_selected: bool, stock_accel_max: float, previous_should_stop: bool) -> float:
    self.accel_controller_result = self.accel_controller.update(
      sm['radarState'], base_speed=base_speed, v_ego=sm['carState'].vEgo, a_ego=sm['carState'].aEgo,
      profile=self.accel_personality, follow_personality=sm['selfdriveState'].personality,
      enabled=self.accel_personality_enabled, acc_selected=acc_selected, engaged=engaged, cruise_initialized=cruise_initialized,
      stock_accel_max=stock_accel_max, previous_should_stop=previous_should_stop,
      radar_fresh=getattr(self, '_radar_fresh_this_cycle', True),
      previous_mpc_source=getattr(getattr(self, 'mpc', None), 'source', None),
      planner_speed=getattr(getattr(self, 'v_desired_filter', None), 'x', sm['carState'].vEgo),
      planner_accel=getattr(self, 'a_desired', sm['carState'].aEgo),
    )
    return self.accel_controller_result.target_speed

  def _run_mpc(self, sm: messaging.SubMaster, v_cruise: float, prev_accel_constraint: bool, accel_max=None,
               *, jerk_cost_multiplier: float = 1.0) -> None:
    self.mpc.set_weights(
      prev_accel_constraint, personality=sm['selfdriveState'].personality, jerk_cost_multiplier=jerk_cost_multiplier,
    )
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality, accel_max=accel_max)

  def update_accel_controller_mpc(self, sm: messaging.SubMaster, base_v_cruise: float, mpc_v_cruise: float,
                                  prev_accel_constraint: bool, *, reset_state: bool, cruise_initialized: bool,
                                  available_accel_max: float, previous_should_stop: bool, force_decel: bool):
    is_e2e = self.is_e2e(sm)
    previous_mpc_failed = getattr(getattr(self, 'mpc', None), 'last_solution_status', 0) != 0
    if previous_mpc_failed and hasattr(self, 'accel_controller'):
      self.accel_controller.reset()

    self.update_accel_controller(
      sm, base_v_cruise, engaged=not reset_state and not force_decel, cruise_initialized=cruise_initialized,
      acc_selected=not is_e2e and not previous_mpc_failed, stock_accel_max=available_accel_max, previous_should_stop=previous_should_stop,
    )
    result = self.accel_controller_result
    actuating = result.active and not is_e2e and not force_decel and not previous_mpc_failed
    valid_lead_stop_hold = (actuating and result.state == AccelControllerState.stopHold
                            and result.selected_lead >= 0)
    controller_v_cruise = mpc_v_cruise if valid_lead_stop_hold else min(mpc_v_cruise, result.target_speed) if actuating else mpc_v_cruise
    accel_max = result.mpc_accel_max if actuating else None
    target_reduction = mpc_v_cruise - controller_v_cruise
    lead_restriction = (
      actuating and prev_accel_constraint and result.state == AccelControllerState.restrict and result.selected_lead >= 0
      and not result.launching and target_reduction > 1e-6
    )
    smoothing_eligible = (lead_restriction and target_reduction < MPC_DECEL_JERK_MAX_TARGET_REDUCTION
                          and 0.0 < result.required_decel < MPC_DECEL_JERK_MAX_REQUIRED_DECEL)
    smoothing_blocked = getattr(self, '_accel_jerk_smoothing_blocked', False)
    if previous_mpc_failed:
      smoothing_blocked = True
    elif not lead_restriction:
      smoothing_blocked = False
    elif not smoothing_blocked and (getattr(self.mpc, 'source', None) != MpcLongitudinalPlanSource.cruise or not smoothing_eligible):
      smoothing_blocked = True
    self._accel_jerk_smoothing_blocked = smoothing_blocked
    jerk_cost_multiplier = MPC_DECEL_JERK_COST_MULTIPLIER if smoothing_eligible and not smoothing_blocked else 1.0
    self._run_mpc(sm, controller_v_cruise, prev_accel_constraint, accel_max, jerk_cost_multiplier=jerk_cost_multiplier)

    return is_e2e

  def accel_controller_should_stop(self, should_stop: bool, is_e2e: bool) -> bool:
    result = self.accel_controller_result
    if result is None or not result.active or is_e2e:
      return should_stop
    if result.departure_launching:
      return False
    return should_stop or result.state == AccelControllerState.stopHold

  def update(self, sm: messaging.SubMaster) -> None:
    self._radar_fresh_this_cycle = self._update_radar_freshness(sm)
    self._read_accel_controller_params()
    self.events_sp.clear()
    self.dec.update(sm, radar_fresh=self._radar_fresh_this_cycle)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    if self.accel_controller_result is not None:
      result = self.accel_controller_result
      accel_controller = longitudinalPlanSP.accelController
      accel_controller.enabled = result.enabled
      accel_controller.active = result.active
      accel_controller.shadowOnly = result.shadow_active and not result.active
      accel_controller.profile = int(result.profile)
      accel_controller.state = int(result.state if result.active else result.shadow_state)
      accel_controller.vTargetBase = float(result.base_speed)
      accel_controller.vTargetRaw = float(result.raw_energy_cap)
      accel_controller.vTargetFiltered = float(result.live_filtered_cap)
      accel_controller.vTargetShadow = float(result.shadow_filtered_cap)
      accel_controller.leadIndex = result.selected_lead
      accel_controller.usableGap = float(result.usable_gap)
      accel_controller.closingSpeed = float(result.closing_speed)
      accel_controller.requiredDecel = float(result.required_decel)
      accel_controller.aMaxProfile = float(result.profile_accel_max)
      accel_controller.aMaxEffective = float(result.effective_accel_max)

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
