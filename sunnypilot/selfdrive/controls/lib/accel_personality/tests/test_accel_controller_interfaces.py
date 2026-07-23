import inspect
import math
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import custom, log, messaging
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import N, LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource as MpcLongitudinalPlanSource
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelControllerState, AccelProfile
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import (
  MPC_DECEL_JERK_COST_MULTIPLIER, MPC_DECEL_JERK_MAX_REQUIRED_DECEL, MPC_DECEL_JERK_MAX_TARGET_REDUCTION,
)
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, LongitudinalPlanSource


def radar_state():
  return messaging.new_message("radarState").radarState


class PlannerSM(dict):
  def __init__(self, radar_log_mono_time: int):
    super().__init__(
      radarState=radar_state(),
      carState=SimpleNamespace(vEgo=10.0, aEgo=0.0),
      selfdriveState=SimpleNamespace(personality=0),
    )
    self.valid = {"radarState": True}
    self.alive = {"radarState": True}
    self.logMonoTime = {"radarState": radar_log_mono_time}


def planner_for_mpc_test(*, target_speed=15.0, active=True, is_e2e=False, mpc_accel_max=None,
                         state=AccelControllerState.free, selected_lead=-1, launching=False,
                         departure_launching=False, required_decel=0.0,
                         mpc_source=MpcLongitudinalPlanSource.lead0):
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  is_e2e_calls = []
  planner.is_e2e = lambda _sm: is_e2e_calls.append(True) or is_e2e
  planner._accel_jerk_smoothing_blocked = False
  planner.mpc = SimpleNamespace(source=mpc_source, last_solution_status=0)
  planner.update_accel_controller = lambda *_args, **_kwargs: setattr(
    planner, "accel_controller_result",
    SimpleNamespace(
      target_speed=target_speed, active=active, state=state, selected_lead=selected_lead,
      launching=launching, departure_launching=departure_launching,
      required_decel=required_decel, mpc_accel_max=mpc_accel_max,
    ),
  )
  return planner, is_e2e_calls


def run_controller_mpc(planner, *, mpc_v_cruise=20.0, force_decel=False):
  calls = []
  planner._run_mpc = lambda *args, **kwargs: calls.append((args, kwargs))
  is_e2e = planner.update_accel_controller_mpc(
    {}, 20.0, mpc_v_cruise, True, reset_state=False, cruise_initialized=True,
    available_accel_max=ACCEL_MAX, previous_should_stop=False, force_decel=force_decel,
  )
  return is_e2e, calls


def test_profile_enum_keeps_toyota_importable():
  expected = {"eco": 0, "normal": 1, "sport": 2}
  assert custom.LongitudinalPlanSP.AccelerationPersonality.schema.enumerants == expected
  assert custom.LongitudinalPlanSP.AccelController.Profile.schema.enumerants == expected
  from opendbc.car.toyota.carstate import AccelPersonality, CarState

  assert AccelPersonality.schema.enumerants == expected
  assert CarState.__module__ == "opendbc.car.toyota.carstate"


def test_mpc_accepts_optional_acceleration_ceiling_without_changing_stock_bounds():
  assert tuple(inspect.signature(LongitudinalMpc.update).parameters) == ("self", "radarstate", "v_cruise", "personality", "accel_max")
  mpc = LongitudinalMpc()
  radar = radar_state()
  mpc.run = lambda: None

  mpc.set_cur_state(10.0, 0.8)
  mpc.update(radar, 30.0)
  np.testing.assert_array_equal(mpc.params[:, 0], ACCEL_MIN)
  np.testing.assert_array_equal(mpc.params[:, 1], ACCEL_MAX)

  requested_ceiling = np.full(N + 1, 0.4)
  mpc.update(radar, 30.0, accel_max=requested_ceiling)
  np.testing.assert_array_equal(mpc.params[:, 0], ACCEL_MIN)
  assert mpc.params[0, 1] == pytest.approx(0.8)
  np.testing.assert_array_equal(mpc.params[1:, 1], requested_ceiling[1:])

  for malformed_ceiling in ("bad", [0.4] * N, np.full(N + 1, math.nan), [10**10000] * (N + 1)):
    mpc.update(radar, 30.0, accel_max=malformed_ceiling)
    np.testing.assert_array_equal(mpc.params[:, 0], ACCEL_MIN)
    np.testing.assert_array_equal(mpc.params[:, 1], ACCEL_MAX)


def test_mpc_jerk_cost_multiplier_is_backward_compatible_and_does_not_change_other_costs():
  mpc = LongitudinalMpc.__new__(LongitudinalMpc)
  captured = []
  mpc.set_cost_weights = lambda costs, constraints: captured.append((np.asarray(costs), np.asarray(constraints)))

  mpc.set_weights(True, personality=log.LongitudinalPersonality.standard)
  default_costs, default_constraints = captured[-1]
  mpc.set_weights(True, personality=log.LongitudinalPersonality.standard, jerk_cost_multiplier=1.0)
  explicit_costs, explicit_constraints = captured[-1]
  mpc.set_weights(True, personality=log.LongitudinalPersonality.standard, jerk_cost_multiplier=1.2)
  smoothed_costs, smoothed_constraints = captured[-1]

  np.testing.assert_array_equal(explicit_costs, default_costs)
  np.testing.assert_array_equal(explicit_constraints, default_constraints)
  np.testing.assert_array_equal(smoothed_costs[:-1], default_costs[:-1])
  assert smoothed_costs[-1] == pytest.approx(default_costs[-1] * 1.2)
  np.testing.assert_array_equal(smoothed_constraints, default_constraints)

  mpc.set_weights(False, personality=log.LongitudinalPersonality.standard, jerk_cost_multiplier=1.2)
  assert captured[-1][0][-2] == 0.0
  assert captured[-1][0][-1] == pytest.approx(default_costs[-1] * 1.2)


def test_inherited_planner_uses_real_state_raw_radar_and_one_mpc_solve():
  radar = radar_state()
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner.a_desired = -0.2
  planner.v_desired_filter = SimpleNamespace(x=12.0)
  calls = []

  def update_mpc(radar_arg, target, *, personality, accel_max):
    calls.append(("update", radar_arg, target, personality, accel_max))

  planner.mpc = SimpleNamespace(
    set_weights=lambda constraint, personality, jerk_cost_multiplier: calls.append(
      ("weights", constraint, personality, jerk_cost_multiplier),
    ),
    set_cur_state=lambda speed, accel: calls.append(("state", speed, accel)),
    update=update_mpc,
  )
  ceiling = tuple(np.linspace(0.8, 0.4, N + 1))
  sm = {"radarState": radar, "selfdriveState": SimpleNamespace(personality=2)}
  planner._run_mpc(sm, 17.5, True, ceiling, jerk_cost_multiplier=1.2)

  assert calls == [
    ("weights", True, 2, 1.2),
    ("state", 12.0, -0.2),
    ("update", radar, 17.5, 2, ceiling),
  ]
  assert calls[-1][1] is radar


def test_active_acc_uses_target_and_ceiling_in_exactly_one_solve():
  ceiling = tuple(np.linspace(0.8, 0.4, N + 1))
  planner, mode_calls = planner_for_mpc_test(mpc_accel_max=ceiling)
  is_e2e, calls = run_controller_mpc(planner)

  assert not is_e2e
  assert len(mode_calls) == 1
  assert calls == [(({}, 15.0, True, ceiling), {"jerk_cost_multiplier": 1.0})]


def test_valid_lead_stop_hold_preplans_from_raw_target_without_an_accel_ceiling():
  planner, _ = planner_for_mpc_test(
    target_speed=0.0, mpc_accel_max=None, state=AccelControllerState.stopHold, selected_lead=0,
  )
  _, calls = run_controller_mpc(planner)

  assert calls == [(({}, 20.0, True, None), {"jerk_cost_multiplier": 1.0})]


def test_missing_lead_stop_hold_keeps_zero_mpc_target_without_an_accel_ceiling():
  planner, _ = planner_for_mpc_test(
    target_speed=0.0, mpc_accel_max=None, state=AccelControllerState.stopHold, selected_lead=-1,
  )
  _, calls = run_controller_mpc(planner)

  assert calls == [(({}, 0.0, True, None), {"jerk_cost_multiplier": 1.0})]


@pytest.mark.parametrize(
  ("active", "departure_launching", "is_e2e", "expected"),
  [
    (True, True, False, False),
    (True, False, False, True),
    (False, True, False, True),
    (True, True, True, True),
  ],
)
def test_only_confirmed_live_acc_departure_clears_should_stop(active, departure_launching, is_e2e, expected):
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner.accel_controller_result = SimpleNamespace(
    active=active, departure_launching=departure_launching, state=AccelControllerState.stopHold,
  )
  assert planner.accel_controller_should_stop(True, is_e2e) is expected
  expected_hold = active and not departure_launching and not is_e2e
  assert planner.accel_controller_should_stop(False, is_e2e) is expected_hold


@pytest.mark.parametrize(("active", "is_e2e"), [(False, False), (True, True)])
def test_disabled_or_e2e_is_an_exact_mpc_bypass(active, is_e2e):
  ceiling = tuple(np.linspace(0.8, 0.4, N + 1))
  planner, mode_calls = planner_for_mpc_test(active=active, is_e2e=is_e2e, mpc_accel_max=ceiling)
  returned_e2e, calls = run_controller_mpc(planner)

  assert returned_e2e is is_e2e
  assert len(mode_calls) == 1
  assert calls == [(({}, 20.0, True, None), {"jerk_cost_multiplier": 1.0})]


def test_force_decel_target_remains_authoritative_and_disables_ceiling():
  ceiling = tuple(np.linspace(0.8, 0.4, N + 1))
  planner, mode_calls = planner_for_mpc_test(mpc_accel_max=ceiling)
  _, calls = run_controller_mpc(planner, mpc_v_cruise=0.0, force_decel=True)

  assert len(mode_calls) == 1
  assert calls == [(({}, 0.0, True, None), {"jerk_cost_multiplier": 1.0})]


def test_previous_mpc_failure_gets_one_stock_recovery_cycle():
  ceiling = tuple(np.linspace(0.8, 0.4, N + 1))
  planner, mode_calls = planner_for_mpc_test(mpc_accel_max=ceiling)
  resets = []
  planner.accel_controller = SimpleNamespace(reset=lambda: resets.append(True))
  planner.mpc = SimpleNamespace(last_solution_status=4)

  _, failed_recovery_calls = run_controller_mpc(planner)
  assert resets == [True]
  assert len(mode_calls) == 1
  assert failed_recovery_calls == [(({}, 20.0, True, None), {"jerk_cost_multiplier": 1.0})]

  planner.mpc.last_solution_status = 0
  _, recovered_calls = run_controller_mpc(planner)
  assert resets == [True]
  assert len(mode_calls) == 2
  assert recovered_calls == [(({}, 15.0, True, ceiling), {"jerk_cost_multiplier": 1.0})]


def test_routine_governor_restriction_forwards_the_jerk_cost_multiplier():
  planner, _ = planner_for_mpc_test(
    state=AccelControllerState.restrict, selected_lead=0, required_decel=0.30,
    mpc_source=MpcLongitudinalPlanSource.cruise,
  )
  _, calls = run_controller_mpc(planner)

  assert calls == [(({}, 15.0, True, None), {"jerk_cost_multiplier": MPC_DECEL_JERK_COST_MULTIPLIER})]


def test_lead_source_blocks_smoothing_only_until_the_restriction_episode_ends():
  planner, _ = planner_for_mpc_test(
    state=AccelControllerState.restrict, selected_lead=0, required_decel=0.30,
    mpc_source=MpcLongitudinalPlanSource.cruise,
  )
  _, initial_calls = run_controller_mpc(planner)
  routine_result = planner.accel_controller_result
  assert initial_calls[0][1] == {"jerk_cost_multiplier": MPC_DECEL_JERK_COST_MULTIPLIER}

  planner.mpc.source = MpcLongitudinalPlanSource.lead0
  _, lead_calls = run_controller_mpc(planner)
  assert lead_calls[0][1] == {"jerk_cost_multiplier": 1.0}

  planner.mpc.source = MpcLongitudinalPlanSource.cruise
  _, flicker_calls = run_controller_mpc(planner)
  assert flicker_calls[0][1] == {"jerk_cost_multiplier": 1.0}

  free_result = SimpleNamespace(**(vars(routine_result) | {"state": AccelControllerState.free, "target_speed": 20.0}))
  planner.update_accel_controller = lambda *_args, **_kwargs: setattr(planner, "accel_controller_result", free_result)
  run_controller_mpc(planner)
  planner.update_accel_controller = lambda *_args, **_kwargs: setattr(planner, "accel_controller_result", routine_result)
  _, rearmed_calls = run_controller_mpc(planner)
  assert rearmed_calls[0][1] == {"jerk_cost_multiplier": MPC_DECEL_JERK_COST_MULTIPLIER}


@pytest.mark.parametrize(
  ("state", "selected_lead", "launching", "required_decel", "target_speed", "mpc_source"),
  [
    (AccelControllerState.free, 0, False, 0.30, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.hold, 0, False, 0.30, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.stopHold, 0, False, 0.30, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, -1, False, 0.30, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, True, 0.30, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, MPC_DECEL_JERK_MAX_REQUIRED_DECEL, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, math.inf, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, math.nan, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, 0.0, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, -0.01, 15.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, 0.30, 20.0 - MPC_DECEL_JERK_MAX_TARGET_REDUCTION, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, 0.30, 20.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, 0.30, 25.0, MpcLongitudinalPlanSource.cruise),
    (AccelControllerState.restrict, 0, False, 0.30, 15.0, MpcLongitudinalPlanSource.lead0),
    (AccelControllerState.restrict, 0, False, 0.30, 15.0, MpcLongitudinalPlanSource.lead1),
  ],
)
def test_non_routine_or_stock_lead_states_keep_stock_jerk_cost(
  state, selected_lead, launching, required_decel, target_speed, mpc_source,
):
  planner, _ = planner_for_mpc_test(
    state=state, selected_lead=selected_lead, launching=launching,
    required_decel=required_decel, target_speed=target_speed, mpc_source=mpc_source,
  )
  _, calls = run_controller_mpc(planner)

  assert calls[0][1] == {"jerk_cost_multiplier": 1.0}


def test_controller_receives_previous_mpc_state_and_cached_radar_freshness():
  radar = radar_state()
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner.accel_personality = int(AccelProfile.normal)
  planner.accel_personality_enabled = True
  planner._radar_fresh_this_cycle = True
  planner.a_desired = -0.4
  planner.v_desired_filter = SimpleNamespace(x=9.5)
  planner.mpc = SimpleNamespace(source=log.LongitudinalPlan.LongitudinalPlanSource.lead0)
  received = {}
  planner.accel_controller = SimpleNamespace(
    update=lambda *_args, **kwargs: received.update(kwargs) or SimpleNamespace(target_speed=12.0),
  )
  sm = {
    "radarState": radar,
    "carState": SimpleNamespace(vEgo=10.0, aEgo=-0.2),
    "selfdriveState": SimpleNamespace(personality=0),
  }
  planner.update_accel_controller(sm, 20.0, True, True, True, ACCEL_MAX, False)

  assert received["previous_mpc_source"] == log.LongitudinalPlan.LongitudinalPlanSource.lead0
  assert received["planner_speed"] == 9.5
  assert received["planner_accel"] == -0.4
  assert received["radar_fresh"] is True


def test_radar_freshness_is_computed_once_and_shared_with_dec_and_controller():
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner._radar_log_mono_time = None
  planner._radar_fresh_this_cycle = True
  planner._read_accel_controller_params = lambda: None
  planner.events_sp = SimpleNamespace(clear=lambda: None)
  dec_freshness = []
  planner.dec = SimpleNamespace(update=lambda _sm, *, radar_fresh: dec_freshness.append(radar_fresh))
  planner.e2e_alerts_helper = SimpleNamespace(update=lambda *_args: None)
  planner.accel_personality = int(AccelProfile.normal)
  planner.accel_personality_enabled = True
  planner.a_desired = 0.0
  planner.v_desired_filter = SimpleNamespace(x=10.0)
  planner.mpc = SimpleNamespace(source=log.LongitudinalPlan.LongitudinalPlanSource.cruise)
  controller_freshness = []
  planner.accel_controller = SimpleNamespace(
    update=lambda *_args, **kwargs: controller_freshness.append(kwargs["radar_fresh"]) or SimpleNamespace(target_speed=20.0),
  )

  sm = PlannerSM(100)
  for expected in (True, False):
    planner.update(sm)
    planner.update_accel_controller(sm, 20.0, True, True, True, ACCEL_MAX, False)
    assert dec_freshness[-1] is expected and controller_freshness[-1] is expected

  sm.logMonoTime["radarState"] = 101
  planner.update(sm)
  planner.update_accel_controller(sm, 20.0, True, True, True, ACCEL_MAX, False)
  assert dec_freshness[-1] is True and controller_freshness[-1] is True


def test_shadow_telemetry_publishes_controller_fields():
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner.source = LongitudinalPlanSource.cruise
  planner.output_v_target = 20.0
  planner.output_a_target = 0.0
  planner.events_sp = SimpleNamespace(to_msg=list)
  planner.dec = SimpleNamespace(mode=lambda: "acc", enabled=lambda: False, active=lambda: False)
  planner.accel_controller_result = SimpleNamespace(
    enabled=True, active=False, shadow_active=True, profile=AccelProfile.normal,
    state=AccelControllerState.inactive, shadow_state=AccelControllerState.restrict,
    base_speed=20.0, raw_energy_cap=15.0, live_filtered_cap=math.inf, shadow_filtered_cap=12.5,
    selected_lead=1, usable_gap=30.0, closing_speed=5.0, required_decel=0.4,
    profile_accel_max=math.inf, effective_accel_max=math.inf,
  )
  planner.scc = SimpleNamespace(
    vision=SimpleNamespace(state=0, output_v_target=20.0, output_a_target=0.0, current_lat_acc=0.0, max_pred_lat_acc=0.0, is_enabled=False, is_active=False),
    map=SimpleNamespace(state=0, output_v_target=20.0, output_a_target=0.0, is_enabled=False, is_active=False),
  )
  planner.resolver = SimpleNamespace(
    speed_limit=0.0, speed_limit_last=0.0, speed_limit_final=0.0, speed_limit_final_last=0.0,
    speed_limit_valid=False, speed_limit_last_valid=False, speed_limit_offset=0.0, distance=0.0,
    source=custom.LongitudinalPlanSP.SpeedLimit.Source.none,
  )
  planner.sla = SimpleNamespace(
    state=custom.LongitudinalPlanSP.SpeedLimit.AssistState.disabled, is_enabled=False, is_active=False,
    output_v_target=20.0, output_a_target=0.0,
  )
  planner.e2e_alerts_helper = SimpleNamespace(green_light_alert=False, lead_depart_alert=False)
  sent = {}
  planner.publish_longitudinal_plan_sp(
    SimpleNamespace(all_checks=lambda service_list: True),
    SimpleNamespace(send=lambda service, message: sent.update({service: message})),
  )

  telemetry = sent["longitudinalPlanSP"].longitudinalPlanSP.accelController
  assert telemetry.enabled and not telemetry.active and telemetry.shadowOnly
  assert telemetry.profile == int(AccelProfile.normal)
  assert telemetry.state == int(AccelControllerState.restrict)
  assert telemetry.vTargetBase == pytest.approx(20.0)
  assert telemetry.vTargetRaw == pytest.approx(15.0)
  assert telemetry.vTargetShadow == pytest.approx(12.5)
  assert telemetry.leadIndex == 1
  assert telemetry.usableGap == pytest.approx(30.0)
  assert telemetry.closingSpeed == pytest.approx(5.0)
  assert telemetry.requiredDecel == pytest.approx(0.4)
  assert telemetry.aMaxProfile == math.inf
  assert telemetry.aMaxEffective == math.inf
