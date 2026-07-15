from types import SimpleNamespace

import numpy as np
import pytest

from cereal import custom, messaging
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import N, LongitudinalMpc
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelControllerState, AccelProfile
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, LongitudinalPlanSource


def test_legacy_profile_enum_keeps_toyota_importable():
  expected = {"eco": 0, "normal": 1, "sport": 2}
  legacy_profile = custom.LongitudinalPlanSP.AccelerationPersonality

  assert legacy_profile.schema.enumerants == expected
  assert custom.LongitudinalPlanSP.AccelController.Profile.schema.enumerants == expected

  from opendbc.car.toyota.carstate import AccelPersonality, CarState

  assert AccelPersonality.schema.enumerants == expected
  assert CarState.__module__ == "opendbc.car.toyota.carstate"


def test_mpc_profile_preshapes_accel_bound_and_reachable_cruise_reference():
  radar_state = messaging.new_message('radarState').radarState
  mpc = LongitudinalMpc()
  mpc.set_cur_state(10.0, 0.0)
  mpc.run = lambda: None
  accel_max = np.linspace(0.4, 1.0, N + 1)

  mpc.update(radar_state, 30.0, accel_max=accel_max, shape_accel_max_in_cruise=True)
  shaped_params = mpc.params.copy()
  mpc.update(radar_state, 30.0)
  stock_params = mpc.params.copy()

  np.testing.assert_array_equal(shaped_params[:, 0], ACCEL_MIN)
  np.testing.assert_array_equal(shaped_params[:, 1], accel_max)
  assert np.any(shaped_params[:, 2] < stock_params[:, 2])
  np.testing.assert_array_equal(shaped_params[:, 3:], stock_params[:, 3:])
  np.testing.assert_array_equal(stock_params[:, 0], ACCEL_MIN)
  np.testing.assert_array_equal(stock_params[:, 1], ACCEL_MAX)


def test_mpc_preshape_keeps_current_accel_feasible_only_at_initial_node():
  radar_state = messaging.new_message('radarState').radarState
  mpc = LongitudinalMpc()
  mpc.set_cur_state(10.0, 0.8)
  mpc.run = lambda: None

  mpc.update(radar_state, 30.0, accel_max=np.full(N + 1, 0.3))
  shaped_params = mpc.params.copy()
  mpc.update(radar_state, 30.0)
  stock_params = mpc.params.copy()

  assert shaped_params[0, 1] == pytest.approx(0.8)
  np.testing.assert_array_equal(shaped_params[1:, 1], 0.3)
  np.testing.assert_array_equal(shaped_params[:, 0], ACCEL_MIN)
  np.testing.assert_array_equal(shaped_params[:, 2:], stock_params[:, 2:])


def test_mpc_last_solve_failure_survives_internal_solver_reset():
  mpc = LongitudinalMpc()
  mpc.last_solution_status = 3

  mpc.reset()

  assert mpc.solution_status == 0
  assert mpc.last_solution_status == 3


@pytest.mark.parametrize("accel_max", [None, np.inf, np.nan, np.ones(N), np.r_[np.ones(N), np.nan]])
def test_mpc_missing_or_invalid_preshape_is_exact_stock(accel_max):
  radar_state = messaging.new_message('radarState').radarState
  mpc = LongitudinalMpc()
  mpc.set_cur_state(10.0, 0.0)
  mpc.run = lambda: None
  mpc.update(radar_state, 30.0)
  stock_params = mpc.params.copy()

  mpc.update(radar_state, 30.0, accel_max=accel_max)

  np.testing.assert_array_equal(mpc.params, stock_params)


def test_shadow_target_telemetry_publishes_filtered_cap():
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner.source = LongitudinalPlanSource.cruise
  planner.output_v_target = 20.0
  planner.output_a_target = 0.0
  planner.events_sp = SimpleNamespace(to_msg=list)
  planner.dec = SimpleNamespace(mode=lambda: "acc", enabled=lambda: False, active=lambda: False)
  planner.accel_controller_result = SimpleNamespace(
    enabled=True,
    active=False,
    shadow_active=True,
    launching=False,
    profile=AccelProfile.normal,
    state=AccelControllerState.inactive,
    shadow_state=AccelControllerState.restrict,
    base_speed=20.0,
    raw_energy_cap=15.0,
    live_filtered_cap=99.0,
    shadow_filtered_cap=12.5,
    shadow_pace=7.25,
    selected_lead=1,
    usable_gap=30.0,
    closing_speed=5.0,
    required_decel=0.4,
    profile_accel_max=1.0,
    effective_accel_max=0.85,
    mpc_accel_max=tuple(np.full(N + 1, 0.85)),
    mpc_shape_cruise=True,
  )
  planner.scc = SimpleNamespace(
    vision=SimpleNamespace(
      state=0,
      output_v_target=20.0,
      output_a_target=0.0,
      current_lat_acc=0.0,
      max_pred_lat_acc=0.0,
      is_enabled=False,
      is_active=False,
    ),
    map=SimpleNamespace(state=0, output_v_target=20.0, output_a_target=0.0, is_enabled=False, is_active=False),
  )
  planner.resolver = SimpleNamespace(
    speed_limit=0.0,
    speed_limit_last=0.0,
    speed_limit_final=0.0,
    speed_limit_final_last=0.0,
    speed_limit_valid=False,
    speed_limit_last_valid=False,
    speed_limit_offset=0.0,
    distance=0.0,
    source=custom.LongitudinalPlanSP.SpeedLimit.Source.none,
  )
  planner.sla = SimpleNamespace(
    state=custom.LongitudinalPlanSP.SpeedLimit.AssistState.disabled,
    is_enabled=False,
    is_active=False,
    output_v_target=20.0,
    output_a_target=0.0,
  )
  planner.e2e_alerts_helper = SimpleNamespace(green_light_alert=False, lead_depart_alert=False)

  sent = {}
  sm = SimpleNamespace(all_checks=lambda service_list: True)
  pm = SimpleNamespace(send=lambda service, message: sent.update({service: message}))
  planner.publish_longitudinal_plan_sp(sm, pm)

  telemetry = sent["longitudinalPlanSP"].longitudinalPlanSP.accelController
  assert telemetry.vTargetShadow == pytest.approx(planner.accel_controller_result.shadow_filtered_cap)
  assert telemetry.vTargetShadow != pytest.approx(planner.accel_controller_result.shadow_pace)
  assert telemetry.aMaxProfile == pytest.approx(planner.accel_controller_result.profile_accel_max)
  assert telemetry.aMaxEffective == pytest.approx(planner.accel_controller_result.effective_accel_max)
