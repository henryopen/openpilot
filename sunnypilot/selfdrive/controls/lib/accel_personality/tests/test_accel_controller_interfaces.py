import math
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import custom, messaging
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import N, LongitudinalMpc
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelControllerState, AccelProfile
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, LongitudinalPlanSource


def radar_state():
  return messaging.new_message('radarState').radarState


def test_legacy_profile_enum_keeps_toyota_importable():
  expected = {"eco": 0, "normal": 1, "sport": 2}
  assert custom.LongitudinalPlanSP.AccelerationPersonality.schema.enumerants == expected
  assert custom.LongitudinalPlanSP.AccelController.Profile.schema.enumerants == expected
  from opendbc.car.toyota.carstate import AccelPersonality, CarState
  assert AccelPersonality.schema.enumerants == expected
  assert CarState.__module__ == "opendbc.car.toyota.carstate"


def test_stock_mpc_parameters_are_unchanged():
  mpc = LongitudinalMpc()
  mpc.set_cur_state(10.0, 0.0)
  mpc.run = lambda: None
  mpc.update(radar_state(), 30.0)
  np.testing.assert_array_equal(mpc.params[:, 0], ACCEL_MIN)
  np.testing.assert_array_equal(mpc.params[:, 1], ACCEL_MAX)


def test_positive_scalar_changes_only_acceleration_ceiling():
  radar = radar_state()
  mpc = LongitudinalMpc()
  mpc.set_cur_state(10.0, 0.0)
  mpc.run = lambda: None
  mpc.update(radar, 30.0)
  stock = mpc.params.copy()
  stock_source = mpc.source
  mpc.update(radar, 30.0, accel_max=0.8)
  np.testing.assert_array_equal(mpc.params[:, 0], stock[:, 0])
  np.testing.assert_array_equal(mpc.params[:, 1], 0.8)
  np.testing.assert_array_equal(mpc.params[:, 2:], stock[:, 2:])
  assert mpc.source == stock_source


def test_negative_finite_horizon_ceiling_is_applied_exactly():
  ceiling = np.linspace(0.2, -0.8, N + 1)
  mpc = LongitudinalMpc()
  mpc.set_cur_state(10.0, 0.2)
  mpc.run = lambda: None
  mpc.update(radar_state(), 30.0, accel_max=ceiling)
  np.testing.assert_allclose(mpc.params[:, 1], ceiling)
  np.testing.assert_array_equal(mpc.params[:, 0], ACCEL_MIN)


@pytest.mark.parametrize("accel_max", [np.inf, np.nan, -0.4, ACCEL_MIN, np.ones(N), "invalid"])
def test_invalid_or_negative_scalar_limit_is_exact_stock(accel_max):
  radar = radar_state()
  mpc = LongitudinalMpc()
  mpc.set_cur_state(10.0, 0.0)
  mpc.run = lambda: None
  mpc.update(radar, 30.0)
  stock = mpc.params.copy()
  mpc.update(radar, 30.0, accel_max=accel_max)
  np.testing.assert_array_equal(mpc.params, stock)


def test_custom_ceiling_keeps_raw_lead_obstacle_and_source_authoritative():
  radar = radar_state()
  radar.leadOne.status = True
  radar.leadOne.dRel = 30.0
  radar.leadOne.vLead = 5.0
  radar.leadOne.aLeadK = 0.0
  radar.leadOne.aLeadTau = 1.0
  before = (radar.leadOne.dRel, radar.leadOne.vLead, radar.leadOne.aLeadK)
  mpc = LongitudinalMpc()
  mpc.set_cur_state(20.0, 0.0)
  mpc.run = lambda: None
  mpc.update(radar, 30.0)
  stock = mpc.params.copy()
  stock_source = mpc.source
  mpc.update(radar, 30.0, accel_max=np.linspace(0.0, -0.5, N + 1))
  np.testing.assert_array_equal(mpc.params[:, 0], stock[:, 0])
  np.testing.assert_array_equal(mpc.params[:, 2:], stock[:, 2:])
  assert mpc.source == stock_source
  assert (radar.leadOne.dRel, radar.leadOne.vLead, radar.leadOne.aLeadK) == before


def test_retry_seed_is_bounded_and_nonnegative_in_speed():
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner.mpc = LongitudinalMpc()
  states = []
  planner.mpc.solver = SimpleNamespace(set=lambda _stage, field, value: states.append(np.asarray(value)) if field == 'x' else None)
  planner.mpc.set_cur_state(0.0, ACCEL_MIN)
  planner._seed_mpc_current_state()
  states = np.asarray(states)
  assert len(states) == N + 1
  assert np.all(np.diff(states[:, 0]) >= 0.0)
  assert np.all(states[:, 1] >= 0.0)
  assert np.all((states[:, 2] >= ACCEL_MIN) & (states[:, 2] <= ACCEL_MAX))


def test_last_solve_failure_survives_internal_reset():
  mpc = LongitudinalMpc()
  mpc.last_solution_status = 3
  mpc.reset()
  assert mpc.solution_status == 0
  assert mpc.last_solution_status == 3


def test_shadow_telemetry_publishes_controller_fields():
  planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
  planner.source = LongitudinalPlanSource.cruise
  planner.output_v_target = 20.0
  planner.output_a_target = 0.0
  planner.events_sp = SimpleNamespace(to_msg=list)
  planner.dec = SimpleNamespace(mode=lambda: "acc", enabled=lambda: False, active=lambda: False)
  planner.accel_controller_result = SimpleNamespace(
    enabled=True, active=False, shadow_active=True, profile=AccelProfile.normal, state=AccelControllerState.inactive,
    shadow_state=AccelControllerState.restrict, base_speed=20.0, raw_energy_cap=15.0, live_filtered_cap=np.inf,
    shadow_filtered_cap=12.5, selected_lead=1, usable_gap=30.0, closing_speed=5.0, required_decel=0.4,
    profile_accel_max=np.inf, effective_accel_max=np.inf,
  )
  planner.scc = SimpleNamespace(
    vision=SimpleNamespace(state=0, output_v_target=20.0, output_a_target=0.0, current_lat_acc=0.0, max_pred_lat_acc=0.0,
                           is_enabled=False, is_active=False),
    map=SimpleNamespace(state=0, output_v_target=20.0, output_a_target=0.0, is_enabled=False, is_active=False),
  )
  planner.resolver = SimpleNamespace(speed_limit=0.0, speed_limit_last=0.0, speed_limit_final=0.0, speed_limit_final_last=0.0,
                                     speed_limit_valid=False, speed_limit_last_valid=False, speed_limit_offset=0.0,
                                     distance=0.0, source=custom.LongitudinalPlanSP.SpeedLimit.Source.none)
  planner.sla = SimpleNamespace(state=custom.LongitudinalPlanSP.SpeedLimit.AssistState.disabled, is_enabled=False, is_active=False,
                                output_v_target=20.0, output_a_target=0.0)
  planner.e2e_alerts_helper = SimpleNamespace(green_light_alert=False, lead_depart_alert=False)
  sent = {}
  planner.publish_longitudinal_plan_sp(SimpleNamespace(all_checks=lambda service_list: True),
                                       SimpleNamespace(send=lambda service, message: sent.update({service: message})))
  telemetry = sent["longitudinalPlanSP"].longitudinalPlanSP.accelController
  assert telemetry.vTargetShadow == pytest.approx(12.5)
  assert telemetry.aMaxProfile == math.inf
  assert telemetry.aMaxEffective == math.inf
