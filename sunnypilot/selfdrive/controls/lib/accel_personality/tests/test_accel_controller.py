#!/usr/bin/env python3
import math
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import log
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import N, LongitudinalPlanSource, STOP_DISTANCE, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import (
  ACCEL_LIMIT_JERK,
  ACCEL_PROFILE_MAX_BP,
  ACCEL_PROFILE_MAX_V,
  BREAKAWAY_ACCEL_MAX,
  CLEAR_LAUNCH_ACCEL_RATE,
  DECEL_LIMIT_JERK,
  INITIAL_LAUNCH_ACCEL_MAX,
  LAUNCH_ACCEL_RATE,
  LAUNCH_DELTA_V,
  RELATIVE_PACE_PREVIEW_TIME,
  URGENT_BYPASS_REQUIRED_DECEL,
  AccelController,
  AccelControllerState,
  AccelProfile,
  PROFILE_CONFIGS,
)


def make_lead(*, status: bool = False, d_rel: float = 0.0, v_lead_k: float = 0.0, a_lead_k: float = 0.0, a_lead_tau: float = 1.5,
              radar_track_id: int = -1):
  return SimpleNamespace(
    status=status,
    dRel=d_rel,
    vLeadK=v_lead_k,
    aLeadK=a_lead_k,
    aLeadTau=a_lead_tau,
    radarTrackId=radar_track_id,
  )


def make_radar(lead_one=None, lead_two=None):
  return SimpleNamespace(leadOne=lead_one or make_lead(), leadTwo=lead_two or make_lead())


def make_governor(delay: float = 0.10):
  return AccelController(SimpleNamespace(longitudinalActuatorDelay=delay))


def update(governor, radar_state=None, **overrides):
  args = {
    "base_speed": 20.0,
    "v_ego": 20.0,
    "a_ego": 0.0,
    "profile": AccelProfile.normal,
    "follow_personality": log.LongitudinalPersonality.standard,
    "enabled": True,
    "acc_selected": True,
    "engaged": True,
    "cruise_initialized": True,
    "previous_mpc_source": LongitudinalPlanSource.cruise,
    "planner_speed": 20.0,
    "stock_accel_max": 2.0,
    "planner_accel": 0.0,
    "previous_should_stop": False,
  }
  args.update(overrides)
  return governor.update(radar_state or make_radar(), **args)


def assert_profile_trajectory(result, expected: float) -> None:
  assert result.mpc_accel_max is not None
  np.testing.assert_array_equal(result.mpc_accel_max, expected)


class TestAccelProfileLimits:
  def test_profile_table_matches_tuned_values(self):
    assert ACCEL_PROFILE_MAX_BP == [0.0, 10.0, 25.0, 40.0]
    assert ACCEL_PROFILE_MAX_V == {
      AccelProfile.eco: [1.55, 0.30, 0.20, 0.10],
      AccelProfile.normal: [1.70, 0.90, 0.40, 0.20],
      AccelProfile.sport: [2.00, 1.70, 1.20, 0.90],
    }

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_profile_accel_max_matches_lookup_table(self, profile):
    for speed, expected in zip(ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V[profile], strict=True):
      assert AccelController.get_profile_accel_max(profile, speed) == expected

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_profile_accel_max_interpolates_and_clamps(self, profile):
    expected_midpoint = (ACCEL_PROFILE_MAX_V[profile][1] + ACCEL_PROFILE_MAX_V[profile][2]) / 2.0

    assert AccelController.get_profile_accel_max(profile, 17.5) == pytest.approx(expected_midpoint)
    assert AccelController.get_profile_accel_max(profile, -1.0) == ACCEL_PROFILE_MAX_V[profile][0]
    assert AccelController.get_profile_accel_max(profile, 50.0) == ACCEL_PROFILE_MAX_V[profile][-1]

  @pytest.mark.parametrize("speed", ACCEL_PROFILE_MAX_BP)
  def test_profile_accel_max_order_is_distinct(self, speed):
    limits = [AccelController.get_profile_accel_max(profile, speed) for profile in AccelProfile]

    assert limits[AccelProfile.eco] < limits[AccelProfile.normal] < limits[AccelProfile.sport]

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_profile_table_stays_within_global_accel_limit(self, profile):
    for step in range(161):
      speed = step * 0.25
      assert 0.0 <= AccelController.get_profile_accel_max(profile, speed) <= ACCEL_MAX

  @pytest.mark.parametrize("profile", list(AccelProfile))
  @pytest.mark.parametrize("speed", ACCEL_PROFILE_MAX_BP)
  def test_stock_dynamic_output_limit_remains_authoritative(self, profile, speed):
    governor = make_governor()
    stock_limit = get_max_accel(speed)

    result = update(governor, profile=profile, v_ego=speed, planner_speed=speed, stock_accel_max=stock_limit)

    assert result.effective_accel_max <= stock_limit

  @pytest.mark.parametrize("speed", ACCEL_PROFILE_MAX_BP[1:])
  def test_effective_profiles_remain_distinct_below_stock_output_limit(self, speed):
    stock_limit = get_max_accel(speed)
    limits = []
    for profile in AccelProfile:
      governor = make_governor()
      result = update(governor, profile=profile, v_ego=speed, planner_speed=speed, stock_accel_max=stock_limit)
      limits.append(result.effective_accel_max)

    assert limits[AccelProfile.eco] < limits[AccelProfile.normal] < limits[AccelProfile.sport]

  def test_active_result_exposes_profile_accel_max(self):
    governor = make_governor()

    result = update(governor, profile=AccelProfile.eco, v_ego=17.5, planner_speed=17.5)

    assert result.profile_accel_max == pytest.approx(0.25)

  def test_clear_road_profile_is_a_separate_pre_mpc_trajectory(self):
    governor = make_governor()

    result = update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.4)

    assert result.mpc_accel_max is not None
    assert result.mpc_shape_cruise
    assert len(result.mpc_accel_max) == N + 1
    assert_profile_trajectory(result, 0.90)

  def test_ordinary_lead_keeps_profile_pre_mpc_accel_bound(self):
    governor = make_governor()
    radar_state = make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=15.0))

    result = update(governor, radar_state)

    assert result.active
    assert result.selected_lead == 0
    assert_profile_trajectory(result, result.profile_accel_max)
    assert result.mpc_shape_cruise

  def test_filtered_lead_history_keeps_profile_bound_through_two_dropouts(self):
    governor = make_governor()
    radar_state = make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=15.0))
    for _ in range(3):
      update(governor, radar_state)

    dropouts = [update(governor), update(governor)]

    assert all(math.isfinite(result.live_filtered_cap) for result in dropouts)
    assert all(result.mpc_accel_max is not None for result in dropouts)
    assert all(result.mpc_shape_cruise for result in dropouts)

  def test_stop_hold_pins_zero_target_with_coherent_zero_accel_horizon(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))

    held = update(governor, stopped, base_speed=5.0, v_ego=0.1, planner_speed=0.1)
    confirming = update(governor, moving, base_speed=5.0, v_ego=0.1, planner_speed=0.1)

    assert held.state == AccelControllerState.stopHold
    assert held.target_speed == 0.0
    assert held.effective_accel_max == 0.0
    assert_profile_trajectory(held, 0.0)
    assert held.mpc_shape_cruise
    assert held.lead_obstacle_weights == (0.0, 0.0)
    assert confirming.state == AccelControllerState.stopHold
    assert confirming.target_speed == 0.0
    assert confirming.effective_accel_max == 0.0
    assert_profile_trajectory(confirming, 0.0)
    assert confirming.mpc_shape_cruise
    assert confirming.lead_obstacle_weights == (0.0, 0.0)

  def test_stop_hold_keeps_raw_lead_above_vehicle_should_stop_threshold(self):
    governor = AccelController(SimpleNamespace(longitudinalActuatorDelay=0.10, vEgoStopping=0.25))
    stopped = make_radar(make_lead(status=True, d_rel=2.0, v_lead_k=0.0))

    held = update(governor, stopped, base_speed=5.0, v_ego=0.28, planner_speed=0.28)

    assert held.state == AccelControllerState.stopHold
    assert held.target_speed == 0.0
    assert held.lead_obstacle_weights == (1.0, 1.0)

  def test_normal_active_limits_are_bounded_by_stock_and_profile(self):
    governor = make_governor()

    result = update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.40)

    assert result.profile_accel_max == 0.90
    assert result.effective_accel_max == 0.90
    assert_profile_trajectory(result, 0.90)

  def test_first_enable_seeds_from_positive_planner_accel_within_stock(self):
    governor = make_governor()

    first = update(
      governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.30, planner_accel=1.20
    )
    second = update(
      governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.30, planner_accel=1.20
    )

    assert first.effective_accel_max == 1.20
    assert second.effective_accel_max == pytest.approx(first.effective_accel_max - ACCEL_LIMIT_JERK * DT_MDL)

  def test_first_enable_seed_preserves_current_plan_but_effective_limit_stays_stock_bounded(self):
    governor = make_governor()

    result = update(
      governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.10, planner_accel=1.80
    )

    assert result.effective_accel_max == 1.10
    assert_profile_trajectory(result, 1.80)

  def test_profile_switch_slews_at_one_meter_per_second_cubed(self):
    governor = make_governor()
    sport = update(governor, profile=AccelProfile.sport, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0)

    eco = update(governor, profile=AccelProfile.eco, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0)

    assert sport.effective_accel_max == 1.70
    assert eco.effective_accel_max == pytest.approx(sport.effective_accel_max - ACCEL_LIMIT_JERK * DT_MDL)
    assert eco.effective_accel_max > eco.profile_accel_max

  def test_dynamic_stock_tightening_does_not_enter_controller_comfort_state(self):
    governor = make_governor()
    update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.40)

    tightened = update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=0.40)
    released = update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.40)

    assert tightened.effective_accel_max == 0.40
    assert_profile_trajectory(tightened, 0.90)
    assert released.effective_accel_max == 0.90
    assert_profile_trajectory(released, 0.90)

  def test_negative_stock_max_remains_authoritative_outside_the_mpc_profile_bound(self):
    governor = make_governor()

    result = update(governor, v_ego=10.0, planner_speed=10.0, stock_accel_max=-0.20, planner_accel=1.0)

    assert result.effective_accel_max == -0.20
    assert_profile_trajectory(result, 1.0)

  def test_profile_tightening_can_converge_below_positive_planner_accel(self):
    governor = make_governor()
    update(
      governor, profile=AccelProfile.sport, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0, planner_accel=1.15
    )

    results = [
      update(governor, profile=AccelProfile.eco, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0, planner_accel=1.15)
      for _ in range(30)
    ]

    assert results[-1].effective_accel_max == pytest.approx(ACCEL_PROFILE_MAX_V[AccelProfile.eco][1])
    assert results[-1].effective_accel_max < 1.15
    assert all(result.mpc_accel_max is not None for result in results)

  @pytest.mark.parametrize("bypass", [{"enabled": False}, {"acc_selected": False}])
  def test_bypass_does_not_expose_an_active_accel_limit(self, bypass):
    governor = make_governor()

    result = update(governor, **bypass)

    assert math.isinf(result.profile_accel_max)
    assert math.isinf(result.effective_accel_max)
    assert result.mpc_accel_max is None
    assert not result.mpc_shape_cruise

  @pytest.mark.parametrize("invalid", [{"stock_accel_max": math.nan}, {"planner_accel": math.nan}])
  def test_invalid_accel_input_bypasses_and_resets_limits(self, invalid):
    governor = make_governor()
    update(governor)

    result = update(governor, **invalid)

    assert not result.active
    assert governor.live.accel_limit is None
    assert math.isinf(result.effective_accel_max)
    assert not result.mpc_shape_cruise


class TestLeadObstacleAcquisition:
  benign_lead = make_lead(status=True, d_rel=126.0, v_lead_k=17.0, radar_track_id=7)

  def test_lead_already_present_at_enable_has_full_authority(self):
    governor = make_governor()

    result = update(governor, make_radar(self.benign_lead), base_speed=30.0, v_ego=20.0, planner_speed=20.0)

    assert result.lead_obstacle_weights == (1.0, 1.0)

  def test_benign_new_lead_reaches_full_authority_in_point_three_seconds(self):
    governor = make_governor()
    args = {"base_speed": 30.0, "v_ego": 20.0, "planner_speed": 20.0}
    update(governor, **args)

    results = [update(governor, make_radar(self.benign_lead), **args) for _ in range(7)]

    expected = np.linspace(0.2, 1.0, 7)
    np.testing.assert_allclose([result.lead_obstacle_weights[0] for result in results], expected, atol=1e-12, rtol=0.0)
    assert all(result.mpc_accel_max is not None for result in results)

  def test_route_shaped_urgent_acquisition_is_immediate(self):
    governor = make_governor()
    args = {"base_speed": 40.0, "v_ego": 34.8, "planner_speed": 34.8}
    update(governor, **args)
    route_lead = make_lead(status=True, d_rel=93.6, v_lead_k=23.4, radar_track_id=22)

    result = update(governor, make_radar(route_lead), **args)

    assert result.required_decel > 1.0
    assert result.lead_obstacle_weights == (1.0, 1.0)

  @pytest.mark.parametrize(
    "lead",
    [
      make_lead(status=True, d_rel=25.0, v_lead_k=0.0),
      make_lead(status=True, d_rel=126.0, v_lead_k=17.0, a_lead_k=-0.6),
    ],
  )
  def test_close_or_braking_new_lead_is_immediate(self, lead):
    governor = make_governor()
    args = {"base_speed": 30.0, "v_ego": 20.0, "planner_speed": 20.0}
    update(governor, **args)

    result = update(governor, make_radar(lead), **args)

    assert result.lead_obstacle_weights == (1.0, 1.0)

  def test_dropout_discards_authority_state_and_reacquisition_starts_fresh(self):
    governor = make_governor()
    args = {"base_speed": 30.0, "v_ego": 20.0, "planner_speed": 20.0}
    update(governor, **args)
    first = update(governor, make_radar(self.benign_lead), **args)
    dropout = update(governor, **args)
    reacquired = update(governor, make_radar(self.benign_lead), **args)

    assert first.lead_obstacle_weights[0] == pytest.approx(0.2)
    assert dropout.lead_obstacle_weights == (1.0, 1.0)
    assert reacquired.lead_obstacle_weights[0] == pytest.approx(0.2)

  @pytest.mark.parametrize("bypass", [{"enabled": False}, {"acc_selected": False}])
  def test_non_actuating_mode_always_requests_raw_lead_authority(self, bypass):
    governor = make_governor()
    update(governor)

    result = update(governor, make_radar(self.benign_lead), **bypass)

    assert result.lead_obstacle_weights == (1.0, 1.0)


class TestEnergyEnvelope:
  def test_correct_relative_energy_formula_and_lead_selection(self):
    governor = make_governor()
    lead_one = make_lead(status=True, d_rel=60.0, v_lead_k=10.0)
    lead_two = make_lead(status=True, d_rel=100.0, v_lead_k=15.0)
    radar_state = make_radar(lead_one, lead_two)

    envelope = governor.calculate_energy_envelope(radar_state, 20.0, 0.0, AccelProfile.normal)

    delay = governor.CP.longitudinalActuatorDelay + DT_MDL
    x_ego = 20.0 * delay
    x_lead = lead_one.dRel + lead_one.vLeadK * delay
    usable_gap = x_lead - x_ego - STOP_DISTANCE - get_T_FOLLOW() * lead_one.vLeadK
    anticipated_gap = max(usable_gap - (20.0 - lead_one.vLeadK) * RELATIVE_PACE_PREVIEW_TIME, 0.0)
    expected = lead_one.vLeadK + math.sqrt(2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * anticipated_gap)
    incorrect_fixed_target_formula = math.sqrt(
      lead_one.vLeadK**2 + 2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * anticipated_gap,
    )

    assert envelope.selected_lead == 0
    assert envelope.usable_gap == pytest.approx(usable_gap)
    assert envelope.cap == pytest.approx(expected)
    assert envelope.cap != pytest.approx(incorrect_fixed_target_formula)

  def test_lead_two_can_be_more_restrictive(self):
    governor = make_governor()
    lead_one = make_lead(status=True, d_rel=100.0, v_lead_k=18.0)
    lead_two = make_lead(status=True, d_rel=35.0, v_lead_k=5.0)

    envelope = governor.calculate_energy_envelope(make_radar(lead_one, lead_two), 20.0, 0.0, AccelProfile.normal)

    assert envelope.selected_lead == 1
    assert envelope.cap < 10.0

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_profile_uses_its_comfort_deceleration(self, profile):
    governor = make_governor()
    radar_state = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0))

    envelope = governor.calculate_energy_envelope(radar_state, 20.0, 0.0, profile)
    anticipated_gap = max(envelope.usable_gap - envelope.closing_speed * RELATIVE_PACE_PREVIEW_TIME, 0.0)
    expected = 10.0 + math.sqrt(2.0 * PROFILE_CONFIGS[profile].comfort_decel * anticipated_gap)

    assert envelope.cap == pytest.approx(expected)

  def test_profile_caps_order_eco_normal_sport(self):
    governor = make_governor()
    radar_state = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0))

    caps = [governor.calculate_energy_envelope(radar_state, 20.0, 0.0, profile).cap for profile in AccelProfile]

    assert caps[AccelProfile.eco] < caps[AccelProfile.normal] < caps[AccelProfile.sport]

  def test_stock_follow_personality_is_independent(self):
    governor = make_governor()
    radar_state = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0))

    aggressive = governor.calculate_energy_envelope(radar_state, 20.0, 0.0, AccelProfile.normal, log.LongitudinalPersonality.aggressive)
    relaxed = governor.calculate_energy_envelope(radar_state, 20.0, 0.0, AccelProfile.normal, log.LongitudinalPersonality.relaxed)

    assert relaxed.usable_gap < aggressive.usable_gap
    assert relaxed.cap < aggressive.cap

  def test_lead_acceleration_is_clipped_before_extrapolation(self):
    governor = make_governor(delay=0.30)
    extreme = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0, a_lead_k=-100.0))
    clipped = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0, a_lead_k=-10.0))

    extreme_envelope = governor.calculate_energy_envelope(extreme, 20.0, 0.0, AccelProfile.normal)
    clipped_envelope = governor.calculate_energy_envelope(clipped, 20.0, 0.0, AccelProfile.normal)

    assert extreme_envelope == clipped_envelope

  def test_ego_projection_stops_at_zero_velocity(self):
    x_ego, v_ego = AccelController._project_ego(0.2, -4.0, 0.15)

    assert x_ego == pytest.approx(0.005)
    assert v_ego == 0.0

  def test_invalid_lead_is_ignored(self):
    governor = make_governor()
    radar_state = make_radar(make_lead(status=True, d_rel=math.nan, v_lead_k=10.0))

    envelope = governor.calculate_energy_envelope(radar_state, 20.0, 0.0, AccelProfile.normal)

    assert math.isinf(envelope.cap)
    assert envelope.selected_lead == -1


class TestAccelControllerState:
  restrictive_lead = make_lead(status=True, d_rel=40.0, v_lead_k=5.0)

  @pytest.mark.parametrize("v_ego", [4.25, 9.39])
  def test_clear_road_rolling_engagement_immediately_targets_base_speed(self, v_ego):
    governor = make_governor()

    result = update(governor, base_speed=20.0, v_ego=v_ego, planner_speed=v_ego)

    assert result.state == AccelControllerState.free
    assert result.live_pace == result.base_speed
    assert result.target_speed == result.base_speed
    assert not result.launching

  def test_five_frame_median_requires_three_observations_and_holds_two_dropouts(self):
    governor = make_governor()
    restrictive_radar = make_radar(self.restrictive_lead)

    first = update(governor, restrictive_radar)
    second = update(governor, restrictive_radar)
    third = update(governor, restrictive_radar)

    assert math.isinf(first.live_filtered_cap)
    assert math.isinf(second.live_filtered_cap)
    assert math.isfinite(third.live_filtered_cap)

    dropout_one = update(governor)
    dropout_two = update(governor)
    dropout_three = update(governor)

    assert math.isfinite(dropout_one.live_filtered_cap)
    assert math.isfinite(dropout_two.live_filtered_cap)
    assert math.isinf(dropout_three.live_filtered_cap)

  def test_restriction_is_limited_by_profile_deceleration(self):
    governor = make_governor()
    # Restrictive enough to start early comfort shaping, but below the urgent
    # stock-MPC bypass threshold.
    radar_state = make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=10.0))

    update(governor, radar_state)
    update(governor, radar_state)
    first_restriction = update(governor, radar_state)
    next_restriction = update(governor, radar_state)

    expected_step = PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * DT_MDL
    assert first_restriction.live_pace == pytest.approx(20.0 - expected_step)
    assert next_restriction.live_pace == pytest.approx(first_restriction.live_pace - expected_step)
    assert next_restriction.state == AccelControllerState.restrict
    initial_limit = AccelController.get_profile_accel_max(AccelProfile.normal, 20.0)
    assert_profile_trajectory(first_restriction, initial_limit - DECEL_LIMIT_JERK * DT_MDL)
    assert_profile_trajectory(next_restriction, first_restriction.mpc_accel_max[0] - DECEL_LIMIT_JERK * DT_MDL)

  def test_urgent_closing_bypasses_comfort_shaping_for_stock_mpc(self):
    governor = make_governor()
    result = update(governor, make_radar(self.restrictive_lead))

    assert result.required_decel > URGENT_BYPASS_REQUIRED_DECEL
    assert result.target_speed == result.base_speed
    assert result.mpc_accel_max is None
    assert result.lead_obstacle_weights == (1.0, 1.0)

  def test_release_waits_for_confirmation_then_uses_profile_rate(self):
    governor = make_governor()
    radar_state = make_radar(self.restrictive_lead)
    for _ in range(30):
      update(governor, radar_state)

    result = update(governor)
    while math.isfinite(result.live_filtered_cap):
      result = update(governor)

    held_pace = result.live_pace
    assert result.state == AccelControllerState.hold

    confirmation_updates = 0
    while result.state != AccelControllerState.release:
      assert result.live_pace == held_pace
      result = update(governor)
      confirmation_updates += 1
      assert confirmation_updates < 20

    assert confirmation_updates >= 6
    expected_rate = PROFILE_CONFIGS[AccelProfile.normal].release_rate
    assert result.live_pace == pytest.approx(held_pace + expected_rate * DT_MDL)

  def test_live_state_never_adopts_shadow_history(self):
    governor = make_governor()
    radar_state = make_radar(self.restrictive_lead)
    for _ in range(20):
      active = update(governor, radar_state)
    assert active.live_pace < 20.0

    shadow_only = update(governor, radar_state, acc_selected=False)
    assert shadow_only.target_speed == 20.0
    assert shadow_only.state == AccelControllerState.inactive
    assert math.isinf(shadow_only.live_pace)
    assert shadow_only.shadow_pace < active.shadow_pace

    reactivated = update(governor, radar_state)
    assert reactivated.live_pace == 20.0
    assert reactivated.target_speed == 20.0
    assert reactivated.shadow_pace < 20.0

  def test_previous_lead_plan_synchronizes_pace_downward(self):
    governor = make_governor()
    update(governor, base_speed=30.0, v_ego=20.0, planner_speed=20.0)

    result = update(governor, base_speed=30.0, v_ego=20.0, planner_speed=15.0, previous_mpc_source=LongitudinalPlanSource.lead0)

    assert result.live_pace == 15.0
    assert result.target_speed == 15.0

  def test_stop_hold_requires_four_confirmed_departure_frames(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    stop_args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.1}

    for _ in range(3):
      result = update(governor, stopped, **stop_args)
      assert result.state == AccelControllerState.stopHold
      assert result.live_pace == 0.0
      assert result.target_speed == 0.0
      assert result.effective_accel_max == 0.0
      assert_profile_trajectory(result, 0.0)
      assert result.lead_obstacle_weights == (0.0, 0.0)

    for _ in range(3):
      result = update(governor, moving, **stop_args)
      assert result.state == AccelControllerState.stopHold
      assert not result.launching
      assert result.live_pace == 0.0
      assert result.target_speed == 0.0
      assert result.effective_accel_max == 0.0
      assert_profile_trajectory(result, 0.0)
      assert result.lead_obstacle_weights == (0.0, 0.0)

    departed = update(governor, moving, **stop_args)
    assert departed.state == AccelControllerState.release
    assert departed.launching
    assert departed.live_pace == pytest.approx(stop_args["v_ego"] + LAUNCH_DELTA_V)
    assert departed.target_speed == stop_args["base_speed"]
    assert departed.effective_accel_max == pytest.approx(LAUNCH_ACCEL_RATE * DT_MDL)
    assert_profile_trajectory(departed, departed.effective_accel_max)
    assert departed.lead_obstacle_weights == (0.0, 1.0)

  def test_far_irrelevant_stopped_lead_does_not_block_departure(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    mixed = make_radar(
      make_lead(status=True, d_rel=20.0, v_lead_k=5.0),
      make_lead(status=True, d_rel=200.0, v_lead_k=0.0),
    )
    args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.0}
    update(governor, stopped, **args)

    results = [update(governor, mixed, **args) for _ in range(4)]

    assert all(result.raw_energy_cap > 0.8 for result in results)
    assert all(result.state == AccelControllerState.stopHold for result in results[:3])
    assert all(not result.launching for result in results[:3])
    assert results[3].state == AccelControllerState.release
    assert results[3].launching
    assert governor.live.departure_frames == 0

  def test_stock_relevant_stopped_lead_two_blocks_departure(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    mixed = make_radar(
      make_lead(status=True, d_rel=20.0, v_lead_k=5.0),
      make_lead(status=True, d_rel=5.0, v_lead_k=0.0),
    )
    args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.0}
    update(governor, stopped, **args)

    results = [update(governor, mixed, **args) for _ in range(5)]

    assert all(result.state == AccelControllerState.stopHold for result in results)
    assert all(not result.launching for result in results)
    assert governor.live.departure_frames == 0

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_confirmed_departure_ramps_to_common_breakaway_then_profiles(self, profile):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.0, "stock_accel_max": 2.0, "profile": profile}

    for _ in range(3):
      update(governor, stopped, **args)
    departure = [update(governor, moving, **args) for _ in range(4)]

    assert [result.target_speed for result in departure[:3]] == [0.0] * 3
    assert all(result.effective_accel_max == 0.0 for result in departure[:3])
    assert all(result.mpc_accel_max is not None for result in departure[:3])
    expected_launch_pace = min(args["base_speed"], departure[-1].live_filtered_cap, args["v_ego"] + LAUNCH_DELTA_V)
    assert departure[-1].live_pace == pytest.approx(expected_launch_pace)
    assert departure[-1].target_speed == args["base_speed"]
    assert departure[-1].effective_accel_max == pytest.approx(LAUNCH_ACCEL_RATE * DT_MDL)
    assert_profile_trajectory(departure[-1], departure[-1].effective_accel_max)

    still_breaking_away = update(governor, moving, **(args | {"v_ego": 0.04, "planner_speed": 0.04}))
    assert still_breaking_away.launching
    assert still_breaking_away.target_speed == args["base_speed"]
    assert still_breaking_away.effective_accel_max == pytest.approx(2.0 * LAUNCH_ACCEL_RATE * DT_MDL)
    assert_profile_trajectory(still_breaking_away, still_breaking_away.effective_accel_max)

    moving_result = update(governor, moving, **(args | {"v_ego": 0.05, "planner_speed": 0.05}))
    assert not moving_result.launching
    assert moving_result.mpc_accel_max is not None
    assert moving_result.mpc_shape_cruise

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_departure_confirmation_uses_controlling_lead_speed_not_energy_cap(self, profile):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    barely_moving = make_radar(make_lead(status=True, d_rel=5.0, v_lead_k=0.35))
    args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.0, "profile": profile}
    update(governor, stopped, **args)

    confirmation = [update(governor, barely_moving, **args) for _ in range(4)]

    assert all(result.raw_energy_cap < 0.8 for result in confirmation)
    assert all(result.state == AccelControllerState.stopHold for result in confirmation[:3])
    assert confirmation[3].state == AccelControllerState.release
    assert confirmation[3].launching

  def test_departure_confirmation_must_be_four_consecutive_frames(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.0}
    update(governor, stopped, **args)

    assert all(update(governor, moving, **args).state == AccelControllerState.stopHold for _ in range(2))
    interrupted = update(governor, stopped, **args)
    assert interrupted.state == AccelControllerState.stopHold
    assert governor.live.departure_frames == 0

    confirmation = [update(governor, moving, **args) for _ in range(4)]
    assert all(result.state == AccelControllerState.stopHold for result in confirmation[:3])
    assert confirmation[3].state == AccelControllerState.release

  def test_stopped_lead_departure_releases_while_mpc_source_remains_lead(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    lead_args = {
      "base_speed": 5.0,
      "v_ego": 0.1,
      "planner_speed": 0.0,
      "previous_mpc_source": LongitudinalPlanSource.lead0,
    }

    for _ in range(3):
      update(governor, stopped, **lead_args)

    departure = [update(governor, moving, **lead_args) for _ in range(4)]
    assert [result.live_pace for result in departure[:3]] == [0.0] * 3
    assert departure[3].live_pace > 0.0
    assert [result.target_speed for result in departure[:3]] == [0.0] * 3
    assert departure[-1].target_speed == lead_args["base_speed"]
    assert len(departure) * DT_MDL < 1.0

    continued_release = update(governor, moving, **lead_args)
    assert continued_release.state == AccelControllerState.release
    assert continued_release.live_pace > departure[-1].live_pace

  def test_stale_should_stop_does_not_restart_departure_confirmation(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    stale_stop_args = {
      "base_speed": 5.0,
      "v_ego": 0.1,
      "planner_speed": 0.0,
      "previous_mpc_source": LongitudinalPlanSource.lead0,
      "previous_should_stop": True,
    }

    for _ in range(3):
      update(governor, stopped, **stale_stop_args)

    departure = [update(governor, moving, **stale_stop_args) for _ in range(4)]
    assert [result.live_pace for result in departure[:3]] == [0.0] * 3
    assert departure[3].live_pace > 0.0
    assert len(departure) * DT_MDL < 1.0

    continued_paces = [update(governor, moving, **stale_stop_args).live_pace for _ in range(60)]
    assert all(current >= previous for previous, current in zip(continued_paces[:-1], continued_paces[1:], strict=True))
    assert continued_paces[-1] > departure[-1].live_pace

  def test_renewed_stopped_lead_interrupts_confirmed_departure(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    stale_stop_args = {
      "base_speed": 5.0,
      "v_ego": 0.1,
      "planner_speed": 0.0,
      "previous_mpc_source": LongitudinalPlanSource.lead0,
      "previous_should_stop": True,
    }

    for _ in range(3):
      update(governor, stopped, **stale_stop_args)
    for _ in range(8):
      departing = update(governor, moving, **stale_stop_args)
    assert departing.target_speed > 0.0

    renewed_stop = update(governor, stopped, **stale_stop_args)
    assert renewed_stop.state == AccelControllerState.stopHold
    assert renewed_stop.live_pace == 0.0
    assert renewed_stop.target_speed == 0.0
    assert renewed_stop.effective_accel_max == 0.0
    assert_profile_trajectory(renewed_stop, 0.0)
    assert not governor.live.departing_from_stop

  def test_low_speed_moving_lead_never_bypasses_bounded_pace(self):
    governor = make_governor()
    noisy_moving_lead = make_radar(make_lead(status=True, d_rel=10.0, v_lead_k=1.5))

    first = update(governor, noisy_moving_lead, base_speed=5.0, v_ego=0.0, planner_speed=0.0)
    second = update(governor, noisy_moving_lead, base_speed=5.0, v_ego=0.0, planner_speed=0.0)

    assert first.selected_lead == 0
    assert first.live_pace == 0.0
    assert first.target_speed == first.live_pace
    assert not governor.live.stopped_lead_hold
    assert second.target_speed == second.live_pace
    assert second.target_speed < second.base_speed

  def test_real_stopped_evidence_latches_hold_after_noisy_first_frame(self):
    governor = make_governor()
    noisy_moving_lead = make_radar(make_lead(status=True, d_rel=10.0, v_lead_k=1.5))
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    args = {"base_speed": 5.0, "v_ego": 0.0, "planner_speed": 0.0}

    initial_noise = update(governor, noisy_moving_lead, **args)
    stopped_evidence = update(governor, stopped, **args)
    repeated_noise = update(governor, noisy_moving_lead, **args)

    assert initial_noise.target_speed == initial_noise.live_pace
    assert stopped_evidence.state == AccelControllerState.stopHold
    assert governor.live.stopped_lead_hold
    assert stopped_evidence.target_speed == 0.0
    assert repeated_noise.target_speed == 0.0
    assert_profile_trajectory(stopped_evidence, 0.0)
    assert_profile_trajectory(repeated_noise, 0.0)

  def test_later_continuously_moving_lead_does_not_latch_stopped_hold(self):
    governor = make_governor()
    moving_lead = make_radar(make_lead(status=True, d_rel=10.0, v_lead_k=1.5))
    update(governor, base_speed=5.0, v_ego=1.0, planner_speed=1.0)

    observations = [update(governor, moving_lead, base_speed=5.0, v_ego=0.0, planner_speed=0.0) for _ in range(3)]
    settled = observations[-1]

    assert settled.selected_lead == 0
    assert not governor.live.stopped_lead_hold
    assert settled.target_speed == settled.live_pace
    assert observations[0].target_speed == observations[0].base_speed
    assert settled.target_speed < settled.base_speed

  def test_stop_hold_dropout_pins_target_without_losing_hold_state(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    stop_args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.1}
    for _ in range(3):
      update(governor, stopped, **stop_args)

    dropout = update(governor, **stop_args)

    assert dropout.selected_lead == -1
    assert dropout.state == AccelControllerState.stopHold
    assert dropout.live_pace == 0.0
    assert dropout.target_speed == 0.0
    assert_profile_trajectory(dropout, 0.0)
    assert governor.live.stopped_lead_hold

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_no_lead_start_uses_solver_safe_seed_then_common_breakaway_floor(self, profile):
    governor = make_governor()

    args = {"base_speed": 5.0, "v_ego": 0.0, "planner_speed": 0.0, "profile": profile}
    first = update(governor, **args)
    second = update(governor, **args)
    third = update(governor, **args)

    assert first.selected_lead == -1
    assert first.launching
    assert first.live_pace == first.base_speed
    assert first.target_speed == first.base_speed
    assert first.effective_accel_max == INITIAL_LAUNCH_ACCEL_MAX
    assert second.effective_accel_max == pytest.approx(INITIAL_LAUNCH_ACCEL_MAX + CLEAR_LAUNCH_ACCEL_RATE * DT_MDL)
    assert third.effective_accel_max == BREAKAWAY_ACCEL_MAX
    assert_profile_trajectory(first, INITIAL_LAUNCH_ACCEL_MAX)
    assert_profile_trajectory(third, BREAKAWAY_ACCEL_MAX)

  def test_confirmed_departure_has_no_later_pace_jump(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    lead_args = {
      "base_speed": 5.0,
      "v_ego": 0.04,
      "planner_speed": 0.0,
      "previous_mpc_source": LongitudinalPlanSource.lead0,
      "previous_should_stop": True,
    }
    for _ in range(3):
      update(governor, stopped, **lead_args)
    for _ in range(8):
      departing = update(governor, moving, **lead_args)
    assert governor.live.departing_from_stop

    handed_back = update(governor, moving, **(lead_args | {"v_ego": 0.05, "planner_speed": 0.05}))

    assert not governor.live.departing_from_stop
    assert not governor.live.stopped_lead_hold
    expected_step = PROFILE_CONFIGS[AccelProfile.normal].release_rate * DT_MDL
    assert handed_back.live_pace == pytest.approx(departing.live_pace + expected_step)
    assert handed_back.live_pace < min(handed_back.base_speed, handed_back.live_filtered_cap)
    assert handed_back.target_speed == handed_back.live_pace
    assert handed_back.mpc_accel_max is not None
    assert handed_back.mpc_shape_cruise

  @pytest.mark.parametrize(
    "bypass",
    [
      {"enabled": False},
      {"acc_selected": False},
      {"engaged": False},
      {"cruise_initialized": False},
      {"controller_fault": True},
      {"a_ego": math.nan},
    ],
  )
  def test_bypass_returns_base_and_resets_live(self, bypass):
    governor = make_governor()
    radar_state = make_radar(self.restrictive_lead)
    for _ in range(5):
      update(governor, radar_state)

    result = update(governor, radar_state, **bypass)

    assert result.target_speed == 20.0
    assert not result.active
    assert result.state == AccelControllerState.inactive
    assert math.isinf(result.live_pace)
    assert governor.live.accel_limit is None
    assert math.isinf(result.effective_accel_max)
    assert result.mpc_accel_max is None
    assert not result.mpc_shape_cruise

  def test_disabled_acc_mode_keeps_shadow_running(self):
    governor = make_governor()
    radar_state = make_radar(self.restrictive_lead)

    results = [update(governor, radar_state, enabled=False) for _ in range(3)]

    assert all(not result.active for result in results)
    assert all(result.shadow_active for result in results)
    assert results[-1].shadow_state == AccelControllerState.restrict
    assert math.isfinite(results[-1].shadow_filtered_cap)

  def test_invalid_profile_defaults_to_normal(self):
    governor = make_governor()

    result = update(governor, profile=99)

    assert result.profile == AccelProfile.normal

  def test_small_negative_ego_speed_is_sanitized_without_resetting_state(self):
    governor = make_governor()
    update(governor)
    cap_samples = governor.live.cap_samples

    result = update(governor, v_ego=-0.04, planner_speed=0.0)

    assert result.active
    assert math.isfinite(result.live_pace)
    assert governor.live.cap_samples is cap_samples

  def test_negative_ego_speed_below_noise_tolerance_resets_live_state(self):
    governor = make_governor()
    update(governor)
    cap_samples = governor.live.cap_samples

    result = update(governor, v_ego=-0.101, planner_speed=0.0)

    assert not result.active
    assert math.isinf(result.live_pace)
    assert governor.live.cap_samples is not cap_samples

  def test_invalid_delay_resets_and_bypasses(self):
    governor = AccelController(SimpleNamespace(longitudinalActuatorDelay=None))

    result = update(governor)

    assert result.target_speed == 20.0
    assert not result.active
    assert not result.shadow_active

  def test_nonfinite_base_is_preserved_on_bypass(self):
    governor = make_governor()

    result = update(governor, base_speed=math.nan)

    assert math.isnan(result.target_speed)
    assert not result.active

  def test_radar_input_is_not_mutated(self):
    governor = make_governor()
    lead = make_lead(status=True, d_rel=50.0, v_lead_k=10.0, a_lead_k=-2.0, a_lead_tau=1.2)
    radar_state = make_radar(lead)
    before = (lead.status, lead.dRel, lead.vLeadK, lead.aLeadK, lead.aLeadTau)

    update(governor, radar_state)

    assert (lead.status, lead.dRel, lead.vLeadK, lead.aLeadK, lead.aLeadTau) == before
