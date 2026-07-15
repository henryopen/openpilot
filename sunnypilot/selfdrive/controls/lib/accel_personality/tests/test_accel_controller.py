#!/usr/bin/env python3
import math
from types import SimpleNamespace

import pytest

from cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource, STOP_DISTANCE, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import (
  ACCEL_LIMIT_JERK,
  ACCEL_PROFILE_MAX_BP,
  ACCEL_PROFILE_MAX_V,
  LAUNCH_ACCEL_JERK,
  LAUNCH_DELTA_V,
  AccelController,
  AccelControllerState,
  AccelProfile,
  PROFILE_CONFIGS,
)


def make_lead(*, status: bool = False, d_rel: float = 0.0, v_lead_k: float = 0.0, a_lead_k: float = 0.0, a_lead_tau: float = 1.5):
  return SimpleNamespace(status=status, dRel=d_rel, vLeadK=v_lead_k, aLeadK=a_lead_k, aLeadTau=a_lead_tau)


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


class TestAccelProfileLimits:
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
  def test_profile_table_never_exceeds_stock_speed_limit(self, profile):
    for step in range(161):
      speed = step * 0.25
      assert AccelController.get_profile_accel_max(profile, speed) <= get_max_accel(speed)

  def test_active_result_exposes_profile_accel_max(self):
    governor = make_governor()

    result = update(governor, profile=AccelProfile.eco, v_ego=17.5, planner_speed=17.5)

    assert result.profile_accel_max == pytest.approx(0.60)

  def test_normal_active_limits_are_bounded_by_stock_and_profile(self):
    governor = make_governor()

    result = update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.40)

    assert result.profile_accel_max == 1.0
    assert result.effective_accel_max == 1.0
    assert result.output_accel_max == 1.0
    assert result.mpc_cruise_accel_max == 1.0

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
    assert result.output_accel_max == 1.80

  def test_profile_switch_slews_at_one_meter_per_second_cubed(self):
    governor = make_governor()
    sport = update(governor, profile=AccelProfile.sport, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0)

    eco = update(governor, profile=AccelProfile.eco, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0)

    assert sport.effective_accel_max == 1.15
    assert eco.effective_accel_max == pytest.approx(sport.effective_accel_max - ACCEL_LIMIT_JERK * DT_MDL)
    assert eco.effective_accel_max > eco.profile_accel_max

  def test_dynamic_stock_tightening_does_not_enter_controller_comfort_state(self):
    governor = make_governor()
    update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.40)

    tightened = update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=0.40)
    released = update(governor, profile=AccelProfile.normal, v_ego=10.0, planner_speed=10.0, stock_accel_max=1.40)

    assert tightened.effective_accel_max == 0.40
    assert tightened.output_accel_max == 1.0
    assert released.effective_accel_max == 1.0
    assert released.output_accel_max == 1.0

  def test_negative_stock_max_never_becomes_positive(self):
    governor = make_governor()

    result = update(governor, v_ego=10.0, planner_speed=10.0, stock_accel_max=-0.20, planner_accel=1.0)

    assert result.effective_accel_max == -0.20
    assert result.output_accel_max == 1.0

  def test_profile_tightening_can_converge_below_positive_planner_accel(self):
    governor = make_governor()
    update(
      governor, profile=AccelProfile.sport, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0, planner_accel=1.15
    )

    results = [
      update(governor, profile=AccelProfile.eco, v_ego=10.0, planner_speed=10.0, stock_accel_max=2.0, planner_accel=1.15)
      for _ in range(8)
    ]

    assert results[-1].output_accel_max == pytest.approx(ACCEL_PROFILE_MAX_V[AccelProfile.eco][1])
    assert results[-1].output_accel_max < 1.15
    assert all(math.isfinite(result.output_accel_max) for result in results)

  @pytest.mark.parametrize("bypass", [{"enabled": False}, {"acc_selected": False}])
  def test_bypass_does_not_expose_an_active_accel_limit(self, bypass):
    governor = make_governor()

    result = update(governor, **bypass)

    assert math.isinf(result.profile_accel_max)
    assert math.isinf(result.effective_accel_max)
    assert math.isinf(result.output_accel_max)
    assert math.isinf(result.mpc_cruise_accel_max)

  @pytest.mark.parametrize("invalid", [{"stock_accel_max": math.nan}, {"planner_accel": math.nan}])
  def test_invalid_accel_input_bypasses_and_resets_limits(self, invalid):
    governor = make_governor()
    update(governor)

    result = update(governor, **invalid)

    assert not result.active
    assert governor.live.accel_limit is None
    assert math.isinf(result.effective_accel_max)


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
    expected = lead_one.vLeadK + math.sqrt(2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * usable_gap)
    incorrect_fixed_target_formula = math.sqrt(lead_one.vLeadK**2 + 2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * usable_gap)

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
    expected = 10.0 + math.sqrt(2.0 * PROFILE_CONFIGS[profile].comfort_decel * envelope.usable_gap)

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
    radar_state = make_radar(self.restrictive_lead)

    update(governor, radar_state)
    update(governor, radar_state)
    first_restriction = update(governor, radar_state)
    next_restriction = update(governor, radar_state)

    expected_step = PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * DT_MDL
    assert first_restriction.live_pace == pytest.approx(20.0 - expected_step)
    assert next_restriction.live_pace == pytest.approx(first_restriction.live_pace - expected_step)
    assert next_restriction.state == AccelControllerState.restrict

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
    expected_rate = AccelController.get_profile_accel_max(AccelProfile.normal, 20.0)
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
      assert result.target_speed == stop_args["base_speed"]
      assert result.effective_accel_max == 0.0
      assert result.output_accel_max == 0.0
      assert math.isinf(result.mpc_cruise_accel_max)

    for _ in range(3):
      result = update(governor, moving, **stop_args)
      assert result.state == AccelControllerState.stopHold
      assert not result.launching
      assert result.live_pace == 0.0
      assert result.target_speed == stop_args["base_speed"]
      assert result.effective_accel_max == 0.0
      assert result.output_accel_max == 0.0
      assert math.isinf(result.mpc_cruise_accel_max)

    departed = update(governor, moving, **stop_args)
    assert departed.state == AccelControllerState.release
    assert departed.launching
    assert departed.live_pace == pytest.approx(stop_args["v_ego"] + LAUNCH_DELTA_V)
    assert departed.target_speed == stop_args["base_speed"]
    assert departed.effective_accel_max == pytest.approx(LAUNCH_ACCEL_JERK * DT_MDL)
    assert departed.output_accel_max == departed.effective_accel_max
    assert math.isinf(departed.mpc_cruise_accel_max)

  def test_second_nearly_stopped_lead_blocks_departure_confirmation(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    mixed = make_radar(
      make_lead(status=True, d_rel=20.0, v_lead_k=5.0),
      make_lead(status=True, d_rel=200.0, v_lead_k=0.0),
    )
    args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.0}
    update(governor, stopped, **args)

    results = [update(governor, mixed, **args) for _ in range(5)]

    assert all(result.raw_energy_cap > 0.8 for result in results)
    assert all(result.state == AccelControllerState.stopHold for result in results)
    assert all(not result.launching for result in results)
    assert governor.live.departure_frames == 0

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_confirmed_departure_launch_is_immediate_bounded_and_profiled(self, profile):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    args = {"base_speed": 5.0, "v_ego": 0.1, "planner_speed": 0.0, "stock_accel_max": 2.0, "profile": profile}

    for _ in range(3):
      update(governor, stopped, **args)
    departure = [update(governor, moving, **args) for _ in range(4)]

    assert [result.target_speed for result in departure[:3]] == [args["base_speed"]] * 3
    expected_launch_pace = min(args["base_speed"], departure[-1].live_filtered_cap, args["v_ego"] + LAUNCH_DELTA_V)
    assert departure[-1].live_pace == pytest.approx(expected_launch_pace)
    assert departure[-1].target_speed == args["base_speed"]
    assert departure[-1].effective_accel_max == pytest.approx(LAUNCH_ACCEL_JERK * DT_MDL)
    assert departure[-1].output_accel_max == departure[-1].effective_accel_max

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
    assert [result.target_speed for result in departure[:3]] == [lead_args["base_speed"]] * 3
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
    assert renewed_stop.target_speed == stale_stop_args["base_speed"]
    assert renewed_stop.output_accel_max == 0.0
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
    assert stopped_evidence.target_speed == stopped_evidence.base_speed
    assert repeated_noise.target_speed == args["base_speed"]

  def test_later_continuously_moving_lead_does_not_latch_stopped_hold(self):
    governor = make_governor()
    moving_lead = make_radar(make_lead(status=True, d_rel=10.0, v_lead_k=1.5))
    update(governor, base_speed=5.0, v_ego=1.0, planner_speed=1.0)

    settled = update(governor, moving_lead, base_speed=5.0, v_ego=0.0, planner_speed=0.0)

    assert settled.selected_lead == 0
    assert not governor.live.stopped_lead_hold
    assert settled.target_speed == settled.live_pace
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
    assert dropout.target_speed == dropout.base_speed
    assert dropout.output_accel_max == 0.0
    assert governor.live.stopped_lead_hold

  def test_no_lead_start_launches_immediately_with_profile_limit(self):
    governor = make_governor()

    result = update(governor, base_speed=5.0, v_ego=0.1, planner_speed=0.1)

    assert result.selected_lead == -1
    assert result.launching
    assert result.live_pace == pytest.approx(0.1 + LAUNCH_DELTA_V)
    assert result.target_speed == result.live_pace
    assert result.effective_accel_max == pytest.approx(LAUNCH_ACCEL_JERK * DT_MDL)
    assert result.output_accel_max == result.effective_accel_max
    assert math.isinf(result.mpc_cruise_accel_max)

  def test_confirmed_departure_has_no_later_pace_jump(self):
    governor = make_governor()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    lead_args = {
      "base_speed": 5.0,
      "v_ego": 0.1,
      "planner_speed": 0.0,
      "previous_mpc_source": LongitudinalPlanSource.lead0,
      "previous_should_stop": True,
    }
    for _ in range(3):
      update(governor, stopped, **lead_args)
    for _ in range(8):
      departing = update(governor, moving, **lead_args)
    assert governor.live.departing_from_stop

    handed_back = update(governor, moving, **(lead_args | {"v_ego": 0.31, "planner_speed": 0.31}))

    assert not governor.live.departing_from_stop
    assert not governor.live.stopped_lead_hold
    expected_step = AccelController.get_profile_accel_max(AccelProfile.normal, 0.31) * DT_MDL
    assert handed_back.live_pace == pytest.approx(departing.live_pace + expected_step)
    assert handed_back.live_pace < min(handed_back.base_speed, handed_back.live_filtered_cap)
    assert handed_back.target_speed == handed_back.live_pace
    assert handed_back.mpc_cruise_accel_max == handed_back.output_accel_max

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
    assert math.isinf(result.output_accel_max)

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
