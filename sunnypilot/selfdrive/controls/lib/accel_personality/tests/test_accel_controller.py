import math
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import log
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE, T_IDXS, LongitudinalMpc, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import (
  ACCEL_PROFILE_MAX_BP,
  ACCEL_PROFILE_MAX_V,
  CAP_FILTER_FRAMES,
  HORIZON_SPEED_BUDGET,
  POSITIVE_MPC_HEADROOM,
  PROFILE_CONFIGS,
  PROFILE_TRANSITION_JERK,
  RADAR_STALE_TIMEOUT,
  RELIEF_CONFIRM_FRAMES,
  SHALLOW_BRAKE_BOUND,
  STOP_GAP_RESERVE,
  STOP_HOLD_CREEP_ABORT_FRAMES,
  STOP_HOLD_EXIT_FRAMES,
  AccelController,
  AccelControllerState,
  AccelProfile,
)


def make_lead(*, status=False, d_rel=0.0, v_lead_k=0.0, a_lead_k=0.0, a_lead_tau=1.5):
  return SimpleNamespace(status=status, dRel=d_rel, vLeadK=v_lead_k, aLeadK=a_lead_k, aLeadTau=a_lead_tau)


def make_radar(lead_one=None, lead_two=None):
  return SimpleNamespace(leadOne=lead_one or make_lead(), leadTwo=lead_two or make_lead())


def make_controller(delay=0.10):
  return AccelController(SimpleNamespace(longitudinalActuatorDelay=delay))


def update(controller, radar_state=None, **overrides):
  args = {
    "base_speed": 25.0,
    "v_ego": 10.0,
    "a_ego": 0.0,
    "profile": AccelProfile.normal,
    "follow_personality": log.LongitudinalPersonality.standard,
    "enabled": True,
    "acc_selected": True,
    "engaged": True,
    "cruise_initialized": True,
    "planner_accel": 0.0,
    "action_accel": 0.0,
    "stock_accel_max": ACCEL_MAX,
    "previous_should_stop": False,
  }
  args.update(overrides)
  return controller.update(radar_state or make_radar(), **args)


def restrictive_radar():
  return make_radar(make_lead(status=True, d_rel=25.0, v_lead_k=8.0, a_lead_k=-0.5))


class TestProfiles:
  def test_lookup_table_is_explicit_and_tunable(self):
    assert ACCEL_PROFILE_MAX_BP == [0.0, 3.0, 10.0, 25.0, 40.0]
    assert ACCEL_PROFILE_MAX_V == {
      AccelProfile.eco: [1.55, 1.25, 0.72, 0.32, 0.16],
      AccelProfile.normal: [1.70, 1.40, 0.97, 0.48, 0.30],
      AccelProfile.sport: [2.00, 1.90, 1.55, 0.80, 0.50],
    }

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_lookup_interpolates_and_stays_inside_global_limit(self, profile):
    for speed, expected in zip(ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V[profile], strict=True):
      assert AccelController.get_profile_accel_max(profile, speed) == expected
    limits = [AccelController.get_profile_accel_max(profile, speed) for speed in np.linspace(-1.0, 50.0, 201)]
    assert all(0.0 <= limit <= ACCEL_MAX for limit in limits)
    post_launch_limits = [AccelController.get_profile_accel_max(profile, speed) for speed in np.linspace(3.0, 40.0, 149)]
    assert np.all(np.diff(post_launch_limits) <= 0.0)

  @pytest.mark.parametrize("speed", [0.0, 3.0, 10.0, 25.0, 40.0])
  def test_profile_order_is_distinct(self, speed):
    limits = [AccelController.get_profile_accel_max(profile, speed) for profile in AccelProfile]
    assert limits[0] < limits[1] < limits[2]

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_clear_road_applies_profile_immediately(self, profile):
    result = update(make_controller(), v_ego=0.0, profile=profile)
    expected = ACCEL_PROFILE_MAX_V[profile][0]
    assert result.active and result.state == AccelControllerState.free
    assert result.target_speed == result.base_speed == 25.0
    assert result.positive_accel_max == expected
    assert result.effective_accel_max == expected
    if expected == ACCEL_MAX:
      assert result.mpc_accel_max is None
    else:
      np.testing.assert_array_equal(result.mpc_accel_max, min(expected + POSITIVE_MPC_HEADROOM, ACCEL_MAX))

  @pytest.mark.parametrize(("profile", "expected"), [
    (AccelProfile.eco, 1.25), (AccelProfile.normal, 1.40), (AccelProfile.sport, 1.90),
  ])
  def test_launch_strength_is_preserved_through_three_meters_per_second(self, profile, expected):
    result = update(make_controller(), v_ego=3.0, profile=profile)
    assert result.profile_accel_max == expected
    assert result.positive_accel_max == expected
    assert result.effective_accel_max == expected

  def test_turn_or_throttle_limit_intersects_profile(self):
    result = update(make_controller(), profile=AccelProfile.sport, stock_accel_max=0.0)
    assert result.positive_accel_max == 0.0
    assert result.effective_accel_max == 0.0
    np.testing.assert_array_equal(result.mpc_accel_max, 0.0)

  def test_nonbinding_clear_road_ceiling_releases_after_initialization(self):
    controller = make_controller()
    initial = update(controller, v_ego=10.0)
    released = update(controller, v_ego=10.0)

    assert initial.mpc_accel_max is not None
    assert released.mpc_accel_max is None
    assert released.effective_accel_max == released.profile_accel_max

  def test_positive_ceiling_uses_demand_hysteresis(self):
    controller = make_controller()
    initial = update(controller, v_ego=10.0)
    bound = initial.effective_accel_max
    released = update(controller, v_ego=10.0)
    entered = update(controller, v_ego=10.0, planner_accel=bound - 0.09)
    retained = update(controller, v_ego=10.0, planner_accel=bound - 0.15)
    exited = update(controller, v_ego=10.0, planner_accel=bound - 0.21)

    assert released.mpc_accel_max is None
    assert entered.mpc_accel_max is not None
    assert retained.mpc_accel_max is not None
    assert exited.mpc_accel_max is None

  def test_low_speed_and_valid_leads_keep_positive_ceiling_active(self):
    low_speed_controller = make_controller()
    update(low_speed_controller, v_ego=3.0)
    low_speed = update(low_speed_controller, v_ego=3.0)

    lead_controller = make_controller()
    far_lead = make_radar(make_lead(status=True, d_rel=200.0, v_lead_k=10.0))
    for _ in range(CAP_FILTER_FRAMES):
      with_lead = update(lead_controller, far_lead, v_ego=10.0)

    assert low_speed.mpc_accel_max is not None
    assert with_lead.state == AccelControllerState.free
    assert with_lead.mpc_accel_max is not None

  def test_profile_switch_changes_ceiling_without_a_step(self):
    controller = make_controller()
    sport = update(controller, profile=AccelProfile.sport, v_ego=10.0)
    eco = update(controller, profile=AccelProfile.eco, v_ego=10.0)
    assert sport.effective_accel_max > eco.effective_accel_max > eco.positive_accel_max
    assert sport.effective_accel_max - eco.effective_accel_max == pytest.approx(PROFILE_TRANSITION_JERK * DT_MDL)

  def test_invalid_profile_defaults_to_normal(self):
    result = update(make_controller(), profile=999)
    assert result.profile == AccelProfile.normal


class TestEnergyEnvelope:
  def test_relative_pace_energy_formula(self):
    controller = make_controller()
    lead = make_lead(status=True, d_rel=50.0, v_lead_k=8.0)
    envelope = controller.calculate_energy_envelope(make_radar(lead), 10.0, 0.0, AccelProfile.normal)
    delay = controller._delay()
    lead_xv = LongitudinalMpc.extrapolate_lead(lead.dRel, lead.vLeadK, lead.aLeadK, lead.aLeadTau)
    x_lead = float(np.interp(delay, T_IDXS, lead_xv[:, 0]))
    v_lead = float(np.interp(delay, T_IDXS, lead_xv[:, 1]))
    x_ego, _ = controller._project_ego(10.0, 0.0, delay)
    gap = max(x_lead - x_ego - STOP_DISTANCE - get_T_FOLLOW(log.LongitudinalPersonality.standard) * v_lead, 0.0)
    expected = v_lead + math.sqrt(2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * gap)
    assert envelope.cap == pytest.approx(expected)
    assert envelope.cap != pytest.approx(math.sqrt(v_lead**2 + 2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * gap))

  def test_profile_order_controls_approach_timing(self):
    controller = make_controller()
    radar = make_radar(make_lead(status=True, d_rel=50.0, v_lead_k=8.0))
    caps = [controller.calculate_energy_envelope(radar, 10.0, 0.0, profile).cap for profile in AccelProfile]
    assert caps[0] < caps[1] < caps[2]

  def test_stopped_lead_reserve_only_reduces_comfort_gap(self):
    controller = make_controller()
    lead = make_lead(status=True, d_rel=20.0, v_lead_k=0.0)
    envelope = controller.calculate_energy_envelope(make_radar(lead), 2.0, 0.0, AccelProfile.normal)
    expected = math.sqrt(2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * envelope.usable_gap)

    assert envelope.required_decel < 0.30
    assert envelope.safety_usable_gap - envelope.usable_gap == pytest.approx(STOP_GAP_RESERVE)
    assert envelope.cap == pytest.approx(expected)
    assert envelope.departure_cap > envelope.cap

  def test_stop_reserve_fades_out_of_urgent_braking(self):
    controller = make_controller()
    lead = make_lead(status=True, d_rel=20.0, v_lead_k=0.0)
    envelope = controller.calculate_energy_envelope(make_radar(lead), 20.0, 0.0, AccelProfile.normal)

    assert envelope.required_decel > 0.80
    assert envelope.usable_gap == envelope.safety_usable_gap
    assert envelope.cap == envelope.departure_cap
    assert controller._ttc(envelope) == pytest.approx(envelope.safety_usable_gap / envelope.closing_speed)

  def test_more_restrictive_lead_is_selected(self):
    radar = make_radar(make_lead(status=True, d_rel=70.0, v_lead_k=12.0), make_lead(status=True, d_rel=25.0, v_lead_k=8.0))
    envelope = make_controller().calculate_energy_envelope(radar, 10.0, 0.0, AccelProfile.normal)
    assert envelope.selected_lead == 1

  @pytest.mark.parametrize("field,value", [("aLeadK", math.nan), ("aLeadK", math.inf), ("aLeadTau", math.nan), ("aLeadTau", -1.0)])
  def test_nonessential_invalid_lead_fields_are_sanitized(self, field, value):
    lead = make_lead(status=True, d_rel=30.0, v_lead_k=8.0)
    setattr(lead, field, value)
    envelope = make_controller().calculate_energy_envelope(make_radar(lead), 10.0, 0.0, AccelProfile.normal)
    assert envelope.selected_lead == 0
    assert math.isfinite(envelope.cap)

  @pytest.mark.parametrize("field,value", [("dRel", math.nan), ("dRel", -1.0), ("vLeadK", math.nan), ("vLeadK", -2.0)])
  def test_invalid_geometry_is_not_used(self, field, value):
    lead = make_lead(status=True, d_rel=30.0, v_lead_k=8.0)
    setattr(lead, field, value)
    envelope = make_controller().calculate_energy_envelope(make_radar(lead), 10.0, 0.0, AccelProfile.normal)
    assert envelope.selected_lead == -1
    assert envelope.lead_status

  def test_raw_radar_is_never_mutated(self):
    lead = make_lead(status=True, d_rel=30.0, v_lead_k=8.0, a_lead_k=-15.0, a_lead_tau=math.nan)
    before = vars(lead).copy()
    make_controller().calculate_energy_envelope(make_radar(lead), 10.0, 0.0, AccelProfile.normal)
    assert vars(lead) == before


class TestAccelControllerState:
  def test_five_frame_median_needs_three_restrictive_samples(self):
    controller = make_controller()
    results = [update(controller, restrictive_radar()) for _ in range(CAP_FILTER_FRAMES)]
    assert math.isinf(results[1].live_filtered_cap)
    assert math.isfinite(results[2].live_filtered_cap)

  def test_routine_approach_builds_safe_finite_horizon_ceiling(self):
    controller = make_controller()
    result = None
    for _ in range(CAP_FILTER_FRAMES):
      result = update(controller, restrictive_radar())
    assert result is not None and result.state == AccelControllerState.restrict
    ceiling = np.asarray(result.mpc_accel_max)
    assert ceiling.shape == T_IDXS.shape
    assert np.all(np.isfinite(ceiling))
    assert np.all((ceiling >= ACCEL_MIN) & (ceiling <= ACCEL_MAX))
    assert ceiling[0] >= 0.0
    assert np.min(ceiling) < -0.05 and ceiling[-1] == pytest.approx(0.0)
    assert np.trapezoid(-np.minimum(ceiling, 0.0), T_IDXS) <= HORIZON_SPEED_BUDGET * 10.0 + 1e-9

  def test_ongoing_mpc_braking_does_not_ratchet_the_controller(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES):
      previous = update(controller, restrictive_radar())
    result = update(controller, restrictive_radar(), action_accel=-1.2, planner_accel=-1.0)
    assert result.effective_accel_max >= previous.effective_accel_max - 0.60 * DT_MDL - 1e-9

  def test_two_dropouts_cannot_release_restriction(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES):
      restricted = update(controller, restrictive_radar())
    results = [update(controller) for _ in range(2)]
    assert all(result.active and result.effective_accel_max <= 0.0 for result in results)
    assert all(result.effective_accel_max >= restricted.effective_accel_max for result in results)

  def test_relief_requires_consecutive_confirmation(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES):
      update(controller, restrictive_radar())
    moving_away = make_radar(make_lead(status=True, d_rel=45.0, v_lead_k=13.0))
    early = [update(controller, moving_away) for _ in range(RELIEF_CONFIRM_FRAMES - 1)]
    assert all(result.state == AccelControllerState.hold and result.effective_accel_max <= 0.0 for result in early)
    released = update(controller, moving_away)
    assert released.state == AccelControllerState.free
    assert released.effective_accel_max <= 0.0
    accelerating = update(controller, moving_away)
    assert released.effective_accel_max < accelerating.effective_accel_max <= accelerating.positive_accel_max

  def test_shallow_brake_relief_uses_long_confirmation_without_delaying_tightening(self):
    controller = make_controller()
    controller.live.state = AccelControllerState.restrict
    controller.live.bound = SHALLOW_BRAKE_BOUND + 0.05
    matched = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=10.0))

    held = [update(controller, matched) for _ in range(controller.shallow_brake_relief_frames - 1)]
    assert all(result.effective_accel_max == pytest.approx(SHALLOW_BRAKE_BOUND + 0.05) for result in held)
    relaxed = update(controller, matched)
    assert relaxed.effective_accel_max > held[-1].effective_accel_max

    for _ in range(CAP_FILTER_FRAMES):
      tightened = update(controller, restrictive_radar())
    assert tightened.effective_accel_max < relaxed.effective_accel_max

  def test_strong_brake_relief_keeps_existing_confirmation(self):
    controller = make_controller()
    controller.live.state = AccelControllerState.restrict
    controller.live.bound = SHALLOW_BRAKE_BOUND - 0.25
    matched = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=10.0))

    held = [update(controller, matched) for _ in range(RELIEF_CONFIRM_FRAMES - 1)]
    assert all(result.effective_accel_max == pytest.approx(SHALLOW_BRAKE_BOUND - 0.25) for result in held)
    relaxed = update(controller, matched)
    assert relaxed.effective_accel_max > held[-1].effective_accel_max
    continuing = [update(controller, matched) for _ in range(8)]
    assert all(current.effective_accel_max > previous.effective_accel_max
               for previous, current in zip([relaxed, *continuing[:-1]], continuing, strict=True))

  def test_urgent_frame_uses_exact_stock_path(self):
    urgent = make_radar(make_lead(status=True, d_rel=18.0, v_lead_k=0.0))
    result = update(make_controller(), urgent, v_ego=20.0)
    assert result.active and result.stock_mode
    assert result.mpc_accel_max is None
    assert math.isinf(result.effective_accel_max)

  def test_urgent_relief_stays_stock_until_braking_has_recovered(self):
    controller = make_controller()
    urgent = make_radar(make_lead(status=True, d_rel=18.0, v_lead_k=0.0))
    update(controller, urgent, v_ego=20.0)
    result = update(controller, action_accel=-1.5, planner_accel=-1.2, v_ego=19.8)
    assert result.stock_mode
    assert result.mpc_accel_max is None
    recovered = [update(controller, action_accel=0.0, planner_accel=0.0, v_ego=19.8) for _ in range(RELIEF_CONFIRM_FRAMES)]
    assert all(sample.stock_mode for sample in recovered[:-1])
    assert recovered[-1].state == AccelControllerState.free

  def test_stop_hold_needs_four_departure_frames(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    held = update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)
    assert held.state == AccelControllerState.stopHold
    np.testing.assert_array_equal(held.mpc_accel_max, 0.0)

    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))
    confirmation = [update(controller, departing, base_speed=8.0, v_ego=0.1) for _ in range(STOP_HOLD_EXIT_FRAMES)]
    assert all(result.effective_accel_max == 0.0 for result in confirmation[:-1])
    launched = confirmation[-1]
    assert launched.launching and launched.state == AccelControllerState.free
    assert launched.effective_accel_max == launched.positive_accel_max

  def test_false_creep_speed_without_range_gain_stays_held(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)

    false_creep = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.30))
    held = [update(controller, false_creep, base_speed=8.0, v_ego=0.1) for _ in range(20)]
    assert all(result.state == AccelControllerState.stopHold and not result.launching for result in held)

  def test_invalid_lead_geometry_cannot_confirm_departure(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)

    invalid = make_radar(make_lead(status=True, d_rel=math.nan, v_lead_k=0.30))
    held = [update(controller, invalid, base_speed=8.0, v_ego=0.1) for _ in range(2 * STOP_HOLD_EXIT_FRAMES)]
    assert all(result.state == AccelControllerState.stopHold and not result.launching for result in held)

  def test_short_range_drop_and_restore_cannot_fake_creep_departure(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)

    low_range = make_radar(make_lead(status=True, d_rel=5.0, v_lead_k=0.0))
    for _ in range(2):
      update(controller, low_range, base_speed=8.0, v_ego=0.1)
    restored = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.30))
    results = [update(controller, restored, base_speed=8.0, v_ego=0.1) for _ in range(8)]

    assert all(result.state == AccelControllerState.stopHold and not result.launching for result in results)

  def test_slow_creep_with_confirmed_range_gain_releases(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)

    results = []
    for frame in range(20):
      creep = make_radar(make_lead(status=True, d_rel=6.0 + 0.05 * frame, v_lead_k=0.30))
      results.append(update(controller, creep, base_speed=8.0, v_ego=0.1))

    launched = [frame for frame, result in enumerate(results) if result.launching]
    assert launched and launched[0] >= STOP_HOLD_EXIT_FRAMES
    assert all(result.state == AccelControllerState.stopHold for result in results[:launched[0]])

  def test_slow_creep_survives_lead_slot_switching(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0), make_lead(status=True, d_rel=6.1))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)

    results = []
    for frame in range(24):
      distance = 6.0 + 0.04 * frame
      offset = 0.05 if frame % 2 else 0.0
      lead_one = make_lead(status=True, d_rel=distance + offset, v_lead_k=0.30)
      lead_two = make_lead(status=True, d_rel=distance + 0.05 - offset, v_lead_k=0.30)
      results.append(update(controller, make_radar(lead_one, lead_two), base_speed=8.0, v_ego=0.1))

    assert any(result.launching for result in results)

  def test_departure_that_stops_again_returns_to_hold(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)
    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))
    for _ in range(STOP_HOLD_EXIT_FRAMES):
      launched = update(controller, departing, base_speed=8.0, v_ego=0.1)
    assert launched.launching

    stalled = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.11))
    settling = [update(controller, stalled, base_speed=8.0, v_ego=0.1) for _ in range(STOP_HOLD_CREEP_ABORT_FRAMES)]
    assert all(result.launching for result in settling[:-1])
    assert settling[-1].state == AccelControllerState.stopHold and not settling[-1].launching

  def test_invalid_geometry_after_departure_returns_to_hold(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, previous_should_stop=True)
    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))
    for _ in range(STOP_HOLD_EXIT_FRAMES):
      launched = update(controller, departing, base_speed=8.0, v_ego=0.1)
    assert launched.launching

    invalid = make_radar(make_lead(status=True, d_rel=math.nan, v_lead_k=0.30))
    settling = [update(controller, invalid, base_speed=8.0, v_ego=0.1) for _ in range(STOP_HOLD_CREEP_ABORT_FRAMES)]
    assert settling[-1].state == AccelControllerState.stopHold and not settling[-1].launching

  def test_stale_radar_freezes_then_discards_live_state(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES):
      restricted = update(controller, restrictive_radar())
    hold_frames = math.ceil(RADAR_STALE_TIMEOUT / DT_MDL) - 1
    frozen = [update(controller, radar_fresh=False) for _ in range(hold_frames)]
    assert all(result.active and result.effective_accel_max == restricted.effective_accel_max for result in frozen)
    timed_out = update(controller, radar_fresh=False)
    assert not timed_out.active and timed_out.mpc_accel_max is None

  def test_stale_radar_preserves_positive_lead_ceiling_until_timeout(self):
    controller = make_controller()
    far_lead = make_radar(make_lead(status=True, d_rel=200.0, v_lead_k=10.0))
    for _ in range(CAP_FILTER_FRAMES):
      active = update(controller, far_lead, v_ego=10.0)
    hold_frames = math.ceil(RADAR_STALE_TIMEOUT / DT_MDL) - 1
    frozen = [update(controller, far_lead, radar_fresh=False, v_ego=10.0) for _ in range(hold_frames)]

    assert active.state == AccelControllerState.free and active.mpc_accel_max is not None
    assert all(result.active and result.mpc_accel_max is not None for result in frozen)
    timed_out = update(controller, far_lead, radar_fresh=False, v_ego=10.0)
    assert not timed_out.active and timed_out.mpc_accel_max is None

  def test_filtered_lead_dropout_guard_preserves_positive_ceiling(self):
    controller = make_controller()
    far_lead = make_radar(make_lead(status=True, d_rel=200.0, v_lead_k=10.0))
    for _ in range(CAP_FILTER_FRAMES):
      with_lead = update(controller, far_lead, v_ego=10.0)
    guarded = [update(controller, v_ego=10.0) for _ in range(CAP_FILTER_FRAMES // 2)]
    released = update(controller, v_ego=10.0)

    assert with_lead.state == AccelControllerState.free and with_lead.mpc_accel_max is not None
    assert all(result.mpc_accel_max is not None for result in guarded)
    assert released.mpc_accel_max is None

  def test_stale_radar_preserves_urgent_stock_passthrough_until_timeout(self):
    controller = make_controller()
    urgent = make_radar(make_lead(status=True, d_rel=18.0, v_lead_k=0.0))
    result = update(controller, urgent, v_ego=20.0)
    assert result.active and result.shadow_active and result.stock_mode

    hold_frames = math.ceil(RADAR_STALE_TIMEOUT / DT_MDL) - 1
    frozen = [update(controller, radar_fresh=False, v_ego=20.0) for _ in range(hold_frames)]
    assert all(sample.active and sample.shadow_active and sample.stock_mode for sample in frozen)
    assert all(sample.state == AccelControllerState.hold and sample.mpc_accel_max is None for sample in frozen)

    timed_out = update(controller, radar_fresh=False, v_ego=20.0)
    assert not timed_out.active and not timed_out.shadow_active and not timed_out.stock_mode

  @pytest.mark.parametrize("override", [
    {"enabled": False}, {"acc_selected": False}, {"engaged": False}, {"cruise_initialized": False}, {"controller_fault": True},
  ])
  def test_bypass_never_actuates(self, override):
    result = update(make_controller(), restrictive_radar(), **override)
    assert not result.active
    assert result.target_speed == result.base_speed
    assert result.mpc_accel_max is None
    assert math.isinf(result.effective_accel_max)

  def test_shadow_history_never_enters_live_actuation(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES):
      shadow = update(controller, restrictive_radar(), enabled=False)
    assert shadow.shadow_state == AccelControllerState.restrict
    live = update(controller)
    assert live.state == AccelControllerState.free
    assert live.effective_accel_max > 0.0


@pytest.mark.parametrize("v_ego", [0.0, 0.2, 0.5, 1.0, 2.0, 10.0, 40.0])
@pytest.mark.parametrize("bound", [-3.5, -2.0, -1.0, -0.1, 0.0, 0.8, 2.0])
def test_accel_ceiling_properties(v_ego, bound):
  result = AccelController._build_accel_ceiling(bound, v_ego, planner_accel=0.3, action_time=0.25)
  if bound >= ACCEL_MAX:
    assert result is None
    return
  ceiling = np.asarray(result)
  assert ceiling.shape == T_IDXS.shape
  assert np.all(np.isfinite(ceiling))
  assert np.all((ceiling >= ACCEL_MIN) & (ceiling <= ACCEL_MAX))
  assert ceiling[0] >= 0.3 - 1e-9
  if bound > 0.0:
    np.testing.assert_array_equal(ceiling, min(bound + POSITIVE_MPC_HEADROOM, ACCEL_MAX))
  if bound < 0.0:
    assert np.trapezoid(-np.minimum(ceiling, 0.0), T_IDXS) <= HORIZON_SPEED_BUDGET * v_ego + 1e-9
