import math
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import log
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  STOP_DISTANCE, T_IDXS, LongitudinalMpc, LongitudinalPlanSource, get_T_FOLLOW,
)
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelController, AccelControllerState, AccelProfile
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import (
  ACCEL_LIMIT_HORIZON_JERK, ACCEL_LIMIT_TRANSITION_JERK, ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V, CAP_FILTER_FRAMES,
  LAUNCH_END_SPEED, LAUNCH_TARGET_HEADROOM, LAUNCH_TARGET_SLEW, LEAD_MATCH_ACCEL_SLEW, MATCHED_PACE_DECEL_RATE, PROFILE_CONFIGS,
  RADAR_STALE_TIMEOUT, STOP_GAP_RESERVE, STOP_HOLD_ACCEL_MAX, STOP_HOLD_DEPARTURE_ACCEL_MAX, STOP_HOLD_EXIT_FRAMES,
)


def make_lead(*, status=False, d_rel=0.0, v_lead_k=0.0, a_lead_k=0.0, a_lead_tau=1.5, radar_track_id=-1):
  return SimpleNamespace(status=status, dRel=d_rel, vLeadK=v_lead_k, aLeadK=a_lead_k, aLeadTau=a_lead_tau,
                         radarTrackId=radar_track_id)


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
    "stock_accel_max": ACCEL_MAX,
    "previous_should_stop": False,
  }
  args.update(overrides)
  return controller.update(radar_state or make_radar(), **args)


def restrictive_radar():
  return make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0, a_lead_k=-0.5))


def enter_stop_hold(controller, *, base_speed=8.0, v_ego=0.1):
  stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
  return update(controller, stopped, base_speed=base_speed, v_ego=v_ego, previous_should_stop=True)


class TestProfiles:
  def test_lookup_table_is_explicit_and_tunable(self):
    assert ACCEL_PROFILE_MAX_BP == [0.0, 3.0, 10.0, 25.0, 40.0]
    assert ACCEL_PROFILE_MAX_V == {
      AccelProfile.eco: [1.55, 1.25, 0.72, 0.32, 0.16],
      AccelProfile.normal: [1.70, 1.40, 0.97, 0.48, 0.30],
      AccelProfile.sport: [2.00, 1.90, 1.15, 0.68, 0.42],
    }

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_lookup_interpolates_and_stays_inside_global_limit(self, profile):
    for speed, expected in zip(ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V[profile], strict=True):
      assert AccelController.get_profile_accel_max(profile, speed) == expected

    limits = [AccelController.get_profile_accel_max(profile, speed) for speed in np.linspace(-1.0, 50.0, 201)]
    assert all(0.0 <= limit <= ACCEL_MAX for limit in limits)
    assert np.all(np.diff(limits) <= 0.0)

  @pytest.mark.parametrize("speed", ACCEL_PROFILE_MAX_BP)
  def test_profile_order_is_distinct(self, speed):
    eco, normal, sport = [AccelController.get_profile_accel_max(profile, speed) for profile in AccelProfile]
    assert eco < normal < sport

  def test_invalid_profile_defaults_to_normal(self):
    assert update(make_controller(), profile=999).profile == AccelProfile.normal

  def test_stock_limit_intersects_profile_before_mpc(self):
    controller = make_controller()
    results = [update(controller, v_ego=10.0, profile=AccelProfile.sport, stock_accel_max=0.30)
               for _ in range(controller.lead_loss_hold_frames)]
    result = results[-1]
    assert result.profile_accel_max == pytest.approx(1.15)
    assert result.positive_accel_max == pytest.approx(0.30)
    assert result.effective_accel_max == pytest.approx(0.30)
    assert all(sample.mpc_accel_max is None for sample in results[:-1])
    assert result.mpc_accel_max is not None
    assert max(result.mpc_accel_max) <= ACCEL_MAX

  def test_runtime_profile_switch_smooths_the_scalar_limit(self):
    controller = make_controller()
    sport = [update(controller, v_ego=10.0, profile=AccelProfile.sport, stock_accel_max=1.20)
             for _ in range(controller.lead_loss_hold_frames)][-1]
    eco = update(controller, v_ego=10.0, profile=AccelProfile.eco, stock_accel_max=1.20)
    max_step = ACCEL_LIMIT_TRANSITION_JERK * DT_MDL

    assert sport.effective_accel_max == pytest.approx(1.15)
    assert sport.effective_accel_max - eco.effective_accel_max == pytest.approx(max_step)

    settled = [update(controller, v_ego=10.0, profile=AccelProfile.eco, stock_accel_max=1.20) for _ in range(30)][-1]
    assert settled.effective_accel_max == pytest.approx(0.72)

  def test_matched_lead_waits_until_ego_catches_the_lead(self):
    radar = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0))
    slow_controller, caught_controller = make_controller(), make_controller()
    for controller in (slow_controller, caught_controller):
      for _ in range(CAP_FILTER_FRAMES + 10):
        update(controller, radar, v_ego=10.0, planner_accel=-0.2)

    update(slow_controller, radar, v_ego=3.0, planner_accel=-0.2)
    update(caught_controller, radar, v_ego=8.0, planner_accel=-0.2)

    assert not slow_controller.live.matched_lead
    assert caught_controller.live.matched_lead

  def test_stock_limit_reduction_applies_immediately(self):
    controller = make_controller()
    for _ in range(controller.lead_loss_hold_frames):
      update(controller, v_ego=10.0, profile=AccelProfile.sport, stock_accel_max=1.20)

    reduced = update(controller, v_ego=10.0, profile=AccelProfile.sport, stock_accel_max=0.30)
    assert reduced.effective_accel_max == pytest.approx(0.30)
    assert reduced.mpc_accel_max is not None
    assert max(reduced.mpc_accel_max) <= 0.30 + 1e-9

  def test_one_frame_stock_zero_does_not_poison_profile_recovery(self):
    clean_controller, glitch_controller = make_controller(), make_controller()
    for _ in range(clean_controller.lead_loss_hold_frames + 10):
      clean = update(clean_controller, v_ego=10.0, stock_accel_max=1.5)
      recovered = update(glitch_controller, v_ego=10.0, stock_accel_max=1.5)

    limited = update(glitch_controller, v_ego=10.0, stock_accel_max=0.0)
    clean = update(clean_controller, v_ego=10.0, stock_accel_max=1.5)
    recovered = update(glitch_controller, v_ego=10.0, stock_accel_max=1.5)

    assert limited.effective_accel_max == 0.0
    assert recovered.effective_accel_max == pytest.approx(clean.effective_accel_max)

  @pytest.mark.parametrize("radar_fresh", (True, False), ids=("dropout", "stale"))
  def test_matched_lead_ceiling_obeys_current_stock_limit(self, radar_fresh):
    controller = make_controller()
    radar = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0))
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, radar, v_ego=10.0, planner_accel=-0.2)
    for _ in range(20):
      update(controller, radar, v_ego=8.0, planner_accel=-0.2)
    assert controller.live.matched_lead

    limited = update(controller, stock_accel_max=0.0, radar_fresh=radar_fresh)
    assert limited.effective_accel_max == 0.0
    assert limited.mpc_accel_max is not None
    assert max(limited.mpc_accel_max) == 0.0

  def test_exact_global_max_uses_stock_ceiling(self):
    result = update(make_controller(), base_speed=8.0, v_ego=0.0, profile=AccelProfile.sport)
    assert result.positive_accel_max == ACCEL_MAX
    assert result.mpc_accel_max is None


class TestMpcCeiling:
  @pytest.mark.parametrize("planner_accel", (-1.0, 0.0, 1.2, ACCEL_MAX))
  def test_ceiling_is_finite_feasible_and_jerk_bounded(self, planner_accel):
    limit = 0.50
    ceiling = np.asarray(AccelController._build_accel_ceiling(limit, planner_accel))
    a0 = float(np.clip(planner_accel, ACCEL_MIN, ACCEL_MAX))

    assert ceiling.shape == T_IDXS.shape
    assert np.all(np.isfinite(ceiling))
    assert np.all((0.0 <= ceiling) & (ceiling <= ACCEL_MAX))
    assert ceiling[0] + 1e-9 >= a0
    assert np.all(ceiling + 1e-9 >= limit)
    assert np.all(np.diff(ceiling) <= 1e-9)
    assert np.all(-np.diff(ceiling) <= ACCEL_LIMIT_HORIZON_JERK * np.diff(T_IDXS) + 1e-9)

  def test_zero_limit_remains_feasible_for_positive_x0(self):
    ceiling = np.asarray(AccelController._build_accel_ceiling(0.0, 0.8))
    assert ceiling[0] == pytest.approx(0.8)
    assert ceiling[-1] == pytest.approx(0.0)
    assert np.all(ceiling >= 0.0)

  def test_inactive_controller_has_no_custom_ceiling(self):
    result = update(make_controller(), enabled=False)
    assert not result.active
    assert result.mpc_accel_max is None
    assert math.isinf(result.effective_accel_max)

  def test_braking_ceiling_waits_for_stable_live_history_and_resets(self):
    controller = make_controller()
    radar = restrictive_radar()
    warmup = [update(controller, radar, planner_accel=-0.2) for _ in range(controller.lead_loss_hold_frames)]

    assert all(not sample.mpc_accel_max for sample in warmup[:-1])
    assert controller.live.braking_limited
    assert warmup[-1].mpc_accel_max is not None

    bypassed = update(controller, radar, planner_accel=-0.2, acc_selected=False)
    assert not bypassed.active and bypassed.mpc_accel_max is None
    assert not controller.live.braking_limited

  def test_matched_lead_ceiling_has_no_planner_accel_handoff(self):
    controller = make_controller()
    radar = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0))
    for _ in range(CAP_FILTER_FRAMES + 5):
      update(controller, radar, v_ego=10.0, planner_accel=-0.2)

    braking = update(controller, radar, v_ego=8.0, planner_accel=-0.2)
    accelerating = update(controller, radar, v_ego=8.0, planner_accel=0.2)

    assert controller.live.matched_lead
    assert braking.mpc_accel_max is not None and accelerating.mpc_accel_max is not None
    assert abs(accelerating.effective_accel_max - braking.effective_accel_max) <= LEAD_MATCH_ACCEL_SLEW * DT_MDL + 1e-9
    target_drop = braking.target_speed - accelerating.target_speed
    assert 0.0 <= target_drop <= MATCHED_PACE_DECEL_RATE * DT_MDL + 1e-9
    assert accelerating.state == AccelControllerState.restrict

  def test_matched_lead_ignores_two_frame_speed_jump(self):
    clean_controller, noisy_controller = make_controller(), make_controller()
    radar = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0))
    for controller in (clean_controller, noisy_controller):
      for _ in range(CAP_FILTER_FRAMES + 10):
        update(controller, radar, v_ego=10.0, planner_accel=-0.2)
      for _ in range(20):
        update(controller, radar, v_ego=8.0, planner_accel=-0.2)

    speed_jump = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=16.0))
    for _ in range(2):
      clean = update(clean_controller, radar, v_ego=8.0)
      noisy = update(noisy_controller, speed_jump, v_ego=8.0)
      assert noisy.effective_accel_max == pytest.approx(clean.effective_accel_max)
      assert noisy.target_speed == pytest.approx(clean.target_speed)

  def test_matched_lead_ignores_two_frame_acceleration_jump(self):
    clean_controller, noisy_controller = make_controller(), make_controller()
    steady = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0))
    for controller in (clean_controller, noisy_controller):
      for _ in range(CAP_FILTER_FRAMES + 10):
        update(controller, steady, v_ego=10.0, planner_accel=-0.2)
      for _ in range(20):
        update(controller, steady, v_ego=8.0, planner_accel=-0.2)

    braking_jump = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0, a_lead_k=-1.0))
    for _ in range(2):
      clean = update(clean_controller, steady, v_ego=8.0)
      noisy = update(noisy_controller, braking_jump, v_ego=8.0)
      assert noisy.effective_accel_max == pytest.approx(clean.effective_accel_max)
      assert noisy.target_speed == pytest.approx(clean.target_speed)


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
    safety_gap = max(x_lead - x_ego - STOP_DISTANCE - get_T_FOLLOW(log.LongitudinalPersonality.standard) * v_lead, 0.0)
    expected = v_lead + math.sqrt(2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * safety_gap)

    assert envelope.cap == pytest.approx(expected)
    assert envelope.cap != pytest.approx(math.sqrt(v_lead**2 + 2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * safety_gap))

  def test_profile_order_controls_approach_timing(self):
    radar = make_radar(make_lead(status=True, d_rel=50.0, v_lead_k=8.0))
    caps = [make_controller().calculate_energy_envelope(radar, 10.0, 0.0, profile).cap for profile in AccelProfile]
    assert caps[0] < caps[1] < caps[2]

  def test_stopped_lead_reserve_only_reduces_comfort_gap(self):
    envelope = make_controller().calculate_energy_envelope(
      make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=0.0)), 5.0, 0.0, AccelProfile.normal,
    )
    comfort_decel = PROFILE_CONFIGS[AccelProfile.normal].comfort_decel
    safety_gap = (envelope.departure_cap - envelope.departure_lead_speed) ** 2 / (2.0 * comfort_decel)
    assert envelope.required_decel < 0.30
    assert safety_gap - envelope.usable_gap == pytest.approx(STOP_GAP_RESERVE)
    assert envelope.departure_cap > envelope.cap

  def test_more_restrictive_lead_is_selected(self):
    radar = make_radar(make_lead(status=True, d_rel=70.0, v_lead_k=12.0), make_lead(status=True, d_rel=25.0, v_lead_k=8.0))
    assert make_controller().calculate_energy_envelope(radar, 10.0, 0.0, AccelProfile.normal).selected_lead == 1

  @pytest.mark.parametrize("field,value", [
    ("aLeadK", math.nan), ("aLeadK", math.inf), ("aLeadTau", math.nan), ("aLeadTau", -1.0), ("radarTrackId", math.nan),
  ])
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
    assert math.isinf(envelope.cap)

  def test_raw_radar_is_never_mutated(self):
    lead = make_lead(status=True, d_rel=30.0, v_lead_k=8.0, a_lead_k=-15.0, a_lead_tau=math.nan)
    before = vars(lead).copy()
    make_controller().calculate_energy_envelope(make_radar(lead), 10.0, 0.0, AccelProfile.normal)
    assert vars(lead) == before


class TestPaceAndLifecycle:
  def test_five_frame_median_needs_three_restrictive_samples(self):
    controller = make_controller()
    results = [update(controller, restrictive_radar()) for _ in range(CAP_FILTER_FRAMES)]
    assert math.isinf(results[1].live_filtered_cap)
    assert math.isfinite(results[2].live_filtered_cap)

  def test_restriction_uses_comfort_rate_without_acquisition_step(self):
    controller = make_controller()
    results = [update(controller, restrictive_radar()) for _ in range(CAP_FILTER_FRAMES + 10)]
    targets = np.asarray([result.target_speed for result in results])
    max_step = PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * DT_MDL

    assert np.max(-np.diff(targets)) <= max_step + 1e-9
    assert results[-1].state == AccelControllerState.restrict
    assert results[-1].target_speed < results[0].target_speed

  @pytest.mark.parametrize("clear_frames", (1, 2, CAP_FILTER_FRAMES + 1))
  def test_lead_acquired_after_clear_road_cannot_step_pace_to_planner(self, clear_frames):
    controller = make_controller()
    for _ in range(clear_frames):
      update(controller, base_speed=25.0, v_ego=20.0, planner_speed=25.0)

    results = [update(controller, restrictive_radar(), base_speed=25.0, v_ego=20.0, planner_speed=20.0)
               for _ in range(CAP_FILTER_FRAMES)]
    targets = np.asarray([25.0, *(result.target_speed for result in results)])
    max_step = PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * DT_MDL

    assert np.max(-np.diff(targets)) <= max_step + 1e-9

  def test_lead_slot_is_forgotten_before_reacquisition(self):
    controller = make_controller()
    lead_one = restrictive_radar()
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, lead_one, base_speed=25.0, v_ego=20.0, planner_speed=20.0, planner_accel=-0.2)

    for _ in range(controller.lead_loss_hold_frames):
      before = update(controller, base_speed=25.0, v_ego=20.0, planner_speed=20.0, planner_accel=-0.2)
    assert controller.live.selected_lead == -1

    lead_two = make_radar(lead_two=make_lead(status=True, d_rel=20.0, v_lead_k=8.0, a_lead_k=-0.5))
    results = [update(controller, lead_two, base_speed=25.0, v_ego=20.0, planner_speed=5.0, planner_accel=-0.2)
               for _ in range(CAP_FILTER_FRAMES)]
    targets = np.asarray([before.target_speed, *(result.target_speed for result in results)])
    max_step = PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * DT_MDL

    assert np.max(-np.diff(targets)) <= max_step + 1e-9

  @pytest.mark.parametrize("replacement_track_id", (200, -1), ids=("radar-track", "vision-track"))
  def test_false_relief_track_replacement_freezes_bounded_pace_release(self, replacement_track_id):
    controller = make_controller()
    original = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0, radar_track_id=100))
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, original, base_speed=25.0, v_ego=10.0, planner_speed=10.0, planner_accel=-0.2)
    for _ in range(20):
      before = update(controller, original, base_speed=25.0, v_ego=8.0, planner_speed=8.0, planner_accel=-0.2)
    assert controller.live.matched_lead

    replacement = make_radar(make_lead(status=True, d_rel=40.0, v_lead_k=12.0, radar_track_id=replacement_track_id))
    switched = update(controller, replacement, base_speed=25.0, v_ego=8.0, planner_speed=5.0, planner_accel=-0.2)

    target_drop = before.target_speed - switched.target_speed
    assert 0.0 <= target_drop <= MATCHED_PACE_DECEL_RATE * DT_MDL + 1e-9
    assert switched.target_speed < switched.base_speed
    assert controller.live.lead_switch_guard_frames == controller.lead_loss_hold_frames

  def test_track_id_churn_without_false_relief_does_not_arm_guard(self):
    controller = make_controller()
    original = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0, radar_track_id=100))
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, original, base_speed=25.0, v_ego=10.0, planner_speed=10.0, planner_accel=-0.2)

    replacement = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0, radar_track_id=200))
    update(controller, replacement, base_speed=25.0, v_ego=10.0, planner_speed=10.0, planner_accel=-0.2)

    assert controller.live.lead_switch_guard_frames == 0

  def test_short_dropout_holds_then_releases_at_profile_rate(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES + 20):
      restricted = update(controller, restrictive_radar())

    held = [update(controller) for _ in range(controller.lead_loss_hold_frames - 1)]
    assert all(result.target_speed <= restricted.target_speed + 1e-9 for result in held)

    released = update(controller)
    assert released.target_speed >= held[-1].target_speed
    assert released.target_speed - held[-1].target_speed <= released.effective_accel_max * DT_MDL + 1e-9

  def test_previous_lead_source_synchronizes_down_to_planner(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES + 10):
      restricted = update(controller, restrictive_radar())
    synchronized = update(controller, previous_mpc_source=LongitudinalPlanSource.lead0, planner_speed=restricted.target_speed - 2.0)
    assert synchronized.target_speed == pytest.approx(restricted.target_speed - 2.0)
    assert synchronized.state == AccelControllerState.hold

  def test_matched_lead_dropout_synchronizes_down_to_planner(self):
    controller = make_controller()
    radar = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0))
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, radar, v_ego=10.0, planner_accel=-0.2)
    for _ in range(20):
      matched = update(controller, radar, v_ego=8.0, planner_accel=-0.2)
    assert controller.live.matched_lead

    planner_speed = matched.target_speed - 2.0
    synchronized = update(controller, previous_mpc_source=LongitudinalPlanSource.lead0, planner_speed=planner_speed)
    assert synchronized.target_speed == pytest.approx(planner_speed)
    assert synchronized.state == AccelControllerState.hold

  def test_reused_radar_holds_matched_lead_until_a_fresh_dropout(self):
    controller = make_controller()
    radar = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=8.0))
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, radar, v_ego=10.0, planner_accel=-0.2)
    for _ in range(20):
      matched = update(controller, radar, v_ego=8.0, planner_accel=-0.2)
    assert controller.live.matched_lead

    planner_speed = matched.target_speed - 2.0
    held = update(controller, radar, previous_mpc_source=LongitudinalPlanSource.lead0,
                  planner_speed=planner_speed, radar_fresh=False)
    synchronized = update(controller, previous_mpc_source=LongitudinalPlanSource.lead0, planner_speed=planner_speed)

    assert held.target_speed == pytest.approx(matched.target_speed)
    assert held.state == matched.state
    assert synchronized.target_speed == pytest.approx(planner_speed)
    assert synchronized.state == AccelControllerState.hold

  def test_clear_road_launch_has_immediate_headroom_and_bounded_target_slew(self):
    controller = make_controller()
    initial = update(controller, base_speed=12.0, v_ego=0.0, profile=AccelProfile.normal)
    rolling = update(controller, base_speed=12.0, v_ego=0.31, profile=AccelProfile.normal)

    assert initial.active and initial.launching
    assert LAUNCH_TARGET_HEADROOM <= initial.target_speed <= LAUNCH_TARGET_HEADROOM + LAUNCH_TARGET_SLEW * DT_MDL
    assert rolling.launching
    assert rolling.target_speed >= 0.31 + LAUNCH_TARGET_HEADROOM
    assert rolling.target_speed - max(initial.target_speed, 0.31 + LAUNCH_TARGET_HEADROOM) <= LAUNCH_TARGET_SLEW * DT_MDL + 1e-9

    finished = update(controller, base_speed=12.0, v_ego=LAUNCH_END_SPEED, profile=AccelProfile.normal)
    assert not finished.launching

  def test_far_stopped_lead_does_not_create_stop_hold(self):
    controller = make_controller()
    far_stopped = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=0.0))
    results = [update(controller, far_stopped, base_speed=12.0, v_ego=0.0) for _ in range(4)]
    assert all(result.state != AccelControllerState.stopHold for result in results)

  def test_far_stopped_lead_does_not_use_sticky_braking_history_as_stop_evidence(self):
    controller = make_controller()
    far_stopped = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=0.0))
    for _ in range(controller.lead_loss_hold_frames):
      update(controller, far_stopped, base_speed=12.0, v_ego=10.0, planner_accel=-0.2)
    assert controller.live.braking_limited

    result = update(controller, far_stopped, base_speed=12.0, v_ego=0.2, planner_accel=-0.2)
    assert result.state != AccelControllerState.stopHold
    assert result.target_speed > 0.0

  def test_near_stopped_lead_uses_braking_history_to_hold_completed_stop(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=0.0))
    for _ in range(controller.lead_loss_hold_frames):
      update(controller, stopped, base_speed=12.0, v_ego=10.0, planner_accel=-0.2)
    assert controller.live.braking_limited

    result = update(controller, stopped, base_speed=12.0, v_ego=0.2, planner_accel=-0.2)
    assert result.state == AccelControllerState.stopHold
    assert controller.live.pace == 0.0
    assert result.target_speed == 0.0
    assert result.effective_accel_max == STOP_HOLD_ACCEL_MAX
    assert result.mpc_accel_max is not None and max(result.mpc_accel_max) <= STOP_HOLD_ACCEL_MAX

    stock_limited = update(controller, stopped, base_speed=12.0, v_ego=0.2, stock_accel_max=0.0)
    assert stock_limited.effective_accel_max == 0.0
    assert stock_limited.mpc_accel_max is not None and max(stock_limited.mpc_accel_max) == 0.0

  def test_stop_hold_needs_four_confirmed_departure_frames(self):
    controller = make_controller()
    held = enter_stop_hold(controller)
    assert controller.live.pace == 0.0
    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))
    results = [update(controller, departing, base_speed=8.0, v_ego=0.1) for _ in range(CAP_FILTER_FRAMES + STOP_HOLD_EXIT_FRAMES)]
    launch_index = next(index for index, result in enumerate(results) if result.launching)

    assert held.state == AccelControllerState.stopHold
    assert held.target_speed == 0.0 and held.effective_accel_max == STOP_HOLD_ACCEL_MAX
    assert all(result.state == AccelControllerState.stopHold and not result.launching for result in results[:launch_index])
    assert launch_index == STOP_HOLD_EXIT_FRAMES - 1
    assert results[launch_index].target_speed >= 0.1 + LAUNCH_TARGET_HEADROOM

  def test_reused_radar_does_not_pulse_stop_hold_or_departure_target(self):
    controller = make_controller()
    enter_stop_hold(controller)
    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))

    for frame in range(STOP_HOLD_EXIT_FRAMES):
      fresh = update(controller, departing, base_speed=8.0, v_ego=0.1)
      held = update(controller, departing, base_speed=8.0, v_ego=0.1, radar_fresh=False,
                    previous_mpc_source=LongitudinalPlanSource.lead0, planner_speed=0.01)
      assert held.target_speed == pytest.approx(fresh.target_speed)
      assert held.state == fresh.state
      assert held.selected_lead == fresh.selected_lead == 0
      assert held.effective_accel_max == pytest.approx(fresh.effective_accel_max)
      if frame < STOP_HOLD_EXIT_FRAMES - 1:
        assert fresh.state == AccelControllerState.stopHold
        expected_limit = STOP_HOLD_ACCEL_MAX if frame == 0 else STOP_HOLD_DEPARTURE_ACCEL_MAX
        assert fresh.effective_accel_max == expected_limit

    assert fresh.launching and held.launching
    assert fresh.target_speed == held.target_speed == 8.0

  def test_single_frame_departure_stays_at_the_stationary_hold_limit(self):
    controller = make_controller()
    enter_stop_hold(controller)
    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))
    stopped = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=0.0))

    warm = update(controller, departing, base_speed=8.0, v_ego=0.0)
    held = update(controller, stopped, base_speed=8.0, v_ego=0.0)

    assert warm.state == held.state == AccelControllerState.stopHold
    assert not warm.launching and not held.launching
    assert warm.effective_accel_max == STOP_HOLD_ACCEL_MAX
    assert held.effective_accel_max == STOP_HOLD_ACCEL_MAX
    assert held.target_speed == 0.0

  def test_previous_stop_without_a_lead_does_not_latch_stop_hold(self):
    result = update(
      make_controller(), base_speed=8.0, v_ego=0.0, previous_should_stop=True,
      previous_mpc_source=LongitudinalPlanSource.cruise,
    )

    assert result.state != AccelControllerState.stopHold
    assert result.target_speed >= LAUNCH_TARGET_HEADROOM

  def test_previous_lead_stop_survives_a_fresh_full_field_dropout(self):
    result = update(
      make_controller(), base_speed=8.0, v_ego=0.0, previous_should_stop=True,
      previous_mpc_source=LongitudinalPlanSource.lead0,
    )

    assert result.state == AccelControllerState.stopHold
    assert result.target_speed == 0.0
    assert result.effective_accel_max == 0.0

  def test_stop_hold_without_usable_lead_stays_pinned_to_zero(self):
    controller = make_controller()
    enter_stop_hold(controller)
    missing = update(controller, base_speed=8.0, v_ego=0.1)

    assert missing.state == AccelControllerState.stopHold
    assert missing.target_speed == 0.0
    assert missing.effective_accel_max == 0.0
    assert missing.mpc_accel_max is not None and max(missing.mpc_accel_max) == 0.0

  def test_confirmed_creep_departure_does_not_reenter_stop_hold(self):
    controller = make_controller()
    enter_stop_hold(controller, v_ego=0.0)
    results = []
    for frame in range(60):
      creeping = make_radar(make_lead(status=True, d_rel=6.0 + frame * 0.01, v_lead_k=0.2))
      results.append(update(controller, creeping, base_speed=8.0, v_ego=0.0))

    launch_index = next(index for index, result in enumerate(results) if result.launching)
    assert all(result.state != AccelControllerState.stopHold for result in results[launch_index:])
    assert all(result.target_speed > 0.0 for result in results[launch_index:])

  def test_departure_dropout_holds_without_resurrecting_stop_hold(self):
    controller = make_controller()
    enter_stop_hold(controller)
    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))
    results = [update(controller, departing, base_speed=8.0, v_ego=0.1) for _ in range(CAP_FILTER_FRAMES + STOP_HOLD_EXIT_FRAMES)]
    launched = next(result for result in results if result.launching)
    before_dropout = results[-1]
    dropout = [update(controller, base_speed=8.0, v_ego=0.1) for _ in range(controller.lead_loss_hold_frames + 1)]

    assert launched.launching
    assert all(result.state != AccelControllerState.stopHold for result in dropout)
    assert all(result.target_speed <= before_dropout.target_speed + 1e-9 for result in dropout[:controller.lead_loss_hold_frames - 1])
    assert dropout[controller.lead_loss_hold_frames - 1].target_speed > before_dropout.target_speed
    assert dropout[-1].launching

  def test_invalid_departure_geometry_returns_to_stop_hold(self):
    controller = make_controller()
    enter_stop_hold(controller)
    departing = make_radar(make_lead(status=True, d_rel=8.0, v_lead_k=2.0))
    for _ in range(CAP_FILTER_FRAMES + STOP_HOLD_EXIT_FRAMES):
      launched = update(controller, departing, base_speed=8.0, v_ego=0.1)
    invalid = make_radar(make_lead(status=True, d_rel=math.nan, v_lead_k=2.0))
    guarded = update(controller, invalid, base_speed=8.0, v_ego=0.1)
    assert launched.launching
    assert guarded.state == AccelControllerState.stopHold
    assert guarded.target_speed == 0.0

  def test_stale_timeout_fully_resets_live_and_shadow(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES + 10):
      restricted = update(controller, restrictive_radar())
    stale_frames = math.ceil(RADAR_STALE_TIMEOUT / DT_MDL)
    held = [update(controller, radar_fresh=False) for _ in range(stale_frames - 1)]
    timed_out = update(controller, radar_fresh=False)

    assert all(result.active and result.target_speed == pytest.approx(restricted.target_speed) for result in held)
    assert not timed_out.active and not timed_out.shadow_active
    assert timed_out.target_speed == timed_out.base_speed
    assert timed_out.mpc_accel_max is None
    assert timed_out.selected_lead == -1 and math.isinf(timed_out.raw_energy_cap)
    assert controller.live.pace is None and controller.shadow.pace is None

  @pytest.mark.parametrize("override", [{"enabled": False}, {"acc_selected": False}, {"engaged": False}, {"cruise_initialized": False}, {"a_ego": math.inf}])
  def test_bypass_or_invalid_context_resets_live_state(self, override):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, restrictive_radar())
    result = update(controller, restrictive_radar(), **override)

    assert not result.active
    assert result.target_speed == result.base_speed
    assert result.mpc_accel_max is None
    assert controller.live.pace is None and controller.live.accel_limit is None

  def test_shadow_history_never_steps_into_live_actuation(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES + 20):
      shadow = update(controller, restrictive_radar(), acc_selected=False)
    live = update(controller)

    assert shadow.shadow_active and not shadow.active
    assert shadow.shadow_filtered_cap < math.inf
    assert live.active and live.target_speed == live.base_speed
    assert math.isinf(live.live_filtered_cap)

  def test_explicit_reset_clears_every_path_field(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES + 10):
      update(controller, restrictive_radar())
    controller.reset()

    assert controller._held_envelope is None
    for path in (controller.live, controller.shadow):
      assert path.pace is None and path.accel_limit is None and path.matched_accel_limit is None
      assert path.state == AccelControllerState.inactive
      assert path.departure_frames == path.active_frames == path.lead_loss_frames == path.stale_frames == 0
      assert not path.launching and not path.departure_launch and not path.matched_lead and not path.braking_limited
      assert math.isinf(path.filtered_cap) and math.isinf(path.filtered_lead_speed) and path.filtered_lead_accel == 0.0
