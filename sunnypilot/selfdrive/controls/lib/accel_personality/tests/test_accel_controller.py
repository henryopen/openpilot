import math
from types import SimpleNamespace

import numpy as np
import pytest

from cereal import log
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource, STOP_DISTANCE, T_IDXS, get_T_FOLLOW
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import (
  ACCEL_LIMIT_BYPASS_RATE,
  ACCEL_LIMIT_RAISE_RATE,
  ACCEL_PROFILE_MAX_BP,
  ACCEL_PROFILE_MAX_V,
  ACCEL_SHAPE_WARMUP_FRAMES,
  CAP_FILTER_FRAMES,
  MOVING_LEAD_DECEL_ACCEL_MAX,
  MOVING_LEAD_DECEL_ACCEL_SLEW_RATE,
  MOVING_LEAD_DECEL_CONFIRM_FRAMES,
  MOVING_LEAD_DECEL_EXIT_FRAMES,
  STOP_HOLD_EXIT_FRAMES,
  AccelController,
  AccelControllerState,
  AccelProfile,
  EnergyEnvelope,
  PROFILE_CONFIGS,
)


def make_lead(
  *,
  status: bool = False,
  d_rel: float = 0.0,
  v_lead_k: float = 0.0,
  a_lead_k: float = 0.0,
  a_lead_tau: float = 1.5,
  radar_track_id: int = -1,
):
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


def make_controller(delay: float = 0.10):
  return AccelController(SimpleNamespace(longitudinalActuatorDelay=delay))


def update(controller, radar_state=None, **overrides):
  args = {
    "base_speed": 20.0,
    "v_ego": 10.0,
    "a_ego": 0.0,
    "profile": AccelProfile.normal,
    "follow_personality": log.LongitudinalPersonality.standard,
    "enabled": True,
    "acc_selected": True,
    "engaged": True,
    "cruise_initialized": True,
    "previous_mpc_source": LongitudinalPlanSource.cruise,
    "planner_speed": 10.0,
    "stock_accel_max": 1.2,
    "planner_accel": 0.0,
    "previous_should_stop": False,
  }
  args.update(overrides)
  return controller.update(radar_state or make_radar(), **args)


class TestAccelProfile:
  def test_lookup_table_values(self):
    assert ACCEL_PROFILE_MAX_BP == [0.0, 3.0, 10.0, 25.0, 40.0]
    assert ACCEL_PROFILE_MAX_V == {
      AccelProfile.eco: [1.55, 1.25, 0.85, 0.50, 0.30],
      AccelProfile.normal: [1.70, 1.40, 1.05, 0.65, 0.45],
      AccelProfile.sport: [2.00, 1.90, 1.70, 1.20, 0.90],
    }

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_lookup_table_interpolates_and_clamps(self, profile):
    for speed, expected in zip(ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V[profile], strict=True):
      assert AccelController.get_profile_accel_max(profile, speed) == expected

    assert AccelController.get_profile_accel_max(profile, -1.0) == ACCEL_PROFILE_MAX_V[profile][0]
    assert AccelController.get_profile_accel_max(profile, 50.0) == ACCEL_PROFILE_MAX_V[profile][-1]

  @pytest.mark.parametrize("profile", list(AccelProfile))
  def test_lookup_table_stays_within_global_limit(self, profile):
    for speed in np.linspace(0.0, 40.0, 161):
      assert 0.0 <= AccelController.get_profile_accel_max(profile, speed) <= ACCEL_MAX

  def test_launch_knots_are_prompt_and_profiles_separate_after_walking_speed(self):
    launch_limits = [AccelController.get_profile_accel_max(profile, 0.0) for profile in AccelProfile]
    rolling_limits = [AccelController.get_profile_accel_max(profile, 10.0) for profile in AccelProfile]

    assert min(launch_limits) >= 1.55
    assert rolling_limits[0] < rolling_limits[1] < rolling_limits[2]

  def test_active_profile_is_clipped_to_stock_before_mpc(self):
    controller = make_controller()
    results = [update(controller, profile=AccelProfile.sport, stock_accel_max=1.2) for _ in range(12)]

    assert all(result.profile_accel_max == 1.7 for result in results)
    assert all(result.mpc_accel_max is None for result in results[:ACCEL_SHAPE_WARMUP_FRAMES])
    assert results[-1].effective_accel_max == 1.2
    assert results[-1].mpc_accel_max == tuple(1.2 for _ in T_IDXS)
    assert results[-1].mpc_shape_cruise
    assert results[-1].mpc_apply_accel_constraint
    assert max(results[-1].mpc_accel_max) <= min(1.2, ACCEL_MAX)

  @pytest.mark.parametrize("profile", list(AccelProfile))
  @pytest.mark.parametrize("stock_accel_max", [0.6, 1.2, ACCEL_MAX])
  def test_settled_mpc_limit_respects_profile_stock_and_global_ceiling(self, profile, stock_accel_max):
    controller = make_controller()
    result = [update(controller, profile=profile, stock_accel_max=stock_accel_max) for _ in range(40)][-1]
    expected = min(AccelController.get_profile_accel_max(profile, 10.0), stock_accel_max, ACCEL_MAX)

    assert result.effective_accel_max == pytest.approx(expected)
    assert result.mpc_accel_max is not None
    assert max(result.mpc_accel_max) <= expected
    assert result.mpc_apply_accel_constraint

  def test_profiles_produce_distinct_effective_limits(self):
    results = [update(make_controller(), profile=profile, stock_accel_max=1.2) for profile in AccelProfile]

    assert [result.effective_accel_max for result in results] == [0.85, 1.05, 1.2]
    assert all(result.mpc_accel_max is None for result in results)

  def test_nonpositive_stock_limit_does_not_create_custom_braking_horizon(self):
    result = update(make_controller(), stock_accel_max=-0.2)

    assert result.effective_accel_max == -0.2
    assert result.mpc_accel_max is None
    assert not result.mpc_shape_cruise

  def test_profile_switch_reaches_new_limit_without_a_step(self):
    controller = make_controller()
    eco = [update(controller, profile=AccelProfile.eco) for _ in range(20)]
    sport = [update(controller, profile=AccelProfile.sport) for _ in range(8)]
    limits = np.array([eco[-1].effective_accel_max, *(result.effective_accel_max for result in sport)])

    assert eco[-1].effective_accel_max == 0.85
    assert sport[-1].effective_accel_max == 1.2
    assert np.all(np.diff(limits) >= 0.0)
    assert np.max(np.diff(limits)) <= ACCEL_LIMIT_RAISE_RATE * DT_MDL + 1e-9

  def test_lead_transitions_preserve_profile_without_constraining_a_closing_lead(self):
    controller = make_controller()
    clear = [update(controller, profile=AccelProfile.eco) for _ in range(20)]
    faster_lead = make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=15.0))
    nonclosing = update(controller, faster_lead, profile=AccelProfile.eco)

    assert clear[-1].effective_accel_max == pytest.approx(ACCEL_PROFILE_MAX_V[AccelProfile.eco][2])
    assert nonclosing.effective_accel_max == clear[-1].effective_accel_max
    assert nonclosing.mpc_apply_accel_constraint

    closing_lead = make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=5.0))
    bypass = [update(controller, closing_lead, profile=AccelProfile.eco) for _ in range(20)]

    transition_bounds = [result.mpc_accel_max[0] for result in bypass if result.mpc_accel_max is not None]
    assert transition_bounds[0] == pytest.approx(nonclosing.effective_accel_max + ACCEL_LIMIT_BYPASS_RATE * DT_MDL)
    assert np.max(np.diff(transition_bounds)) <= ACCEL_LIMIT_BYPASS_RATE * DT_MDL + 1e-9
    assert all(not result.mpc_apply_accel_constraint for result in bypass)
    assert bypass[-1].mpc_accel_max is None
    assert not bypass[-1].mpc_shape_cruise

    rejoin = [update(controller, profile=AccelProfile.eco) for _ in range(20)]
    rejoin_bounds = np.array([result.mpc_accel_max[0] for result in rejoin])

    assert all(result.mpc_apply_accel_constraint for result in rejoin)
    assert rejoin_bounds[0] == pytest.approx(ACCEL_MAX - ACCEL_LIMIT_BYPASS_RATE * DT_MDL)
    assert np.max(np.abs(np.diff(rejoin_bounds))) <= ACCEL_LIMIT_BYPASS_RATE * DT_MDL + 1e-9
    assert rejoin_bounds[-1] == ACCEL_PROFILE_MAX_V[AccelProfile.eco][2]
    assert np.max(rejoin_bounds) <= ACCEL_MAX


class TestEnergyEnvelope:
  def test_relative_energy_formula(self):
    controller = make_controller()
    lead = make_lead(status=True, d_rel=60.0, v_lead_k=10.0)
    envelope = controller.calculate_energy_envelope(make_radar(lead), 20.0, 0.0, AccelProfile.normal)

    delay = controller.CP.longitudinalActuatorDelay + DT_MDL
    x_ego = 20.0 * delay
    x_lead = lead.dRel + lead.vLeadK * delay
    usable_gap = x_lead - x_ego - STOP_DISTANCE - get_T_FOLLOW() * lead.vLeadK
    expected = lead.vLeadK + math.sqrt(2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * usable_gap)
    incorrect = math.sqrt(lead.vLeadK**2 + 2.0 * PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * usable_gap)

    assert envelope.usable_gap == pytest.approx(usable_gap)
    assert envelope.cap == pytest.approx(expected)
    assert envelope.cap != pytest.approx(incorrect)

  def test_more_restrictive_lead_is_selected(self):
    leads = make_radar(
      make_lead(status=True, d_rel=100.0, v_lead_k=18.0),
      make_lead(status=True, d_rel=35.0, v_lead_k=5.0),
    )

    envelope = make_controller().calculate_energy_envelope(leads, 20.0, 0.0, AccelProfile.normal)

    assert envelope.selected_lead == 1

  def test_profiles_order_early_deceleration(self):
    radar = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0))
    caps = [make_controller().calculate_energy_envelope(radar, 20.0, 0.0, profile).cap for profile in AccelProfile]

    assert caps[0] < caps[1] < caps[2]

  def test_follow_personality_remains_stock_authority(self):
    controller = make_controller()
    radar = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0))
    aggressive = controller.calculate_energy_envelope(radar, 20.0, 0.0, AccelProfile.normal, log.LongitudinalPersonality.aggressive)
    relaxed = controller.calculate_energy_envelope(radar, 20.0, 0.0, AccelProfile.normal, log.LongitudinalPersonality.relaxed)

    assert relaxed.usable_gap < aggressive.usable_gap
    assert relaxed.cap < aggressive.cap

  def test_lead_acceleration_is_clipped_before_projection(self):
    controller = make_controller(delay=0.30)
    extreme = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0, a_lead_k=-100.0))
    clipped = make_radar(make_lead(status=True, d_rel=60.0, v_lead_k=10.0, a_lead_k=-10.0))

    assert controller.calculate_energy_envelope(extreme, 20.0, 0.0, 1) == controller.calculate_energy_envelope(clipped, 20.0, 0.0, 1)

  def test_ego_projection_stops_at_zero(self):
    assert AccelController._project_ego(0.2, -4.0, 0.15) == pytest.approx((0.005, 0.0))

  @pytest.mark.parametrize("field", ["dRel", "vLeadK", "aLeadK", "aLeadTau"])
  @pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf])
  def test_nonfinite_lead_values_are_ignored(self, field, invalid):
    lead = make_lead(status=True, d_rel=40.0, v_lead_k=10.0)
    setattr(lead, field, invalid)

    envelope = make_controller().calculate_energy_envelope(make_radar(lead), 20.0, 0.0, AccelProfile.normal)

    assert envelope == EnergyEnvelope()

  @pytest.mark.parametrize(
    "field, invalid",
    [
      ("dRel", -0.01),
      ("vLeadK", -1.0),
      ("aLeadTau", 0.0),
      ("aLeadTau", -0.01),
      ("aLeadTau", 1e6),
    ],
  )
  def test_physically_invalid_lead_values_are_ignored(self, field, invalid):
    lead = make_lead(status=True, d_rel=40.0, v_lead_k=10.0)
    setattr(lead, field, invalid)

    envelope = make_controller().calculate_energy_envelope(make_radar(lead), 20.0, 0.0, AccelProfile.normal)

    assert envelope == EnergyEnvelope()

  @pytest.mark.parametrize(
    "malformed_lead",
    [
      SimpleNamespace(status=True, dRel=40.0, vLeadK=10.0, aLeadK=0.0),
      SimpleNamespace(status=True, dRel="invalid", vLeadK=10.0, aLeadK=0.0, aLeadTau=1.5),
      SimpleNamespace(status=True, dRel=40.0, vLeadK=None, aLeadK=0.0, aLeadTau=1.5),
    ],
  )
  def test_malformed_lead_does_not_hide_valid_second_lead(self, malformed_lead):
    valid_lead = make_lead(status=True, d_rel=35.0, v_lead_k=5.0)

    envelope = make_controller().calculate_energy_envelope(
      make_radar(malformed_lead, valid_lead),
      20.0,
      0.0,
      AccelProfile.normal,
    )

    assert envelope.selected_lead == 1
    assert math.isfinite(envelope.cap)

  @pytest.mark.parametrize("invalid_column", [0, 1])
  @pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf])
  def test_nonfinite_projected_lead_is_ignored(self, monkeypatch, invalid_column, invalid):
    projected = np.column_stack((np.full_like(T_IDXS, 40.0), np.full_like(T_IDXS, 10.0)))
    projected[:, invalid_column] = invalid
    monkeypatch.setattr(LongitudinalMpc, "extrapolate_lead", staticmethod(lambda *_args: projected))

    envelope = make_controller().calculate_energy_envelope(
      make_radar(make_lead(status=True, d_rel=40.0, v_lead_k=10.0)),
      20.0,
      0.0,
      AccelProfile.normal,
    )

    assert envelope == EnergyEnvelope()


class TestPaceGovernor:
  restrictive_lead = make_lead(status=True, d_rel=40.0, v_lead_k=5.0)
  nonrestrictive_lead = make_lead(status=True, d_rel=100.0, v_lead_k=15.0)
  gentle_restrictive_lead = make_lead(status=True, d_rel=60.0, v_lead_k=8.0)

  @classmethod
  def establish_gentle_restriction(cls, controller, frames=6):
    for _ in range(CAP_FILTER_FRAMES // 2 + 1):
      update(controller, make_radar(cls.nonrestrictive_lead))
    return [update(controller, make_radar(cls.gentle_restrictive_lead)) for _ in range(frames)]

  def test_clear_road_targets_base_immediately(self):
    controller = make_controller()
    result = update(controller, base_speed=25.0, v_ego=0.0, planner_speed=0.0, stock_accel_max=1.6)
    settled = [update(controller, base_speed=25.0, v_ego=0.0, planner_speed=0.0, stock_accel_max=1.6) for _ in range(13)]

    assert result.target_speed == 25.0
    assert result.live_pace == 25.0
    assert result.effective_accel_max == 0.95
    assert settled[-1].effective_accel_max == 1.6
    assert all(sample.mpc_accel_max is not None for sample in [result, *settled])
    assert all(max(sample.mpc_accel_max) <= min(sample.profile_accel_max, 1.6, ACCEL_MAX) for sample in [result, *settled])
    assert all(sample.target_speed == 25.0 for sample in settled)

  def test_median_needs_three_leads_and_holds_two_dropouts(self):
    controller = make_controller()
    radar = make_radar(self.restrictive_lead)
    acquisitions = [update(controller, radar) for _ in range(3)]
    dropouts = [update(controller) for _ in range(3)]

    assert all(math.isinf(result.live_filtered_cap) for result in acquisitions[:2])
    assert math.isfinite(acquisitions[2].live_filtered_cap)
    assert all(math.isfinite(result.live_filtered_cap) for result in dropouts[:2])
    assert math.isinf(dropouts[2].live_filtered_cap)

  def test_invalid_lead_expires_and_cannot_resurrect_filtered_geometry(self):
    controller = make_controller()
    valid = make_radar(self.restrictive_lead)
    invalid = make_radar(make_lead(status=True, d_rel=-1.0, v_lead_k=5.0))

    acquired = [update(controller, valid) for _ in range(CAP_FILTER_FRAMES // 2 + 1)]
    expired = [update(controller, invalid) for _ in range(CAP_FILTER_FRAMES // 2 + 1)]
    reacquired = [update(controller, valid) for _ in range(CAP_FILTER_FRAMES // 2 + 1)]

    assert math.isfinite(acquired[-1].live_filtered_cap)
    assert math.isinf(expired[-1].live_filtered_cap)
    assert all(math.isinf(result.live_filtered_cap) for result in reacquired[:-1])
    assert math.isfinite(reacquired[-1].live_filtered_cap)

  @pytest.mark.parametrize(
    "field, invalid",
    [
      ("dRel", -1.0),
      ("vLeadK", -1.0),
      ("aLeadTau", -1.0),
      ("aLeadTau", math.inf),
    ],
  )
  def test_invalid_lead_never_produces_nan_telemetry(self, field, invalid):
    lead = make_lead(status=True, d_rel=40.0, v_lead_k=5.0)
    setattr(lead, field, invalid)

    result = update(make_controller(), make_radar(lead))

    telemetry = (
      result.target_speed,
      result.profile_accel_max,
      result.effective_accel_max,
      result.raw_energy_cap,
      result.live_filtered_cap,
      result.live_pace,
      result.closing_speed,
      result.required_decel,
    )
    assert not any(math.isnan(value) for value in telemetry)

  def test_small_negative_stopped_lead_speed_is_treated_as_zero(self):
    lead = make_lead(status=True, d_rel=6.0, v_lead_k=-0.01)

    result = update(make_controller(), make_radar(lead), base_speed=8.0, v_ego=0.1, planner_speed=0.1)

    assert result.selected_lead == 0
    assert result.raw_energy_cap == 0.0
    assert result.state == AccelControllerState.stopHold

  def test_restriction_changes_only_pace_at_comfort_rate(self):
    controller = make_controller()
    results = self.establish_gentle_restriction(controller, frames=3)
    restricted = results[-1]

    assert all(result.live_pace == 20.0 for result in results[:2])
    assert restricted.live_pace == pytest.approx(20.0 - PROFILE_CONFIGS[AccelProfile.normal].comfort_decel * DT_MDL)
    assert restricted.state == AccelControllerState.restrict
    assert restricted.mpc_accel_max is None
    assert not restricted.mpc_shape_cruise
    assert not restricted.mpc_apply_accel_constraint

  def test_far_slower_lead_acquisition_is_bounded_then_holds_for_mpc(self):
    controller = make_controller()
    radar = make_radar(make_lead(status=True, d_rel=200.0, v_lead_k=15.0))
    results = [update(controller, radar, base_speed=30.0, v_ego=25.0, planner_speed=25.0) for _ in range(3)]

    assert all(result.live_pace == 30.0 for result in results[:2])
    assert 24.5 <= results[-1].live_pace < 25.0
    assert results[-1].state == AccelControllerState.hold
    assert results[-1].mpc_accel_max is None
    assert not results[-1].mpc_apply_accel_constraint

  def test_material_closing_lead_returns_acceleration_authority_to_stock(self):
    controller = make_controller()
    radar = make_radar(self.restrictive_lead)
    results = [update(controller, radar) for _ in range(20)]

    assert all(0.0 < result.effective_accel_max <= min(result.profile_accel_max, 1.2, ACCEL_MAX) for result in results)
    assert all(result.mpc_accel_max is None for result in results)
    assert all(not result.mpc_shape_cruise for result in results)
    assert all(not result.mpc_apply_accel_constraint for result in results)

  def test_route_threshold_crossing_keeps_direct_stock_authority(self):
    controller = make_controller()
    leads = [
      make_radar(make_lead(status=True, d_rel=31.0, v_lead_k=5.0)),
      make_radar(make_lead(status=True, d_rel=29.0, v_lead_k=5.0)),
    ]
    results = [update(controller, radar, v_ego=10.0, planner_speed=10.0) for radar in leads]

    assert all(result.mpc_accel_max is None for result in results)
    assert all(not result.mpc_shape_cruise for result in results)
    assert all(not result.mpc_apply_accel_constraint for result in results)

  def test_decelerating_moving_lead_uses_smooth_ceiling_until_confirmed_match(self):
    controller = make_controller()

    def moving_lead(speed):
      return make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=speed))

    update(controller, moving_lead(15.0), base_speed=30.0, v_ego=20.0, planner_speed=20.0)
    confirmation = []
    for frame in range(MOVING_LEAD_DECEL_CONFIRM_FRAMES):
      confirmation.append(
        update(
          controller,
          moving_lead(15.0 - 0.01 * (frame + 1)),
          base_speed=30.0,
          v_ego=20.0,
          planner_speed=20.0,
        )
      )
      if frame < MOVING_LEAD_DECEL_CONFIRM_FRAMES - 1:
        assert not controller.live.moving_lead_decel

    assert controller.live.moving_lead_decel
    assert confirmation[-1].effective_accel_max == pytest.approx(-MOVING_LEAD_DECEL_ACCEL_SLEW_RATE * DT_MDL)
    assert confirmation[-1].mpc_apply_accel_constraint

    falling_speed = 15.0 - 0.01 * MOVING_LEAD_DECEL_CONFIRM_FRAMES
    constrained = [update(controller, moving_lead(falling_speed - 0.01 * (frame + 1)), base_speed=30.0, v_ego=20.0, planner_speed=20.0) for frame in range(20)]
    limits = np.array([result.effective_accel_max for result in [confirmation[-1], *constrained]])
    assert constrained[-1].effective_accel_max == MOVING_LEAD_DECEL_ACCEL_MAX
    assert np.all(np.diff(limits) <= 0.0)
    assert np.max(np.abs(np.diff(limits))) <= MOVING_LEAD_DECEL_ACCEL_SLEW_RATE * DT_MDL + 1e-9

    matched = [update(controller, moving_lead(5.0), base_speed=30.0, v_ego=5.05, planner_speed=5.05) for _ in range(MOVING_LEAD_DECEL_EXIT_FRAMES)]
    assert all(result.effective_accel_max <= 0.0 for result in matched[:-1])
    assert not controller.live.moving_lead_decel
    assert controller.live.moving_lead_accel_max is None
    assert matched[-1].effective_accel_max > 0.0

  def test_far_or_noisy_decelerating_lead_does_not_force_braking(self):
    far_controller = make_controller()
    far_results = [
      update(
        far_controller,
        make_radar(make_lead(status=True, d_rel=500.0, v_lead_k=15.0 - 0.01 * frame)),
        base_speed=30.0,
        v_ego=20.0,
        planner_speed=20.0,
      )
      for frame in range(10)
    ]

    noisy_controller = make_controller()
    noisy_results = [
      update(
        noisy_controller,
        make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=15.0 - 0.01 * (frame % 2))),
        base_speed=30.0,
        v_ego=20.0,
        planner_speed=20.0,
      )
      for frame in range(10)
    ]

    assert not far_controller.live.moving_lead_decel
    assert not noisy_controller.live.moving_lead_decel
    assert all(result.effective_accel_max >= 0.0 for result in [*far_results, *noisy_results])

  def test_release_waits_then_raises_pace_at_profile_rate(self):
    controller = make_controller()
    restricted = self.establish_gentle_restriction(controller, frames=30)[-1]
    restricted_pace = restricted.live_pace

    confirmation_frames = math.ceil(PROFILE_CONFIGS[AccelProfile.normal].release_confirm / DT_MDL)
    results = [update(controller) for _ in range(CAP_FILTER_FRAMES // 2 + confirmation_frames)]
    released = results[-1]

    assert all(result.state == AccelControllerState.hold for result in results[:-1])
    assert all(result.live_pace == restricted_pace for result in results[:-1])
    assert released.state == AccelControllerState.release
    assert released.live_pace > restricted_pace
    assert released.live_pace == pytest.approx(restricted_pace + PROFILE_CONFIGS[AccelProfile.normal].release_rate * DT_MDL)

  def test_confirmed_clear_finishes_release_at_base(self):
    controller = make_controller()
    self.establish_gentle_restriction(controller, frames=30)

    results = [update(controller) for _ in range(200)]

    assert results[-1].state == AccelControllerState.free
    assert results[-1].target_speed == results[-1].base_speed

  def test_cruise_source_dropout_holds_pace_without_a_target_lurch(self):
    controller = make_controller()
    restricted = self.establish_gentle_restriction(controller)[-1]

    dropout = update(
      controller,
      previous_mpc_source=LongitudinalPlanSource.cruise,
      planner_speed=7.0,
      v_ego=8.0,
    )

    assert math.isfinite(dropout.live_filtered_cap)
    assert dropout.state == AccelControllerState.hold
    assert dropout.live_pace == restricted.live_pace
    assert dropout.target_speed == restricted.target_speed

  def test_far_geometry_jump_cannot_release_filtered_restriction(self):
    controller = make_controller()
    far = make_radar(make_lead(status=True, d_rel=200.0, v_lead_k=20.0))
    restricted = self.establish_gentle_restriction(controller)[-1]

    jumped = update(controller, far, planner_speed=8.0, v_ego=9.0)

    assert math.isfinite(jumped.live_filtered_cap)
    assert jumped.state == AccelControllerState.hold
    assert jumped.live_pace == restricted.live_pace
    assert jumped.target_speed == restricted.target_speed

  def test_previous_lead_plan_synchronizes_pace_after_lead_loss(self):
    controller = make_controller()
    self.establish_gentle_restriction(controller)

    lost = update(
      controller,
      previous_mpc_source=LongitudinalPlanSource.lead0,
      planner_speed=7.0,
    )

    assert lost.state == AccelControllerState.hold
    assert lost.live_pace == 7.0
    assert lost.target_speed == 7.0

  def test_existing_lead_plan_keeps_relative_pace_restriction(self):
    controller = make_controller()
    restricted = self.establish_gentle_restriction(controller)[-1]
    radar = make_radar(self.gentle_restrictive_lead)

    handed_off = update(
      controller,
      radar,
      previous_mpc_source=LongitudinalPlanSource.lead0,
      planner_speed=restricted.live_pace,
    )

    assert restricted.live_pace < restricted.base_speed
    assert handed_off.state == AccelControllerState.restrict
    assert handed_off.live_pace <= restricted.live_pace
    assert handed_off.target_speed < handed_off.base_speed
    assert handed_off.mpc_accel_max is None
    assert not handed_off.mpc_apply_accel_constraint

  def test_stop_hold_requires_four_moving_lead_frames_then_targets_base(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    held = update(controller, stopped, base_speed=8.0, v_ego=0.1, planner_speed=0.1, stock_accel_max=1.6)
    waiting = [update(controller, stopped, base_speed=8.0, v_ego=0.1, planner_speed=0.1, stock_accel_max=1.6) for _ in range(13)]
    confirmations = [update(controller, moving, base_speed=8.0, v_ego=0.1, planner_speed=0.1, stock_accel_max=1.6) for _ in range(STOP_HOLD_EXIT_FRAMES)]
    released = update(controller, moving, base_speed=8.0, v_ego=0.1, planner_speed=0.1, stock_accel_max=1.6)

    assert held.state == AccelControllerState.stopHold
    assert held.target_speed == 0.0
    held_results = [held, *waiting, *confirmations[:-1]]
    assert all(result.effective_accel_max == 0.0 for result in held_results)
    assert all(result.mpc_accel_max[0] < 0.0 and result.mpc_accel_max[-1] == ACCEL_MAX for result in held_results)
    assert all(result.mpc_shape_cruise for result in held_results)
    assert all(result.mpc_apply_accel_constraint for result in held_results)
    assert all(result.target_speed == 0.0 for result in confirmations[:-1])
    assert confirmations[-1].target_speed > 0.0
    assert confirmations[-1].effective_accel_max <= ACCEL_MAX
    assert confirmations[-1].mpc_accel_max is None
    assert confirmations[-1].launching
    assert released.target_speed >= confirmations[-1].target_speed
    assert released.launching

  def test_slow_queue_creep_exits_after_four_frames(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    creeping = make_radar(make_lead(status=True, d_rel=6.1, v_lead_k=0.5))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, planner_speed=0.1)
    results = [update(controller, creeping, base_speed=8.0, v_ego=0.1, planner_speed=0.1) for _ in range(STOP_HOLD_EXIT_FRAMES)]
    released = update(controller, creeping, base_speed=8.0, v_ego=0.1, planner_speed=0.1)

    assert all(result.state == AccelControllerState.stopHold for result in results[:-1])
    assert results[-1].state == AccelControllerState.release
    assert results[-1].target_speed > 0.0
    assert released.target_speed > 0.0

  def test_far_stopped_lead_does_not_enter_stop_hold(self):
    far_stopped = make_radar(make_lead(status=True, d_rel=100.0, v_lead_k=0.0))

    result = update(make_controller(), far_stopped, base_speed=8.0, v_ego=0.1, planner_speed=0.1)

    assert result.state != AccelControllerState.stopHold
    assert result.target_speed == 8.0

  def test_departure_confirmation_must_be_consecutive(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, planner_speed=0.1)
    for _ in range(STOP_HOLD_EXIT_FRAMES - 1):
      update(controller, moving, base_speed=8.0, v_ego=0.1, planner_speed=0.1)
    interrupted = update(controller, stopped, base_speed=8.0, v_ego=0.1, planner_speed=0.1)

    assert interrupted.state == AccelControllerState.stopHold
    assert controller.live.departure_frames == 0

  def test_stale_should_stop_does_not_reenter_after_confirmed_departure(self):
    controller = make_controller()
    stopped = make_radar(make_lead(status=True, d_rel=6.0, v_lead_k=0.0))
    moving = make_radar(make_lead(status=True, d_rel=20.0, v_lead_k=5.0))
    update(controller, stopped, base_speed=8.0, v_ego=0.1, planner_speed=0.1, previous_should_stop=True)
    for _ in range(STOP_HOLD_EXIT_FRAMES):
      result = update(
        controller,
        moving,
        base_speed=8.0,
        v_ego=0.1,
        planner_speed=0.1,
        previous_should_stop=True,
      )
    released = update(
      controller,
      moving,
      base_speed=8.0,
      v_ego=0.1,
      planner_speed=0.1,
      previous_should_stop=True,
    )

    assert result.state != AccelControllerState.stopHold
    assert result.target_speed > 0.0
    assert released.state != AccelControllerState.stopHold
    assert released.target_speed > 0.0

  def test_live_never_adopts_shadow_history(self):
    controller = make_controller()
    for _ in range(CAP_FILTER_FRAMES // 2 + 1):
      update(controller, make_radar(self.nonrestrictive_lead), enabled=False)
    for _ in range(CAP_FILTER_FRAMES // 2 + 1):
      shadow = update(controller, make_radar(self.gentle_restrictive_lead), enabled=False)
    active = update(controller, enabled=True)

    assert shadow.shadow_state == AccelControllerState.restrict
    assert active.target_speed == active.base_speed
    assert active.live_pace == active.base_speed

  @pytest.mark.parametrize("bypass", [{"enabled": False}, {"acc_selected": False}, {"engaged": False}])
  def test_bypass_is_exact_base_and_resets_live(self, bypass):
    controller = make_controller()
    update(controller, make_radar(self.restrictive_lead))
    result = update(controller, **bypass)

    assert not result.active
    assert result.target_speed == result.base_speed
    assert result.mpc_accel_max is None
    assert math.isinf(result.effective_accel_max)
    assert controller.live.pace is None

  @pytest.mark.parametrize("invalid", [{"base_speed": math.nan}, {"stock_accel_max": math.nan}, {"controller_fault": True}])
  def test_invalid_context_resets_without_changing_base(self, invalid):
    controller = make_controller()
    update(controller)
    result = update(controller, **invalid)

    assert not result.active
    assert controller.live.pace is None
    if "base_speed" not in invalid:
      assert result.target_speed == result.base_speed

    recovered = update(controller)
    assert recovered.active
    assert recovered.live_pace == recovered.base_speed

  def test_fault_recovery_is_discarded_when_context_is_invalid(self):
    controller = make_controller()
    for _ in range(ACCEL_SHAPE_WARMUP_FRAMES + 1):
      update(controller)
    assert controller.recovery_accel_max is not None

    result = update(controller, controller_fault=True, base_speed=math.nan)
    assert not result.active
    assert result.mpc_accel_max is None
    assert not controller.recovery_available

  def test_invalid_profile_defaults_to_normal(self):
    result = update(make_controller(), profile=99)

    assert result.profile == AccelProfile.normal

  def test_small_negative_ego_noise_is_sanitized(self):
    result = update(make_controller(), v_ego=-0.05, planner_speed=0.0, stock_accel_max=1.6)

    assert result.active
    assert result.profile_accel_max == ACCEL_PROFILE_MAX_V[AccelProfile.normal][0]

  def test_radar_input_is_not_mutated(self):
    lead = make_lead(status=True, d_rel=60.0, v_lead_k=10.0, a_lead_k=-2.0)
    radar = make_radar(lead)
    before = vars(lead).copy()

    update(make_controller(), radar)

    assert vars(lead) == before
