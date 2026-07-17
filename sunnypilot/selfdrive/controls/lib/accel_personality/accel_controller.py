#!/usr/bin/env python3
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
import math

import numpy as np

from cereal import log
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc,
  LongitudinalPlanSource,
  STOP_DISTANCE,
  T_IDXS,
  get_T_FOLLOW,
  get_stopped_equivalence_factor,
)


class AccelProfile(IntEnum):
  eco = 0
  normal = 1
  sport = 2


class AccelControllerState(IntEnum):
  inactive = 0
  free = 1
  restrict = 2
  hold = 3
  release = 4
  stopHold = 5


@dataclass(frozen=True)
class ProfileConfig:
  comfort_decel: float
  release_rate: float
  release_confirm: float


PROFILE_CONFIGS = {
  AccelProfile.eco: ProfileConfig(comfort_decel=0.25, release_rate=0.90, release_confirm=0.50),
  AccelProfile.normal: ProfileConfig(comfort_decel=0.335, release_rate=1.15, release_confirm=0.35),
  AccelProfile.sport: ProfileConfig(comfort_decel=0.50, release_rate=1.45, release_confirm=0.20),
}

ACCEL_PROFILE_MAX_BP = [0.0, 10.0, 25.0, 40.0]
# Pre-MPC profile requests; stock output limits remain authoritative.
ACCEL_PROFILE_MAX_V = {
  AccelProfile.eco: [1.55, 0.85, 0.45, 0.25],
  AccelProfile.normal: [1.65, 1.10, 0.70, 0.45],
  AccelProfile.sport: [2.00, 1.70, 1.20, 0.90],
}
LAUNCH_DELTA_V = 3.0
LAUNCH_TARGET_HANDOFF_SPEED = 1.0

CAP_FILTER_FRAMES = 5
RESTRICT_DEADBAND = 0.15
RELIEF_DEADBAND = 0.35
STOP_HOLD_EGO_SPEED = 0.30
STOP_HOLD_CAP = 0.50
STOPPED_LEAD_SPEED = 0.30
LEAD_DEPARTURE_SPEED = 0.30
STOP_HOLD_EXIT_FRAMES = 4
LAUNCH_PROFILE_HANDOFF_SPEED = 0.05
VEGO_NOISE_TOLERANCE = 0.10
ACCEL_LIMIT_JERK = 1.0
DECEL_LIMIT_JERK = 1.10
LAUNCH_ACCEL_RATE = 5.0
CLEAR_LAUNCH_ACCEL_RATE = 3.0
INITIAL_LAUNCH_ACCEL_MAX = 0.95
BREAKAWAY_ACCEL_MAX = 1.15
HOLD_ACCEL_MAX = 0.10
MATCHED_LEAD_PROFILE_ACCEL = -0.90
MATCHED_LEAD_ACCEL_TAPER_SPEED = 1.25
MATCHED_LEAD_ACCEL_GAIN = 0.80
NO_PROPULSION_CLOSING_SPEED = 0.02
MATERIAL_CLOSING_DECEL_ENTER = 0.08
MATERIAL_CLOSING_DECEL_EXIT = 0.03
MATERIAL_CLOSING_SPEED_ENTER = 0.25
MATERIAL_CLOSING_SPEED_EXIT = 0.10
MATERIAL_CLOSING_EXIT_FRAMES = 3
LAUNCH_PACE_RATE = 5.0
LEAD_ACQUISITION_INITIAL_AUTHORITY = 0.20
LEAD_ACQUISITION_TIME = 0.30
LEAD_ACQUISITION_MIN_DISTANCE = 60.0
LEAD_ACQUISITION_MIN_HEADWAY = 3.0
LEAD_ACQUISITION_MIN_TTC = 10.0
LEAD_ACQUISITION_MAX_DECEL = 0.25
LEAD_ACQUISITION_MIN_LEAD_ACCEL = -0.50
LEAD_ACQUISITION_MIN_PLANNER_ACCEL = -0.10
LEAD_DEPARTURE_INITIAL_AUTHORITY = 0.0
LEAD_DEPARTURE_HANDOFF_TIME = 0.50
RELATIVE_PACE_PREVIEW_TIME = 3.0
URGENT_BYPASS_REQUIRED_DECEL = 0.45
URGENT_BYPASS_MIN_SPEED = 5.0
URGENT_RELEASE_REQUIRED_DECEL = 0.35
URGENT_LOW_SPEED_LEAD_BYPASS = 5.0
URGENT_REJOIN_ACCEL_MAX = -0.15
URGENT_REJOIN_ACCEL_RATE = 5.0


@dataclass(frozen=True)
class EnergyEnvelope:
  cap: float = math.inf
  selected_lead: int = -1
  departure_lead_speed: float = math.inf
  usable_gap: float = math.inf
  closing_speed: float = 0.0
  required_decel: float = 0.0
  conservative_required_decel: float = 0.0
  has_nearly_stopped_lead: bool = False


@dataclass(frozen=True)
class AccelControllerResult:
  target_speed: float
  enabled: bool
  active: bool
  shadow_active: bool
  launching: bool
  profile: AccelProfile
  profile_accel_max: float
  effective_accel_max: float
  mpc_accel_max: tuple[float, ...] | None
  mpc_shape_cruise: bool
  reset_mpc: bool
  lead_obstacle_weights: tuple[float, float]
  state: AccelControllerState
  shadow_state: AccelControllerState
  base_speed: float
  raw_energy_cap: float
  live_filtered_cap: float
  shadow_filtered_cap: float
  live_pace: float
  shadow_pace: float
  selected_lead: int
  usable_gap: float
  closing_speed: float
  required_decel: float


@dataclass
class _PacePath:
  cap_samples: deque[float] = field(default_factory=lambda: deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES))
  pace: float | None = None
  state: AccelControllerState = AccelControllerState.inactive
  relief_time: float = 0.0
  departure_frames: int = 0
  departing_from_stop: bool = False
  launch_target_active: bool = False
  departure_handoff_active: bool = False
  stop_departure_confirmed: bool = False
  stopped_lead_hold: bool = False
  accel_limit: float | None = None
  decel_limit_active: bool = False
  urgent_bypass_active: bool = False
  urgent_recovery_active: bool = False
  urgent_dropout_frames: int = 0
  material_closing: bool = False
  material_relief_frames: int = 0
  lead_seen: list[bool] = field(default_factory=lambda: [False, False])
  lead_track_ids: list[int] = field(default_factory=lambda: [-1, -1])
  lead_obstacle_weights: list[float] = field(default_factory=lambda: [1.0, 1.0])

  def reset(self) -> None:
    self.cap_samples = deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES)
    self.pace = None
    self.state = AccelControllerState.inactive
    self.relief_time = 0.0
    self.departure_frames = 0
    self.departing_from_stop = False
    self.launch_target_active = False
    self.departure_handoff_active = False
    self.stop_departure_confirmed = False
    self.stopped_lead_hold = False
    self.accel_limit = None
    self.decel_limit_active = False
    self.urgent_bypass_active = False
    self.urgent_recovery_active = False
    self.urgent_dropout_frames = 0
    self.material_closing = False
    self.material_relief_frames = 0
    self.lead_seen = [False, False]
    self.lead_track_ids = [-1, -1]
    self.lead_obstacle_weights = [1.0, 1.0]

  def update_filter(self, cap: float) -> float:
    self.cap_samples.append(cap)
    return sorted(self.cap_samples)[CAP_FILTER_FRAMES // 2]

  @property
  def filtered_cap(self) -> float:
    return sorted(self.cap_samples)[CAP_FILTER_FRAMES // 2]


class AccelController:
  """A relative-pace governor with a pre-MPC acceleration comfort ceiling."""

  def __init__(self, CP, dt: float = DT_MDL):
    if not math.isfinite(dt) or dt <= 0.0:
      raise ValueError("dt must be finite and positive")

    self.CP = CP
    self.dt = dt
    self.live = _PacePath()
    self.shadow = _PacePath()

  @staticmethod
  def _profile(profile: int | AccelProfile) -> AccelProfile:
    try:
      return AccelProfile(profile)
    except (TypeError, ValueError):
      return AccelProfile.normal

  @classmethod
  def get_profile_accel_max(cls, profile: int | AccelProfile, v_ego: float) -> float:
    """Return the profile's positive-acceleration ceiling at the current speed."""
    if not math.isfinite(v_ego):
      return math.nan

    profile = cls._profile(profile)
    return float(np.interp(max(v_ego, 0.0), ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V[profile]))

  def _delay(self) -> float:
    try:
      return float(self.CP.longitudinalActuatorDelay) + DT_MDL
    except (AttributeError, TypeError, ValueError):
      return math.nan

  @staticmethod
  def _project_ego(v_ego: float, a_ego: float, delay: float) -> tuple[float, float]:
    if a_ego < 0.0:
      stop_time = -v_ego / a_ego if v_ego > 0.0 else 0.0
      if stop_time <= delay:
        return -v_ego * v_ego / (2.0 * a_ego) if v_ego > 0.0 else 0.0, 0.0

    return max(v_ego * delay + 0.5 * a_ego * delay * delay, 0.0), max(v_ego + a_ego * delay, 0.0)

  @staticmethod
  def _valid_lead(lead) -> bool:
    return bool(lead.status) and all(math.isfinite(value) for value in (lead.dRel, lead.vLeadK, lead.aLeadK, lead.aLeadTau))

  def calculate_energy_envelope(
    self, radar_state, v_ego: float, a_ego: float, profile: int | AccelProfile, follow_personality=log.LongitudinalPersonality.standard
  ) -> EnergyEnvelope:
    """Calculate the unfiltered relative-energy speed cap without mutating radar state."""
    profile = self._profile(profile)
    config = PROFILE_CONFIGS[profile]
    delay = self._delay()
    if not all(math.isfinite(value) for value in (v_ego, a_ego, delay)) or v_ego < 0.0 or delay < 0.0:
      return EnergyEnvelope()

    try:
      t_follow = get_T_FOLLOW(follow_personality)
    except (NotImplementedError, TypeError, ValueError):
      t_follow = get_T_FOLLOW(log.LongitudinalPersonality.standard)

    x_ego, v_ego_delay = self._project_ego(v_ego, a_ego, delay)
    candidates: list[EnergyEnvelope] = []
    departure_candidates: list[tuple[float, float]] = []

    for lead_index, lead in enumerate((radar_state.leadOne, radar_state.leadTwo)):
      if not self._valid_lead(lead):
        continue

      x_lead = float(lead.dRel)
      v_lead = float(lead.vLeadK)
      a_lead = np.clip(float(lead.aLeadK), -10.0, 5.0)
      a_lead_tau = float(lead.aLeadTau)
      lead_xv = LongitudinalMpc.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
      x_lead_delay = float(np.interp(delay, T_IDXS, lead_xv[:, 0]))
      v_lead_delay = float(np.interp(delay, T_IDXS, lead_xv[:, 1]))
      departure_candidates.append((x_lead_delay + float(get_stopped_equivalence_factor(v_lead_delay)), v_lead_delay))

      match_gap = STOP_DISTANCE + t_follow * v_lead_delay
      usable_gap = max(x_lead_delay - x_ego - match_gap, 0.0)
      closing_speed = max(v_ego_delay - v_lead_delay, 0.0)
      if closing_speed == 0.0:
        required_decel = 0.0
      elif usable_gap == 0.0:
        required_decel = math.inf
      else:
        required_decel = closing_speed * closing_speed / (2.0 * usable_gap)

      # Positive aLeadK spikes can hide urgent acquisition. The zero-accel projection is urgency-only; raw lead data remains unchanged.
      conservative_required_decel = required_decel
      if a_lead > 0.0:
        conservative_lead_xv = LongitudinalMpc.extrapolate_lead(x_lead, v_lead, 0.0, a_lead_tau)
        conservative_x_lead = float(np.interp(delay, T_IDXS, conservative_lead_xv[:, 0]))
        conservative_v_lead = float(np.interp(delay, T_IDXS, conservative_lead_xv[:, 1]))
        conservative_gap = max(conservative_x_lead - x_ego - STOP_DISTANCE - t_follow * conservative_v_lead, 0.0)
        conservative_closing = max(v_ego_delay - conservative_v_lead, 0.0)
        if conservative_closing == 0.0:
          conservative_required_decel = 0.0
        elif conservative_gap == 0.0:
          conservative_required_decel = math.inf
        else:
          conservative_required_decel = conservative_closing * conservative_closing / (2.0 * conservative_gap)
        conservative_required_decel = max(required_decel, conservative_required_decel)

      anticipated_gap = max(usable_gap - closing_speed * RELATIVE_PACE_PREVIEW_TIME, 0.0)
      cap = v_lead_delay + math.sqrt(2.0 * config.comfort_decel * anticipated_gap)
      candidates.append(EnergyEnvelope(
        cap=cap, selected_lead=lead_index, usable_gap=usable_gap, closing_speed=closing_speed, required_decel=required_decel,
        conservative_required_decel=conservative_required_decel,
      ))

    if not candidates:
      return EnergyEnvelope()

    selected = min(candidates, key=lambda candidate: candidate.cap)
    departure_lead_speed = min(departure_candidates, key=lambda candidate: candidate[0])[1]
    return EnergyEnvelope(
      cap=selected.cap, selected_lead=selected.selected_lead, departure_lead_speed=departure_lead_speed, usable_gap=selected.usable_gap,
      closing_speed=selected.closing_speed, required_decel=selected.required_decel,
      conservative_required_decel=selected.conservative_required_decel,
      has_nearly_stopped_lead=departure_lead_speed < STOPPED_LEAD_SPEED,
    )

  def _lead_acquisition_is_benign(self, lead, v_ego: float, a_ego: float, planner_accel: float, follow_personality) -> bool:
    """Return whether a new lead can enter the optimizer gradually without delaying needed braking."""
    if not self._valid_lead(lead) or planner_accel <= LEAD_ACQUISITION_MIN_PLANNER_ACCEL:
      return False

    try:
      t_follow = get_T_FOLLOW(follow_personality)
    except (NotImplementedError, TypeError, ValueError):
      t_follow = get_T_FOLLOW(log.LongitudinalPersonality.standard)

    delay = self._delay()
    x_ego, v_ego_delay = self._project_ego(v_ego, a_ego, delay)
    lead_xv = LongitudinalMpc.extrapolate_lead(
      float(lead.dRel), float(lead.vLeadK), float(np.clip(lead.aLeadK, -10.0, 5.0)), float(lead.aLeadTau),
    )
    x_lead_delay = float(np.interp(delay, T_IDXS, lead_xv[:, 0]))
    v_lead_delay = float(np.interp(delay, T_IDXS, lead_xv[:, 1]))
    separation = max(x_lead_delay - x_ego, 0.0)
    closing_speed = max(v_ego_delay - v_lead_delay, 0.0)
    ttc = separation / closing_speed if closing_speed > 0.0 else math.inf
    usable_gap = max(separation - STOP_DISTANCE - t_follow * v_lead_delay, 0.0)
    if closing_speed == 0.0:
      required_decel = 0.0
    elif usable_gap == 0.0:
      required_decel = math.inf
    else:
      required_decel = closing_speed * closing_speed / (2.0 * usable_gap)

    return (
      separation > max(LEAD_ACQUISITION_MIN_DISTANCE, LEAD_ACQUISITION_MIN_HEADWAY * v_ego_delay)
      and ttc > LEAD_ACQUISITION_MIN_TTC
      and required_decel < LEAD_ACQUISITION_MAX_DECEL
      and float(lead.aLeadK) > LEAD_ACQUISITION_MIN_LEAD_ACCEL
    )

  def _update_lead_obstacle_weights(
    self, path: _PacePath, radar_state, v_ego: float, a_ego: float, planner_accel: float, follow_personality, *, allow_blend: bool,
  ) -> tuple[float, float]:
    """Ramp benign new obstacles into MPC while making every urgent lead immediate."""
    authority_step = (1.0 - LEAD_ACQUISITION_INITIAL_AUTHORITY) * self.dt / LEAD_ACQUISITION_TIME
    departure_step = (1.0 - LEAD_DEPARTURE_INITIAL_AUTHORITY) * self.dt / LEAD_DEPARTURE_HANDOFF_TIME
    for lead_index, lead in enumerate((radar_state.leadOne, radar_state.leadTwo)):
      if not self._valid_lead(lead):
        path.lead_seen[lead_index] = False
        path.lead_track_ids[lead_index] = -1
        path.lead_obstacle_weights[lead_index] = 1.0
        continue

      track_id = int(getattr(lead, "radarTrackId", -1))
      previous_track_id = path.lead_track_ids[lead_index]
      positive_track_change = track_id >= 0 and previous_track_id >= 0 and track_id != previous_track_id
      new_lead = not path.lead_seen[lead_index] or positive_track_change
      benign = self._lead_acquisition_is_benign(lead, v_ego, a_ego, planner_accel, follow_personality)
      safe_departure_handoff = path.departure_handoff_active and float(lead.vLeadK) > LEAD_DEPARTURE_SPEED and v_ego <= float(lead.vLeadK)

      if path.departure_handoff_active:
        if safe_departure_handoff:
          # Clear standstill deadband, then ramp raw-lead authority.
          previous_weight = path.lead_obstacle_weights[lead_index]
          if previous_weight <= LEAD_DEPARTURE_INITIAL_AUTHORITY and v_ego < LAUNCH_PROFILE_HANDOFF_SPEED:
            path.lead_obstacle_weights[lead_index] = LEAD_DEPARTURE_INITIAL_AUTHORITY
          else:
            path.lead_obstacle_weights[lead_index] = min(1.0, max(previous_weight, LEAD_DEPARTURE_INITIAL_AUTHORITY) + departure_step)
        else:
          path.lead_obstacle_weights[lead_index] = 1.0
      elif new_lead:
        path.lead_obstacle_weights[lead_index] = LEAD_ACQUISITION_INITIAL_AUTHORITY if allow_blend and benign else 1.0
      elif path.lead_obstacle_weights[lead_index] < 1.0:
        if benign:
          path.lead_obstacle_weights[lead_index] = min(1.0, path.lead_obstacle_weights[lead_index] + authority_step)
        else:
          path.lead_obstacle_weights[lead_index] = 1.0

      path.lead_seen[lead_index] = True
      path.lead_track_ids[lead_index] = track_id

    if path.state == AccelControllerState.stopHold:
      try:
        v_ego_stopping = float(self.CP.vEgoStopping)
      except (AttributeError, TypeError, ValueError):
        v_ego_stopping = STOP_HOLD_EGO_SPEED
      if not math.isfinite(v_ego_stopping) or v_ego_stopping < 0.0:
        v_ego_stopping = STOP_HOLD_EGO_SPEED

      # Use zero-speed cruise below stock's shouldStop threshold; retain raw authority above it for close stopped leads.
      hold_weight = 0.0 if v_ego < v_ego_stopping else 1.0
      path.lead_obstacle_weights = [hold_weight, hold_weight]
    elif path.departure_handoff_active and not path.departing_from_stop and all(
      not seen or weight >= 1.0 for seen, weight in zip(path.lead_seen, path.lead_obstacle_weights, strict=True)
    ):
      path.departure_handoff_active = False

    return tuple(path.lead_obstacle_weights)

  def reset(self) -> None:
    self.live.reset()
    self.shadow.reset()

  @staticmethod
  def _lead_source(source) -> bool:
    return source in (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1)

  def _update_path(
    self, path: _PacePath, envelope: EnergyEnvelope, base_speed: float, v_ego: float, config: ProfileConfig, previous_mpc_source,
    planner_speed: float, previous_should_stop: bool, selected_lead_speed: float,
  ) -> float:
    raw_cap = envelope.cap
    closing_speed = envelope.closing_speed
    required_decel = envelope.required_decel
    filtered_cap = path.update_filter(raw_cap)

    # Ignore near-zero track-switch chatter without delaying material closing.
    if math.isfinite(raw_cap):
      restrictive_evidence = required_decel >= MATERIAL_CLOSING_DECEL_ENTER or closing_speed >= MATERIAL_CLOSING_SPEED_ENTER
      relief_evidence = required_decel <= MATERIAL_CLOSING_DECEL_EXIT and closing_speed <= MATERIAL_CLOSING_SPEED_EXIT
      if restrictive_evidence:
        path.material_closing = True
        path.material_relief_frames = 0
      elif path.material_closing and relief_evidence:
        path.material_relief_frames += 1
        if path.material_relief_frames >= MATERIAL_CLOSING_EXIT_FRAMES:
          path.material_closing = False
          path.material_relief_frames = 0
      else:
        path.material_relief_frames = 0
    elif not math.isfinite(filtered_cap):
      path.material_closing = False
      path.material_relief_frames = 0

    just_initialized = path.pace is None
    if just_initialized:
      # Clear road seeds base cruise; a present lead seeds ego pace.
      path.pace = base_speed if not math.isfinite(raw_cap) else min(base_speed, v_ego)
      path.state = AccelControllerState.free

    if just_initialized and v_ego < STOP_HOLD_EGO_SPEED and not math.isfinite(raw_cap) and not previous_should_stop:
      path.pace = base_speed
      path.state = AccelControllerState.release
      path.relief_time = 0.0
      path.departing_from_stop = True
      path.launch_target_active = True
      return filtered_cap

    # A lower non-controller target is authoritative, and is also the correct seed if it later clears.
    path.pace = min(path.pace, base_speed)
    if self._lead_source(previous_mpc_source) and not math.isfinite(raw_cap) and planner_speed < path.pace:
      path.pace = max(planner_speed, 0.0)

    if v_ego < STOP_HOLD_EGO_SPEED and (filtered_cap < STOP_HOLD_CAP or envelope.has_nearly_stopped_lead):
      path.stopped_lead_hold = True

    clear_road_launch_complete = path.departing_from_stop and not path.stopped_lead_hold and v_ego >= LAUNCH_PROFILE_HANDOFF_SPEED
    if v_ego >= STOP_HOLD_EGO_SPEED or clear_road_launch_complete:
      path.departing_from_stop = False
      path.stopped_lead_hold = False
      if v_ego >= STOP_HOLD_EGO_SPEED:
        path.stop_departure_confirmed = False

    if path.launch_target_active:
      if v_ego >= LAUNCH_TARGET_HANDOFF_SPEED:
        # Preserve launch preview across the base-cruise handoff.
        path.pace = min(base_speed, max(path.pace, v_ego + LAUNCH_DELTA_V))
        path.launch_target_active = False
      elif envelope.has_nearly_stopped_lead or closing_speed > MATERIAL_CLOSING_SPEED_EXIT or required_decel >= MATERIAL_CLOSING_DECEL_ENTER:
        path.launch_target_active = False

    renewed_stop_evidence = filtered_cap < STOP_HOLD_CAP or envelope.has_nearly_stopped_lead
    stale_plan_stop = previous_should_stop and not path.departing_from_stop and not path.stop_departure_confirmed
    enter_stop_hold = v_ego < STOP_HOLD_EGO_SPEED and (renewed_stop_evidence or stale_plan_stop)
    if enter_stop_hold and path.state != AccelControllerState.stopHold:
      path.pace = 0.0
      path.state = AccelControllerState.stopHold
      path.relief_time = 0.0
      path.departure_frames = 0
      path.departing_from_stop = False
      path.launch_target_active = False
      path.departure_handoff_active = False
      path.stop_departure_confirmed = False
      return filtered_cap

    if path.state == AccelControllerState.stopHold:
      # Departure uses projected lead motion, independent of profile. Lead loss waits for the median's three-observation guard.
      raw_departure = math.isfinite(envelope.departure_lead_speed) and envelope.departure_lead_speed > LEAD_DEPARTURE_SPEED
      guarded_lead_loss = not math.isfinite(raw_cap) and not math.isfinite(filtered_cap)
      if raw_departure or guarded_lead_loss:
        path.departure_frames += 1
      else:
        path.departure_frames = 0

      if path.departure_frames < STOP_HOLD_EXIT_FRAMES:
        path.pace = 0.0
        return filtered_cap

      path.state = AccelControllerState.release
      path.relief_time = config.release_confirm
      path.departure_frames = 0
      path.departing_from_stop = True
      path.launch_target_active = True
      path.departure_handoff_active = True
      path.stop_departure_confirmed = True
      path.stopped_lead_hold = False
      path.pace = min(base_speed, filtered_cap, v_ego + LAUNCH_DELTA_V)
      return filtered_cap

    ceiling = min(base_speed, filtered_cap)
    matched_moving_lead = (math.isfinite(selected_lead_speed) and closing_speed <= MATERIAL_CLOSING_SPEED_EXIT
                           and v_ego <= selected_lead_speed + MATERIAL_CLOSING_SPEED_EXIT)
    if matched_moving_lead:
      # Cap matched traffic at lead speed; upward changes still use the release ramp.
      matched_ceiling = min(base_speed, max(selected_lead_speed, 0.0))
      path.pace = min(path.pace, matched_ceiling)
      ceiling = min(ceiling, matched_ceiling)
    if math.isfinite(raw_cap) and path.material_closing:
      ceiling = min(ceiling, path.pace)
    if ceiling <= path.pace - RESTRICT_DEADBAND:
      path.pace = max(ceiling, path.pace - config.comfort_decel * self.dt)
      path.state = AccelControllerState.restrict
      path.relief_time = 0.0
      path.departing_from_stop = False
      path.launch_target_active = False
      path.departure_handoff_active = False
      return filtered_cap

    relief = ceiling - path.pace
    confirmed_clear = not math.isfinite(raw_cap) and not math.isfinite(filtered_cap)
    relief_deadband = RESTRICT_DEADBAND if confirmed_clear else RELIEF_DEADBAND
    release_allowed = path.state == AccelControllerState.release and relief > RESTRICT_DEADBAND
    if relief >= relief_deadband and not release_allowed:
      path.relief_time += self.dt
      path.state = AccelControllerState.hold
      release_allowed = path.relief_time >= config.release_confirm

    if release_allowed:
      pace_rate = LAUNCH_PACE_RATE if path.departing_from_stop else config.release_rate
      path.pace = min(ceiling, path.pace + pace_rate * self.dt)
      path.state = AccelControllerState.release
    elif relief <= relief_deadband:
      path.relief_time = 0.0
      if confirmed_clear:
        # Close the final clear-road deadband so HOLD cannot persist without a lead.
        path.pace = ceiling
        path.state = AccelControllerState.free
      else:
        path.state = AccelControllerState.free if path.pace >= base_speed else AccelControllerState.hold

    return filtered_cap

  def _update_accel_limit(
    self, path: _PacePath, envelope: EnergyEnvelope, stock_accel_max: float, planner_accel: float, profile_accel_max: float,
    config: ProfileConfig, v_ego: float, selected_lead_speed: float,
  ) -> tuple[float, float]:
    """Return telemetry effective max and the controller's pre-MPC upper bound."""
    profile_limit = float(np.clip(profile_accel_max, 0.0, ACCEL_MAX))
    lead_present = envelope.selected_lead >= 0

    if path.state == AccelControllerState.stopHold:
      # Pin the full MPC horizon at standstill; departure logic reopens it.
      path.accel_limit = 0.0
      path.decel_limit_active = False
      path.urgent_recovery_active = False
      return min(stock_accel_max, 0.0), 0.0

    if path.departing_from_stop:
      path.decel_limit_active = False
      path.urgent_recovery_active = False
      planner_seed = max(0.0, planner_accel)
      if path.departure_handoff_active:
        # A common bounded breakaway ramp starts motion; each profile may then continue toward its table ceiling.
        launch_target = min(ACCEL_MAX, max(BREAKAWAY_ACCEL_MAX, profile_limit, planner_seed))
        previous_limit = path.accel_limit if path.accel_limit is not None else 0.0
        path.accel_limit = min(launch_target, previous_limit + LAUNCH_ACCEL_RATE * self.dt)
      else:
        # Seed below the standstill solver edge; the profile table takes over after the first few centimeters.
        if path.accel_limit is None:
          path.accel_limit = min(INITIAL_LAUNCH_ACCEL_MAX, BREAKAWAY_ACCEL_MAX)
        else:
          path.accel_limit = min(BREAKAWAY_ACCEL_MAX, path.accel_limit + CLEAR_LAUNCH_ACCEL_RATE * self.dt)
      return min(stock_accel_max, path.accel_limit), path.accel_limit

    if path.urgent_recovery_active:
      # Exit on current matched-lead evidence or confirmed relief; missing leads keep the no-gas guard until then.
      if (lead_present and envelope.closing_speed <= 0.10) or path.state in (AccelControllerState.free, AccelControllerState.release):
        path.urgent_recovery_active = False
      else:
        previous_limit = path.accel_limit if path.accel_limit is not None else ACCEL_MAX
        rejoin_target = URGENT_REJOIN_ACCEL_MAX if path.state == AccelControllerState.restrict else HOLD_ACCEL_MAX
        max_step = URGENT_REJOIN_ACCEL_RATE * self.dt
        path.accel_limit = float(np.clip(rejoin_target, previous_limit - max_step, previous_limit + max_step))
        path.decel_limit_active = path.state == AccelControllerState.restrict
        return min(stock_accel_max, path.accel_limit), path.accel_limit

    if path.state == AccelControllerState.restrict and (not lead_present or path.material_closing):
      # Keep a negative horizon while materially closing or through missing-lead restriction; matched traffic recovers the ceiling.
      requested_limit = -config.comfort_decel
      if not path.decel_limit_active:
        # Retain an existing horizon on restriction entry; reseeding from scalar planner accel can cause a one-frame drop.
        if path.accel_limit is None:
          path.accel_limit = max(0.0, planner_accel)
      path.decel_limit_active = True
    elif lead_present and path.material_closing:
      requested_limit = HOLD_ACCEL_MAX
      path.decel_limit_active = False
    elif lead_present and envelope.closing_speed > NO_PROPULSION_CLOSING_SPEED:
      # Tiny closing uses the +0.10 hold ceiling instead of material-closing deceleration.
      requested_limit = HOLD_ACCEL_MAX
      path.decel_limit_active = False
    elif lead_present and math.isfinite(selected_lead_speed) and selected_lead_speed - v_ego <= MATCHED_LEAD_ACCEL_TAPER_SPEED:
      # Taper propulsion before matching lead speed to absorb acceleration already in the planner and actuator.
      speed_error = max(selected_lead_speed - v_ego, 0.0)
      requested_limit = min(profile_limit, max(HOLD_ACCEL_MAX, MATCHED_LEAD_ACCEL_GAIN * speed_error))
      path.decel_limit_active = False
    elif path.state in (AccelControllerState.restrict, AccelControllerState.hold):
      # A clearly faster lead may recover the profile ceiling; otherwise hold near zero through relief confirmation.
      requested_limit = profile_limit if lead_present and planner_accel >= MATCHED_LEAD_PROFILE_ACCEL else HOLD_ACCEL_MAX
      path.decel_limit_active = False
    else:
      requested_limit = profile_limit
      path.decel_limit_active = False

    if path.accel_limit is None:
      # Seed from the current positive command; global and stock output limits still apply.
      path.accel_limit = min(ACCEL_MAX, max(requested_limit, max(0.0, planner_accel)))
    else:
      transition_jerk = DECEL_LIMIT_JERK if path.decel_limit_active or path.accel_limit < 0.0 else ACCEL_LIMIT_JERK
      max_step = transition_jerk * self.dt
      path.accel_limit = float(np.clip(requested_limit, path.accel_limit - max_step, path.accel_limit + max_step))

    effective_limit = min(stock_accel_max, path.accel_limit)
    return effective_limit, path.accel_limit

  def _build_mpc_accel_max(self, accel_limit: float | None) -> tuple[float, ...] | None:
    """Build the controller's pre-MPC acceleration upper-bound trajectory."""
    if accel_limit is None or not math.isfinite(accel_limit):
      return None

    bounded_limit = float(np.clip(accel_limit, ACCEL_MIN, ACCEL_MAX))
    return tuple(bounded_limit for _ in T_IDXS)

  @staticmethod
  def _valid_context(
    base_speed: float, v_ego: float, a_ego: float, planner_speed: float, stock_accel_max: float, planner_accel: float, delay: float,
    engaged: bool, cruise_initialized: bool, controller_fault: bool,
  ) -> bool:
    return (
      engaged
      and cruise_initialized
      and not controller_fault
      and base_speed >= 0.0
      and v_ego >= -VEGO_NOISE_TOLERANCE
      and planner_speed >= 0.0
      and delay >= 0.0
      and all(math.isfinite(value) for value in (base_speed, v_ego, a_ego, planner_speed, stock_accel_max, planner_accel, delay))
    )

  def update(
    self, radar_state, *, base_speed: float, v_ego: float, a_ego: float, profile: int | AccelProfile, follow_personality, enabled: bool,
    acc_selected: bool, engaged: bool, cruise_initialized: bool, previous_mpc_source, planner_speed: float, stock_accel_max: float,
    planner_accel: float, previous_should_stop: bool, controller_fault: bool = False,
  ) -> AccelControllerResult:
    """Update live and shadow acceleration controllers and return the target and additive telemetry."""
    profile = self._profile(profile)
    # Clamp Toyota standstill wheel-speed noise without accepting materially negative speed.
    sanitized_v_ego = max(v_ego, 0.0) if math.isfinite(v_ego) and v_ego >= -VEGO_NOISE_TOLERANCE else v_ego
    config = PROFILE_CONFIGS[profile]
    profile_accel_max = self.get_profile_accel_max(profile, sanitized_v_ego)
    delay = self._delay()
    valid_context = self._valid_context(
      base_speed, sanitized_v_ego, a_ego, planner_speed, stock_accel_max, planner_accel, delay, engaged, cruise_initialized, controller_fault,
    )

    envelope = self.calculate_energy_envelope(radar_state, sanitized_v_ego, a_ego, profile, follow_personality) if valid_context else EnergyEnvelope()
    selected_lead_speed = math.inf
    if envelope.selected_lead in (0, 1):
      selected_lead_speed = float(getattr((radar_state.leadOne, radar_state.leadTwo)[envelope.selected_lead], "vLeadK", math.inf))

    if valid_context:
      shadow_filtered_cap = self._update_path(
        self.shadow, envelope, base_speed, sanitized_v_ego, config, previous_mpc_source, planner_speed, previous_should_stop, selected_lead_speed,
      )
      self._update_accel_limit(
        self.shadow, envelope, stock_accel_max, planner_accel, profile_accel_max, config, sanitized_v_ego, selected_lead_speed,
      )
      shadow_active = True
    else:
      self.shadow.reset()
      shadow_filtered_cap = math.inf
      shadow_active = False

    reset_mpc = False
    live_active = valid_context and bool(enabled) and bool(acc_selected)
    if live_active:
      live_was_initialized = self.live.pace is not None
      established_selected_lead = False
      if envelope.selected_lead in (0, 1):
        selected_lead = (radar_state.leadOne, radar_state.leadTwo)[envelope.selected_lead]
        selected_track_id = int(getattr(selected_lead, "radarTrackId", -1))
        previous_track_id = self.live.lead_track_ids[envelope.selected_lead]
        positive_track_change = selected_track_id >= 0 and previous_track_id >= 0 and selected_track_id != previous_track_id
        established_selected_lead = self.live.lead_seen[envelope.selected_lead] and not positive_track_change
      live_filtered_cap = self._update_path(
        self.live, envelope, base_speed, sanitized_v_ego, config, previous_mpc_source, planner_speed, previous_should_stop, selected_lead_speed,
      )
      lead_obstacle_weights = self._update_lead_obstacle_weights(
        self.live, radar_state, sanitized_v_ego, a_ego, planner_accel, follow_personality, allow_blend=live_was_initialized,
      )
      urgent_was_active = self.live.urgent_bypass_active
      selected_has_full_authority = envelope.selected_lead in (0, 1) and lead_obstacle_weights[envelope.selected_lead] >= 1.0
      urgent_required_decel = envelope.required_decel
      if not live_was_initialized or not established_selected_lead:
        urgent_required_decel = max(urgent_required_decel, envelope.conservative_required_decel)
      low_speed_urgent_lead = selected_lead_speed < URGENT_LOW_SPEED_LEAD_BYPASS and self.live.state != AccelControllerState.stopHold
      urgent_trigger = (
        (sanitized_v_ego >= URGENT_BYPASS_MIN_SPEED or low_speed_urgent_lead)
        and urgent_required_decel >= URGENT_BYPASS_REQUIRED_DECEL
        and (not live_was_initialized or established_selected_lead or selected_has_full_authority)
      )
      if urgent_was_active and envelope.selected_lead < 0:
        self.live.urgent_dropout_frames += 1
      else:
        self.live.urgent_dropout_frames = 0
      urgent_dropout_guard = urgent_was_active and envelope.selected_lead < 0 and self.live.urgent_dropout_frames <= 2
      urgent_bypass = urgent_trigger or (
        urgent_was_active
        and envelope.selected_lead >= 0
        and (
          envelope.required_decel > URGENT_RELEASE_REQUIRED_DECEL
          or (selected_lead_speed < URGENT_LOW_SPEED_LEAD_BYPASS and self.live.state != AccelControllerState.stopHold)
        )
      )
      # Latch urgency across two missing observations, retaining scalar state but never stale lead geometry.
      self.live.urgent_bypass_active = urgent_bypass or urgent_dropout_guard
      if (urgent_was_active and not urgent_bypass and not urgent_dropout_guard and envelope.selected_lead >= 0
          and self.live.state != AccelControllerState.stopHold):
        # Rejoin from stock's achieved speed, not the stale pre-urgent pace.
        rejoin_speed = min(planner_speed, sanitized_v_ego)
        if envelope.closing_speed <= 0.10 and math.isfinite(selected_lead_speed):
          # A matched moving lead is the lower pace reference; seeding below it reinforces residual braking.
          rejoin_speed = max(rejoin_speed, min(base_speed, selected_lead_speed))
          self.live.pace = rejoin_speed
        else:
          self.live.pace = min(self.live.pace if self.live.pace is not None else rejoin_speed, rejoin_speed)
        self.live.state = AccelControllerState.hold
        self.live.relief_time = 0.0
        # Rejoin from the global ceiling so the pre-MPC slew tightens gradually.
        self.live.accel_limit = ACCEL_MAX
        self.live.urgent_recovery_active = True
      if urgent_dropout_guard:
        # Two-frame dropout holds pace and a nonpositive ceiling without stale lead geometry.
        held_limit = self.live.accel_limit if self.live.accel_limit is not None else planner_accel
        self.live.accel_limit = float(np.clip(min(held_limit, 0.0), ACCEL_MIN, ACCEL_MAX))
        self.live.decel_limit_active = self.live.accel_limit < 0.0
        self.live.urgent_recovery_active = False
        effective_accel_max = min(stock_accel_max, self.live.accel_limit)
        mpc_accel_max = self._build_mpc_accel_max(self.live.accel_limit)
        mpc_shape_cruise = True
        lead_obstacle_weights = (1.0, 1.0)
        target_speed = min(base_speed, self.live.pace if self.live.pace is not None else sanitized_v_ego)
      elif urgent_bypass:
        # Urgent entry restores raw leads and stock bounds, then resets MPC before planner state/update.
        reset_mpc = reset_mpc or not urgent_was_active
        self.live.launch_target_active = False
        self.live.accel_limit = None
        self.live.decel_limit_active = False
        self.live.urgent_recovery_active = False
        if self.live.pace is not None:
          self.live.pace = min(self.live.pace, planner_speed, sanitized_v_ego)
        effective_accel_max = stock_accel_max
        mpc_accel_max = None
        mpc_shape_cruise = False
        lead_obstacle_weights = (1.0, 1.0)
        target_speed = base_speed
      else:
        recovery_was_active = self.live.urgent_recovery_active
        effective_accel_max, controller_accel_max = self._update_accel_limit(
          self.live, envelope, stock_accel_max, planner_accel, profile_accel_max, config, sanitized_v_ego, selected_lead_speed,
        )
        if (
          recovery_was_active
          and not self.live.urgent_recovery_active
          and envelope.closing_speed <= 0.10
          and math.isfinite(selected_lead_speed)
          and self.live.pace is not None
        ):
          # Do not carry recovery pace below a matched moving lead.
          self.live.pace = max(self.live.pace, min(base_speed, selected_lead_speed))
        mpc_accel_max = self._build_mpc_accel_max(controller_accel_max)
        mpc_shape_cruise = mpc_accel_max is not None
        if mpc_accel_max is None:
          effective_accel_max = stock_accel_max
        if self.live.state == AccelControllerState.stopHold:
          # Pin cruise because some platforms assert shouldStop below 0.30 m/s while lead authority is muted.
          target_speed = 0.0
        elif self.live.departing_from_stop or self.live.launch_target_active:
          # Base cruise supplies launch incentive; renewed closing cancels it.
          target_speed = base_speed
        elif (
          envelope.selected_lead >= 0
          and math.isfinite(selected_lead_speed)
          and selected_lead_speed > STOPPED_LEAD_SPEED
          and envelope.closing_speed <= MATERIAL_CLOSING_SPEED_EXIT
          and envelope.required_decel <= MATERIAL_CLOSING_DECEL_EXIT
          and planner_accel < -0.20
          and not self.live.material_closing
        ):
          # Base cruise unwinds residual braking after matching a moving lead. Raw-lead authority and mpc_accel_max remain active.
          target_speed = base_speed
          mpc_shape_cruise = False
          lead_obstacle_weights = (1.0, 1.0)
        elif self.live.urgent_recovery_active:
          # Keep raw-lead authority and synchronized pace while the ceiling rejoins.
          target_speed = min(base_speed, self.live.pace if self.live.pace is not None else base_speed)
          lead_obstacle_weights = (1.0, 1.0)
        else:
          target_speed = min(base_speed, self.live.pace if self.live.pace is not None else base_speed)
    else:
      self.live.reset()
      live_filtered_cap = math.inf
      # Preserve the stock target bit-for-bit on every bypass, including stock's own invalid-value handling.
      target_speed = base_speed
      effective_accel_max = math.inf
      mpc_accel_max = None
      mpc_shape_cruise = False
      lead_obstacle_weights = (1.0, 1.0)

    return AccelControllerResult(
      target_speed=target_speed, enabled=bool(enabled), active=live_active, shadow_active=shadow_active,
      launching=live_active and self.live.departing_from_stop, profile=profile,
      profile_accel_max=profile_accel_max if live_active else math.inf, effective_accel_max=effective_accel_max,
      mpc_accel_max=mpc_accel_max, mpc_shape_cruise=mpc_shape_cruise, reset_mpc=reset_mpc, lead_obstacle_weights=lead_obstacle_weights,
      state=self.live.state, shadow_state=self.shadow.state, base_speed=base_speed, raw_energy_cap=envelope.cap,
      live_filtered_cap=live_filtered_cap, shadow_filtered_cap=shadow_filtered_cap,
      live_pace=self.live.pace if self.live.pace is not None else math.inf,
      shadow_pace=self.shadow.pace if self.shadow.pace is not None else math.inf,
      selected_lead=envelope.selected_lead, usable_gap=envelope.usable_gap, closing_speed=envelope.closing_speed,
      required_decel=envelope.required_decel,
    )
