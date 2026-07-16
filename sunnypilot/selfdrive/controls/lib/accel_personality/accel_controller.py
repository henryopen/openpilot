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
  AccelProfile.eco: ProfileConfig(comfort_decel=0.25, release_rate=0.65, release_confirm=0.50),
  AccelProfile.normal: ProfileConfig(comfort_decel=0.335, release_rate=0.85, release_confirm=0.35),
  AccelProfile.sport: ProfileConfig(comfort_decel=0.50, release_rate=1.10, release_confirm=0.20),
}

ACCEL_PROFILE_MAX_BP = [0.0, 10.0, 25.0, 40.0]
# These are pre-MPC profile requests. Values remain inside global ACCEL_MAX;
# the planner's stock speed/turn/coast limit remains the final output authority.
ACCEL_PROFILE_MAX_V = {
  AccelProfile.eco: [1.55, 0.30, 0.20, 0.10],
  AccelProfile.normal: [1.70, 0.90, 0.40, 0.20],
  AccelProfile.sport: [2.00, 1.70, 1.20, 0.90],
}
LAUNCH_DELTA_V = 3.0

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
LAUNCH_ACCEL_RATE = 4.0
CLEAR_LAUNCH_ACCEL_RATE = 3.0
INITIAL_LAUNCH_ACCEL_MAX = 0.95
BREAKAWAY_ACCEL_MAX = 1.15
HOLD_ACCEL_MAX = 0.10
LAUNCH_PACE_RATE = 5.0
LEAD_ACQUISITION_INITIAL_AUTHORITY = 0.20
LEAD_ACQUISITION_TIME = 0.30
LEAD_ACQUISITION_MIN_DISTANCE = 60.0
LEAD_ACQUISITION_MIN_HEADWAY = 3.0
LEAD_ACQUISITION_MIN_TTC = 10.0
LEAD_ACQUISITION_MAX_DECEL = 0.25
LEAD_ACQUISITION_MIN_LEAD_ACCEL = -0.50
LEAD_ACQUISITION_MIN_PLANNER_ACCEL = -0.10
LEAD_DEPARTURE_HANDOFF_TIME = 0.50
RELATIVE_PACE_PREVIEW_TIME = 3.0
URGENT_BYPASS_REQUIRED_DECEL = 0.75
URGENT_BYPASS_MIN_SPEED = 5.0


@dataclass(frozen=True)
class EnergyEnvelope:
  cap: float = math.inf
  selected_lead: int = -1
  departure_lead_speed: float = math.inf
  usable_gap: float = math.inf
  closing_speed: float = 0.0
  required_decel: float = 0.0
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
  departure_handoff_active: bool = False
  stop_departure_confirmed: bool = False
  stopped_lead_hold: bool = False
  accel_limit: float | None = None
  decel_limit_active: bool = False
  urgent_bypass_active: bool = False
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
    self.departure_handoff_active = False
    self.stop_departure_confirmed = False
    self.stopped_lead_hold = False
    self.accel_limit = None
    self.decel_limit_active = False
    self.urgent_bypass_active = False
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

      # Relative kinetic energy: the lead keeps moving while ego sheds closing speed.
      anticipated_gap = max(usable_gap - closing_speed * RELATIVE_PACE_PREVIEW_TIME, 0.0)
      cap = v_lead_delay + math.sqrt(2.0 * config.comfort_decel * anticipated_gap)
      candidates.append(EnergyEnvelope(
        cap=cap,
        selected_lead=lead_index,
        usable_gap=usable_gap,
        closing_speed=closing_speed,
        required_decel=required_decel,
      ))

    if not candidates:
      return EnergyEnvelope()

    selected = min(candidates, key=lambda candidate: candidate.cap)
    departure_lead_speed = min(departure_candidates, key=lambda candidate: candidate[0])[1]
    return EnergyEnvelope(
      cap=selected.cap,
      selected_lead=selected.selected_lead,
      departure_lead_speed=departure_lead_speed,
      usable_gap=selected.usable_gap,
      closing_speed=selected.closing_speed,
      required_decel=selected.required_decel,
      has_nearly_stopped_lead=departure_lead_speed < STOPPED_LEAD_SPEED,
    )

  def _lead_acquisition_is_benign(
    self,
    lead,
    v_ego: float,
    a_ego: float,
    planner_accel: float,
    follow_personality,
  ) -> bool:
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
      float(lead.dRel),
      float(lead.vLeadK),
      float(np.clip(lead.aLeadK, -10.0, 5.0)),
      float(lead.aLeadTau),
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
    self,
    path: _PacePath,
    radar_state,
    v_ego: float,
    a_ego: float,
    planner_accel: float,
    follow_personality,
    *,
    allow_blend: bool,
  ) -> tuple[float, float]:
    """Ramp benign new obstacles into MPC while making every urgent lead immediate."""
    authority_step = (1.0 - LEAD_ACQUISITION_INITIAL_AUTHORITY) * self.dt / LEAD_ACQUISITION_TIME
    departure_step = self.dt / LEAD_DEPARTURE_HANDOFF_TIME
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
      safe_departure_handoff = (
        path.departure_handoff_active
        and float(lead.vLeadK) > LEAD_DEPARTURE_SPEED
        and v_ego <= float(lead.vLeadK)
      )

      if safe_departure_handoff:
        if path.departing_from_stop:
          path.lead_obstacle_weights[lead_index] = 0.0
        else:
          path.lead_obstacle_weights[lead_index] = min(1.0, path.lead_obstacle_weights[lead_index] + departure_step)
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

      # Once stock's shouldStop threshold is reachable, use the zero-speed
      # cruise obstacle to keep the stopped solver warm. Above that threshold,
      # retain full raw-lead authority: a zero acceleration ceiling alone does
      # not require enough braking for a newly detected close stopped lead.
      hold_weight = 0.0 if v_ego < v_ego_stopping else 1.0
      path.lead_obstacle_weights = [hold_weight, hold_weight]
    elif path.departure_handoff_active and all(
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
    self,
    path: _PacePath,
    raw_cap: float,
    base_speed: float,
    v_ego: float,
    config: ProfileConfig,
    previous_mpc_source,
    planner_speed: float,
    previous_should_stop: bool,
    has_nearly_stopped_lead: bool,
    departure_lead_speed: float,
    closing_speed: float,
    launch_delta_v: float,
  ) -> float:
    filtered_cap = path.update_filter(raw_cap)
    just_initialized = path.pace is None
    if just_initialized:
      # A clear road has no prior restriction to release from, so expose base
      # cruise immediately. With any lead present, seed at ego instead: the
      # first radar frame can contain a large aLeadK spike, and using that
      # transient energy cap as pace caused several seconds of acceleration
      # before a late brake.
      path.pace = base_speed if not math.isfinite(raw_cap) else min(base_speed, v_ego)
      path.state = AccelControllerState.free

    # A clear-road standstill engagement should request motion immediately. A
    # stopped/previously-stopping lead still goes through stop-hold confirmation.
    if just_initialized and v_ego < STOP_HOLD_EGO_SPEED and not math.isfinite(raw_cap) and not previous_should_stop:
      path.pace = base_speed
      path.state = AccelControllerState.release
      path.relief_time = 0.0
      path.departing_from_stop = True
      return filtered_cap

    # A lower non-controller target is authoritative, and is also the correct seed if it later clears.
    path.pace = min(path.pace, base_speed)
    if self._lead_source(previous_mpc_source) and not math.isfinite(raw_cap) and planner_speed < path.pace:
      path.pace = max(planner_speed, 0.0)

    if v_ego < STOP_HOLD_EGO_SPEED and (filtered_cap < STOP_HOLD_CAP or has_nearly_stopped_lead):
      path.stopped_lead_hold = True

    clear_road_launch_complete = path.departing_from_stop and not path.stopped_lead_hold and v_ego >= LAUNCH_PROFILE_HANDOFF_SPEED
    if v_ego >= STOP_HOLD_EGO_SPEED or clear_road_launch_complete:
      path.departing_from_stop = False
      path.stopped_lead_hold = False
      if v_ego >= STOP_HOLD_EGO_SPEED:
        path.stop_departure_confirmed = False

    renewed_stop_evidence = filtered_cap < STOP_HOLD_CAP or has_nearly_stopped_lead
    stale_plan_stop = previous_should_stop and not path.departing_from_stop and not path.stop_departure_confirmed
    enter_stop_hold = v_ego < STOP_HOLD_EGO_SPEED and (renewed_stop_evidence or stale_plan_stop)
    if enter_stop_hold and path.state != AccelControllerState.stopHold:
      path.pace = 0.0
      path.state = AccelControllerState.stopHold
      path.relief_time = 0.0
      path.departure_frames = 0
      path.departing_from_stop = False
      path.departure_handoff_active = False
      path.stop_departure_confirmed = False
      return filtered_cap

    if path.state == AccelControllerState.stopHold:
      # Departure is a perception fact, not a comfort-profile decision. Confirm
      # the selected lead's projected motion directly so Eco cannot wait longer
      # merely because its energy envelope is lower. Total lead loss still waits
      # for the five-frame median dropout guard before confirmation begins.
      raw_departure = math.isfinite(departure_lead_speed) and departure_lead_speed > LEAD_DEPARTURE_SPEED
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
      path.departure_handoff_active = True
      path.stop_departure_confirmed = True
      path.stopped_lead_hold = False
      path.pace = min(base_speed, filtered_cap, v_ego + launch_delta_v)
      return filtered_cap

    ceiling = min(base_speed, filtered_cap)
    if math.isfinite(raw_cap) and closing_speed > 0.0:
      # Never spend stored gap by accelerating toward a slower lead. Hold the
      # current pace until relative speed is matched; the energy envelope may
      # still lower it at the configured comfort rate.
      ceiling = min(ceiling, path.pace)
    if ceiling <= path.pace - RESTRICT_DEADBAND:
      path.pace = max(ceiling, path.pace - config.comfort_decel * self.dt)
      path.state = AccelControllerState.restrict
      path.relief_time = 0.0
      path.departing_from_stop = False
      path.departure_handoff_active = False
      return filtered_cap

    relief = ceiling - path.pace
    release_allowed = path.state == AccelControllerState.release and relief > RESTRICT_DEADBAND
    if relief >= RELIEF_DEADBAND and not release_allowed:
      path.relief_time += self.dt
      path.state = AccelControllerState.hold
      release_allowed = path.relief_time >= config.release_confirm

    if release_allowed:
      pace_rate = LAUNCH_PACE_RATE if path.departing_from_stop else config.release_rate
      path.pace = min(ceiling, path.pace + pace_rate * self.dt)
      path.state = AccelControllerState.release
    elif relief <= RELIEF_DEADBAND:
      path.relief_time = 0.0
      path.state = AccelControllerState.free if path.pace >= base_speed else AccelControllerState.hold

    return filtered_cap

  def _update_accel_limit(
    self,
    path: _PacePath,
    stock_accel_max: float,
    planner_accel: float,
    profile_accel_max: float,
    config: ProfileConfig,
  ) -> tuple[float, float]:
    """Return telemetry effective max and the controller's pre-MPC upper bound."""
    profile_limit = float(np.clip(profile_accel_max, 0.0, ACCEL_MAX))

    if path.state == AccelControllerState.stopHold:
      # Keep the entire reachable cruise trajectory at standstill. Unlike the
      # old mixed zero/warm-node horizon, this is internally consistent and
      # opens in time on departure instead of changing shape in one frame.
      path.accel_limit = 0.0
      path.decel_limit_active = False
      return min(stock_accel_max, 0.0), 0.0

    if path.departing_from_stop:
      path.decel_limit_active = False
      planner_seed = max(0.0, planner_accel)
      if path.departure_handoff_active:
        # Open every profile at the same bounded jerk rate until the command is
        # high enough to overcome measured standstill deadband. Normal and
        # Sport may continue above that common floor, but all profiles begin
        # physical motion at the same time.
        launch_target = min(ACCEL_MAX, max(BREAKAWAY_ACCEL_MAX, profile_limit, planner_seed))
        previous_limit = path.accel_limit if path.accel_limit is not None else 0.0
        path.accel_limit = min(launch_target, previous_limit + LAUNCH_ACCEL_RATE * self.dt)
      else:
        # Start below the solver's standstill cold-start edge, then reach the
        # common breakaway floor over two controller frames. The lookup table
        # takes over after the first few centimeters.
        if path.accel_limit is None:
          path.accel_limit = min(INITIAL_LAUNCH_ACCEL_MAX, BREAKAWAY_ACCEL_MAX)
        else:
          path.accel_limit = min(BREAKAWAY_ACCEL_MAX, path.accel_limit + CLEAR_LAUNCH_ACCEL_RATE * self.dt)
      return min(stock_accel_max, path.accel_limit), path.accel_limit

    if path.state == AccelControllerState.restrict:
      requested_limit = -config.comfort_decel
      if not path.decel_limit_active:
        # Preserve an existing pre-MPC ceiling when restriction begins. The
        # planner's scalar acceleration is the near-time state, while the
        # commanded target is sampled later for actuator delay; replacing the
        # ceiling with that scalar can therefore create a one-frame command
        # drop. A newly initialized path has no prior ceiling to preserve and
        # is safely seeded from the current planner state.
        if path.accel_limit is None:
          path.accel_limit = max(0.0, planner_accel)
      path.decel_limit_active = True
    elif path.state == AccelControllerState.hold:
      # No gas while waiting for relief confirmation. This is the main
      # anti-rubber-band rule for a still-closing lead.
      requested_limit = HOLD_ACCEL_MAX
      path.decel_limit_active = False
    else:
      requested_limit = profile_limit
      path.decel_limit_active = False

    if path.accel_limit is None:
      # Avoid a discontinuity when enabling around an already-positive command.
      # The global OP limit bounds this seed; dynamic stock output constraints
      # still retain their existing output-side enforcement and slew.
      path.accel_limit = min(ACCEL_MAX, max(requested_limit, max(0.0, planner_accel)))
    else:
      transition_jerk = DECEL_LIMIT_JERK if path.decel_limit_active or path.accel_limit < 0.0 else ACCEL_LIMIT_JERK
      max_step = transition_jerk * self.dt
      path.accel_limit = float(np.clip(requested_limit, path.accel_limit - max_step, path.accel_limit + max_step))

    effective_limit = min(stock_accel_max, path.accel_limit)
    return effective_limit, path.accel_limit

  def _build_mpc_accel_max(
    self,
    path: _PacePath,
    accel_limit: float,
  ) -> tuple[float, ...] | None:
    """Build the controller's pre-MPC acceleration upper-bound trajectory."""
    if not math.isfinite(accel_limit):
      return None

    bounded_limit = float(np.clip(accel_limit, ACCEL_MIN, ACCEL_MAX))
    return tuple(bounded_limit for _ in T_IDXS)

  @staticmethod
  def _valid_context(
    base_speed: float,
    v_ego: float,
    a_ego: float,
    planner_speed: float,
    stock_accel_max: float,
    planner_accel: float,
    delay: float,
    engaged: bool,
    cruise_initialized: bool,
    controller_fault: bool,
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
    self,
    radar_state,
    *,
    base_speed: float,
    v_ego: float,
    a_ego: float,
    profile: int | AccelProfile,
    follow_personality,
    enabled: bool,
    acc_selected: bool,
    engaged: bool,
    cruise_initialized: bool,
    previous_mpc_source,
    planner_speed: float,
    stock_accel_max: float,
    planner_accel: float,
    previous_should_stop: bool,
    controller_fault: bool = False,
  ) -> AccelControllerResult:
    """Update live and shadow acceleration controllers and return the target and additive telemetry."""
    profile = self._profile(profile)
    # Toyota wheel-speed filtering can report a few cm/s negative at a stop.
    # Treat that as zero without allowing a real invalid state to persist.
    sanitized_v_ego = max(v_ego, 0.0) if math.isfinite(v_ego) and v_ego >= -VEGO_NOISE_TOLERANCE else v_ego
    config = PROFILE_CONFIGS[profile]
    profile_accel_max = self.get_profile_accel_max(profile, sanitized_v_ego)
    launch_delta_v = LAUNCH_DELTA_V
    delay = self._delay()
    valid_context = self._valid_context(
      base_speed,
      sanitized_v_ego,
      a_ego,
      planner_speed,
      stock_accel_max,
      planner_accel,
      delay,
      engaged,
      cruise_initialized,
      controller_fault,
    )

    envelope = self.calculate_energy_envelope(radar_state, sanitized_v_ego, a_ego, profile, follow_personality) if valid_context else EnergyEnvelope()

    if valid_context:
      shadow_filtered_cap = self._update_path(
        self.shadow,
        envelope.cap,
        base_speed,
        sanitized_v_ego,
        config,
        previous_mpc_source,
        planner_speed,
        previous_should_stop,
        envelope.has_nearly_stopped_lead,
        envelope.departure_lead_speed,
        envelope.closing_speed,
        launch_delta_v,
      )
      self._update_accel_limit(self.shadow, stock_accel_max, planner_accel, profile_accel_max, config)
      shadow_active = True
    else:
      self.shadow.reset()
      shadow_filtered_cap = math.inf
      shadow_active = False

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
        self.live,
        envelope.cap,
        base_speed,
        sanitized_v_ego,
        config,
        previous_mpc_source,
        planner_speed,
        previous_should_stop,
        envelope.has_nearly_stopped_lead,
        envelope.departure_lead_speed,
        envelope.closing_speed,
        launch_delta_v,
      )
      lead_obstacle_weights = self._update_lead_obstacle_weights(
        self.live,
        radar_state,
        sanitized_v_ego,
        a_ego,
        planner_accel,
        follow_personality,
        allow_blend=live_was_initialized,
      )
      urgent_trigger = (
        sanitized_v_ego >= URGENT_BYPASS_MIN_SPEED
        and envelope.required_decel >= URGENT_BYPASS_REQUIRED_DECEL
        and (not live_was_initialized or established_selected_lead)
      )
      urgent_bypass = urgent_trigger or (
        self.live.urgent_bypass_active and envelope.selected_lead >= 0 and envelope.closing_speed > 0.10
      )
      self.live.urgent_bypass_active = urgent_bypass
      if urgent_bypass:
        # Comfort shaping must never compete with urgent braking. Hand the raw
        # leads, base cruise target, and stock acceleration bounds directly to
        # MPC. Clearing the stored ceiling gives the later comfort re-entry a
        # fresh non-restrictive seed instead of resurrecting an urgent bound.
        self.live.accel_limit = None
        self.live.decel_limit_active = False
        effective_accel_max = stock_accel_max
        mpc_accel_max = None
        mpc_shape_cruise = False
        lead_obstacle_weights = (1.0, 1.0)
        target_speed = base_speed
      else:
        effective_accel_max, controller_accel_max = self._update_accel_limit(
          self.live, stock_accel_max, planner_accel, profile_accel_max, config
        )
        # Feed only the controller-owned ceiling into MPC. Stock's speed, turn,
        # coast, and no-throttle limits remain in their original output clip.
        mpc_accel_max = self._build_mpc_accel_max(self.live, controller_accel_max)
        mpc_shape_cruise = mpc_accel_max is not None
        if mpc_accel_max is None:
          effective_accel_max = stock_accel_max
        if self.live.state == AccelControllerState.stopHold:
          # Pin the cruise obstacle to zero as well as the acceleration upper
          # bound. Some platforms declare shouldStop below this controller's
          # 0.30 m/s hold threshold; keeping base cruise there can otherwise
          # permit a slow coast while lead authority is intentionally muted.
          target_speed = 0.0
        elif self.live.departing_from_stop:
          # Give all profiles the same prompt stock breakaway. The raw lead still
          # owns obstacle braking, and profile separation begins after motion.
          target_speed = base_speed
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
      target_speed=target_speed,
      enabled=bool(enabled),
      active=live_active,
      shadow_active=shadow_active,
      launching=live_active and self.live.departing_from_stop,
      profile=profile,
      profile_accel_max=profile_accel_max if live_active else math.inf,
      effective_accel_max=effective_accel_max,
      mpc_accel_max=mpc_accel_max,
      mpc_shape_cruise=mpc_shape_cruise,
      lead_obstacle_weights=lead_obstacle_weights,
      state=self.live.state,
      shadow_state=self.shadow.state,
      base_speed=base_speed,
      raw_energy_cap=envelope.cap,
      live_filtered_cap=live_filtered_cap,
      shadow_filtered_cap=shadow_filtered_cap,
      live_pace=self.live.pace if self.live.pace is not None else math.inf,
      shadow_pace=self.shadow.pace if self.shadow.pace is not None else math.inf,
      selected_lead=envelope.selected_lead,
      usable_gap=envelope.usable_gap,
      closing_speed=envelope.closing_speed,
      required_decel=envelope.required_decel,
    )
