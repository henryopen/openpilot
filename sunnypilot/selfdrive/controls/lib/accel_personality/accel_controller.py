#!/usr/bin/env python3
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
import math

import numpy as np

from cereal import log
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc,
  LongitudinalPlanSource,
  STOP_DISTANCE,
  T_IDXS,
  get_T_FOLLOW,
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
ACCEL_PROFILE_MAX_V = {
  AccelProfile.eco: [0.95, 0.70, 0.42, 0.28],
  AccelProfile.normal: [1.30, 1.00, 0.65, 0.45],
  AccelProfile.sport: [1.55, 1.15, 0.78, 0.58],
}
LAUNCH_DELTA_V = 3.0

CAP_FILTER_FRAMES = 5
RESTRICT_DEADBAND = 0.15
RELIEF_DEADBAND = 0.35
STOP_HOLD_EGO_SPEED = 0.30
STOP_HOLD_CAP = 0.50
STOPPED_LEAD_SPEED = 0.30
STOP_HOLD_EXIT_CAP = 0.80
STOP_HOLD_EXIT_FRAMES = 4
CLEAR_ROAD_PROFILE_SPEED = 0.20
ACCEL_LIMIT_JERK = 1.0
LAUNCH_ACCEL_JERK = 3.0
LAUNCH_PACE_RATE = 5.0
MPC_LAUNCH_BOUND_NODES = 2
MPC_STOP_WARM_BLEND = 0.0
MPC_CONFIRM_WARM_BLEND = 0.10
MPC_DEPART_WARM_BLEND = 0.25


@dataclass(frozen=True)
class EnergyEnvelope:
  cap: float = math.inf
  selected_lead: int = -1
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
  stopped_lead_hold: bool = False
  accel_limit: float | None = None

  def reset(self) -> None:
    self.cap_samples = deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES)
    self.pace = None
    self.state = AccelControllerState.inactive
    self.relief_time = 0.0
    self.departure_frames = 0
    self.departing_from_stop = False
    self.stopped_lead_hold = False
    self.accel_limit = None

  def update_filter(self, cap: float) -> float:
    self.cap_samples.append(cap)
    return sorted(self.cap_samples)[CAP_FILTER_FRAMES // 2]

  @property
  def filtered_cap(self) -> float:
    return sorted(self.cap_samples)[CAP_FILTER_FRAMES // 2]


class AccelController:
  """A relative-pace governor with a positive-acceleration comfort ceiling."""

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
    nearly_stopped = False

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
      nearly_stopped = nearly_stopped or v_lead_delay < STOPPED_LEAD_SPEED

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
      cap = v_lead_delay + math.sqrt(2.0 * config.comfort_decel * usable_gap)
      candidates.append(EnergyEnvelope(cap, lead_index, usable_gap, closing_speed, required_decel))

    if not candidates:
      return EnergyEnvelope(has_nearly_stopped_lead=nearly_stopped)

    selected = min(candidates, key=lambda candidate: candidate.cap)
    return EnergyEnvelope(selected.cap, selected.selected_lead, selected.usable_gap, selected.closing_speed, selected.required_decel, nearly_stopped)

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
    launch_delta_v: float,
  ) -> float:
    filtered_cap = path.update_filter(raw_cap)
    just_initialized = path.pace is None
    if just_initialized:
      path.pace = min(base_speed, v_ego)
      path.state = AccelControllerState.free

    # A clear-road standstill engagement should request motion immediately. A
    # stopped/previously-stopping lead still goes through stop-hold confirmation.
    if just_initialized and v_ego < STOP_HOLD_EGO_SPEED and not math.isfinite(raw_cap) and not previous_should_stop:
      path.pace = min(base_speed, v_ego + launch_delta_v)
      path.state = AccelControllerState.release
      path.relief_time = config.release_confirm
      path.departing_from_stop = True
      return filtered_cap

    # A lower non-controller target is authoritative, and is also the correct seed if it later clears.
    path.pace = min(path.pace, base_speed)
    if self._lead_source(previous_mpc_source) and not math.isfinite(raw_cap) and planner_speed < path.pace:
      path.pace = max(planner_speed, 0.0)

    if v_ego < STOP_HOLD_EGO_SPEED and (filtered_cap < STOP_HOLD_CAP or has_nearly_stopped_lead):
      path.stopped_lead_hold = True

    clear_road_launch_complete = path.departing_from_stop and not path.stopped_lead_hold and v_ego >= CLEAR_ROAD_PROFILE_SPEED
    if v_ego >= STOP_HOLD_EGO_SPEED or clear_road_launch_complete:
      path.departing_from_stop = False
      path.stopped_lead_hold = False

    renewed_stop_evidence = filtered_cap < STOP_HOLD_CAP or has_nearly_stopped_lead
    enter_stop_hold = v_ego < STOP_HOLD_EGO_SPEED and (renewed_stop_evidence or (previous_should_stop and not path.departing_from_stop))
    if enter_stop_hold and path.state != AccelControllerState.stopHold:
      path.pace = 0.0
      path.state = AccelControllerState.stopHold
      path.relief_time = 0.0
      path.departure_frames = 0
      path.departing_from_stop = False
      return filtered_cap

    if path.state == AccelControllerState.stopHold:
      # A continuously observed moving lead exits after exactly four raw frames.
      # Total lead loss still waits for the five-frame median dropout guard first.
      raw_departure = math.isfinite(raw_cap) and raw_cap > STOP_HOLD_EXIT_CAP and not has_nearly_stopped_lead
      guarded_lead_loss = not math.isfinite(raw_cap) and filtered_cap > STOP_HOLD_EXIT_CAP
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
      path.pace = min(base_speed, filtered_cap, v_ego + launch_delta_v)
      return filtered_cap

    ceiling = min(base_speed, filtered_cap)
    if ceiling <= path.pace - RESTRICT_DEADBAND:
      path.pace = max(ceiling, path.pace - config.comfort_decel * self.dt)
      path.state = AccelControllerState.restrict
      path.relief_time = 0.0
      path.departing_from_stop = False
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
  ) -> tuple[float, float]:
    """Return telemetry effective max and the controller's pre-MPC positive bound."""
    requested_limit = float(np.clip(profile_accel_max, 0.0, ACCEL_MAX))

    if path.state == AccelControllerState.stopHold:
      path.accel_limit = 0.0
      return min(stock_accel_max, 0.0), 0.0

    if path.departing_from_stop:
      if path.stopped_lead_hold:
        # A confirmed lead departure opens quickly but continuously from zero.
        previous_limit = path.accel_limit if path.accel_limit is not None else 0.0
        path.accel_limit = min(requested_limit, previous_limit + LAUNCH_ACCEL_JERK * self.dt)
      else:
        # The MPC stays completely stock for the first few centimeters of a
        # clear-road launch. Seed the selected table value for a smooth handoff.
        path.accel_limit = requested_limit
      return min(stock_accel_max, path.accel_limit), path.accel_limit

    if path.accel_limit is None:
      # Avoid a discontinuity when enabling around an already-positive command.
      # The global OP limit bounds this seed; dynamic stock output constraints
      # still retain their existing output-side enforcement and slew.
      path.accel_limit = min(ACCEL_MAX, max(requested_limit, max(0.0, planner_accel)))
    else:
      max_step = ACCEL_LIMIT_JERK * self.dt
      path.accel_limit = float(np.clip(requested_limit, path.accel_limit - max_step, path.accel_limit + max_step))

    effective_limit = min(stock_accel_max, path.accel_limit)
    return effective_limit, path.accel_limit

  def _build_mpc_accel_max(
    self,
    path: _PacePath,
    envelope: EnergyEnvelope,
    filtered_cap: float,
    previous_mpc_source,
    accel_limit: float,
  ) -> tuple[float, ...] | None:
    """Build a short pre-MPC bound while leaving the future horizon stock-warm."""
    # Stock tip-in removes launch delay and gives every profile the same initial
    # response. The lookup table becomes active once the car is barely rolling.
    if path.departing_from_stop and not path.stopped_lead_hold:
      return None

    # A short total-lead dropout has no obstacle to hold stock MPC at zero.
    # Bound the whole horizon only while the median guard still says "stopped";
    # genuine loss transitions to the tapered confirmation path below.
    if path.state == AccelControllerState.stopHold and envelope.selected_lead < 0 and path.departure_frames == 0:
      return tuple(0.0 for _ in T_IDXS)

    special_launch_state = path.state == AccelControllerState.stopHold or path.departing_from_stop

    # Ordinary lead following must retain stock MPC constraints and obstacle
    # behavior. Include filtered and previous-source state so a radar dropout
    # cannot switch the profile bound on for only one or two frames.
    lead_guarded = envelope.selected_lead >= 0 or math.isfinite(filtered_cap) or self._lead_source(previous_mpc_source)
    if not special_launch_state and lead_guarded:
      return None

    if not math.isfinite(accel_limit):
      return None

    bounded_limit = float(np.clip(accel_limit, 0.0, ACCEL_MAX))
    accel_max = np.full(len(T_IDXS), bounded_limit, dtype=float)
    if special_launch_state:
      # A hard low bound across the full action-delay horizon cold-soaks the
      # stop solver. Two bounded nodes plus one tapered warm-up node holds the
      # vehicle through confirmation while preserving a ready future solution.
      accel_max[MPC_LAUNCH_BOUND_NODES:] = ACCEL_MAX
      if len(accel_max) > MPC_LAUNCH_BOUND_NODES:
        if path.state == AccelControllerState.stopHold:
          warm_blend = MPC_CONFIRM_WARM_BLEND if path.departure_frames > 0 else MPC_STOP_WARM_BLEND
        else:
          warm_blend = MPC_DEPART_WARM_BLEND
        accel_max[MPC_LAUNCH_BOUND_NODES] = bounded_limit + warm_blend * (ACCEL_MAX - bounded_limit)
    return tuple(float(value) for value in accel_max)

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
      and v_ego >= 0.0
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
    config = PROFILE_CONFIGS[profile]
    profile_accel_max = self.get_profile_accel_max(profile, v_ego)
    launch_delta_v = LAUNCH_DELTA_V
    delay = self._delay()
    valid_context = self._valid_context(
      base_speed,
      v_ego,
      a_ego,
      planner_speed,
      stock_accel_max,
      planner_accel,
      delay,
      engaged,
      cruise_initialized,
      controller_fault,
    )

    envelope = self.calculate_energy_envelope(radar_state, v_ego, a_ego, profile, follow_personality) if valid_context else EnergyEnvelope()

    if valid_context:
      shadow_filtered_cap = self._update_path(
        self.shadow,
        envelope.cap,
        base_speed,
        v_ego,
        config,
        previous_mpc_source,
        planner_speed,
        previous_should_stop,
        envelope.has_nearly_stopped_lead,
        launch_delta_v,
      )
      self._update_accel_limit(self.shadow, stock_accel_max, planner_accel, profile_accel_max)
      shadow_active = True
    else:
      self.shadow.reset()
      shadow_filtered_cap = math.inf
      shadow_active = False

    live_active = valid_context and bool(enabled) and bool(acc_selected)
    if live_active:
      live_filtered_cap = self._update_path(
        self.live,
        envelope.cap,
        base_speed,
        v_ego,
        config,
        previous_mpc_source,
        planner_speed,
        previous_should_stop,
        envelope.has_nearly_stopped_lead,
        launch_delta_v,
      )
      effective_accel_max, controller_accel_max = self._update_accel_limit(
        self.live, stock_accel_max, planner_accel, profile_accel_max
      )
      # Feed only the controller-owned ceiling into MPC. Stock's speed, turn,
      # coast, and no-throttle limits remain in their original output clip.
      mpc_accel_max = self._build_mpc_accel_max(
        self.live, envelope, live_filtered_cap, previous_mpc_source, controller_accel_max,
      )
      mpc_shape_cruise = (
        mpc_accel_max is not None
        and self.live.state != AccelControllerState.stopHold
        and not self.live.departing_from_stop
      )
      if mpc_accel_max is None:
        effective_accel_max = stock_accel_max
      if self.live.state == AccelControllerState.stopHold:
        # Bounds provide the dropout/creep guard while the stock cruise target
        # keeps the solver ready for a confirmed departure.
        target_speed = base_speed
      elif self.live.departing_from_stop and v_ego < STOP_HOLD_EGO_SPEED and envelope.selected_lead >= 0:
        # A moving lead keeps stock MPC well-conditioned during a confirmed
        # departure. Clear-road launches retain the bounded live pace below.
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
