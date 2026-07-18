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


PROFILE_CONFIGS = {
  AccelProfile.eco: ProfileConfig(comfort_decel=0.25),
  AccelProfile.normal: ProfileConfig(comfort_decel=0.32),
  AccelProfile.sport: ProfileConfig(comfort_decel=0.38),
}

# Keep launch responsive, then separate the profiles above walking speed.
ACCEL_PROFILE_MAX_BP = [0.0, 3.0, 10.0, 25.0, 40.0]
ACCEL_PROFILE_MAX_V = {
  AccelProfile.eco: [1.55, 1.25, 0.85, 0.50, 0.30],
  AccelProfile.normal: [1.70, 1.40, 1.05, 0.65, 0.45],
  AccelProfile.sport: [2.00, 1.90, 1.70, 1.20, 0.90],
}

CAP_FILTER_FRAMES = 5
RESTRICT_DEADBAND = 0.15
RESTRICT_EXIT_DEADBAND = 0.05
RELIEF_DEADBAND = 0.35
RELIEF_CONFIRM_FRAMES = 2
PACE_ACQUISITION_MARGIN = 0.45
EARLY_APPROACH_HEADWAY = 5.0
EARLY_APPROACH_MIN_SPEED = 8.0
CLOSING_SPEED_ENTER = 0.25
CLOSING_SPEED_EXIT = 0.10
MOVING_LEAD_DECEL_RATE = 0.15
MOVING_LEAD_DECEL_CONFIRM_FRAMES = 3
MOVING_LEAD_DECEL_ENTER_REQUIRED_DECEL = 0.15
MOVING_LEAD_DECEL_ACCEL_MAX = -0.42
MOVING_LEAD_DECEL_ACCEL_SLEW_RATE = 0.50
MOVING_LEAD_DECEL_RELEASE_RATE = 1.00
MOVING_LEAD_DECEL_EXIT_REQUIRED_DECEL = 0.05
MOVING_LEAD_DECEL_EXIT_FRAMES = 4
STOP_HOLD_EGO_SPEED = 0.30
STOP_HOLD_CAP = 0.50
STOP_HOLD_PRECONDITION_SPEED = 0.05
STOP_HOLD_PRECONDITION_ACCEL_MAX = -0.25
STOP_HOLD_PRECONDITION_EXTRA = 0.20
STOPPED_LEAD_SPEED = 0.30
STOP_HOLD_EXIT_CAP = 0.80
STOP_HOLD_EXIT_FRAMES = 4
MAX_LEAD_ACCEL_TAU = 10.0
VEGO_NOISE_TOLERANCE = 0.10
LEAD_SPEED_NOISE_TOLERANCE = 0.10
ACCEL_LIMIT_TIGHTEN_RATE = 2.0
ACCEL_LIMIT_RAISE_RATE = 1.0
ACCEL_LIMIT_BYPASS_RATE = 2.0
ACCEL_BOUND_HORIZON_RATE = 1.0
STANDSTILL_MPC_ACCEL_SEED = 0.95
STANDSTILL_ACCEL_LIMIT_RAISE_RATE = 2.0
ACCEL_SHAPE_WARMUP_FRAMES = 3


@dataclass(frozen=True)
class EnergyEnvelope:
  cap: float = math.inf
  selected_lead: int = -1
  selected_lead_speed: float = math.inf
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
  mpc_apply_accel_constraint: bool
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
  relief_frames: int = 0
  departure_frames: int = 0
  departing_from_stop: bool = False
  filtered_lead_active: bool = False
  previous_lead_speed: float | None = None
  lead_decel_frames: int = 0
  moving_lead_decel: bool = False
  moving_lead_release: bool = False
  moving_lead_relief_frames: int = 0
  moving_lead_accel_max: float | None = None

  def reset(self) -> None:
    self.cap_samples = deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES)
    self.pace = None
    self.state = AccelControllerState.inactive
    self.relief_frames = 0
    self.departure_frames = 0
    self.departing_from_stop = False
    self.filtered_lead_active = False
    self.previous_lead_speed = None
    self.lead_decel_frames = 0
    self.moving_lead_decel = False
    self.moving_lead_release = False
    self.moving_lead_relief_frames = 0
    self.moving_lead_accel_max = None

  def update_filter(self, cap: float) -> float:
    self.cap_samples.append(cap)
    return sorted(self.cap_samples)[CAP_FILTER_FRAMES // 2]


class AccelController:
  """Pre-MPC acceleration profile and relative-pace cruise governor."""

  def __init__(self, CP, dt: float = DT_MDL):
    if not math.isfinite(dt) or dt <= 0.0:
      raise ValueError("dt must be finite and positive")

    self.CP = CP
    self.dt = dt
    self.live = _PacePath()
    self.shadow = _PacePath()
    self._reset_accel_limit()
    self._clear_recovery_state()

  def _reset_accel_limit(self, value: float | None = None) -> None:
    self.live_accel_max = value
    self.live_accel_bypass = False
    self.live_accel_rejoin = False
    self.live_accel_warmup_frames = 0

  def _clear_recovery_state(self) -> None:
    self.recovery_available = False
    self.recovery_target_speed = math.inf
    self.recovery_accel_max: tuple[float, ...] | None = None
    self.recovery_shape_cruise = False
    self.recovery_apply_accel_constraint = False

  def _cache_recovery_state(self, target_speed: float, accel_max: tuple[float, ...] | None,
                            shape_cruise: bool, apply_accel_constraint: bool) -> None:
    self.recovery_available = True
    self.recovery_target_speed = target_speed
    self.recovery_accel_max = accel_max
    self.recovery_shape_cruise = shape_cruise
    self.recovery_apply_accel_constraint = apply_accel_constraint

  @staticmethod
  def _profile(profile: int | AccelProfile) -> AccelProfile:
    try:
      return AccelProfile(profile)
    except (TypeError, ValueError):
      return AccelProfile.normal

  @classmethod
  def get_profile_accel_max(cls, profile: int | AccelProfile, v_ego: float) -> float:
    if not math.isfinite(v_ego):
      return math.nan

    selected_profile = cls._profile(profile)
    return float(np.interp(max(v_ego, 0.0), ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V[selected_profile]))

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
        distance = -v_ego * v_ego / (2.0 * a_ego) if v_ego > 0.0 else 0.0
        return distance, 0.0

    distance = v_ego * delay + 0.5 * a_ego * delay * delay
    return max(distance, 0.0), max(v_ego + a_ego * delay, 0.0)

  @staticmethod
  def _valid_lead(lead) -> bool:
    try:
      d_rel, v_lead, a_lead, a_lead_tau = (float(lead.dRel), float(lead.vLeadK), float(lead.aLeadK), float(lead.aLeadTau))
      status = bool(lead.status)
    except (AttributeError, TypeError, ValueError, OverflowError):
      return False

    return (
      status
      and all(math.isfinite(value) for value in (d_rel, v_lead, a_lead, a_lead_tau))
      and d_rel >= 0.0
      and v_lead >= -LEAD_SPEED_NOISE_TOLERANCE
      and 0.0 < a_lead_tau <= MAX_LEAD_ACCEL_TAU
    )

  def calculate_energy_envelope(self, radar_state, v_ego: float, a_ego: float, profile: int | AccelProfile,
                                follow_personality=log.LongitudinalPersonality.standard) -> EnergyEnvelope:
    selected_profile = self._profile(profile)
    config = PROFILE_CONFIGS[selected_profile]
    delay = self._delay()
    if not all(math.isfinite(value) for value in (v_ego, a_ego, delay)) or v_ego < 0.0 or delay < 0.0:
      return EnergyEnvelope()

    try:
      t_follow = get_T_FOLLOW(follow_personality)
    except (NotImplementedError, TypeError, ValueError):
      t_follow = get_T_FOLLOW(log.LongitudinalPersonality.standard)

    x_ego, v_ego_delay = self._project_ego(v_ego, a_ego, delay)
    candidates = []
    departure_candidates = []
    for lead_index, lead in enumerate((radar_state.leadOne, radar_state.leadTwo)):
      if not self._valid_lead(lead):
        continue

      try:
        lead_xv = LongitudinalMpc.extrapolate_lead(
          float(lead.dRel),
          float(lead.vLeadK),
          float(np.clip(lead.aLeadK, -10.0, 5.0)),
          float(lead.aLeadTau),
        )
        x_lead = float(np.interp(delay, T_IDXS, lead_xv[:, 0]))
        v_lead = float(np.interp(delay, T_IDXS, lead_xv[:, 1]))
        if not all(math.isfinite(value) and value >= 0.0 for value in (x_lead, v_lead)):
          continue

        match_gap = STOP_DISTANCE + t_follow * v_lead
        usable_gap = max(x_lead - x_ego - match_gap, 0.0)
        closing_speed = max(v_ego_delay - v_lead, 0.0)
        required_decel = 0.0 if closing_speed == 0.0 else math.inf if usable_gap == 0.0 else closing_speed**2 / (2.0 * usable_gap)
        cap = v_lead + math.sqrt(2.0 * config.comfort_decel * usable_gap)
        departure_distance = x_lead + float(get_stopped_equivalence_factor(v_lead))
      except (FloatingPointError, OverflowError, TypeError, ValueError):
        continue

      if not all(math.isfinite(value) for value in (usable_gap, closing_speed, cap, departure_distance)) or math.isnan(required_decel) or required_decel < 0.0:
        continue

      candidates.append(EnergyEnvelope(
        cap=cap, selected_lead=lead_index, selected_lead_speed=v_lead, departure_lead_speed=v_lead,
        usable_gap=usable_gap, closing_speed=closing_speed, required_decel=required_decel,
      ))
      departure_candidates.append((departure_distance, v_lead))

    if not candidates:
      return EnergyEnvelope()

    selected = min(candidates, key=lambda candidate: candidate.cap)
    departure_lead_speed = min(departure_candidates, key=lambda candidate: candidate[0])[1]
    return EnergyEnvelope(
      cap=selected.cap, selected_lead=selected.selected_lead, selected_lead_speed=selected.selected_lead_speed,
      departure_lead_speed=departure_lead_speed, usable_gap=selected.usable_gap, closing_speed=selected.closing_speed,
      required_decel=selected.required_decel, has_nearly_stopped_lead=departure_lead_speed < STOPPED_LEAD_SPEED,
    )

  @staticmethod
  def _lead_source(source) -> bool:
    return source in (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1)

  def _update_moving_lead_decel(self, path: _PacePath, envelope: EnergyEnvelope, base_speed: float, v_ego: float) -> None:
    lead_speed = envelope.selected_lead_speed
    speed_falling = (
      path.previous_lead_speed is not None
      and math.isfinite(path.previous_lead_speed)
      and math.isfinite(lead_speed)
      and lead_speed < path.previous_lead_speed - MOVING_LEAD_DECEL_RATE * self.dt
    )
    path.lead_decel_frames = path.lead_decel_frames + 1 if speed_falling else 0
    if (
      not path.moving_lead_decel
      and path.lead_decel_frames >= MOVING_LEAD_DECEL_CONFIRM_FRAMES
      and path.filtered_lead_active
      and envelope.cap < base_speed - RESTRICT_DEADBAND
      and envelope.required_decel >= MOVING_LEAD_DECEL_ENTER_REQUIRED_DECEL
      and v_ego >= EARLY_APPROACH_MIN_SPEED
      and envelope.closing_speed > CLOSING_SPEED_ENTER
    ):
      path.moving_lead_decel = True
      if not path.moving_lead_release:
        path.moving_lead_accel_max = None
      path.moving_lead_release = False

    matched_moving_lead = (
      path.moving_lead_decel
      and lead_speed > STOPPED_LEAD_SPEED
      and envelope.closing_speed <= CLOSING_SPEED_EXIT
      and envelope.required_decel <= MOVING_LEAD_DECEL_EXIT_REQUIRED_DECEL
    )
    path.moving_lead_relief_frames = path.moving_lead_relief_frames + 1 if matched_moving_lead else 0
    if path.state == AccelControllerState.stopHold:
      path.moving_lead_decel = False
      path.moving_lead_release = False
      path.moving_lead_relief_frames = 0
      path.moving_lead_accel_max = None
    elif path.moving_lead_relief_frames >= MOVING_LEAD_DECEL_EXIT_FRAMES or not path.filtered_lead_active:
      path.moving_lead_decel = False
      path.moving_lead_release = path.moving_lead_accel_max is not None and path.moving_lead_accel_max < 0.0
      path.moving_lead_relief_frames = 0

    path.previous_lead_speed = lead_speed if math.isfinite(lead_speed) else None

  def _update_path(self, path: _PacePath, envelope: EnergyEnvelope, base_speed: float, v_ego: float, config: ProfileConfig,
                   previous_mpc_source, planner_speed: float, previous_should_stop: bool) -> float:
    raw_cap = envelope.cap
    filtered_cap = path.update_filter(raw_cap)
    filtered_lead_active = math.isfinite(filtered_cap)
    acquired_restrictive_lead = filtered_lead_active and not path.filtered_lead_active and envelope.closing_speed > 0.0
    path.filtered_lead_active = filtered_lead_active

    if path.pace is None:
      path.pace = base_speed
      path.state = AccelControllerState.free

    early_approach = envelope.usable_gap >= EARLY_APPROACH_HEADWAY * max(v_ego, 1.0)
    if acquired_restrictive_lead and path.pace >= base_speed - RESTRICT_DEADBAND and filtered_cap < path.pace - RESTRICT_DEADBAND:
      if not early_approach:
        path.pace = min(path.pace, v_ego)
      elif v_ego >= EARLY_APPROACH_MIN_SPEED:
        path.pace = min(path.pace, max(envelope.selected_lead_speed, v_ego - PACE_ACQUISITION_MARGIN))

    path.pace = min(path.pace, base_speed)
    if self._lead_source(previous_mpc_source) and not math.isfinite(raw_cap) and planner_speed < path.pace:
      path.pace = max(planner_speed, 0.0)

    if path.state == AccelControllerState.stopHold:
      moving_lead = raw_cap > STOP_HOLD_EXIT_CAP and filtered_cap > STOP_HOLD_EXIT_CAP
      confirmed_lead_loss = not math.isfinite(raw_cap) and not math.isfinite(filtered_cap)
      path.departure_frames = path.departure_frames + 1 if moving_lead or confirmed_lead_loss else 0
      if path.departure_frames < STOP_HOLD_EXIT_FRAMES:
        path.pace = 0.0
        return filtered_cap

      path.pace = base_speed
      path.state = AccelControllerState.release
      path.relief_frames = 0
      path.departure_frames = 0
      path.departing_from_stop = True
      return filtered_cap

    stopped_lead_is_close = envelope.has_nearly_stopped_lead and raw_cap < STOP_HOLD_CAP
    stop_evidence = filtered_cap < STOP_HOLD_CAP or stopped_lead_is_close
    clearly_moving_lead = math.isfinite(raw_cap) and envelope.departure_lead_speed > STOPPED_LEAD_SPEED
    stale_plan_stop = previous_should_stop and not path.departing_from_stop and not clearly_moving_lead
    if v_ego < STOP_HOLD_EGO_SPEED and (stop_evidence or stale_plan_stop):
      path.pace = 0.0
      path.state = AccelControllerState.stopHold
      path.relief_frames = 0
      path.departure_frames = 0
      path.departing_from_stop = False
      return filtered_cap

    if path.departing_from_stop and v_ego >= STOP_HOLD_EGO_SPEED:
      path.departing_from_stop = False

    filter_warmup_relief = math.isfinite(raw_cap) and not math.isfinite(filtered_cap) and path.pace < base_speed - RESTRICT_DEADBAND
    if filter_warmup_relief:
      path.state = AccelControllerState.hold
      path.relief_frames = 0
      return filtered_cap

    transient_relief = math.isfinite(filtered_cap) and (not math.isfinite(raw_cap) or raw_cap > filtered_cap + RELIEF_DEADBAND)
    if transient_relief and not path.departing_from_stop and path.state != AccelControllerState.release:
      path.state = AccelControllerState.hold
      path.relief_frames = 0
      return filtered_cap

    ceiling = base_speed if path.departing_from_stop else min(base_speed, filtered_cap)
    restrict_deadband = RESTRICT_EXIT_DEADBAND if path.state == AccelControllerState.restrict else RESTRICT_DEADBAND
    cap_restriction = ceiling <= path.pace - restrict_deadband
    closing_threshold = CLOSING_SPEED_EXIT if path.state == AccelControllerState.restrict else CLOSING_SPEED_ENTER
    closing_guard = math.isfinite(raw_cap) and ceiling < base_speed - RESTRICT_DEADBAND and envelope.closing_speed > closing_threshold
    if cap_restriction:
      path.pace = max(ceiling, path.pace - config.comfort_decel * self.dt)
      path.state = AccelControllerState.restrict
      path.relief_frames = 0
      return filtered_cap

    if closing_guard:
      path.state = AccelControllerState.hold
      path.relief_frames = 0
      return filtered_cap

    relief = ceiling - path.pace
    if relief > RELIEF_DEADBAND and path.state != AccelControllerState.release:
      confirmed_lead_loss = not math.isfinite(raw_cap) and not math.isfinite(filtered_cap)
      path.relief_frames = path.relief_frames + 1 if confirmed_lead_loss else RELIEF_CONFIRM_FRAMES
      path.state = AccelControllerState.release if path.relief_frames >= RELIEF_CONFIRM_FRAMES else AccelControllerState.hold
    elif path.state != AccelControllerState.release:
      path.relief_frames = 0
    release_active = path.state == AccelControllerState.release and relief >= 0.0

    if release_active:
      path.pace = ceiling
      path.state = AccelControllerState.free if path.pace >= base_speed else AccelControllerState.release
      path.relief_frames = 0
    elif path.state == AccelControllerState.release:
      pass
    elif not math.isfinite(raw_cap) and not math.isfinite(filtered_cap) and 0.0 < relief <= RELIEF_DEADBAND:
      path.pace = ceiling
      path.state = AccelControllerState.free
    else:
      path.state = AccelControllerState.free if path.pace >= ceiling - RESTRICT_DEADBAND else AccelControllerState.hold

    return filtered_cap

  @staticmethod
  def _valid_context(base_speed: float, v_ego: float, a_ego: float, planner_speed: float, stock_accel_max: float,
                     planner_accel: float, delay: float, engaged: bool, cruise_initialized: bool, controller_fault: bool) -> bool:
    values = (base_speed, v_ego, a_ego, planner_speed, stock_accel_max, planner_accel, delay)
    return (
      engaged
      and cruise_initialized
      and not controller_fault
      and base_speed >= 0.0
      and v_ego >= -VEGO_NOISE_TOLERANCE
      and planner_speed >= 0.0
      and delay >= 0.0
      and all(math.isfinite(value) for value in values)
    )

  def _build_mpc_accel_max(self, profile_accel_max: float, stock_accel_max: float, planner_accel: float, v_ego: float,
                           lead_present: bool, closing_speed: float, stop_hold: bool) -> tuple[float, tuple[float, ...] | None, bool, bool]:
    def build_trajectory(limit: float) -> tuple[float, ...]:
      initial_limit = max(limit, min(planner_accel, ACCEL_MAX))
      return tuple(max(limit, initial_limit - ACCEL_BOUND_HORIZON_RATE * t) for t in T_IDXS)

    requested_accel_max = min(profile_accel_max, stock_accel_max)
    if stop_hold:
      self._reset_accel_limit()
      if v_ego > STOP_HOLD_PRECONDITION_SPEED:
        return requested_accel_max, None, False, False
      guard_time = self._delay() + STOP_HOLD_PRECONDITION_EXTRA
      hold_accel_max = tuple(STOP_HOLD_PRECONDITION_ACCEL_MAX if t <= guard_time else ACCEL_MAX for t in T_IDXS)
      return STOP_HOLD_PRECONDITION_ACCEL_MAX, hold_accel_max, True, True

    if self.live.departing_from_stop:
      self._reset_accel_limit(requested_accel_max)
      return requested_accel_max, None, False, False

    if self.live.moving_lead_decel and self.live.filtered_lead_active:
      if self.live.moving_lead_accel_max is None:
        self.live.moving_lead_accel_max = max(min(planner_accel, 0.0), MOVING_LEAD_DECEL_ACCEL_MAX)
      self.live.moving_lead_accel_max = max(
        MOVING_LEAD_DECEL_ACCEL_MAX,
        self.live.moving_lead_accel_max - MOVING_LEAD_DECEL_ACCEL_SLEW_RATE * self.dt,
      )
      moving_lead_accel_max = min(stock_accel_max, self.live.moving_lead_accel_max)
      return moving_lead_accel_max, build_trajectory(moving_lead_accel_max), True, True

    if self.live.moving_lead_release:
      self.live.moving_lead_accel_max = min(0.0, self.live.moving_lead_accel_max + MOVING_LEAD_DECEL_RELEASE_RATE * self.dt)
      moving_lead_accel_max = min(stock_accel_max, self.live.moving_lead_accel_max)
      if self.live.moving_lead_accel_max < 0.0:
        return moving_lead_accel_max, build_trajectory(moving_lead_accel_max), True, True
      self.live.moving_lead_release = False
      self.live.moving_lead_accel_max = None
      self._reset_accel_limit(requested_accel_max)
      return moving_lead_accel_max, build_trajectory(moving_lead_accel_max), True, True

    if requested_accel_max <= 0.0:
      self._reset_accel_limit()
      return requested_accel_max, None, False, False

    requested_accel_max = float(np.clip(requested_accel_max, 0.0, ACCEL_MAX))
    closing_threshold = CLOSING_SPEED_EXIT if self.live_accel_bypass else CLOSING_SPEED_ENTER
    if lead_present and closing_speed > closing_threshold:
      self.live_accel_bypass = True
      self.live_accel_rejoin = False
    elif self.live_accel_bypass and (not lead_present or closing_speed <= CLOSING_SPEED_EXIT):
      self.live_accel_bypass = False
      self.live_accel_rejoin = self.live_accel_max is not None and self.live_accel_max < ACCEL_MAX

    if self.live_accel_max is None and not self.live_accel_bypass and v_ego >= STOP_HOLD_EGO_SPEED:
      if self.live_accel_warmup_frames < ACCEL_SHAPE_WARMUP_FRAMES:
        self.live_accel_warmup_frames += 1
        return requested_accel_max, None, False, False

    if self.live_accel_max is None:
      self.live_accel_max = min(requested_accel_max, STANDSTILL_MPC_ACCEL_SEED) if v_ego < STOP_HOLD_EGO_SPEED else ACCEL_MAX
      self.live_accel_warmup_frames = 0
      if v_ego < STOP_HOLD_EGO_SPEED and not self.live_accel_bypass:
        return self.live_accel_max, build_trajectory(self.live_accel_max), True, True

    if self.live_accel_bypass or self.live_accel_rejoin:
      self.live_accel_max = min(ACCEL_MAX, self.live_accel_max + ACCEL_LIMIT_BYPASS_RATE * self.dt)
      if self.live_accel_max < ACCEL_MAX:
        return self.live_accel_max, build_trajectory(self.live_accel_max), True, False
      if self.live_accel_bypass:
        return requested_accel_max, None, False, False
      self.live_accel_rejoin = False
      return self.live_accel_max, build_trajectory(self.live_accel_max), True, True

    if requested_accel_max >= self.live_accel_max:
      raise_rate = STANDSTILL_ACCEL_LIMIT_RAISE_RATE if v_ego < STOP_HOLD_EGO_SPEED else ACCEL_LIMIT_RAISE_RATE
      self.live_accel_max = min(requested_accel_max, self.live_accel_max + raise_rate * self.dt)
    else:
      self.live_accel_max = max(requested_accel_max, self.live_accel_max - ACCEL_LIMIT_TIGHTEN_RATE * self.dt)

    self.live_accel_max = float(np.clip(self.live_accel_max, 0.0, ACCEL_MAX))
    return self.live_accel_max, build_trajectory(self.live_accel_max), True, True

  def reset(self) -> None:
    self.live.reset()
    self.shadow.reset()
    self._reset_accel_limit()
    self._clear_recovery_state()

  def update(self, radar_state, *, base_speed: float, v_ego: float, a_ego: float, profile: int | AccelProfile, follow_personality,
             enabled: bool, acc_selected: bool, engaged: bool, cruise_initialized: bool, previous_mpc_source,
             planner_speed: float, stock_accel_max: float, planner_accel: float, previous_should_stop: bool,
             controller_fault: bool = False) -> AccelControllerResult:
    selected_profile = self._profile(profile)
    sanitized_v_ego = max(v_ego, 0.0) if math.isfinite(v_ego) and v_ego >= -VEGO_NOISE_TOLERANCE else v_ego
    profile_accel_max = self.get_profile_accel_max(selected_profile, sanitized_v_ego)
    delay = self._delay()
    valid_context = self._valid_context(base_speed, sanitized_v_ego, a_ego, planner_speed, stock_accel_max, planner_accel,
                                        delay, engaged, cruise_initialized, controller_fault)
    recovery_context = self._valid_context(base_speed, sanitized_v_ego, a_ego, planner_speed, stock_accel_max, planner_accel,
                                            delay, engaged, cruise_initialized, False)
    envelope = (self.calculate_energy_envelope(radar_state, sanitized_v_ego, a_ego, selected_profile, follow_personality)
                if valid_context else EnergyEnvelope())
    config = PROFILE_CONFIGS[selected_profile]

    if valid_context:
      shadow_filtered_cap = self._update_path(self.shadow, envelope, base_speed, sanitized_v_ego, config, previous_mpc_source,
                                              planner_speed, previous_should_stop)
      self._update_moving_lead_decel(self.shadow, envelope, base_speed, sanitized_v_ego)
      shadow_active = True
    else:
      self.shadow.reset()
      shadow_filtered_cap = math.inf
      shadow_active = False

    live_active = valid_context and bool(enabled) and bool(acc_selected)
    if live_active:
      if self.live.pace is None and self.recovery_available:
        self.live.pace = min(base_speed, self.recovery_target_speed)
      live_filtered_cap = self._update_path(self.live, envelope, base_speed, sanitized_v_ego, config, previous_mpc_source,
                                            planner_speed, previous_should_stop)
      self._update_moving_lead_decel(self.live, envelope, base_speed, sanitized_v_ego)
      target_speed = min(base_speed, self.live.pace if self.live.pace is not None else base_speed)
      effective_accel_max, mpc_accel_max, mpc_shape_cruise, mpc_apply_accel_constraint = self._build_mpc_accel_max(
        profile_accel_max, stock_accel_max, planner_accel, sanitized_v_ego, envelope.selected_lead >= 0,
        envelope.closing_speed, self.live.state == AccelControllerState.stopHold,
      )
      self._cache_recovery_state(target_speed, mpc_accel_max, mpc_shape_cruise, mpc_apply_accel_constraint)
    else:
      self.live.reset()
      self._reset_accel_limit()
      live_filtered_cap = math.inf
      target_speed = base_speed
      retain_recovery_state = bool(controller_fault) and recovery_context and self.recovery_available
      if retain_recovery_state:
        target_speed = min(base_speed, self.recovery_target_speed)
        mpc_accel_max = (tuple(min(value, stock_accel_max, ACCEL_MAX) for value in self.recovery_accel_max)
                         if self.recovery_accel_max is not None else None)
        mpc_shape_cruise = self.recovery_shape_cruise if mpc_accel_max is not None else False
        mpc_apply_accel_constraint = self.recovery_apply_accel_constraint if mpc_accel_max is not None else False
        effective_accel_max = min(mpc_accel_max) if mpc_accel_max is not None else math.inf
      else:
        self._clear_recovery_state()
        effective_accel_max = math.inf
        mpc_accel_max = None
        mpc_shape_cruise = False
        mpc_apply_accel_constraint = False

    return AccelControllerResult(
      target_speed=target_speed, enabled=bool(enabled), active=live_active, shadow_active=shadow_active,
      launching=live_active and self.live.departing_from_stop, profile=selected_profile,
      profile_accel_max=profile_accel_max if live_active else math.inf, effective_accel_max=effective_accel_max,
      mpc_accel_max=mpc_accel_max, mpc_shape_cruise=mpc_shape_cruise, mpc_apply_accel_constraint=mpc_apply_accel_constraint,
      state=self.live.state, shadow_state=self.shadow.state, base_speed=base_speed, raw_energy_cap=envelope.cap,
      live_filtered_cap=live_filtered_cap, shadow_filtered_cap=shadow_filtered_cap,
      live_pace=self.live.pace if self.live.pace is not None else math.inf,
      shadow_pace=self.shadow.pace if self.shadow.pace is not None else math.inf,
      selected_lead=envelope.selected_lead, usable_gap=envelope.usable_gap, closing_speed=envelope.closing_speed,
      required_decel=envelope.required_decel,
    )
