#!/usr/bin/env python3
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
import math

import numpy as np

from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, STOP_DISTANCE, T_IDXS, get_T_FOLLOW, get_stopped_equivalence_factor
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import (
  ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V, APPROACH_CLOSING_SPEED, APPROACH_LEAD_DECEL, APPROACH_LEAD_SPEED_MARGIN, APPROACH_MIN_SPEED,
  BRAKE_CAP_MARGIN, CAP_FILTER_FRAMES, CAP_RELAX_JERK, CAP_TIGHTEN_JERK, COAST_MATCH_CLOSING_SPEED, COAST_MATCH_USABLE_GAP,
  DROPOUT_ACTION_ACCEL_MARGIN, HORIZON_DOWN_JERK, HORIZON_HOLD_TIME, HORIZON_SPEED_BUDGET, HORIZON_UP_JERK, MAX_LEAD_ACCEL_TAU,
  MIN_LEAD_SPEED, POSITIVE_MPC_HEADROOM, PROFILE_CONFIGS, PROFILE_TRANSITION_JERK, RADAR_STALE_TIMEOUT, RELIEF_CAP_MARGIN,
  RELIEF_CONFIRM_FRAMES, RELIEF_LEAD_SPEED_STEP, RELIEF_MPC_JERK, REQUIRED_DECEL_MARGIN, ROUTINE_DECEL_MAX, STOP_HOLD_EGO_SPEED,
  SHALLOW_BRAKE_BOUND, SHALLOW_BRAKE_RELIEF_TIME, STOP_GAP_RESERVE, STOP_GAP_RESERVE_DECEL_BP, STOP_GAP_RESERVE_LEAD_SPEED,
  STOP_HOLD_CREEP_ABORT_FRAMES, STOP_HOLD_CREEP_DISTANCE, STOP_HOLD_CREEP_SPEED,
  STOP_HOLD_EXIT_FRAMES, STOP_HOLD_EXIT_SPEED,
  STOPPED_LEAD_SPEED, URGENT_CLOSING_SPEED, URGENT_RELEASE_ACCEL, URGENT_REQUIRED_DECEL, URGENT_TTC, URGENT_TTC_MIN_CLOSING,
  VEGO_NOISE_TOLERANCE, AccelProfile,
)


class AccelControllerState(IntEnum):
  inactive = 0
  free = 1
  restrict = 2
  hold = 3
  release = 4
  stopHold = 5


@dataclass(frozen=True)
class EnergyEnvelope:
  cap: float = math.inf
  selected_lead: int = -1
  selected_lead_speed: float = math.inf
  selected_lead_decel: float = 0.0
  departure_lead_index: int = -1
  departure_lead_speed: float = math.inf
  departure_cap: float = math.inf
  departure_lead_speeds: tuple[float, float] = (math.inf, math.inf)
  departure_lead_separations: tuple[float, float] = (-math.inf, -math.inf)
  usable_gap: float = math.inf
  safety_usable_gap: float = math.inf
  closing_speed: float = 0.0
  required_decel: float = 0.0
  has_nearly_stopped_lead: bool = False
  lead_status: bool = False


@dataclass(frozen=True)
class AccelControllerResult:
  target_speed: float
  enabled: bool
  active: bool
  shadow_active: bool
  launching: bool
  stock_mode: bool
  profile: AccelProfile
  profile_accel_max: float
  positive_accel_max: float
  effective_accel_max: float
  mpc_accel_max: tuple[float, ...] | None
  state: AccelControllerState
  shadow_state: AccelControllerState
  base_speed: float
  raw_energy_cap: float
  live_filtered_cap: float
  shadow_filtered_cap: float
  selected_lead: int
  selected_lead_speed: float
  usable_gap: float
  closing_speed: float
  required_decel: float


@dataclass
class _ControllerPath:
  cap_samples: deque[float] = field(default_factory=lambda: deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES))
  required_samples: deque[float] = field(default_factory=lambda: deque(maxlen=CAP_FILTER_FRAMES))
  lead_decel_samples: deque[float] = field(default_factory=lambda: deque(maxlen=CAP_FILTER_FRAMES))
  departure_samples: tuple[deque[float], deque[float]] = field(
    default_factory=lambda: (deque(maxlen=CAP_FILTER_FRAMES), deque(maxlen=CAP_FILTER_FRAMES)),
  )
  departure_references: list[float | None] = field(default_factory=lambda: [None, None])
  bound: float | None = None
  state: AccelControllerState = AccelControllerState.inactive
  relief_frames: int = 0
  bound_relief_frames: int = 0
  bound_relief_required_frames: int = 0
  departure_frames: int = 0
  creep_abort_frames: int = 0
  stale_frames: int = 0
  urgent: bool = False
  urgent_severe: bool = False
  urgent_safe_frames: int = 0
  departing_from_stop: bool = False
  previous_lead_speed: float | None = None
  lead_speed_relief: bool = False

  @property
  def filtered_cap(self) -> float:
    return sorted(self.cap_samples)[CAP_FILTER_FRAMES // 2]

  @property
  def robust_required_decel(self) -> float:
    return float(np.median(self.required_samples)) if self.required_samples else 0.0

  @property
  def robust_lead_decel(self) -> float:
    return float(np.median(self.lead_decel_samples)) if self.lead_decel_samples else 0.0

  def robust_departure_separation(self, lead_index: int) -> float:
    samples = self.departure_samples[lead_index]
    return float(np.median(samples)) if samples else -math.inf

  def reset(self) -> None:
    self.cap_samples = deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES)
    self.required_samples.clear()
    self.lead_decel_samples.clear()
    self.departure_samples = (deque(maxlen=CAP_FILTER_FRAMES), deque(maxlen=CAP_FILTER_FRAMES))
    self.departure_references = [None, None]
    self.bound = None
    self.state = AccelControllerState.inactive
    self.relief_frames = 0
    self.bound_relief_frames = 0
    self.bound_relief_required_frames = 0
    self.departure_frames = 0
    self.creep_abort_frames = 0
    self.stale_frames = 0
    self.urgent = False
    self.urgent_severe = False
    self.urgent_safe_frames = 0
    self.departing_from_stop = False
    self.previous_lead_speed = None
    self.lead_speed_relief = False


class AccelController:
  def __init__(self, CP, dt: float = DT_MDL):
    if not math.isfinite(dt) or dt <= 0.0:
      raise ValueError("dt must be finite and positive")

    self.CP = CP
    self.dt = dt
    self.radar_stale_frames = max(1, math.ceil(RADAR_STALE_TIMEOUT / dt))
    self.shallow_brake_relief_frames = max(RELIEF_CONFIRM_FRAMES, math.ceil(SHALLOW_BRAKE_RELIEF_TIME / dt))
    self.live = _ControllerPath()
    self.shadow = _ControllerPath()

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
    except (AttributeError, OverflowError, TypeError, ValueError):
      return math.nan

  @staticmethod
  def _project_ego(v_ego: float, a_ego: float, delay: float) -> tuple[float, float]:
    if a_ego < 0.0:
      stop_time = -v_ego / a_ego if v_ego > 0.0 else 0.0
      if stop_time <= delay:
        distance = -v_ego**2 / (2.0 * a_ego) if v_ego > 0.0 else 0.0
        return distance, 0.0
    return max(v_ego * delay + 0.5 * a_ego * delay**2, 0.0), max(v_ego + a_ego * delay, 0.0)

  @staticmethod
  def _lead_values(lead) -> tuple[float, float, float, float] | None:
    try:
      if not lead.status:
        return None
      d_rel = float(lead.dRel)
      v_lead = float(lead.vLeadK)
    except (AttributeError, OverflowError, TypeError, ValueError):
      return None
    if not math.isfinite(d_rel) or d_rel < 0.0 or not math.isfinite(v_lead) or v_lead < MIN_LEAD_SPEED:
      return None

    try:
      a_lead = float(lead.aLeadK)
    except (AttributeError, OverflowError, TypeError, ValueError):
      a_lead = 0.0
    if not math.isfinite(a_lead):
      a_lead = 0.0

    try:
      a_lead_tau = float(lead.aLeadTau)
    except (AttributeError, OverflowError, TypeError, ValueError):
      a_lead_tau = _LEAD_ACCEL_TAU
    if not math.isfinite(a_lead_tau) or not 0.0 < a_lead_tau <= MAX_LEAD_ACCEL_TAU:
      a_lead_tau = _LEAD_ACCEL_TAU
    return d_rel, max(v_lead, 0.0), float(np.clip(a_lead, -10.0, 5.0)), a_lead_tau

  def calculate_energy_envelope(self, radar_state, v_ego: float, a_ego: float, profile: int | AccelProfile,
                                follow_personality=log.LongitudinalPersonality.standard) -> EnergyEnvelope:
    delay = self._delay()
    if not all(math.isfinite(value) for value in (v_ego, a_ego, delay)) or v_ego < 0.0 or delay < 0.0:
      return EnergyEnvelope()

    try:
      leads = (radar_state.leadOne, radar_state.leadTwo)
      lead_status = any(bool(lead.status) for lead in leads)
    except (AttributeError, TypeError, ValueError):
      return EnergyEnvelope()

    try:
      t_follow = get_T_FOLLOW(follow_personality)
    except (NotImplementedError, TypeError, ValueError):
      t_follow = get_T_FOLLOW(log.LongitudinalPersonality.standard)
    if not math.isfinite(t_follow) or t_follow < 0.0:
      return EnergyEnvelope(lead_status=lead_status)

    x_ego, v_ego_delay = self._project_ego(v_ego, a_ego, delay)
    comfort_decel = PROFILE_CONFIGS[self._profile(profile)].comfort_decel
    candidates: list[EnergyEnvelope] = []
    departure_candidates: list[tuple[float, int]] = []
    departure_speeds = [math.inf, math.inf]
    departure_separations = [-math.inf, -math.inf]
    departure_caps = [math.inf, math.inf]
    for lead_index, lead in enumerate(leads):
      values = self._lead_values(lead)
      if values is None:
        continue
      try:
        d_rel, v_lead, a_lead, a_lead_tau = values
        lead_xv = LongitudinalMpc.extrapolate_lead(d_rel, v_lead, a_lead, a_lead_tau)
        x_lead = float(np.interp(delay, T_IDXS, lead_xv[:, 0]))
        v_lead_delay = float(np.interp(delay, T_IDXS, lead_xv[:, 1]))
        safety_usable_gap = max(x_lead - x_ego - STOP_DISTANCE - t_follow * v_lead_delay, 0.0)
        closing_speed = max(v_ego_delay - v_lead_delay, 0.0)
        required_decel = (0.0 if closing_speed == 0.0 else math.inf if safety_usable_gap == 0.0
                          else closing_speed**2 / (2.0 * safety_usable_gap))
        reserve_speed = float(np.interp(v_lead_delay, (0.0, STOP_GAP_RESERVE_LEAD_SPEED), (STOP_GAP_RESERVE, 0.0)))
        reserve_scale = float(np.interp(required_decel, STOP_GAP_RESERVE_DECEL_BP, (1.0, 0.0)))
        usable_gap = max(safety_usable_gap - reserve_speed * reserve_scale, 0.0)
        cap = v_lead_delay + math.sqrt(2.0 * comfort_decel * usable_gap)
        departure_cap = v_lead_delay + math.sqrt(2.0 * comfort_decel * safety_usable_gap)
        projected_separation = x_lead - x_ego
        departure_distance = x_lead + float(get_stopped_equivalence_factor(v_lead_delay))
      except (FloatingPointError, OverflowError, TypeError, ValueError):
        continue
      finite_values = (x_lead, v_lead_delay, usable_gap, safety_usable_gap, closing_speed, cap, departure_cap, departure_distance)
      if not all(math.isfinite(value) and value >= 0.0 for value in finite_values) or math.isnan(required_decel) or required_decel < 0.0:
        continue
      if not math.isfinite(projected_separation):
        continue
      candidates.append(EnergyEnvelope(cap=cap, selected_lead=lead_index, selected_lead_speed=v_lead_delay,
                                       selected_lead_decel=max(-a_lead, 0.0), usable_gap=usable_gap,
                                       safety_usable_gap=safety_usable_gap, closing_speed=closing_speed,
                                       required_decel=required_decel, lead_status=lead_status))
      departure_candidates.append((departure_distance, lead_index))
      departure_speeds[lead_index] = v_lead_delay
      departure_separations[lead_index] = projected_separation
      departure_caps[lead_index] = departure_cap

    if not candidates:
      return EnergyEnvelope(lead_status=lead_status)
    selected = min(candidates, key=lambda candidate: candidate.cap)
    departure_lead_index = min(departure_candidates, key=lambda candidate: candidate[0])[1]
    departure_lead_speed = departure_speeds[departure_lead_index]
    return EnergyEnvelope(
      cap=selected.cap, selected_lead=selected.selected_lead, selected_lead_speed=selected.selected_lead_speed,
      selected_lead_decel=selected.selected_lead_decel, departure_lead_index=departure_lead_index,
      departure_lead_speed=departure_lead_speed, departure_cap=departure_caps[departure_lead_index],
      departure_lead_speeds=tuple(departure_speeds), departure_lead_separations=tuple(departure_separations),
      usable_gap=selected.usable_gap, safety_usable_gap=selected.safety_usable_gap, closing_speed=selected.closing_speed,
      required_decel=selected.required_decel, has_nearly_stopped_lead=departure_lead_speed < STOPPED_LEAD_SPEED,
      lead_status=lead_status,
    )

  @staticmethod
  def _move(value: float, target: float, rate: float, dt: float) -> float:
    return float(np.clip(target, value - rate * dt, value + rate * dt))

  @staticmethod
  def _ttc(envelope: EnergyEnvelope) -> float:
    return envelope.safety_usable_gap / envelope.closing_speed if envelope.closing_speed > 0.0 else math.inf

  def _update_samples(self, path: _ControllerPath, envelope: EnergyEnvelope) -> None:
    has_lead = envelope.selected_lead >= 0
    path.lead_speed_relief = (has_lead and path.previous_lead_speed is not None
                              and envelope.selected_lead_speed > path.previous_lead_speed + RELIEF_LEAD_SPEED_STEP)
    path.previous_lead_speed = envelope.selected_lead_speed if has_lead else None
    path.cap_samples.append(envelope.cap if has_lead else math.inf)
    for lead_index, separation in enumerate(envelope.departure_lead_separations):
      if math.isfinite(separation):
        path.departure_samples[lead_index].append(separation)
    if has_lead:
      if math.isfinite(envelope.required_decel):
        path.required_samples.append(envelope.required_decel)
      if math.isfinite(envelope.selected_lead_decel):
        path.lead_decel_samples.append(envelope.selected_lead_decel)
    else:
      path.required_samples.append(0.0)
      path.lead_decel_samples.append(0.0)

  @staticmethod
  def _reset_departure_tracking(path: _ControllerPath, envelope: EnergyEnvelope) -> None:
    path.departure_samples = (deque(maxlen=CAP_FILTER_FRAMES), deque(maxlen=CAP_FILTER_FRAMES))
    path.departure_references = [None, None]
    for lead_index, separation in enumerate(envelope.departure_lead_separations):
      if math.isfinite(separation):
        path.departure_samples[lead_index].append(separation)
        path.departure_references[lead_index] = separation
    path.departure_frames = 0
    path.creep_abort_frames = 0

  @staticmethod
  def _clear_bound_relief(path: _ControllerPath) -> None:
    path.bound_relief_frames = 0
    path.bound_relief_required_frames = 0

  @staticmethod
  def _creep_departure(path: _ControllerPath, envelope: EnergyEnvelope) -> bool:
    lead_index = envelope.departure_lead_index
    if lead_index < 0 or envelope.departure_lead_speed <= STOP_HOLD_CREEP_SPEED:
      return False

    separation = path.robust_departure_separation(lead_index)
    reference = path.departure_references[lead_index]
    return reference is not None and separation - reference >= STOP_HOLD_CREEP_DISTANCE

  def _update_path(self, path: _ControllerPath, envelope: EnergyEnvelope, base_speed: float, v_ego: float, action_accel: float,
                   positive_accel_max: float, profile: AccelProfile, previous_should_stop: bool) -> bool:
    self._update_samples(path, envelope)
    has_lead = envelope.selected_lead >= 0
    filtered_cap = path.filtered_cap
    robust_required = path.robust_required_decel
    robust_lead_decel = path.robust_lead_decel
    ttc = self._ttc(envelope)
    moving_away = (has_lead and not envelope.has_nearly_stopped_lead
                   and envelope.selected_lead_speed > v_ego + APPROACH_CLOSING_SPEED
                   and envelope.cap > v_ego + RELIEF_CAP_MARGIN)

    if path.departing_from_stop:
      if v_ego >= STOP_HOLD_EGO_SPEED:
        path.departing_from_stop = False
        path.creep_abort_frames = 0
      elif envelope.lead_status and (not has_lead or envelope.departure_lead_speed <= STOP_HOLD_CREEP_SPEED):
        path.creep_abort_frames += 1
        if path.creep_abort_frames >= STOP_HOLD_CREEP_ABORT_FRAMES:
          path.departing_from_stop = False
          path.creep_abort_frames = 0
      else:
        path.creep_abort_frames = 0

    stop_hold = (v_ego < STOP_HOLD_EGO_SPEED and not path.departing_from_stop
                 and (previous_should_stop or (envelope.lead_status and not has_lead)
                      or (has_lead and (envelope.has_nearly_stopped_lead or envelope.cap < 0.50))))

    if path.state == AccelControllerState.stopHold:
      self._clear_bound_relief(path)
      for lead_index in range(len(path.departure_references)):
        separation = path.robust_departure_separation(lead_index)
        if math.isfinite(separation) and path.departure_references[lead_index] is None:
          path.departure_references[lead_index] = separation
      departed = (not envelope.lead_status
                  or (has_lead and envelope.departure_lead_speed > STOP_HOLD_EXIT_SPEED
                      and envelope.departure_cap > STOP_HOLD_EXIT_SPEED)
                  or self._creep_departure(path, envelope))
      path.departure_frames = path.departure_frames + 1 if departed else 0
      path.bound = 0.0
      if path.departure_frames < STOP_HOLD_EXIT_FRAMES:
        return False
      path.state = AccelControllerState.free
      path.bound = positive_accel_max
      path.departure_frames = 0
      path.departing_from_stop = True
      return False

    if stop_hold:
      path.state = AccelControllerState.stopHold
      path.bound = 0.0
      path.relief_frames = 0
      self._clear_bound_relief(path)
      path.departure_frames = 0
      path.urgent = False
      path.urgent_severe = False
      path.urgent_safe_frames = 0
      path.departing_from_stop = False
      self._reset_departure_tracking(path, envelope)
      return False

    urgent_closing = envelope.closing_speed > URGENT_TTC_MIN_CLOSING
    raw_urgent = (has_lead and v_ego >= STOP_HOLD_EGO_SPEED
                  and (envelope.closing_speed >= URGENT_CLOSING_SPEED
                       or (urgent_closing and envelope.required_decel >= URGENT_REQUIRED_DECEL)
                       or (urgent_closing and ttc <= URGENT_TTC)))
    if raw_urgent:
      path.urgent = True
      path.urgent_severe |= envelope.closing_speed >= URGENT_CLOSING_SPEED or envelope.required_decel >= URGENT_REQUIRED_DECEL
      path.urgent_safe_frames = 0
      path.bound = None
      path.state = AccelControllerState.hold
      path.relief_frames = 0
      self._clear_bound_relief(path)
      return True

    if path.urgent:
      matched = has_lead and envelope.closing_speed <= APPROACH_CLOSING_SPEED and robust_lead_decel <= 0.05
      urgent_safe = (not has_lead or moving_away or matched) and (not path.urgent_severe or action_accel >= URGENT_RELEASE_ACCEL)
      path.urgent_safe_frames = path.urgent_safe_frames + 1 if urgent_safe else 0
      if path.urgent_safe_frames < RELIEF_CONFIRM_FRAMES:
        path.bound = None
        path.state = AccelControllerState.hold
        self._clear_bound_relief(path)
        return True
      path.urgent = False
      path.urgent_severe = False
      path.urgent_safe_frames = 0
      if not has_lead or moving_away:
        path.state = AccelControllerState.free
        path.bound = min(action_accel, 0.0)
      else:
        path.state = AccelControllerState.hold
        path.bound = 0.0

    if path.state == AccelControllerState.inactive and has_lead and not math.isfinite(filtered_cap):
      path.bound = min(action_accel, 0.0)
      self._clear_bound_relief(path)
      return False

    dropout_guard = (not has_lead and math.isfinite(filtered_cap)
                     and path.state in (AccelControllerState.restrict, AccelControllerState.hold) and path.bound is not None)
    if dropout_guard:
      path.bound = min(path.bound, action_accel + DROPOUT_ACTION_ACCEL_MARGIN)

    profile_config = PROFILE_CONFIGS[profile]
    lead_demand = (envelope.closing_speed > APPROACH_CLOSING_SPEED
                   or (robust_lead_decel > APPROACH_LEAD_DECEL
                       and envelope.selected_lead_speed < v_ego + APPROACH_LEAD_SPEED_MARGIN))
    braking_zone = filtered_cap < v_ego + BRAKE_CAP_MARGIN
    anticipation = filtered_cap < base_speed - profile_config.anticipation_margin
    approach = (has_lead and (v_ego > APPROACH_MIN_SPEED or path.state == AccelControllerState.restrict)
                and lead_demand and (braking_zone or anticipation))
    retaining_lead = path.state in (AccelControllerState.restrict, AccelControllerState.hold) and has_lead and not moving_away
    if approach or retaining_lead:
      entering = path.state not in (AccelControllerState.restrict, AccelControllerState.hold)
      if path.bound is None or entering:
        path.bound = action_accel
      matched = envelope.closing_speed <= APPROACH_CLOSING_SPEED and robust_lead_decel <= 0.05
      coast_cap = envelope.selected_lead_speed + math.sqrt(2.0 * profile_config.comfort_decel * COAST_MATCH_USABLE_GAP)
      coast_to_match = (robust_lead_decel <= 0.05 and envelope.closing_speed <= COAST_MATCH_CLOSING_SPEED
                        and filtered_cap > coast_cap)
      if matched or coast_to_match:
        target_decel = 0.0
      elif braking_zone:
        target_decel = min(max(robust_required + REQUIRED_DECEL_MARGIN, robust_lead_decel), ROUTINE_DECEL_MAX)
      else:
        target_decel = profile_config.glide_decel
      target = -target_decel
      bound_relief = has_lead and path.bound < 0.0 and target > path.bound + 1e-9
      if bound_relief and path.bound_relief_frames == 0:
        path.bound_relief_required_frames = (self.shallow_brake_relief_frames
                                             if path.bound >= SHALLOW_BRAKE_BOUND else RELIEF_CONFIRM_FRAMES)
      path.bound_relief_frames = path.bound_relief_frames + 1 if bound_relief else 0
      if not bound_relief:
        self._clear_bound_relief(path)
      if bound_relief and path.bound_relief_frames < path.bound_relief_required_frames:
        target = path.bound
      path.bound = self._move(path.bound, target, CAP_RELAX_JERK if target > path.bound else CAP_TIGHTEN_JERK, self.dt)
      path.state = AccelControllerState.hold if matched or coast_to_match else AccelControllerState.restrict
      path.relief_frames = 0
      return False

    if path.state in (AccelControllerState.restrict, AccelControllerState.hold):
      self._clear_bound_relief(path)
      relief = not has_lead or moving_away
      path.relief_frames = path.relief_frames + 1 if relief else 0
      path.bound = min(path.bound if path.bound is not None else action_accel, 0.0)
      if path.relief_frames < RELIEF_CONFIRM_FRAMES:
        path.state = AccelControllerState.hold
        return False
      path.state = AccelControllerState.free
      path.relief_frames = 0
      return False

    if path.bound is None:
      path.bound = positive_accel_max
    else:
      path.bound = self._move(path.bound, positive_accel_max, PROFILE_TRANSITION_JERK, self.dt)
    self._clear_bound_relief(path)
    path.state = AccelControllerState.free
    return False

  @staticmethod
  def _build_accel_ceiling(bound: float, v_ego: float, planner_accel: float, action_time: float) -> tuple[float, ...] | None:
    if bound >= ACCEL_MAX - 1e-9:
      return None
    a0 = float(np.clip(planner_accel, ACCEL_MIN, ACCEL_MAX))
    if bound > 0.0:
      ceiling = np.full(len(T_IDXS), min(bound + POSITIVE_MPC_HEADROOM, ACCEL_MAX))
    elif bound == 0.0:
      ceiling = np.maximum(0.0, a0 - HORIZON_DOWN_JERK * T_IDXS)
    else:
      descent = np.maximum(bound, a0 - HORIZON_DOWN_JERK * T_IDXS)
      reach_time = max((a0 - bound) / HORIZON_DOWN_JERK, 0.0)
      release_time = max(action_time + HORIZON_HOLD_TIME, reach_time + HORIZON_HOLD_TIME)
      recovery = np.clip(bound + HORIZON_UP_JERK * np.maximum(T_IDXS - release_time, 0.0), bound, 0.0)
      ceiling = np.where(T_IDXS <= release_time, descent, np.maximum(descent, recovery))
      budget = HORIZON_SPEED_BUDGET * max(v_ego, 0.0)
      negative_area = float(np.trapezoid(-np.minimum(ceiling, 0.0), T_IDXS))
      if negative_area > budget and negative_area > 1e-9:
        ceiling = np.where(ceiling < 0.0, ceiling * budget / negative_area, ceiling)
    ceiling = np.clip(ceiling, ACCEL_MIN, ACCEL_MAX)
    ceiling[0] = max(ceiling[0], a0)
    return tuple(float(value) for value in ceiling)

  @staticmethod
  def _valid_context(base_speed: float, v_ego: float, a_ego: float, planner_accel: float, action_accel: float,
                     positive_accel_max: float, delay: float, engaged: bool, cruise_initialized: bool, controller_fault: bool) -> bool:
    values = (base_speed, v_ego, a_ego, planner_accel, action_accel, positive_accel_max, delay)
    return (engaged and cruise_initialized and not controller_fault and base_speed >= 0.0 and v_ego >= -VEGO_NOISE_TOLERANCE
            and delay >= 0.0 and all(math.isfinite(value) for value in values))

  def _update_freshness(self, path: _ControllerPath, radar_fresh: bool) -> bool:
    if radar_fresh:
      path.stale_frames = 0
      return True
    path.stale_frames += 1
    if path.stale_frames < self.radar_stale_frames and (path.bound is not None or path.urgent):
      return False
    path.reset()
    return False

  def reset(self) -> None:
    self.live.reset()
    self.shadow.reset()

  def update(self, radar_state, *, base_speed: float, v_ego: float, a_ego: float, profile: int | AccelProfile, follow_personality,
             enabled: bool, acc_selected: bool, engaged: bool, cruise_initialized: bool, planner_accel: float, action_accel: float,
             stock_accel_max: float, previous_should_stop: bool, controller_fault: bool = False,
             radar_fresh: bool = True) -> AccelControllerResult:
    selected_profile = self._profile(profile)
    sanitized_v_ego = max(v_ego, 0.0) if math.isfinite(v_ego) and v_ego >= -VEGO_NOISE_TOLERANCE else v_ego
    profile_accel_max = self.get_profile_accel_max(selected_profile, sanitized_v_ego)
    try:
      stock_accel_max = float(stock_accel_max)
    except (OverflowError, TypeError, ValueError):
      stock_accel_max = math.nan
    positive_accel_max = (max(0.0, min(profile_accel_max, stock_accel_max, ACCEL_MAX))
                          if math.isfinite(profile_accel_max) and math.isfinite(stock_accel_max) else math.nan)
    valid_context = self._valid_context(base_speed, sanitized_v_ego, a_ego, planner_accel, action_accel, positive_accel_max,
                                        self._delay(), engaged, cruise_initialized, controller_fault)
    envelope = (self.calculate_energy_envelope(radar_state, sanitized_v_ego, a_ego, selected_profile, follow_personality)
                if valid_context and radar_fresh else EnergyEnvelope(lead_status=self._radar_has_lead(radar_state)))

    shadow_fresh = self._update_freshness(self.shadow, radar_fresh) if valid_context else False
    if valid_context and radar_fresh:
      self._update_path(self.shadow, envelope, base_speed, sanitized_v_ego, action_accel, positive_accel_max,
                        selected_profile, previous_should_stop)
      shadow_active = True
    elif valid_context and not shadow_fresh and (self.shadow.bound is not None or self.shadow.urgent):
      shadow_active = True
    else:
      self.shadow.reset()
      shadow_active = False

    live_context = valid_context and bool(enabled) and bool(acc_selected)
    live_fresh = self._update_freshness(self.live, radar_fresh) if live_context else False
    if live_context and radar_fresh:
      stock_mode = self._update_path(self.live, envelope, base_speed, sanitized_v_ego, action_accel,
                                     positive_accel_max, selected_profile, previous_should_stop)
      live_active = True
    elif live_context and not live_fresh and (self.live.bound is not None or self.live.urgent):
      stock_mode = self.live.urgent
      live_active = True
    else:
      self.live.reset()
      stock_mode = False
      live_active = False

    if live_active and not stock_mode and self.live.bound is not None:
      effective_accel_max = float(np.clip(self.live.bound, ACCEL_MIN, ACCEL_MAX))
      if self.live.bound_relief_frames and self.live.lead_speed_relief:
        effective_accel_max = min(effective_accel_max, action_accel + RELIEF_MPC_JERK * self.dt)
      mpc_accel_max = self._build_accel_ceiling(effective_accel_max, sanitized_v_ego, planner_accel, self._delay())
    else:
      effective_accel_max = math.inf
      mpc_accel_max = None

    return AccelControllerResult(
      target_speed=0.0 if live_active and self.live.state == AccelControllerState.stopHold else base_speed,
      enabled=bool(enabled), active=live_active, shadow_active=shadow_active,
      launching=live_active and self.live.departing_from_stop, stock_mode=stock_mode, profile=selected_profile,
      profile_accel_max=profile_accel_max if live_active else math.inf,
      positive_accel_max=positive_accel_max if live_active else math.inf, effective_accel_max=effective_accel_max,
      mpc_accel_max=mpc_accel_max,
      state=self.live.state, shadow_state=self.shadow.state, base_speed=base_speed, raw_energy_cap=envelope.cap,
      live_filtered_cap=self.live.filtered_cap if live_active else math.inf,
      shadow_filtered_cap=self.shadow.filtered_cap if shadow_active else math.inf, selected_lead=envelope.selected_lead,
      selected_lead_speed=envelope.selected_lead_speed, usable_gap=envelope.usable_gap,
      closing_speed=envelope.closing_speed, required_decel=envelope.required_decel,
    )

  @staticmethod
  def _radar_has_lead(radar_state) -> bool:
    try:
      return bool(radar_state.leadOne.status or radar_state.leadTwo.status)
    except (AttributeError, TypeError, ValueError):
      return True
