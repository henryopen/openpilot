#!/usr/bin/env python3
from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum
import math

import numpy as np

from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
  LongitudinalMpc, LongitudinalPlanSource, STOP_DISTANCE, T_IDXS, get_T_FOLLOW, get_stopped_equivalence_factor,
)
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.constants import (
  ACCEL_LIMIT_HORIZON_JERK, ACCEL_LIMIT_TRANSITION_JERK, ACCEL_PROFILE_MAX_BP, ACCEL_PROFILE_MAX_V, BRAKING_ACCEL_LIMIT_THRESHOLD,
  CAP_FILTER_FRAMES, LAUNCH_END_SPEED, LAUNCH_TARGET_HEADROOM, LAUNCH_TARGET_SLEW, LEAD_LOSS_HOLD_TIME, LEAD_MATCH_ACCEL_GAIN,
  LEAD_MATCH_ACCEL_SLEW, LEAD_MATCH_GAP_GAIN, LEAD_MATCH_SPEED_HEADROOM, MATCHED_PACE_DECEL_RATE, MAX_LEAD_ACCEL_TAU, MIN_LEAD_SPEED,
  PACE_RELIEF_DEADBAND, PACE_RESTRICT_DEADBAND, PROFILE_CONFIGS, RADAR_STALE_TIMEOUT, STOP_GAP_RESERVE, STOP_GAP_RESERVE_DECEL_BP,
  STOP_GAP_RESERVE_LEAD_SPEED,
  STOP_HOLD_CREEP_DISTANCE, STOP_HOLD_CREEP_SPEED, STOP_HOLD_EGO_SPEED, STOP_HOLD_EXIT_FRAMES, STOP_HOLD_EXIT_SPEED,
  STOP_HOLD_ACCEL_MAX, STOP_HOLD_DEPARTURE_ACCEL_MAX, STOP_HOLD_MAX_LEAD_DISTANCE,
  STOPPED_LEAD_SPEED, VEGO_NOISE_TOLERANCE, AccelProfile,
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
  selected_lead_track_id: int = -1
  selected_lead_speed: float = math.inf
  selected_lead_accel: float = 0.0
  departure_lead_index: int = -1
  departure_lead_speed: float = math.inf
  departure_cap: float = math.inf
  departure_lead_separations: tuple[float, float] = (-math.inf, -math.inf)
  usable_gap: float = math.inf
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
  lead_speed_samples: deque[float] = field(default_factory=lambda: deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES))
  lead_accel_samples: deque[float] = field(default_factory=lambda: deque([0.0] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES))
  departure_samples: tuple[deque[float], deque[float]] = field(
    default_factory=lambda: (deque(maxlen=CAP_FILTER_FRAMES), deque(maxlen=CAP_FILTER_FRAMES)),
  )
  departure_references: list[float | None] = field(default_factory=lambda: [None, None])
  pace: float | None = None
  state: AccelControllerState = AccelControllerState.inactive
  departure_frames: int = 0
  active_frames: int = 0
  lead_loss_frames: int = 0
  lead_switch_guard_frames: int = 0
  selected_lead: int = -1
  selected_lead_track_id: int = -1
  stale_frames: int = 0
  launching: bool = False
  departure_launch: bool = False
  matched_lead: bool = False
  braking_limited: bool = False
  accel_limit: float | None = None
  matched_accel_limit: float | None = None

  @property
  def filtered_cap(self) -> float:
    return sorted(self.cap_samples)[CAP_FILTER_FRAMES // 2]

  @property
  def filtered_lead_speed(self) -> float:
    return sorted(self.lead_speed_samples)[CAP_FILTER_FRAMES // 2]

  @property
  def filtered_lead_accel(self) -> float:
    return sorted(self.lead_accel_samples)[CAP_FILTER_FRAMES // 2]

  def robust_departure_separation(self, lead_index: int) -> float:
    samples = self.departure_samples[lead_index]
    return float(np.median(samples)) if samples else -math.inf

  def reset(self) -> None:
    self.cap_samples = deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES)
    self.lead_speed_samples = deque([math.inf] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES)
    self.lead_accel_samples = deque([0.0] * CAP_FILTER_FRAMES, maxlen=CAP_FILTER_FRAMES)
    self.departure_samples = (deque(maxlen=CAP_FILTER_FRAMES), deque(maxlen=CAP_FILTER_FRAMES))
    self.departure_references = [None, None]
    self.pace = None
    self.state = AccelControllerState.inactive
    self.departure_frames = 0
    self.active_frames = 0
    self.lead_loss_frames = 0
    self.lead_switch_guard_frames = 0
    self.selected_lead = -1
    self.selected_lead_track_id = -1
    self.stale_frames = 0
    self.launching = False
    self.departure_launch = False
    self.matched_lead = False
    self.braking_limited = False
    self.accel_limit = None
    self.matched_accel_limit = None


class AccelController:
  def __init__(self, CP, dt: float = DT_MDL):
    if not math.isfinite(dt) or dt <= 0.0:
      raise ValueError("dt must be finite and positive")

    self.CP = CP
    self.dt = dt
    self.lead_loss_hold_frames = max(CAP_FILTER_FRAMES, math.ceil(LEAD_LOSS_HOLD_TIME / dt))
    self.radar_stale_frames = max(1, math.ceil(RADAR_STALE_TIMEOUT / dt))
    self.live = _ControllerPath()
    self.shadow = _ControllerPath()
    self._held_envelope: EnergyEnvelope | None = None

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
      d_rel, v_lead = float(lead.dRel), float(lead.vLeadK)
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

  @staticmethod
  def _lead_track_id(lead) -> int:
    try:
      return max(int(lead.radarTrackId), -1)
    except (AttributeError, OverflowError, TypeError, ValueError):
      return -1

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
        safety_gap = max(x_lead - x_ego - STOP_DISTANCE - t_follow * v_lead_delay, 0.0)
        closing_speed = max(v_ego_delay - v_lead_delay, 0.0)
        required_decel = 0.0 if closing_speed == 0.0 else math.inf if safety_gap == 0.0 else closing_speed**2 / (2.0 * safety_gap)
        reserve = float(np.interp(v_lead_delay, (0.0, STOP_GAP_RESERVE_LEAD_SPEED), (STOP_GAP_RESERVE, 0.0)))
        reserve_scale = float(np.interp(required_decel, STOP_GAP_RESERVE_DECEL_BP, (1.0, 0.0)))
        usable_gap = max(safety_gap - reserve * reserve_scale, 0.0)
        cap = v_lead_delay + math.sqrt(2.0 * comfort_decel * usable_gap)
        departure_cap = v_lead_delay + math.sqrt(2.0 * comfort_decel * safety_gap)
        separation = x_lead - x_ego
        departure_distance = x_lead + float(get_stopped_equivalence_factor(v_lead_delay))
      except (FloatingPointError, OverflowError, TypeError, ValueError):
        continue

      finite_values = (x_lead, v_lead_delay, safety_gap, usable_gap, closing_speed, cap, departure_cap, departure_distance)
      if not all(math.isfinite(value) and value >= 0.0 for value in finite_values) or math.isnan(required_decel) or required_decel < 0.0:
        continue
      if not math.isfinite(separation):
        continue

      candidates.append(EnergyEnvelope(
        cap=cap, selected_lead=lead_index, selected_lead_track_id=self._lead_track_id(lead),
        selected_lead_speed=v_lead_delay, selected_lead_accel=a_lead,
        usable_gap=usable_gap, closing_speed=closing_speed, required_decel=required_decel, lead_status=lead_status,
      ))
      departure_candidates.append((departure_distance, lead_index))
      departure_speeds[lead_index] = v_lead_delay
      departure_separations[lead_index] = separation
      departure_caps[lead_index] = departure_cap

    if not candidates:
      return EnergyEnvelope(lead_status=lead_status)

    selected = min(candidates, key=lambda candidate: candidate.cap)
    departure_lead_index = min(departure_candidates, key=lambda candidate: candidate[0])[1]
    departure_lead_speed = departure_speeds[departure_lead_index]
    return EnergyEnvelope(
      cap=selected.cap, selected_lead=selected.selected_lead, selected_lead_track_id=selected.selected_lead_track_id,
      selected_lead_speed=selected.selected_lead_speed,
      selected_lead_accel=selected.selected_lead_accel,
      departure_lead_index=departure_lead_index, departure_lead_speed=departure_lead_speed,
      departure_cap=departure_caps[departure_lead_index], departure_lead_separations=tuple(departure_separations),
      usable_gap=selected.usable_gap, closing_speed=selected.closing_speed, required_decel=selected.required_decel,
      has_nearly_stopped_lead=departure_lead_speed < STOPPED_LEAD_SPEED, lead_status=lead_status,
    )

  @staticmethod
  def _move(value: float, target: float, rate: float, dt: float) -> float:
    return float(np.clip(target, value - rate * dt, value + rate * dt))

  @staticmethod
  def _lead_source(source) -> bool:
    return source in (LongitudinalPlanSource.lead0, LongitudinalPlanSource.lead1)

  @staticmethod
  def _update_samples(path: _ControllerPath, envelope: EnergyEnvelope) -> bool:
    had_filtered_lead = math.isfinite(path.filtered_cap)
    has_lead = envelope.selected_lead >= 0
    path.cap_samples.append(envelope.cap if has_lead else math.inf)
    path.lead_speed_samples.append(envelope.selected_lead_speed if has_lead else math.inf)
    path.lead_accel_samples.append(envelope.selected_lead_accel if has_lead else 0.0)
    path.lead_loss_frames = 0 if has_lead else path.lead_loss_frames + 1
    for lead_index, separation in enumerate(envelope.departure_lead_separations):
      if math.isfinite(separation):
        path.departure_samples[lead_index].append(separation)
    return not had_filtered_lead and math.isfinite(path.filtered_cap)

  @staticmethod
  def _seed_departure_tracking(path: _ControllerPath, envelope: EnergyEnvelope) -> None:
    path.departure_samples = (deque(maxlen=CAP_FILTER_FRAMES), deque(maxlen=CAP_FILTER_FRAMES))
    path.departure_references = [None, None]
    for lead_index, separation in enumerate(envelope.departure_lead_separations):
      if math.isfinite(separation):
        path.departure_samples[lead_index].append(separation)
        path.departure_references[lead_index] = separation
    path.departure_frames = 0

  @staticmethod
  def _creep_departure(path: _ControllerPath, envelope: EnergyEnvelope) -> bool:
    lead_index = envelope.departure_lead_index
    if lead_index < 0 or envelope.departure_lead_speed <= STOP_HOLD_CREEP_SPEED:
      return False
    reference = path.departure_references[lead_index]
    separation = path.robust_departure_separation(lead_index)
    return reference is not None and separation - reference >= STOP_HOLD_CREEP_DISTANCE

  def _enter_stop_hold(self, path: _ControllerPath, envelope: EnergyEnvelope) -> None:
    if path.state != AccelControllerState.stopHold:
      self._seed_departure_tracking(path, envelope)
    path.pace = 0.0
    path.state = AccelControllerState.stopHold
    path.departure_frames = 0
    path.launching = False
    path.departure_launch = False
    path.matched_lead = False
    path.matched_accel_limit = None

  def _update_path(self, path: _ControllerPath, envelope: EnergyEnvelope, base_speed: float, v_ego: float,
                   profile: AccelProfile, profile_accel_max: float, previous_should_stop: bool,
                   previous_mpc_source, planner_speed: float, planner_accel: float) -> float:
    confirmed_lead = self._update_samples(path, envelope)
    path.active_frames += 1
    has_lead = envelope.selected_lead >= 0
    filtered_cap = path.filtered_cap
    slot_changed = has_lead and path.selected_lead >= 0 and envelope.selected_lead != path.selected_lead
    track_changed = (has_lead and path.selected_lead >= 0 and envelope.selected_lead == path.selected_lead
                     and envelope.selected_lead_track_id != path.selected_lead_track_id
                     and (path.selected_lead_track_id >= 0 or envelope.selected_lead_track_id >= 0))
    false_relief = (has_lead and math.isfinite(filtered_cap)
                    and envelope.cap >= filtered_cap + PACE_RELIEF_DEADBAND)
    if (slot_changed or track_changed) and false_relief and path.lead_switch_guard_frames == 0 and planner_accel <= BRAKING_ACCEL_LIMIT_THRESHOLD:
      path.lead_switch_guard_frames = self.lead_loss_hold_frames
    elif path.lead_switch_guard_frames > 0:
      path.lead_switch_guard_frames -= 1
    if has_lead:
      path.selected_lead = envelope.selected_lead
      path.selected_lead_track_id = envelope.selected_lead_track_id
    elif path.lead_loss_frames >= self.lead_loss_hold_frames:
      path.lead_switch_guard_frames = 0
      path.selected_lead = -1
      path.selected_lead_track_id = -1
    departure_separation = (envelope.departure_lead_separations[envelope.departure_lead_index]
                            if envelope.departure_lead_index >= 0 else math.inf)
    stopped_lead_hold = (has_lead and envelope.has_nearly_stopped_lead
                         and (envelope.departure_cap < 0.50
                              or (path.braking_limited and departure_separation <= STOP_HOLD_MAX_LEAD_DISTANCE)))
    invalid_lead = envelope.lead_status and not has_lead
    prior_lead_context = self._lead_source(previous_mpc_source) or math.isfinite(filtered_cap) or path.braking_limited
    previous_stop = (previous_should_stop and prior_lead_context
                     and (not has_lead or envelope.departure_lead_speed < STOP_HOLD_EXIT_SPEED))
    stop_evidence = (stopped_lead_hold or envelope.cap < 0.50 or filtered_cap < 0.50
                     or (previous_stop and not path.launching) or invalid_lead)
    confirmed_creep_departure = path.launching and path.departure_launch and self._creep_departure(path, envelope)

    if path.accel_limit is None:
      path.accel_limit = profile_accel_max
    else:
      path.accel_limit = self._move(path.accel_limit, profile_accel_max, ACCEL_LIMIT_TRANSITION_JERK, self.dt)
    if (path.active_frames >= self.lead_loss_hold_frames and math.isfinite(filtered_cap)
        and has_lead and planner_accel <= BRAKING_ACCEL_LIMIT_THRESHOLD):
      path.braking_limited = True
    elif not has_lead and path.lead_loss_frames >= self.lead_loss_hold_frames:
      path.braking_limited = False

    if path.pace is None:
      path.pace = min(base_speed, v_ego) if has_lead else base_speed
      path.state = AccelControllerState.free
      if v_ego < STOP_HOLD_EGO_SPEED and not stop_evidence:
        path.pace = min(base_speed, v_ego + LAUNCH_TARGET_HEADROOM)
        path.state = AccelControllerState.release
        path.launching = True
        path.departure_launch = False

    path.pace = min(path.pace, base_speed)
    if (v_ego < STOP_HOLD_EGO_SPEED and stop_evidence and not confirmed_creep_departure
        and path.state != AccelControllerState.stopHold):
      self._enter_stop_hold(path, envelope)
      return path.pace

    if path.state == AccelControllerState.stopHold:
      for lead_index in range(len(path.departure_references)):
        separation = path.robust_departure_separation(lead_index)
        if math.isfinite(separation) and path.departure_references[lead_index] is None:
          path.departure_references[lead_index] = separation
      raw_departure = ((has_lead and envelope.departure_lead_speed > STOP_HOLD_EXIT_SPEED
                        and envelope.departure_cap > STOP_HOLD_EXIT_SPEED)
                       or (not envelope.lead_status and path.lead_loss_frames >= self.lead_loss_hold_frames))
      departed = self._creep_departure(path, envelope) or raw_departure
      path.departure_frames = path.departure_frames + 1 if departed else 0
      path.pace = 0.0
      if path.departure_frames < STOP_HOLD_EXIT_FRAMES:
        return path.pace
      path.pace = base_speed
      path.state = AccelControllerState.release
      path.departure_frames = 0
      path.launching = True
      path.departure_launch = has_lead
      return path.pace

    if path.launching:
      invalid_lead = envelope.lead_status and not has_lead
      renewed_stop = (has_lead and not confirmed_creep_departure
                      and (envelope.cap < STOP_HOLD_EXIT_SPEED
                           or (envelope.has_nearly_stopped_lead and envelope.departure_cap < STOP_HOLD_EXIT_SPEED)))
      guarded_departure_loss = path.departure_launch and not envelope.lead_status and path.lead_loss_frames < self.lead_loss_hold_frames
      if invalid_lead:
        path.launching = False
        path.departure_launch = False
        if v_ego < STOP_HOLD_EGO_SPEED:
          self._enter_stop_hold(path, envelope)
          return path.pace
        path.state = AccelControllerState.hold
        return path.pace
      if guarded_departure_loss:
        path.state = AccelControllerState.hold
        return path.pace
      if path.departure_launch and not has_lead:
        path.departure_launch = False
      if renewed_stop:
        path.launching = False
        path.departure_launch = False
        if v_ego < STOP_HOLD_EGO_SPEED:
          self._enter_stop_hold(path, envelope)
          return path.pace
      if path.departure_launch:
        path.pace = base_speed
      else:
        launch_target = min(base_speed, v_ego + LAUNCH_TARGET_HEADROOM)
        path.pace = min(base_speed, max(path.pace, launch_target) + LAUNCH_TARGET_SLEW * self.dt)
      if v_ego >= LAUNCH_END_SPEED:
        path.launching = False
        path.departure_launch = False

    comfort_decel = PROFILE_CONFIGS[profile].comfort_decel
    if (has_lead and not path.launching and path.state == AccelControllerState.restrict and envelope.closing_speed <= 0.0
        and v_ego >= path.filtered_lead_speed - VEGO_NOISE_TOLERANCE):
      path.matched_lead = True
    elif not has_lead and path.lead_loss_frames >= self.lead_loss_hold_frames:
      path.matched_lead = False

    if path.matched_lead:
      if not has_lead:
        if self._lead_source(previous_mpc_source) and planner_speed < path.pace:
          path.pace = max(planner_speed, path.pace - MATCHED_PACE_DECEL_RATE * self.dt)
        path.state = AccelControllerState.hold
        return path.pace
      if math.isfinite(path.filtered_lead_speed):
        recovery_speed = min(base_speed, path.filtered_lead_speed + min(LEAD_MATCH_SPEED_HEADROOM, LEAD_MATCH_GAP_GAIN * envelope.usable_gap))
        recovery_error = max(recovery_speed - v_ego, 0.0)
        desired_accel_limit = min(path.accel_limit, LEAD_MATCH_ACCEL_GAIN[profile] * recovery_error)
      else:
        desired_accel_limit = 0.0
      if path.filtered_lead_accel < BRAKING_ACCEL_LIMIT_THRESHOLD:
        desired_accel_limit = path.accel_limit
      if path.matched_accel_limit is None:
        path.matched_accel_limit = path.accel_limit
      if path.lead_switch_guard_frames > 0:
        desired_accel_limit = min(desired_accel_limit, path.matched_accel_limit)
      path.matched_accel_limit = min(path.accel_limit, self._move(path.matched_accel_limit, desired_accel_limit, LEAD_MATCH_ACCEL_SLEW, self.dt))
      matched_ceiling = min(base_speed, filtered_cap)
      if matched_ceiling <= path.pace - PACE_RESTRICT_DEADBAND:
        path.pace = max(matched_ceiling, path.pace - MATCHED_PACE_DECEL_RATE * self.dt)
        path.state = AccelControllerState.restrict
      elif path.lead_switch_guard_frames == 0 and matched_ceiling >= path.pace + PACE_RELIEF_DEADBAND:
        path.pace = min(matched_ceiling, path.pace + path.accel_limit * self.dt)
        path.state = AccelControllerState.free if path.pace >= base_speed - PACE_RESTRICT_DEADBAND else AccelControllerState.release
      else:
        path.state = AccelControllerState.free if path.pace >= base_speed - PACE_RESTRICT_DEADBAND else AccelControllerState.hold
      return path.pace
    path.matched_accel_limit = None

    ceiling = min(base_speed, filtered_cap)
    if (confirmed_lead and path.active_frames == CAP_FILTER_FRAMES // 2 + 1 and not path.launching
        and planner_speed < path.pace):
      path.pace = max(planner_speed, path.pace - comfort_decel * self.dt)

    if self._lead_source(previous_mpc_source) and not has_lead and planner_speed < path.pace:
      path.pace = max(planner_speed, path.pace - MATCHED_PACE_DECEL_RATE * self.dt)
      path.state = AccelControllerState.hold
      return path.pace

    if ceiling <= path.pace - PACE_RESTRICT_DEADBAND or (path.state == AccelControllerState.restrict and ceiling < path.pace):
      path.pace = max(ceiling, path.pace - comfort_decel * self.dt)
      path.state = AccelControllerState.restrict
      return path.pace

    filter_warmup = has_lead and not math.isfinite(filtered_cap)
    guarded_lead_loss = not has_lead and path.lead_loss_frames < self.lead_loss_hold_frames
    if (filter_warmup or guarded_lead_loss) and path.pace < base_speed - PACE_RESTRICT_DEADBAND:
      path.state = AccelControllerState.hold
      return path.pace

    confirmed_clear_road = not math.isfinite(filtered_cap) and not guarded_lead_loss
    relief = not has_lead or envelope.closing_speed <= 0.0
    if relief and (ceiling >= path.pace + PACE_RELIEF_DEADBAND or (confirmed_clear_road and ceiling > path.pace)):
      if path.lead_switch_guard_frames == 0:
        path.pace = min(ceiling, path.pace + path.accel_limit * self.dt)
      path.state = AccelControllerState.free if path.pace >= base_speed - PACE_RESTRICT_DEADBAND else AccelControllerState.release
    else:
      path.state = AccelControllerState.free if path.pace >= base_speed - PACE_RESTRICT_DEADBAND else AccelControllerState.hold
    return path.pace

  @staticmethod
  def _valid_context(base_speed: float, v_ego: float, a_ego: float, planner_speed: float, planner_accel: float, stock_accel_max: float,
                     delay: float, engaged: bool, cruise_initialized: bool) -> bool:
    values = (base_speed, v_ego, a_ego, planner_speed, planner_accel, stock_accel_max, delay)
    return (engaged and cruise_initialized and base_speed >= 0.0 and v_ego >= -VEGO_NOISE_TOLERANCE
            and planner_speed >= 0.0 and stock_accel_max >= 0.0 and delay >= 0.0 and all(math.isfinite(value) for value in values))

  def _update_freshness(self, path: _ControllerPath, radar_fresh: bool) -> bool:
    if radar_fresh:
      path.stale_frames = 0
      return True
    path.stale_frames += 1
    if path.stale_frames >= self.radar_stale_frames:
      path.reset()
    return False

  @staticmethod
  def _build_accel_ceiling(limit: float, planner_accel: float) -> tuple[float, ...] | None:
    if limit >= ACCEL_MAX - 1e-9:
      return None
    a0 = float(np.clip(planner_accel, ACCEL_MIN, ACCEL_MAX))
    ceiling = np.maximum(limit, a0 - ACCEL_LIMIT_HORIZON_JERK * T_IDXS)
    ceiling = np.clip(ceiling, 0.0, ACCEL_MAX)
    ceiling[0] = max(ceiling[0], a0)
    return tuple(float(value) for value in ceiling)

  def reset(self) -> None:
    self.live.reset()
    self.shadow.reset()
    self._held_envelope = None

  def update(self, radar_state, *, base_speed: float, v_ego: float, a_ego: float, profile: int | AccelProfile, follow_personality,
             enabled: bool, acc_selected: bool, engaged: bool, cruise_initialized: bool, stock_accel_max: float,
             previous_should_stop: bool, radar_fresh: bool = True,
             previous_mpc_source=None, planner_speed: float | None = None, planner_accel: float = 0.0) -> AccelControllerResult:
    selected_profile = self._profile(profile)
    sanitized_v_ego = max(v_ego, 0.0) if math.isfinite(v_ego) and v_ego >= -VEGO_NOISE_TOLERANCE else v_ego
    profile_accel_max = self.get_profile_accel_max(selected_profile, sanitized_v_ego)
    try:
      stock_accel_max = float(stock_accel_max)
    except (OverflowError, TypeError, ValueError):
      stock_accel_max = math.nan
    positive_accel_max = (max(0.0, min(profile_accel_max, stock_accel_max, ACCEL_MAX))
                          if math.isfinite(profile_accel_max) and math.isfinite(stock_accel_max) else math.nan)
    planner_speed = sanitized_v_ego if planner_speed is None else planner_speed
    valid_context = self._valid_context(base_speed, sanitized_v_ego, a_ego, planner_speed, planner_accel, stock_accel_max, self._delay(),
                                        engaged, cruise_initialized)
    if valid_context and radar_fresh:
      envelope = self.calculate_energy_envelope(radar_state, sanitized_v_ego, a_ego, selected_profile, follow_personality)
      self._held_envelope = envelope
    elif valid_context and self._held_envelope is not None:
      envelope = self._held_envelope
    else:
      envelope = EnergyEnvelope(lead_status=self._radar_has_lead(radar_state))
      if not valid_context:
        self._held_envelope = None

    shadow_fresh = self._update_freshness(self.shadow, radar_fresh) if valid_context else False
    if valid_context and radar_fresh:
      self._update_path(self.shadow, envelope, base_speed, sanitized_v_ego, selected_profile, profile_accel_max, previous_should_stop,
                        previous_mpc_source, planner_speed, planner_accel)
      shadow_active = True
    elif valid_context and not shadow_fresh and self.shadow.pace is not None:
      shadow_active = True
    else:
      self.shadow.reset()
      shadow_active = False

    live_context = valid_context and bool(enabled) and bool(acc_selected)
    live_fresh = self._update_freshness(self.live, radar_fresh) if live_context else False
    if live_context and radar_fresh:
      pace_target = self._update_path(self.live, envelope, base_speed, sanitized_v_ego, selected_profile, profile_accel_max,
                                      previous_should_stop,
                                      previous_mpc_source, planner_speed, planner_accel)
      live_active = True
    elif live_context and not live_fresh and self.live.pace is not None:
      pace_target = self.live.pace
      live_active = True
    else:
      self.live.reset()
      pace_target = base_speed
      live_active = False

    if not radar_fresh and not shadow_active and not live_active:
      self._held_envelope = None
      envelope = EnergyEnvelope(lead_status=self._radar_has_lead(radar_state))

    stop_hold_active = live_active and self.live.state == AccelControllerState.stopHold
    valid_lead_stop_hold = stop_hold_active and envelope.selected_lead >= 0
    matched_limit_active = live_active and self.live.matched_lead and self.live.matched_accel_limit is not None
    braking_limited = live_active and self.live.braking_limited
    confirmed_clear_road = (live_active and not stop_hold_active and not envelope.lead_status
                            and not math.isfinite(self.live.filtered_cap) and self.live.lead_loss_frames >= self.lead_loss_hold_frames)
    if stop_hold_active:
      stop_hold_accel_max = STOP_HOLD_DEPARTURE_ACCEL_MAX if self.live.departure_frames > 1 else STOP_HOLD_ACCEL_MAX
      effective_accel_max = min(stop_hold_accel_max, stock_accel_max) if valid_lead_stop_hold else 0.0
    elif matched_limit_active:
      effective_accel_max = min(self.live.matched_accel_limit, stock_accel_max)
    else:
      effective_accel_max = (min(self.live.accel_limit, stock_accel_max)
                             if (braking_limited or confirmed_clear_road) and self.live.accel_limit is not None else math.inf)
    if stop_hold_active or matched_limit_active or braking_limited or confirmed_clear_road:
      mpc_accel_max = self._build_accel_ceiling(effective_accel_max, planner_accel)
    else:
      mpc_accel_max = None

    return AccelControllerResult(
      target_speed=0.0 if stop_hold_active else pace_target,
      enabled=bool(enabled), active=live_active, shadow_active=shadow_active, launching=live_active and self.live.launching,
      profile=selected_profile, profile_accel_max=profile_accel_max if live_active else math.inf,
      positive_accel_max=positive_accel_max if live_active else math.inf, effective_accel_max=effective_accel_max,
      mpc_accel_max=mpc_accel_max, state=self.live.state,
      shadow_state=self.shadow.state, base_speed=base_speed, raw_energy_cap=envelope.cap,
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
