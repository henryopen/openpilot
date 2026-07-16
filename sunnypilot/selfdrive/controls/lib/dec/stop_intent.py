"""A compact fixed-world stop tracker for Dynamic Experimental Control."""

from dataclasses import dataclass
from enum import IntEnum
import math

from openpilot.common.realtime import DT_MDL


class StopIntentState(IntEnum):
  clear = 0
  qualifying = 1
  approach = 2
  hold = 3
  suppressed = 4


@dataclass(frozen=True)
class StopConstraint:
  active: bool = False
  distance_m: float = 0.0
  hold: bool = False
  state: StopIntentState = StopIntentState.clear
  confidence: float = 0.0
  accel_ceiling_mps2: float = math.inf


@dataclass(frozen=True)
class _Observation:
  distance_m: float
  evidence: bool


class StopIntentController:
  TRAJECTORY_SIZE = 33
  QUALIFY_TIME = 0.75
  QUALIFY_TRAVEL_MARGIN = 1.0
  APPROACH_CLEAR_TIME = 0.5
  HOLD_MIN_TIME = 1.0
  HOLD_CLEAR_TIME = 0.5

  MAX_OBSERVATION_DISTANCE = 170.0
  MAX_APPROACH_SPEED = 20.0
  MAX_REQUIRED_DECEL = 2.2
  ACTUATION_LOOKAHEAD = 0.25
  ENDPOINT_SETTLE_MARGIN = 2.5
  HOLD_DISTANCE = 3.0
  HOLD_SPEED = 0.7

  ACTION_DECEL_MIN = 0.2
  ACTION_DECEL_RATIO = 0.4
  FREE_FLOW_HORIZON = 7.0
  SHORT_TRAJECTORY_RATIO = 0.95
  TERMINAL_SPEED_MAX = 0.75
  TERMINAL_WINDOW_SIZE = 4
  TERMINAL_WINDOW_SPEED_MAX = 2.0

  WORLD_ERROR_MIN = 2.0
  WORLD_ERROR_MAX = 3.0
  WORLD_SPEED_MIN_TIME = 0.4
  WORLD_SPEED_MAX = 0.75
  ASSOCIATION_ERROR_MIN = 2.0

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self._state = StopIntentState.clear
    self._distance_m = 0.0
    self._start_distance = 0.0
    self._travel_m = 0.0
    self._elapsed = 0.0
    self._evidence_time = 0.0
    self._clear_time = 0.0
    self._hold_time = 0.0
    self._last_v_ego = 0.0

  def update(self, model, car_state, *, enabled: bool, lead_present: bool, observation_valid: bool = True, dt: float = DT_MDL) -> StopConstraint:
    dt = self._sanitize_dt(dt)
    if not enabled:
      self.reset()
      return self._constraint()

    v_ego = self._read_speed(car_state)
    self._last_v_ego = v_ego
    standstill = bool(getattr(car_state, 'standstill', False))
    observation = self._read_observation(model, v_ego) if observation_valid else None
    observation_step = min(dt, DT_MDL)

    if bool(getattr(car_state, 'gasPressed', False)):
      self.reset()
      self._state = StopIntentState.suppressed
      self._distance_m = observation.distance_m if observation is not None else 0.0
      return self._constraint()
    if self._state == StopIntentState.suppressed:
      self.reset()

    if self._state == StopIntentState.clear:
      if lead_present or observation is None or not observation.evidence:
        return self._constraint()
      reserve = self._qualification_reserve(observation.distance_m, v_ego)
      if not self._feasible(observation.distance_m, v_ego, reserve):
        return self._constraint()
      self._start_candidate(observation.distance_m, observation_step)
      return self._constraint()

    step_distance = v_ego * dt
    if self._state == StopIntentState.qualifying:
      self._distance_m -= step_distance
      self._travel_m += step_distance
      self._elapsed += dt
      reserve = self._qualification_reserve(
        self._start_distance,
        v_ego,
        evidence_time=self._evidence_time,
        travel_m=self._travel_m,
      )
      if lead_present or observation is None or not observation.evidence or not self._feasible(self._distance_m, v_ego, reserve):
        self.reset()
        return self._constraint()

      expected_distance = self._start_distance - self._travel_m
      world_displacement = observation.distance_m - expected_distance
      moving_target = self._elapsed >= self.WORLD_SPEED_MIN_TIME and abs(world_displacement) / self._elapsed > self.WORLD_SPEED_MAX
      if abs(world_displacement) > self._world_tolerance(self._start_distance) or moving_target:
        self._start_candidate(observation.distance_m, observation_step)
        return self._constraint()

      self._fuse(observation.distance_m)
      self._evidence_time = min(self.QUALIFY_TIME, self._evidence_time + observation_step)
      self._commit_if_ready(v_ego, standstill)
      return self._constraint()

    self._distance_m -= step_distance
    if self._state == StopIntentState.hold:
      self._hold_time += dt
      self._clear_time = self._clear_time + observation_step if observation is not None and not observation.evidence else 0.0
      if self._hold_time >= self.HOLD_MIN_TIME - 1e-6 and self._clear_time >= self.HOLD_CLEAR_TIME - 1e-6:
        self.reset()
      return self._constraint()

    if observation is None:
      self._clear_time = 0.0
    elif observation.evidence:
      self._clear_time = 0.0
      if observation.distance_m < self._distance_m:
        self._distance_m = observation.distance_m
      elif observation.distance_m - self._distance_m <= self.ASSOCIATION_ERROR_MIN:
        self._fuse(observation.distance_m)
    else:
      self._clear_time += observation_step
      if self._clear_time >= self.APPROACH_CLEAR_TIME - 1e-6:
        self.reset()
        return self._constraint()

    if standstill or (self._distance_m <= self.HOLD_DISTANCE and v_ego <= self.HOLD_SPEED):
      self._state = StopIntentState.hold
      self._hold_time = 0.0
      self._clear_time = 0.0
    return self._constraint()

  def _start_candidate(self, distance_m: float, evidence_step: float) -> None:
    self._state = StopIntentState.qualifying
    self._distance_m = distance_m
    self._start_distance = distance_m
    self._travel_m = 0.0
    self._elapsed = 0.0
    self._evidence_time = evidence_step

  def _commit_if_ready(self, v_ego: float, standstill: bool) -> None:
    if self._evidence_time < self.QUALIFY_TIME:
      return
    if self._distance_m <= self.HOLD_DISTANCE and (standstill or v_ego <= self.HOLD_SPEED):
      self._state = StopIntentState.hold
      self._hold_time = 0.0
      self._clear_time = 0.0
    elif self._travel_m >= self._world_tolerance(self._start_distance) + self.QUALIFY_TRAVEL_MARGIN:
      self._state = StopIntentState.approach

  def _qualification_reserve(self, start_distance: float, v_ego: float, *, evidence_time: float = 0.0, travel_m: float = 0.0) -> float:
    time_reserve = v_ego * max(0.0, self.QUALIFY_TIME - evidence_time)
    travel_reserve = max(0.0, self._world_tolerance(start_distance) + self.QUALIFY_TRAVEL_MARGIN - travel_m)
    return max(time_reserve, travel_reserve)

  def _feasible(self, distance_m: float, v_ego: float, reserve_m: float = 0.0) -> bool:
    if v_ego > self.MAX_APPROACH_SPEED or not 0.1 <= distance_m <= self.MAX_OBSERVATION_DISTANCE:
      return False
    if distance_m <= self.HOLD_DISTANCE and v_ego <= self.HOLD_SPEED:
      return True
    usable_distance = self._control_distance(distance_m, v_ego) - max(0.0, reserve_m)
    return usable_distance > 0.0 and v_ego * v_ego / (2.0 * usable_distance) <= self.MAX_REQUIRED_DECEL

  def _read_observation(self, model, v_ego: float) -> _Observation | None:
    position = getattr(model, 'position', None)
    xs, ys = getattr(position, 'x', ()), getattr(position, 'y', ())
    velocity = getattr(model, 'velocity', None)
    vxs, vys = getattr(velocity, 'x', ()), getattr(velocity, 'y', ())
    if not all(len(values) == self.TRAJECTORY_SIZE for values in (xs, ys, vxs, vys)):
      return None

    try:
      points = [(float(x), float(y)) for x, y in zip(xs, ys, strict=True)]
      terminal_speeds = [math.hypot(float(vx), float(vy)) for vx, vy in zip(vxs[-self.TERMINAL_WINDOW_SIZE :], vys[-self.TERMINAL_WINDOW_SIZE :], strict=True)]
    except (TypeError, ValueError):
      return None
    if not all(math.isfinite(value) for point in points for value in point) or not all(math.isfinite(speed) for speed in terminal_speeds):
      return None
    if points[-1][0] <= 0.0:
      return None

    distance_m = math.hypot(*points[0]) + sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(points, points[1:], strict=False))
    if not 0.1 <= distance_m <= self.MAX_OBSERVATION_DISTANCE:
      return None

    terminal_profile = (
      terminal_speeds[-1] <= self.TERMINAL_SPEED_MAX
      and max(terminal_speeds) <= self.TERMINAL_WINDOW_SPEED_MAX
      and all(next_speed <= speed + 0.25 for speed, next_speed in zip(terminal_speeds, terminal_speeds[1:], strict=False))
    )
    expected_distance = max(20.0, self.FREE_FLOW_HORIZON * v_ego)

    action = getattr(model, 'action', None)
    try:
      desired_accel = float(getattr(action, 'desiredAcceleration', 0.0))
    except (TypeError, ValueError):
      return None
    if not math.isfinite(desired_accel):
      return None
    control_distance = max(0.1, self._control_distance(distance_m, v_ego))
    required_decel = v_ego * v_ego / (2.0 * control_distance)
    action_threshold = max(self.ACTION_DECEL_MIN, self.ACTION_DECEL_RATIO * required_decel)
    action_evidence = bool(getattr(action, 'shouldStop', False)) or desired_accel <= -action_threshold
    evidence = action_evidence and terminal_profile and distance_m <= expected_distance * self.SHORT_TRAJECTORY_RATIO
    return _Observation(distance_m, evidence)

  def _fuse(self, observed_distance: float) -> None:
    self._distance_m += max(-1.0, min(1.0, observed_distance - self._distance_m)) * 0.25

  @classmethod
  def _sanitize_dt(cls, dt: float) -> float:
    try:
      dt = float(dt)
    except (TypeError, ValueError):
      return DT_MDL
    return max(0.01, min(0.5, dt)) if math.isfinite(dt) and dt > 0.0 else DT_MDL

  @staticmethod
  def _read_speed(car_state) -> float:
    try:
      v_ego = float(getattr(car_state, 'vEgo', 0.0))
    except (TypeError, ValueError):
      return 0.0
    return max(0.0, v_ego) if math.isfinite(v_ego) else 0.0

  def _world_tolerance(self, distance_m: float) -> float:
    return min(self.WORLD_ERROR_MAX, max(self.WORLD_ERROR_MIN, 0.02 * distance_m))

  def _control_distance(self, distance_m: float, v_ego: float) -> float:
    return distance_m - self.ENDPOINT_SETTLE_MARGIN - v_ego * self.ACTUATION_LOOKAHEAD

  def _constraint(self) -> StopConstraint:
    active = self._state in (StopIntentState.approach, StopIntentState.hold)
    accel_ceiling = math.inf
    if active:
      control_distance = max(0.1, self._control_distance(max(0.0, self._distance_m), self._last_v_ego))
      accel_ceiling = -min(self.MAX_REQUIRED_DECEL, self._last_v_ego**2 / (2.0 * control_distance))
    confidence = 1.0 if active else min(1.0, self._evidence_time / self.QUALIFY_TIME)
    return StopConstraint(active, max(0.0, self._distance_m), self._state == StopIntentState.hold, self._state, confidence, accel_ceiling)
