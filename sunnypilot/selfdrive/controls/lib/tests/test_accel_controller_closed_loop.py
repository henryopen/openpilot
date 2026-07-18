from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
import pytest

from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import PRIUS_TSS2_ROUTE_MODEL, LeadObservation, Plant
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelControllerState
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality.accel_controller import MOVING_LEAD_DECEL_ACCEL_MAX

ROUTINE_GAP_TOLERANCE = 0.02


@dataclass
class ClosedLoopTrace:
  time: np.ndarray
  speed: np.ndarray
  distance: np.ndarray
  distance_lead: np.ndarray
  a_target: np.ndarray
  acceleration: np.ndarray
  should_stop: np.ndarray
  fcw: np.ndarray
  source: list
  active: np.ndarray
  shadow_active: np.ndarray
  launching: np.ndarray
  target_speed: np.ndarray
  pace: np.ndarray
  raw_cap: np.ndarray
  filtered_cap: np.ndarray
  selected_lead: np.ndarray
  profile_accel_max: np.ndarray
  effective_accel_max: np.ndarray
  state: np.ndarray
  required_decel: np.ndarray
  controller_fault: np.ndarray
  solver_failures: int


def _set_params(*, enabled: bool, profile: int = 1, dec_enabled: bool = False) -> None:
  params = Params()
  params.put_bool("AccelPersonalityEnabled", enabled, block=True)
  params.put("AccelPersonality", profile, block=True)
  params.put_bool("DynamicExperimentalControl", dec_enabled, block=True)


def _run(
  *,
  duration: float,
  controller_enabled: bool,
  profile: int = 1,
  v_lead: float | Callable[[float], float] = 0.0,
  v_cruise: float = 30.0,
  dec_enabled: bool = False,
  **plant_kwargs,
) -> ClosedLoopTrace:
  _set_params(enabled=controller_enabled, profile=profile, dec_enabled=dec_enabled)
  plant = Plant(**plant_kwargs)
  plant.v_lead_prev = float(v_lead) if isinstance(v_lead, (int, float)) else float(v_lead(0.0))
  solver_failures = 0
  original_mpc_reset = plant.planner.mpc.reset

  def count_failed_solve() -> None:
    nonlocal solver_failures
    if plant.planner.mpc.solution_status != 0:
      solver_failures += 1
    original_mpc_reset()

  plant.planner.mpc.reset = count_failed_solve
  rows = []
  sources = []
  while plant.current_time < duration:
    lead_speed = float(v_lead) if isinstance(v_lead, (int, float)) else v_lead(plant.current_time)
    controller_fault = plant.planner.mpc.last_solution_status != 0
    result = plant.step(v_lead=lead_speed, v_cruise=v_cruise)
    controller = plant.planner.accel_controller_result
    rows.append(
      (
        plant.current_time,
        result["speed"],
        result["distance"],
        result["distance_lead"],
        result["a_target"],
        result["realized_acceleration"],
        result["should_stop"],
        result["fcw"],
        controller.active,
        controller.shadow_active,
        controller.launching,
        controller.target_speed,
        controller.live_pace,
        controller.raw_energy_cap,
        controller.live_filtered_cap,
        controller.selected_lead,
        controller.profile_accel_max,
        controller.effective_accel_max,
        controller.state,
        controller.required_decel,
        controller_fault,
      )
    )
    sources.append(result["mpc_source"])

  data = np.asarray(rows, dtype=float)
  return ClosedLoopTrace(
    time=data[:, 0],
    speed=data[:, 1],
    distance=data[:, 2],
    distance_lead=data[:, 3],
    a_target=data[:, 4],
    acceleration=data[:, 5],
    should_stop=data[:, 6].astype(bool),
    fcw=data[:, 7].astype(bool),
    source=sources,
    active=data[:, 8].astype(bool),
    shadow_active=data[:, 9].astype(bool),
    launching=data[:, 10].astype(bool),
    target_speed=data[:, 11],
    pace=data[:, 12],
    raw_cap=data[:, 13],
    filtered_cap=data[:, 14],
    selected_lead=data[:, 15].astype(int),
    profile_accel_max=data[:, 16],
    effective_accel_max=data[:, 17],
    state=data[:, 18].astype(int),
    required_decel=data[:, 19],
    controller_fault=data[:, 20].astype(bool),
    solver_failures=solver_failures,
  )


def _first_time_below(trace: ClosedLoopTrace, threshold: float) -> float:
  indices = np.flatnonzero(trace.a_target <= threshold)
  assert len(indices), f"never reached {threshold} m/s²"
  return float(trace.time[indices[0]])


def _sustained_time_below(trace: ClosedLoopTrace, threshold: float, *, after: float = 0.5, duration: float = 0.5) -> float:
  required_frames = round(duration / DT_MDL)
  below = (trace.time >= after) & (trace.a_target <= threshold)
  sustained = np.convolve(below.astype(int), np.ones(required_frames, dtype=int), mode="valid") == required_frames
  indices = np.flatnonzero(sustained)
  assert len(indices), f"never sustained {threshold} m/s² for {duration} s"
  return float(trace.time[indices[0]])


def _command_jerk(trace: ClosedLoopTrace, after: float = 0.0) -> np.ndarray:
  indices = np.flatnonzero(trace.time >= after)
  assert len(indices) >= 2
  return np.diff(trace.a_target[indices]) / DT_MDL


def _filtered_realized_jerk(trace: ClosedLoopTrace, after: float = 1.0) -> np.ndarray:
  filtered_acceleration = np.convolve(trace.acceleration, np.ones(3) / 3.0, mode="valid")
  return (np.diff(filtered_acceleration) / DT_MDL)[trace.time[2:-1] >= after]


def _has_brake_gas_brake(values: np.ndarray, threshold: float = 0.2, frames: int = 5) -> bool:
  phase = 0
  brake_seen = False
  gas_after_brake_seen = False
  for index in range(len(values) - frames + 1):
    window = values[index : index + frames]
    candidate = -1 if np.all(window <= -threshold) else 1 if np.all(window >= threshold) else 0
    if candidate == 0 or candidate == phase:
      continue
    phase = candidate
    if candidate < 0:
      if gas_after_brake_seen:
        return True
      brake_seen = True
    elif brake_seen:
      gas_after_brake_seen = True
  return False


@pytest.fixture(autouse=True)
def _restore_defaults():
  yield
  _set_params(enabled=False, profile=1, dec_enabled=False)


@pytest.mark.parametrize(
  ("plant_kwargs", "expect_shadow"),
  [
    ({"enabled": False, "lead_relevancy": True, "speed": 20.0, "distance_lead": 70.0}, False),
    ({"e2e": True, "lead_relevancy": False, "speed": 20.0}, True),
  ],
  ids=("disengaged", "e2e-shadow"),
)
def test_non_actuating_modes_match_clean_base(plant_kwargs, expect_shadow):
  common = dict(duration=2.0, v_lead=14.0, **plant_kwargs)
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)

  np.testing.assert_allclose(trace.a_target, baseline.a_target, atol=1e-6, rtol=0.0)
  np.testing.assert_array_equal(trace.should_stop, baseline.should_stop)
  np.testing.assert_array_equal(trace.fcw, baseline.fcw)
  assert trace.source == baseline.source
  assert not trace.active.any()
  np.testing.assert_array_equal(trace.shadow_active, ~trace.controller_fault if expect_shadow else np.zeros_like(trace.active))


def test_disabled_profiles_match_clean_base():
  common = dict(duration=2.0, controller_enabled=False, lead_relevancy=True, speed=20.0, distance_lead=70.0, v_lead=14.0)
  traces = [_run(profile=profile, **common) for profile in range(3)]
  for trace in traces[1:]:
    np.testing.assert_allclose(trace.a_target, traces[0].a_target, atol=1e-6, rtol=0.0)
    np.testing.assert_array_equal(trace.should_stop, traces[0].should_stop)
    assert trace.source == traces[0].source
  assert all(np.isinf(trace.effective_accel_max).all() for trace in traces)


def test_active_controller_is_pre_mpc_and_preserves_stock_lead_authority():
  _set_params(enabled=True, profile=0)
  plant = Plant(lead_relevancy=False, speed=0.0, actuator_delay=0.15, actuator_lag=0.20)
  for _ in range(10):
    result = plant.step(v_cruise=15.0)
    if plant.planner.accel_controller_result.mpc_shape_cruise:
      break
  controller = plant.planner.accel_controller_result

  assert controller.mpc_shape_cruise
  assert controller.mpc_accel_max is not None
  assert controller.mpc_apply_accel_constraint
  assert min(controller.mpc_accel_max) > 0.0
  np.testing.assert_array_equal(plant.planner.mpc.params[:, 1], controller.mpc_accel_max)
  assert ACCEL_MIN <= result["a_target"] <= get_max_accel(plant.speed)

  lead_plant = Plant(lead_relevancy=True, speed=0.0, distance_lead=6.0, actuator_delay=0.15, actuator_lag=0.20)
  lead_plant.step(v_lead=0.0, v_cruise=15.0)
  controller = lead_plant.planner.accel_controller_result
  assert controller.target_speed == 0.0
  assert min(controller.mpc_accel_max) == -0.25
  assert controller.mpc_accel_max[-1] == ACCEL_MAX
  assert controller.mpc_shape_cruise
  assert controller.mpc_apply_accel_constraint
  preconditioned = lead_plant.planner.mpc.params[(T_IDXS > 0.0) & (T_IDXS <= 0.40), 1]
  assert len(preconditioned) and np.max(preconditioned) <= -0.25


def test_clear_road_launch_is_immediate_and_profiles_separate():
  common = dict(
    duration=6.0,
    controller_enabled=True,
    lead_relevancy=False,
    speed=0.0,
    v_cruise=15.0,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )
  traces = [_run(profile=profile, **common) for profile in range(3)]

  for trace in traces:
    positive = np.flatnonzero(trace.a_target > 0.05)
    moving = np.flatnonzero(trace.speed > 0.01)
    assert len(positive) and trace.time[positive[0]] <= 4 * DT_MDL
    assert len(moving) and trace.time[moving[0]] <= 1.0
    assert np.all(trace.effective_accel_max[trace.active] > 0.0)
    assert not np.any(trace.a_target < -0.05)
    assert trace.solver_failures == 0

  onset_times = [float(trace.time[np.flatnonzero(trace.a_target > 0.05)[0]]) for trace in traces]
  assert max(onset_times) - min(onset_times) <= DT_MDL


def test_profiles_have_distinct_moving_speed_preshape():
  traces = [
    _run(
      duration=18.0,
      controller_enabled=True,
      profile=profile,
      lead_relevancy=False,
      speed=0.0,
      v_cruise=30.0,
      actuator_delay=0.15,
      actuator_lag=0.20,
    )
    for profile in range(3)
  ]
  samples = [np.flatnonzero(trace.speed >= 10.0)[0] for trace in traces]
  configured = [float(trace.profile_accel_max[index]) for trace, index in zip(traces, samples, strict=True)]
  effective = [float(trace.effective_accel_max[index]) for trace, index in zip(traces, samples, strict=True)]
  assert configured[0] < configured[1] < configured[2]
  assert effective[0] < effective[1] < effective[2]
  moving_acceleration = [float(np.mean(trace.a_target[(trace.speed >= 14.0) & (trace.speed <= 16.0)])) for trace in traces]
  assert moving_acceleration[0] < moving_acceleration[1] < moving_acceleration[2]
  assert all(trace.solver_failures == 0 for trace in traces)


def test_clear_road_acceleration_crosses_lut_without_solver_failure():
  trace = _run(
    duration=12.0,
    controller_enabled=True,
    profile=1,
    lead_relevancy=False,
    speed=0.0,
    v_cruise=22.352,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )
  assert np.max(trace.speed) > 10.0
  assert trace.solver_failures == 0
  assert np.all(trace.effective_accel_max[trace.active] > 0.0)


def test_prius_route_model_launches_without_a_dead_pedal():
  trace = _run(
    duration=3.0,
    controller_enabled=True,
    profile=1,
    lead_relevancy=False,
    speed=0.0,
    v_cruise=22.352,
    actuator_model=PRIUS_TSS2_ROUTE_MODEL,
  )
  positive = np.flatnonzero(trace.a_target > 0.05)
  moving = np.flatnonzero(trace.speed > 0.05)
  assert len(positive) and trace.time[positive[0]] <= 4 * DT_MDL
  assert len(moving) and trace.time[moving[0]] <= 1.0
  assert trace.solver_failures == 0


@pytest.mark.parametrize(
  ("actuator_delay", "actuator_lag"),
  [(0.10, 0.20), (0.15, 0.25), (0.20, 0.20), (0.25, 0.30), (0.30, 0.35)],
  ids=("toyota", "honda", "gm", "hyundai", "ford"),
)
def test_stopped_lead_requires_four_departure_frames_and_launches_promptly(actuator_delay, actuator_lag):
  departure_time = 1.0

  def lead_speed(current_time: float) -> float:
    return 0.0 if current_time < departure_time else 2.0

  trace = _run(
    duration=2.5,
    controller_enabled=True,
    lead_relevancy=True,
    speed=0.0,
    distance_lead=6.0,
    v_lead=lead_speed,
    v_cruise=8.0,
    actuator_delay=actuator_delay,
    actuator_lag=actuator_lag,
  )

  first_three = (trace.time > departure_time) & (trace.time <= departure_time + 3 * DT_MDL + 1e-9)
  assert np.max(trace.speed[first_three]) < 1e-3
  assert not trace.launching[first_three].any()
  positive_departure_pace = np.flatnonzero((trace.time >= departure_time) & (trace.target_speed > 0.0))
  assert len(positive_departure_pace)
  before_ego_moves = np.arange(len(trace.time)) >= positive_departure_pace[0]
  before_ego_moves &= trace.speed < 0.30
  assert np.all(trace.target_speed[before_ego_moves] > 0.0)
  moving = np.flatnonzero((trace.time >= departure_time) & (trace.speed > 0.05))
  assert len(moving) and trace.time[moving[0]] <= departure_time + 4 * DT_MDL + 1.0
  prelaunch_pace = trace.target_speed[positive_departure_pace[0] : moving[0] + 1]
  assert np.all(np.diff(prelaunch_pace) >= -1e-9)
  assert not _has_brake_gas_brake(trace.a_target[trace.time >= departure_time])
  assert trace.solver_failures == 0


def test_creeping_lead_departure_is_prompt_and_does_not_lurch():
  departure_time = 1.0

  def lead_speed(current_time: float) -> float:
    if current_time < departure_time:
      return 0.0
    if current_time < departure_time + 0.5:
      return 1.6 * (current_time - departure_time)
    return min(2.5, 0.8 + 0.7 * (current_time - departure_time - 0.5))

  def observe(_current_time: float, lead_name: str, truth: LeadObservation) -> LeadObservation | None:
    return None if lead_name == "leadTwo" else truth | {"aLeadK": 0.0, "radarTrackId": 2133, "radar": True}

  common = dict(
    duration=6.0,
    profile=0,
    lead_relevancy=True,
    speed=0.0,
    distance_lead=3.6,
    v_lead=lead_speed,
    v_cruise=22.352,
    lead_observation_fn=observe,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)
  after_departure = trace.time >= departure_time
  lead_speeds = np.array([lead_speed(max(0.0, current_time - DT_MDL)) for current_time in trace.time])
  baseline_moving = np.flatnonzero((baseline.time >= departure_time) & (baseline.speed > 0.05))
  moving = np.flatnonzero(after_departure & (trace.speed > 0.05))
  assert len(baseline_moving) and len(moving)
  assert trace.time[moving[0]] <= baseline.time[baseline_moving[0]]
  assert np.all(trace.speed[after_departure] <= lead_speeds[after_departure] + 0.20)
  assert not _has_brake_gas_brake(trace.a_target[after_departure])
  assert np.min(trace.distance_lead - trace.distance) >= np.min(baseline.distance_lead - baseline.distance) - 1e-3
  assert trace.solver_failures <= baseline.solver_failures


def test_stop_hold_ignores_two_frame_total_lead_dropout():
  def observe(current_time: float, _lead_name: str, truth: LeadObservation) -> LeadObservation | None:
    return None if 1.0 <= current_time < 1.1 else truth

  trace = _run(
    duration=2.0,
    controller_enabled=True,
    lead_relevancy=True,
    speed=0.0,
    distance_lead=6.0,
    v_lead=0.0,
    v_cruise=8.0,
    lead_observation_fn=observe,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )
  assert np.max(trace.speed) < 1e-3
  assert np.max(trace.pace) == 0.0
  assert trace.solver_failures == 0


def test_low_speed_stopped_lead_never_accelerates_during_stop_hold():
  def lead_speed(current_time: float) -> float:
    return max(0.0, 1.9 - 1.16 * current_time)

  def observe(current_time: float, lead_name: str, truth: LeadObservation) -> LeadObservation | None:
    if lead_name == "leadTwo":
      return None
    moving = lead_speed(current_time) > 0.0
    return truth | {"vLeadK": truth["vLeadK"] if moving else -0.01, "aLeadK": -1.16 if moving else 0.0, "radarTrackId": 7, "radar": True}

  common = dict(
    duration=6.0,
    profile=0,
    lead_relevancy=True,
    speed=4.5,
    distance_lead=18.0,
    v_lead=lead_speed,
    v_cruise=23.056,
    lead_observation_fn=observe,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)

  urgent_demand = (trace.required_decel >= 0.45) & (trace.speed >= 0.30) & ~trace.should_stop
  stop_hold = trace.state == int(AccelControllerState.stopHold)
  assert urgent_demand.any() and stop_hold.any()
  assert np.max(trace.a_target[urgent_demand]) < 0.0
  assert np.max(trace.a_target[stop_hold]) < 0.1
  assert np.max(trace.speed[stop_hold]) < 0.30
  assert np.max(trace.acceleration[stop_hold]) <= 0.0
  assert not _has_brake_gas_brake(trace.a_target[trace.time >= 1.0])
  assert _first_time_below(trace, -1.0) <= _first_time_below(baseline, -1.0) + 1e-9
  assert np.min(trace.distance_lead - trace.distance) >= np.min(baseline.distance_lead - baseline.distance) - 1e-3
  assert trace.solver_failures == 0


def test_moving_lead_dropout_and_false_relief_do_not_release_pace():
  def observe(current_time: float, _lead_name: str, truth: LeadObservation) -> LeadObservation | None:
    if 2.0 <= current_time < 2.1:
      return None
    if 3.0 <= current_time < 3.1:
      return {"dRel": truth["dRel"] + 5.0}
    return truth

  common = dict(
    duration=5.0,
    lead_relevancy=True,
    speed=22.0,
    distance_lead=85.0,
    v_lead=14.0,
    lead_observation_fn=observe,
    actuator_delay=0.20,
    actuator_lag=0.25,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)
  for start in (2.0, 3.0):
    before = trace.pace[np.flatnonzero(trace.time < start)[-1]]
    guard = (trace.time >= start) & (trace.time < start + 0.2) & trace.active
    assert np.all(trace.pace[guard] <= before + 1e-9)
  assert not _has_brake_gas_brake(trace.a_target[trace.time >= 1.0])
  assert trace.solver_failures <= baseline.solver_failures


def test_confirmed_finite_relief_stays_latched_and_smooth():
  def lead_speed(current_time: float) -> float:
    return 8.0 if current_time < 5.0 else min(15.0, 8.0 + 3.5 * (current_time - 5.0))

  common = dict(
    duration=9.0,
    profile=1,
    lead_relevancy=True,
    speed=12.0,
    distance_lead=50.0,
    v_lead=lead_speed,
    v_cruise=20.0,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)

  released = np.flatnonzero((trace.time >= 5.0) & (trace.state == int(AccelControllerState.release)))
  assert len(released)
  reached_base = np.flatnonzero((np.arange(len(trace.time)) >= released[0]) & (trace.target_speed >= 20.0 - 1e-6))
  release_end = reached_base[0] if len(reached_base) else len(trace.time)
  rising = (np.arange(len(trace.time)) >= released[0]) & (np.arange(len(trace.time)) < release_end)
  assert rising.any()
  assert np.all(trace.state[rising] == int(AccelControllerState.release))
  assert np.all(np.diff(trace.target_speed[rising]) >= -1e-9)
  assert not _has_brake_gas_brake(trace.a_target[trace.time >= 5.0])
  assert np.max(np.abs(_command_jerk(trace, after=5.0))) <= np.max(np.abs(_command_jerk(baseline, after=5.0))) + 0.1
  assert trace.solver_failures <= baseline.solver_failures


def test_low_speed_far_lead_acquisition_does_not_fault_or_lurch():
  acquisition_time = 5.0

  def observe(current_time: float, _lead_name: str, truth: LeadObservation) -> LeadObservation | None:
    return None if current_time < acquisition_time else truth

  trace = _run(
    duration=8.0,
    controller_enabled=True,
    profile=0,
    lead_relevancy=True,
    speed=0.0,
    distance_lead=200.0,
    v_lead=3.0,
    v_cruise=30.0,
    lead_observation_fn=observe,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )

  acquired = (trace.time >= acquisition_time) & (trace.selected_lead >= 0)
  response = trace.time >= acquisition_time
  jerk_response = trace.time[1:] >= acquisition_time
  assert acquired.any()
  assert not trace.controller_fault[response].any()
  assert trace.solver_failures == 0
  assert np.max(np.abs(np.diff(trace.a_target)[jerk_response] / DT_MDL)) < 3.0
  assert not _has_brake_gas_brake(trace.a_target[response])


def test_alternating_range_glitch_has_bounded_jerk_and_no_reversal():
  glitch_start = 5.0
  glitch_end = 5.5

  def observe(current_time: float, _lead_name: str, truth: LeadObservation) -> LeadObservation:
    if glitch_start <= current_time < glitch_end:
      frame = round(current_time / DT_MDL)
      return truth | {"dRel": truth["dRel"] + (5.0 if frame % 2 else 0.0)}
    return truth

  common = dict(
    duration=10.0,
    controller_enabled=True,
    lead_relevancy=True,
    speed=8.0,
    distance_lead=20.0,
    v_lead=1.5,
    actuator_delay=0.20,
    actuator_lag=0.25,
  )
  control = _run(**common)
  trace = _run(lead_observation_fn=observe, **common)
  window = (trace.time[1:] >= glitch_start) & (trace.time[1:] < glitch_end + 0.5)
  assert np.max(np.abs(np.diff(trace.a_target)[window] / DT_MDL)) < 3.0
  response = (trace.time >= glitch_start) & (trace.time < glitch_end + 1.0)
  assert not _has_brake_gas_brake((trace.a_target - control.a_target)[response])
  assert trace.solver_failures == 0


@pytest.mark.parametrize(
  ("actuator_delay", "actuator_lag"),
  [(0.10, 0.20), (0.15, 0.25), (0.20, 0.20), (0.25, 0.30), (0.30, 0.35)],
  ids=("toyota", "honda", "gm", "hyundai", "ford"),
)
def test_slow_lead_approach_is_smooth_across_actuator_dynamics(actuator_delay, actuator_lag):
  lead_speed = 10.0
  common = dict(
    duration=15.0,
    profile=1,
    lead_relevancy=True,
    speed=20.0,
    distance_lead=100.0,
    v_lead=lead_speed,
    v_cruise=30.0,
    actuator_delay=actuator_delay,
    actuator_lag=actuator_lag,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)
  baseline_peak_jerk = np.max(np.abs(_command_jerk(baseline, after=1.0)))
  trace_peak_jerk = np.max(np.abs(_command_jerk(trace, after=1.0)))
  assert trace_peak_jerk <= baseline_peak_jerk + 0.1
  assert not _has_brake_gas_brake(trace.a_target[trace.time >= 1.0])
  closing = trace.speed > lead_speed + 0.2
  assert np.max(trace.a_target[closing]) <= 0.2
  assert np.min(trace.distance_lead - trace.distance) >= np.min(baseline.distance_lead - baseline.distance) - 1e-3
  assert trace.solver_failures <= baseline.solver_failures


@pytest.mark.parametrize("profile", range(3), ids=("eco", "normal", "sport"))
def test_decelerating_moving_lead_unwinds_without_brake_gas_brake(profile):
  def lead_speed(current_time: float) -> float:
    if current_time < 2.0:
      return 15.0
    if current_time >= 8.0:
      return 5.0
    progress = (current_time - 2.0) / 6.0
    return 15.0 - 10.0 * (3.0 * progress**2 - 2.0 * progress**3)

  common = dict(
    duration=18.0,
    profile=profile,
    lead_relevancy=True,
    speed=20.0,
    distance_lead=110.0,
    v_lead=lead_speed,
    v_cruise=30.0,
    actuator_delay=0.20,
    actuator_lag=0.25,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)
  after_lead_settles = trace.time >= 8.0
  lead_decelerating = (trace.time >= 2.0) & (trace.time <= 8.0) & trace.active
  moving_lead_guard = trace.effective_accel_max < 0.0
  assert moving_lead_guard.any()
  assert np.min(trace.effective_accel_max[moving_lead_guard]) >= MOVING_LEAD_DECEL_ACCEL_MAX - 1e-9
  assert np.all(np.diff(trace.pace[lead_decelerating]) <= 1e-9)
  assert not trace.should_stop[after_lead_settles].any()
  assert np.max(np.abs(_command_jerk(trace, after=1.0))) < 3.5
  assert np.min(trace.speed[after_lead_settles]) >= 2.0
  assert not _has_brake_gas_brake(trace.a_target[trace.time >= 1.0])
  assert np.min(trace.distance_lead - trace.distance) > 20.0
  assert trace.solver_failures <= baseline.solver_failures


def test_severe_closing_never_delays_stock_braking_or_reduces_clearance():
  common = dict(
    duration=12.0,
    lead_relevancy=True,
    speed=20.0,
    distance_lead=160.0,
    v_lead=3.5,
    actuator_delay=0.20,
    actuator_lag=0.20,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)
  for threshold in (-1.0, -2.0):
    assert _first_time_below(trace, threshold) <= _first_time_below(baseline, threshold) + 1e-9
  baseline_gap = baseline.distance_lead - baseline.distance
  controlled_gap = trace.distance_lead - trace.distance
  assert np.min(controlled_gap) >= np.min(baseline_gap) - 1e-3
  onset = (trace.time[1:] > 0.5) & (trace.time[1:] < 3.0)
  assert np.max(np.abs(np.diff(trace.a_target)[onset] / DT_MDL)) < 4.0
  assert trace.solver_failures == 0


@pytest.mark.parametrize(
  ("actuator_delay", "actuator_lag"),
  [(0.10, 0.20), (0.15, 0.25), (0.20, 0.20), (0.25, 0.30), (0.30, 0.35)],
  ids=("toyota", "honda", "gm", "hyundai", "ford"),
)
@pytest.mark.parametrize("profile", range(3), ids=("eco", "normal", "sport"))
def test_far_lead_deceleration_starts_early_and_stays_smooth(profile, actuator_delay, actuator_lag):
  common = dict(
    duration=11.0,
    lead_relevancy=True,
    speed=25.0,
    distance_lead=200.0,
    v_lead=15.0,
    actuator_delay=actuator_delay,
    actuator_lag=actuator_lag,
  )
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, profile=profile, **common)
  baseline_onset = _sustained_time_below(baseline, -0.10)
  trace_onset = _sustained_time_below(trace, -0.10)
  early_restrictive_pace = (trace.pace < trace.speed - 0.01) & (trace.time <= baseline_onset)
  assert early_restrictive_pace.any()
  assert np.max(trace.a_target[early_restrictive_pace] - baseline.a_target[early_restrictive_pace]) <= 1e-6
  assert trace_onset <= baseline_onset - 0.5
  assert trace.acceleration.min() >= baseline.acceleration.min() - 0.1
  baseline_p95 = float(np.percentile(np.abs(_filtered_realized_jerk(baseline)), 95))
  trace_p95 = float(np.percentile(np.abs(_filtered_realized_jerk(trace)), 95))
  assert trace_p95 <= max(0.15, baseline_p95 + 0.02)
  assert not _has_brake_gas_brake(trace.a_target[trace.time >= 1.0])
  assert trace.solver_failures == 0


def test_solver_fault_discards_live_state_and_recovers():
  _set_params(enabled=True, profile=1)
  plant = Plant(speed=0.0, actuator_delay=0.15, actuator_lag=0.20)
  plant.step(v_cruise=15.0)
  assert plant.planner.accel_controller_result.active
  assert plant.planner.mpc.last_solution_status == 0

  plant.planner.mpc.last_solution_status = 3
  plant.planner.mpc.reset()
  for _ in range(3):
    plant.step(v_cruise=15.0)
    faulted = plant.planner.accel_controller_result
    assert not faulted.active
    assert np.isinf(faulted.live_pace)
    assert faulted.target_speed == 15.0
    assert faulted.mpc_accel_max is not None
    if plant.planner.mpc.last_solution_status == 0:
      break
  assert plant.planner.mpc.last_solution_status == 0

  plant.step(v_cruise=15.0)
  recovered = plant.planner.accel_controller_result
  assert recovered.active
  assert np.isfinite(recovered.live_pace)


def test_solver_fault_keeps_restrictive_lead_target_until_recovery():
  _set_params(enabled=True, profile=1)
  plant = Plant(lead_relevancy=True, speed=25.0, distance_lead=200.0, actuator_delay=0.15, actuator_lag=0.20)
  plant.v_lead_prev = 15.0
  for _ in range(30):
    plant.step(v_lead=15.0, v_cruise=30.0)

  previous_target = plant.planner.accel_controller_result.target_speed
  assert previous_target < 30.0
  assert plant.planner.mpc.last_solution_status == 0

  plant.planner.mpc.last_solution_status = 3
  plant.planner.mpc.reset()
  for _ in range(3):
    result = plant.step(v_lead=15.0, v_cruise=30.0)
    faulted = plant.planner.accel_controller_result
    assert not faulted.active
    assert faulted.target_speed <= previous_target + 1e-9
    assert result["a_target"] <= 0.2
    if plant.planner.mpc.last_solution_status == 0:
      break
  assert plant.planner.mpc.last_solution_status == 0

  plant.step(v_lead=15.0, v_cruise=30.0)
  assert plant.planner.accel_controller_result.active


@pytest.mark.parametrize("pre_frames", (1, 2))
@pytest.mark.parametrize("mode", ("disabled", "e2e"))
def test_early_launch_transition_returns_to_stock_without_solver_fault(pre_frames, mode):
  _set_params(enabled=True, profile=1)
  plant = Plant(speed=0.0, actuator_delay=0.15, actuator_lag=0.20)
  for _ in range(pre_frames):
    plant.step(v_cruise=15.0)

  if mode == "disabled":
    plant.planner.accel_personality_enabled = False
    plant.planner._read_accel_controller_params = lambda: None
  else:
    plant.e2e = True

  for _ in range(4):
    plant.step(v_cruise=15.0)
    controller = plant.planner.accel_controller_result
    assert not controller.active
    assert controller.mpc_accel_max is None
    assert plant.planner.mpc.last_solution_status == 0
    np.testing.assert_array_equal(plant.planner.mpc.params[:, 1], ACCEL_MAX)


@pytest.mark.parametrize("profile", range(3), ids=("eco", "normal", "sport"))
@pytest.mark.parametrize("mode", ("disabled", "e2e"))
def test_launch_transition_after_crossing_standstill_threshold(profile, mode):
  _set_params(enabled=True, profile=profile)
  plant = Plant(speed=0.29, actuator_delay=0.15, actuator_lag=0.20)
  plant.acceleration = 0.5
  plant.planner.a_desired = 0.5
  plant.step(v_cruise=15.0)
  assert plant.speed > 0.30

  if mode == "disabled":
    plant.planner.accel_personality_enabled = False
    plant.planner._read_accel_controller_params = lambda: None
  else:
    plant.e2e = True

  for _ in range(4):
    plant.step(v_cruise=15.0)
    controller = plant.planner.accel_controller_result
    assert not controller.active
    assert controller.mpc_accel_max is None
    assert plant.planner.mpc.last_solution_status == 0
    np.testing.assert_array_equal(plant.planner.mpc.params[:, 1], ACCEL_MAX)
