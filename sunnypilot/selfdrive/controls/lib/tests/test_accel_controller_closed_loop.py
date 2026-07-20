from collections.abc import Callable
from dataclasses import dataclass
import gc

import numpy as np
import pytest

from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import PRIUS_TSS2_ROUTE_MODEL, LeadObservation, Plant
from openpilot.sunnypilot.selfdrive.controls.lib.accel_personality import AccelControllerState, AccelProfile

ROUTINE_GAP_TOLERANCE = 0.10


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
  stock_mode: np.ndarray
  raw_cap: np.ndarray
  filtered_cap: np.ndarray
  selected_lead: np.ndarray
  profile_accel_max: np.ndarray
  effective_accel_max: np.ndarray
  state: np.ndarray
  required_decel: np.ndarray
  controller_fault: np.ndarray
  controller_fault_latched: np.ndarray
  mpc_accel_max: np.ndarray
  actuator_command: np.ndarray
  solver_status: np.ndarray
  solver_failures: int
  solver_failure_times: list[float]


def _configure_plant(plant: Plant, *, enabled: bool, profile: int = 1, dec_enabled: bool = False) -> None:
  plant.planner.accel_personality_enabled = enabled
  plant.planner.accel_personality = profile
  plant.planner._read_accel_controller_params = lambda: None
  plant.planner.dec._enabled = dec_enabled
  plant.planner.dec._read_params = lambda: None


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
  gc.collect()
  plant = Plant(**plant_kwargs)
  _configure_plant(plant, enabled=controller_enabled, profile=profile, dec_enabled=dec_enabled)
  plant.v_lead_prev = float(v_lead) if isinstance(v_lead, (int, float)) else float(v_lead(0.0))
  solver_failures = 0
  solver_failure_times = []
  original_mpc_reset = plant.planner.mpc.reset

  def count_failed_solve() -> None:
    nonlocal solver_failures
    if plant.planner.mpc.solution_status != 0:
      solver_failures += 1
      solver_failure_times.append(plant.current_time)
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
        controller.stock_mode,
        controller.raw_energy_cap,
        controller.live_filtered_cap,
        controller.selected_lead,
        controller.profile_accel_max,
        controller.effective_accel_max,
        controller.state,
        controller.required_decel,
        controller_fault,
        plant.planner.accel_controller_fault_latched,
        min(controller.mpc_accel_max) if controller.mpc_accel_max is not None else np.nan,
        result["actuator_command"],
        plant.planner.mpc.last_solution_status,
      )
    )
    sources.append(result["mpc_source"])

  data = np.asarray(rows, dtype=float)
  trace = ClosedLoopTrace(
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
    stock_mode=data[:, 12].astype(bool),
    raw_cap=data[:, 13],
    filtered_cap=data[:, 14],
    selected_lead=data[:, 15].astype(int),
    profile_accel_max=data[:, 16],
    effective_accel_max=data[:, 17],
    state=data[:, 18].astype(int),
    required_decel=data[:, 19],
    controller_fault=data[:, 20].astype(bool),
    controller_fault_latched=data[:, 21].astype(bool),
    mpc_accel_max=data[:, 22],
    actuator_command=data[:, 23],
    solver_status=data[:, 24].astype(int),
    solver_failures=solver_failures,
    solver_failure_times=solver_failure_times,
  )
  plant.planner.mpc.reset = original_mpc_reset
  gc.collect()
  return trace


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


def _filtered_realized_jerk(trace: ClosedLoopTrace, after: float = 1.0, min_speed: float = 0.0) -> np.ndarray:
  filtered_acceleration = np.convolve(trace.acceleration, np.ones(3) / 3.0, mode="valid")
  samples = (trace.time[2:-1] >= after) & (trace.speed[2:-1] >= min_speed)
  return (np.diff(filtered_acceleration) / DT_MDL)[samples]


def _has_brake_coast_brake(values: np.ndarray, brake: float = -0.8, coast: float = -0.35, frames: int = 2) -> bool:
  phase = 0
  for index in range(len(values) - frames + 1):
    window = values[index : index + frames]
    if np.all(window <= brake):
      if phase == 2:
        return True
      phase = 1
    elif phase == 1 and np.all(window >= coast):
      phase = 2
  return False


def _has_propulsion_after_braking(values: np.ndarray, propulsion: float = 0.2, brake: float = -0.2, frames: int = 2) -> bool:
  braking = False
  for index in range(len(values) - frames + 1):
    window = values[index : index + frames]
    if np.all(window <= brake):
      braking = True
    elif braking and np.all(window >= propulsion):
      return True
  return False


def _has_propulsion_brake_cycle(values: np.ndarray, propulsion: float = 0.2, brake: float = -0.2, frames: int = 2) -> bool:
  phases = []
  for index in range(len(values) - frames + 1):
    window = values[index : index + frames]
    phase = 1 if np.all(window >= propulsion) else -1 if np.all(window <= brake) else 0
    if phase and (not phases or phase != phases[-1]):
      phases.append(phase)
      if len(phases) >= 3 and phases[-1] == phases[-3]:
        return True
  return False


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
  np.testing.assert_array_equal(trace.shadow_active, np.full_like(trace.active, expect_shadow))


def test_disabled_profiles_match_clean_base():
  common = dict(duration=2.0, controller_enabled=False, lead_relevancy=True, speed=20.0, distance_lead=70.0, v_lead=14.0)
  traces = [_run(profile=profile, **common) for profile in range(3)]
  for trace in traces[1:]:
    np.testing.assert_allclose(trace.a_target, traces[0].a_target, atol=1e-6, rtol=0.0)
    np.testing.assert_array_equal(trace.should_stop, traces[0].should_stop)
    assert trace.source == traces[0].source
  assert all(np.isinf(trace.effective_accel_max).all() for trace in traces)


@pytest.mark.parametrize("lead_relevancy", (False, True), ids=("clear-road", "lead"))
def test_force_decel_matches_controller_off(lead_relevancy):
  common = dict(duration=2.0, force_decel=True, lead_relevancy=lead_relevancy, speed=20.0,
                distance_lead=70.0, v_lead=14.0, profile=0)
  baseline = _run(controller_enabled=False, **common)
  trace = _run(controller_enabled=True, **common)
  np.testing.assert_allclose(trace.a_target, baseline.a_target, atol=1e-6, rtol=0.0)
  np.testing.assert_array_equal(trace.should_stop, baseline.should_stop)
  np.testing.assert_array_equal(trace.fcw, baseline.fcw)
  assert trace.source == baseline.source


def test_e2e_to_radar_acc_handoff_keeps_braking_continuous():
  plant = Plant(
    lead_relevancy=True, speed=10.0, distance_lead=30.0, actuator_delay=0.15, actuator_lag=0.20,
    model_action_fn=lambda current_time, _v_ego, _a_ego: (-1.0 if current_time < 2.0 else 0.0, False),
  )
  _configure_plant(plant, enabled=True)
  rows = []
  while plant.current_time < 2.4:
    plant.e2e = plant.current_time < 2.0
    result = plant.step(v_lead=8.0, v_cruise=20.0)
    rows.append((plant.current_time, result["a_target"], plant.planner.mpc.last_solution_status,
                 plant.planner.accel_controller_result.active))

  time_values, acceleration, solver_status, active = np.asarray(rows, dtype=float).T
  transition = np.flatnonzero(time_values > 2.0)[0]
  assert acceleration[transition] - acceleration[transition - 1] < 0.15
  assert np.max(np.diff(acceleration[transition - 1:]) / DT_MDL) < 3.0
  assert not solver_status[transition:].any()
  assert active[transition]


def test_dec_retains_acc_through_route_like_radar_marker_dropout():
  dropout_start = 1.0
  reacquisition_time = 1.8

  def observe(current_time: float, lead_name: str, truth: LeadObservation) -> LeadObservation:
    frame = round(current_time / DT_MDL)
    if current_time < dropout_start:
      marked_slot = "leadOne" if frame % 2 == 0 else "leadTwo"
      return truth | {"radar": lead_name == marked_slot, "radarTrackId": 985 + frame if lead_name == marked_slot else -1}
    if current_time < reacquisition_time:
      return truth | {"radar": False, "radarTrackId": -1}
    return truth | {"radar": lead_name == "leadOne", "radarTrackId": 1263 if lead_name == "leadOne" else -1}

  plant = Plant(
    e2e=True, lead_relevancy=True, speed=20.0, distance_lead=35.0, lead_observation_fn=observe,
    model_action_fn=lambda _current_time, _v_ego, _a_ego: (-2.0, False), actuator_delay=0.15, actuator_lag=0.20,
  )
  _configure_plant(plant, enabled=True, dec_enabled=True)
  rows = []
  while plant.current_time < 2.5:
    result = plant.step(v_lead=18.0, v_cruise=30.0)
    rows.append((plant.current_time, result["a_target"], result["dec_mode"], str(result["mpc_source"]), result["fcw"]))

  time_values = np.asarray([row[0] for row in rows])
  acceleration = np.asarray([row[1] for row in rows])
  dropout = (time_values >= dropout_start) & (time_values < reacquisition_time)
  response = (time_values >= dropout_start - DT_MDL) & (time_values <= reacquisition_time + 0.5)
  assert all(row[2] == "acc" for row in rows)
  assert all(row[3] != "e2e" for row in rows)
  assert not any(row[4] for row in rows)
  assert dropout.any()
  assert not _has_propulsion_brake_cycle(acceleration[response])
  assert np.max(np.abs(np.diff(acceleration[response]) / DT_MDL)) < 3.0
  assert not plant.planner.accel_controller_fault_latched


def test_active_controller_is_pre_mpc_and_preserves_stock_lead_authority():
  plant = Plant(lead_relevancy=False, speed=0.0, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(plant, enabled=True, profile=0)
  result = plant.step(v_cruise=15.0)
  controller = plant.planner.accel_controller_result

  assert controller.mpc_accel_max is not None
  np.testing.assert_allclose(plant.planner.mpc.params[:, 1], controller.mpc_accel_max)
  assert np.all((plant.planner.mpc.params[:, 1] >= 0.0) & (plant.planner.mpc.params[:, 1] <= ACCEL_MAX))
  assert ACCEL_MIN <= result["a_target"] <= get_max_accel(plant.speed)
  for _ in range(100):
    result = plant.step(v_cruise=15.0)
    if plant.speed >= 0.30:
      break
  assert plant.speed >= 0.30
  controller = plant.planner.accel_controller_result
  assert controller.mpc_accel_max is not None
  np.testing.assert_allclose(plant.planner.mpc.params[:, 1], controller.mpc_accel_max)
  assert np.all((plant.planner.mpc.params[:, 1] >= 0.0) & (plant.planner.mpc.params[:, 1] <= ACCEL_MAX))

  lead_plant = Plant(lead_relevancy=True, speed=0.0, distance_lead=6.0, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(lead_plant, enabled=True, profile=0)
  lead_plant.step(v_lead=0.0, v_cruise=15.0)
  controller = lead_plant.planner.accel_controller_result
  assert controller.target_speed == 0.0
  np.testing.assert_array_equal(controller.mpc_accel_max, 0.0)
  np.testing.assert_array_equal(lead_plant.planner.mpc.params[:, 1], 0.0)


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
  speed_grid = np.linspace(5.0, 16.0, 45)
  moving_acceleration = [np.interp(speed_grid, trace.speed, trace.a_target) for trace in traces]
  assert np.all(moving_acceleration[1] - moving_acceleration[0] > 0.10)
  assert np.all(moving_acceleration[2] - moving_acceleration[1] > 0.05)
  assert all(trace.solver_failures == 0 for trace in traces)


def test_runtime_profile_switch_is_distinct_and_smooth():
  plant = Plant(lead_relevancy=False, speed=0.0, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(plant, enabled=True, profile=AccelProfile.sport)
  while plant.speed < 10.0 and plant.current_time < 15.0:
    plant.step(v_cruise=30.0)
  assert plant.speed >= 10.0
  switch_start = plant.current_time
  rows = []
  while plant.current_time < switch_start + 5.0:
    elapsed = plant.current_time - switch_start
    profile = AccelProfile.sport if elapsed < 1.0 or elapsed >= 3.0 else AccelProfile.eco
    plant.planner.accel_personality = profile
    result = plant.step(v_cruise=30.0)
    controller = plant.planner.accel_controller_result
    rows.append((plant.current_time - switch_start, profile, result["a_target"], controller.effective_accel_max,
                 plant.planner.mpc.last_solution_status, plant.planner.accel_controller_fault_latched))

  data = np.asarray(rows, dtype=float)
  time_values, profiles, acceleration, effective_max, solver_status, fault_latched = data.T
  settled_eco = (profiles == AccelProfile.eco) & (time_values >= 2.0) & (time_values < 3.0)
  settled_sport = (profiles == AccelProfile.sport) & (time_values >= 4.0)
  assert np.max(effective_max[settled_eco]) < np.min(effective_max[settled_sport])
  assert np.mean(acceleration[settled_eco]) + 0.15 < np.mean(acceleration[settled_sport])
  switch_window = ((time_values[1:] >= 0.5) & (time_values[1:] <= 1.5)) | ((time_values[1:] >= 2.5) & (time_values[1:] <= 3.5))
  assert np.max(np.abs(np.diff(acceleration)[switch_window] / DT_MDL)) < 3.0
  assert np.min(acceleration) >= -0.05
  assert not solver_status.any()
  assert not fault_latched.any()


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
  departure_release = np.flatnonzero((trace.time >= departure_time) & trace.launching)
  assert len(departure_release) and trace.time[departure_release[0]] >= departure_time + 3 * DT_MDL
  moving = np.flatnonzero((trace.time >= departure_time) & (trace.speed > 0.05))
  assert len(moving) and trace.time[moving[0]] <= departure_time + 4 * DT_MDL + 1.0
  assert np.min(trace.effective_accel_max[departure_release[0] : moving[0] + 1]) > 1.5
  assert not _has_brake_coast_brake(trace.a_target[trace.time >= departure_time])
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
  assert not _has_brake_coast_brake(trace.a_target[after_departure])
  assert np.min(trace.distance_lead - trace.distance) >= np.min(baseline.distance_lead - baseline.distance) - 1e-3
  assert trace.solver_failures == 0


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
  assert np.max(trace.effective_accel_max[np.isfinite(trace.effective_accel_max)]) == 0.0
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
  hold_indices = np.flatnonzero(stop_hold)
  assert np.max(trace.acceleration[stop_hold]) < 0.25
  assert np.max(trace.speed[stop_hold]) < 0.30
  assert trace.distance[hold_indices[-1]] - trace.distance[hold_indices[0]] < 0.05
  assert not _has_brake_coast_brake(trace.a_target[trace.time >= 1.0])
  assert np.min(trace.a_target) >= np.min(baseline.a_target) - ROUTINE_GAP_TOLERANCE
  assert np.min(trace.distance_lead - trace.distance) >= np.min(baseline.distance_lead - baseline.distance) - ROUTINE_GAP_TOLERANCE
  assert not trace.fcw.any()
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
  trace = _run(controller_enabled=True, **common)
  for start in (2.0, 3.0):
    before = trace.effective_accel_max[np.flatnonzero(trace.time < start)[-1]]
    guard = (trace.time >= start) & (trace.time < start + 0.2) & trace.active
    assert np.all(trace.effective_accel_max[guard] <= before + 0.02)
  assert not _has_brake_coast_brake(trace.a_target[trace.time >= 1.0])
  assert not _has_propulsion_after_braking(trace.a_target[trace.time >= 1.0])
  assert np.max(np.abs(_command_jerk(trace, after=1.0))) < 3.0
  assert trace.solver_failures == 0


def test_positive_approach_bound_dropout_does_not_fault_mpc():
  def observe(current_time: float, _lead_name: str, truth: LeadObservation) -> LeadObservation | None:
    return truth if 0.50 <= current_time < 0.70 or current_time >= 0.95 else None

  trace = _run(
    duration=2.0,
    controller_enabled=True,
    profile=1,
    lead_relevancy=True,
    speed=20.0,
    distance_lead=60.0,
    v_lead=14.0,
    v_cruise=30.0,
    lead_observation_fn=observe,
    actuator_delay=0.20,
    actuator_lag=0.25,
  )

  assert not trace.solver_status.any()
  assert trace.solver_failures == 0
  assert not trace.controller_fault_latched.any()


@pytest.mark.parametrize(
  ("actuator_delay", "actuator_lag"),
  [(0.10, 0.20), (0.15, 0.25), (0.20, 0.20), (0.25, 0.30), (0.30, 0.35)],
  ids=("toyota", "honda", "gm", "hyundai", "ford"),
)
def test_confirmed_finite_relief_transitions_smoothly(actuator_delay, actuator_lag):
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
    actuator_delay=actuator_delay,
    actuator_lag=actuator_lag,
  )
  trace = _run(controller_enabled=True, **common)

  released = np.flatnonzero((trace.time >= 5.0) & (trace.state == int(AccelControllerState.free)))
  assert len(released)
  reached_profile = np.flatnonzero((np.arange(len(trace.time)) >= released[0]) &
                                   (trace.effective_accel_max >= trace.profile_accel_max - 1e-6))
  assert len(reached_profile)
  rising = (np.arange(len(trace.time)) >= released[0]) & (np.arange(len(trace.time)) <= reached_profile[0])
  assert rising.any()
  assert np.all(np.diff(trace.effective_accel_max[rising]) >= -1e-9)
  assert not _has_brake_coast_brake(trace.a_target[trace.time >= 5.0])
  assert not _has_propulsion_brake_cycle(trace.a_target[trace.time >= 5.0])
  assert np.max(np.abs(_command_jerk(trace, after=5.0))) < 3.0
  assert trace.solver_failures == 0


def test_low_speed_far_lead_acquisition_does_not_fault_or_lurch():
  acquisition_time = 5.0

  def observe(current_time: float, _lead_name: str, truth: LeadObservation) -> LeadObservation | None:
    return None if current_time < acquisition_time else truth

  common = dict(
    duration=8.0,
    profile=0,
    lead_relevancy=True,
    speed=0.0,
    distance_lead=180.0,
    v_lead=3.0,
    v_cruise=30.0,
    lead_observation_fn=observe,
    actuator_delay=0.15,
    actuator_lag=0.20,
  )
  trace = _run(controller_enabled=True, **common)

  acquired = (trace.time >= acquisition_time) & (trace.selected_lead >= 0)
  response = trace.time >= acquisition_time
  jerk_response = trace.time[1:] >= acquisition_time
  assert acquired.any()
  assert not trace.controller_fault[response].any()
  assert not trace.solver_status.any()
  assert not trace.controller_fault_latched.any()
  assert trace.solver_failures == 0
  assert np.max(np.abs(np.diff(trace.a_target)[jerk_response] / DT_MDL)) < 3.0
  assert not _has_brake_coast_brake(trace.a_target[response])
  assert not _has_propulsion_after_braking(trace.a_target[response])


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
    lead_relevancy=True,
    speed=8.0,
    distance_lead=20.0,
    v_lead=1.5,
    actuator_delay=0.20,
    actuator_lag=0.25,
  )
  control = _run(controller_enabled=True, **common)
  baseline = _run(controller_enabled=False, lead_observation_fn=observe, **common)
  trace = _run(controller_enabled=True, lead_observation_fn=observe, **common)
  window = (trace.time[1:] >= glitch_start) & (trace.time[1:] < glitch_end)
  assert np.max(np.abs(np.diff(trace.a_target)[window] / DT_MDL)) < 3.0
  response = (trace.time >= glitch_start) & (trace.time < glitch_end + 1.0)
  assert np.max(np.abs((trace.a_target - control.a_target)[response])) < 0.07
  np.testing.assert_array_equal(trace.should_stop[response], baseline.should_stop[response])
  np.testing.assert_array_equal(trace.fcw[response], baseline.fcw[response])
  assert not _has_brake_coast_brake(trace.a_target[response])
  assert not _has_propulsion_after_braking(trace.a_target[response])
  assert np.min(trace.distance_lead - trace.distance) >= np.min(baseline.distance_lead - baseline.distance) - ROUTINE_GAP_TOLERANCE
  assert trace.solver_failures == 0


@pytest.mark.parametrize(
  ("actuator_delay", "actuator_lag"),
  [(0.10, 0.20), (0.15, 0.25), (0.20, 0.20), (0.25, 0.30), (0.30, 0.35)],
  ids=("toyota", "honda", "gm", "hyundai", "ford"),
)
def test_slow_lead_approach_is_smooth_across_actuator_dynamics(actuator_delay, actuator_lag):
  lead_speed = 10.0
  trace = _run(
    duration=70.0,
    controller_enabled=True,
    profile=1,
    lead_relevancy=True,
    speed=20.0,
    distance_lead=100.0,
    v_lead=lead_speed,
    v_cruise=30.0,
    actuator_delay=actuator_delay,
    actuator_lag=actuator_lag,
  )
  desired_gap = STOP_DISTANCE + get_T_FOLLOW() * lead_speed
  gap = trace.distance_lead - trace.distance
  closing_speed = trace.speed - lead_speed
  closing = closing_speed > 0.1
  meaningful_closing = closing_speed > 0.3
  settled = trace.time >= trace.time[-1] - 3.0
  moving = (trace.time[1:] >= 0.5) & (trace.speed[1:] >= 2.0) & ~trace.should_stop[1:] & ~trace.should_stop[:-1]
  assert np.max(np.abs(np.diff(trace.a_target)[moving] / DT_MDL)) < 3.0
  assert not _has_brake_coast_brake(trace.a_target[trace.time >= 1.0])
  assert not _has_propulsion_brake_cycle(trace.a_target[trace.time >= 1.0])
  assert np.max(trace.a_target[meaningful_closing]) <= 0.2
  assert np.percentile(np.abs(_filtered_realized_jerk(trace)), 95) < 0.35
  assert np.min(trace.a_target) >= -1.1
  assert np.min(trace.acceleration) >= -1.1
  assert np.min(gap) >= desired_gap - 1.6
  assert np.min(gap[closing] / closing_speed[closing]) >= 2.0
  assert abs(np.median(trace.speed[settled]) - lead_speed) <= 0.5
  assert desired_gap - 1.6 <= np.median(gap[settled]) <= desired_gap + 6.0
  assert not trace.fcw.any()
  assert not trace.should_stop.any()
  assert not trace.solver_status.any()
  assert not trace.controller_fault_latched.any()
  assert trace.solver_failures == 0


def test_decelerating_moving_lead_stays_smooth_and_safe():
  def lead_speed(current_time: float) -> float:
    if current_time < 2.0:
      return 15.0
    progress = min((current_time - 2.0) / 6.0, 1.0)
    return 15.0 - 5.0 * (3.0 * progress**2 - 2.0 * progress**3)

  common = dict(
    duration=14.0,
    profile=1,
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
  lead_decelerating = (trace.time >= 2.0) & (trace.time <= 8.0) & trace.active
  settled = trace.time >= 8.0
  assert np.any(trace.effective_accel_max[lead_decelerating] < 0.0)
  assert not trace.should_stop.any()
  assert np.max(np.abs(_command_jerk(trace, after=1.0))) < 4.25
  baseline_p95 = np.percentile(np.abs(_filtered_realized_jerk(baseline)), 95)
  trace_p95 = np.percentile(np.abs(_filtered_realized_jerk(trace)), 95)
  assert trace_p95 <= max(0.20, baseline_p95 + 0.02)
  assert not _has_brake_coast_brake(trace.a_target[trace.time >= 1.0])
  assert not _has_propulsion_after_braking(trace.a_target[trace.time >= 1.0])
  assert np.max(trace.a_target[settled]) <= 0.2
  assert np.min(trace.distance_lead - trace.distance) >= np.min(baseline.distance_lead - baseline.distance) - ROUTINE_GAP_TOLERANCE
  assert not trace.fcw.any()
  assert trace.solver_failures == 0


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
  assert np.min(controlled_gap) >= np.min(baseline_gap) - 0.02
  baseline_closing = baseline.speed - 3.5
  controlled_closing = trace.speed - 3.5
  baseline_ttc = np.min(baseline_gap[baseline_closing > 0.1] / baseline_closing[baseline_closing > 0.1])
  controlled_ttc = np.min(controlled_gap[controlled_closing > 0.1] / controlled_closing[controlled_closing > 0.1])
  assert controlled_ttc >= baseline_ttc - 0.02
  assert np.min(controlled_gap) > 0.0
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
  negative_bound = np.isfinite(trace.mpc_accel_max) & (trace.mpc_accel_max < -0.05)
  assert negative_bound.any()
  assert trace.time[np.flatnonzero(negative_bound)[0]] <= baseline_onset - 0.5
  assert trace_onset <= baseline_onset - 0.5
  assert trace.acceleration.min() >= baseline.acceleration.min() - 0.1
  trace_p95 = float(np.percentile(np.abs(_filtered_realized_jerk(trace)), 95))
  assert trace_p95 < 0.45
  assert np.max(np.abs(_command_jerk(trace, after=0.5))) < 3.0
  assert not _has_brake_coast_brake(trace.a_target[trace.time >= 1.0])
  assert not _has_propulsion_brake_cycle(trace.a_target[trace.time >= 1.0])
  assert not trace.fcw.any()
  assert not trace.solver_status.any()
  assert not trace.controller_fault_latched.any()
  assert trace.solver_failures == 0


def test_far_lead_profile_order_is_monotonic():
  traces = [
    _run(
      duration=6.0,
      controller_enabled=True,
      profile=profile,
      lead_relevancy=True,
      speed=25.0,
      distance_lead=200.0,
      v_lead=15.0,
      actuator_delay=0.10,
      actuator_lag=0.20,
    )
    for profile in range(3)
  ]
  bound_onsets = [
    float(trace.time[np.flatnonzero(np.isfinite(trace.mpc_accel_max) & (trace.mpc_accel_max < -0.05))[0]])
    for trace in traces
  ]
  decel_onsets = [_sustained_time_below(trace, -0.10) for trace in traces]
  assert bound_onsets[0] <= bound_onsets[1] <= bound_onsets[2]
  assert decel_onsets[0] <= decel_onsets[1] <= decel_onsets[2]
  assert traces[0].raw_cap[0] < traces[1].raw_cap[0] < traces[2].raw_cap[0]


def test_prior_stock_solver_status_does_not_disable_clear_road_controller():
  plant = Plant(speed=0.0, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(plant, enabled=True, profile=1)
  plant.step(v_cruise=15.0)
  assert plant.planner.accel_controller_result.active
  assert plant.planner.mpc.last_solution_status == 0

  plant.planner.mpc.last_solution_status = 3
  plant.step(v_cruise=15.0)
  assert plant.planner.mpc.last_solution_status == 0
  recovered = plant.planner.accel_controller_result
  assert recovered.active
  assert not plant.planner.accel_controller_fault_latched
  assert np.isfinite(recovered.effective_accel_max)


def test_prior_stock_solver_status_does_not_disable_lead_controller():
  plant = Plant(lead_relevancy=True, speed=25.0, distance_lead=200.0, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(plant, enabled=True, profile=1)
  plant.v_lead_prev = 15.0
  for _ in range(30):
    plant.step(v_lead=15.0, v_cruise=30.0)

  assert plant.planner.accel_controller_result.effective_accel_max < 0.0
  assert plant.planner.mpc.last_solution_status == 0

  plant.planner.mpc.last_solution_status = 3
  result = plant.step(v_lead=15.0, v_cruise=30.0)
  assert plant.planner.mpc.last_solution_status == 0
  assert plant.planner.accel_controller_result.active
  assert not plant.planner.accel_controller_fault_latched
  assert result["a_target"] <= 0.2


@pytest.mark.parametrize(("profile", "speed", "expects_ceiling"), ((0, 10.0, True), (2, 0.0, False)), ids=("ceiling", "seed-only"))
def test_failed_custom_solve_restores_stock_state_and_counts_fcw_once(profile, speed, expects_ceiling):
  plant = Plant(lead_relevancy=False, speed=speed, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(plant, enabled=True, profile=profile)
  saved_a_prev = np.full_like(plant.planner.mpc.a_prev, -0.25)
  accepted_a_prev = np.full_like(saved_a_prev, 0.15)
  plant.planner.mpc.a_prev = saved_a_prev.copy()
  plant.planner.mpc.crash_cnt = 2.0
  if not expects_ceiling:
    plant.planner.accel_controller._build_accel_ceiling = lambda *_args: None
  calls = []

  def update_mpc(_radar_state, _v_cruise, personality, accel_max=None):
    calls.append((personality, accel_max))
    if len(calls) == 1:
      plant.planner.mpc.last_solution_status = plant.planner.mpc.solution_status = 4
      plant.planner.mpc.a_prev = np.zeros_like(saved_a_prev)
      plant.planner.mpc.crash_cnt = 0.0
    else:
      np.testing.assert_array_equal(plant.planner.mpc.a_prev, saved_a_prev)
      assert plant.planner.mpc.crash_cnt == 2.0
      plant.planner.mpc.last_solution_status = plant.planner.mpc.solution_status = 0
      plant.planner.mpc.a_prev = accepted_a_prev.copy()
      plant.planner.mpc.crash_cnt += 1.0

  plant.planner.mpc.update = update_mpc
  result = plant.step(v_cruise=30.0)
  assert len(calls) == 2
  assert (calls[0][1] is not None) == expects_ceiling and calls[1][1] is None
  assert plant.planner.accel_controller_fault_latched
  assert not plant.planner.accel_controller_result.active
  assert plant.planner.mpc.crash_cnt == 3.0
  np.testing.assert_array_equal(plant.planner.mpc.a_prev, accepted_a_prev)
  assert result["fcw"] == (speed > 0.0)


@pytest.mark.parametrize("mode", ("disabled", "e2e"))
def test_stock_solver_recovery_is_not_warm_seeded_when_controller_cannot_actuate(mode):
  plant = Plant(lead_relevancy=False, speed=10.0, actuator_delay=0.15, actuator_lag=0.20, e2e=mode == "e2e")
  _configure_plant(plant, enabled=mode != "disabled", profile=0)
  plant.planner.mpc.last_solution_status = 3
  seeds = []
  plant.planner._seed_mpc_current_state = lambda _target=None: seeds.append(True)
  plant.step(v_cruise=30.0)
  assert not seeds


@pytest.mark.parametrize("pre_frames", (1, 2))
@pytest.mark.parametrize("mode", ("disabled", "e2e"))
def test_early_launch_transition_returns_to_stock_without_solver_fault(pre_frames, mode):
  plant = Plant(speed=0.0, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(plant, enabled=True, profile=1)
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
  plant = Plant(speed=0.29, actuator_delay=0.15, actuator_lag=0.20)
  _configure_plant(plant, enabled=True, profile=profile)
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
