from collections import deque

import numpy as np
import pytest

from cereal import car, custom
from opendbc.car.interfaces import ACCEL_MAX, ACCEL_MIN
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.sunnypilot.selfdrive.controls.lib.dec.stop_intent import StopIntentController, StopIntentState


class CarState:
  def __init__(self, v_ego: float):
    self.vEgo = v_ego
    self.aEgo = 0.0
    self.standstill = False
    self.gasPressed = False
    self.brakePressed = False
    self.cruiseState = type('CruiseState', (), {'standstill': False})()


class StopModel:
  def __init__(self, distance_m: float, v_ego: float, *, valid: bool = True):
    size = StopIntentController.TRAJECTORY_SIZE if valid else 0
    self.position = type(
      'Position',
      (),
      {
        'x': np.linspace(0.0, max(distance_m, 0.1), size).tolist(),
        'y': np.zeros(size).tolist(),
      },
    )()
    self.velocity = type(
      'Velocity',
      (),
      {
        'x': np.linspace(v_ego, 0.0, size).tolist(),
        'y': np.zeros(size).tolist(),
      },
    )()
    control_distance = max(0.1, distance_m - StopIntentController.ENDPOINT_SETTLE_MARGIN - v_ego * StopIntentController.ACTUATION_LOOKAHEAD)
    required_decel = v_ego * v_ego / (2.0 * control_distance)
    rolling_decel = min(1.5, max(StopIntentController.ACTION_DECEL_MIN, 0.45 * required_decel))
    self.action = type(
      'Action',
      (),
      {'shouldStop': valid and v_ego < 0.3, 'desiredAcceleration': -rolling_decel if valid else 0.0},
    )()


def run_stop_approach(*, initial_speed: float = 12.0, stop_line: float = 60.0, dropout: range = range(0), duration: float = 30.0):
  distance = 0.0
  v_ego = initial_speed
  a_ego = 0.0
  controller = StopIntentController()
  car_state = CarState(v_ego)
  CP = car.CarParams.new_message()
  CP.stopAccel = -2.0
  CP.stoppingDecelRate = 0.8
  CP.vEgoStopping = 0.5
  CP.vEgoStarting = 0.5
  CP.longitudinalActuatorDelay = StopIntentController.ACTUATION_LOOKAHEAD
  CP.longitudinalTuning.kpBP = [0.0]
  CP.longitudinalTuning.kpV = [0.0]
  CP.longitudinalTuning.kiBP = [0.0]
  CP.longitudinalTuning.kiV = [0.0]
  long_control = LongControl(CP, custom.CarParamsSP.new_message())
  actuator_delay = deque([0.0] * round(CP.longitudinalActuatorDelay / DT_CTRL))
  trace = []

  for frame in range(round(duration / DT_MDL)):
    remaining = stop_line - distance
    car_state.vEgo = v_ego
    car_state.aEgo = a_ego
    car_state.standstill = v_ego < 0.01
    car_state.cruiseState.standstill = car_state.standstill
    observation_valid = frame not in dropout
    model = StopModel(remaining, v_ego, valid=observation_valid)
    constraint = controller.update(
      model,
      car_state,
      enabled=True,
      lead_present=False,
      observation_valid=observation_valid,
      dt=DT_MDL,
    )

    a_target = min(model.action.desiredAcceleration, 0.0)
    if constraint.active:
      a_target = min(a_target, constraint.accel_ceiling_mps2)
    should_stop = constraint.hold or (model.action.shouldStop and not constraint.active)
    for _ in range(round(DT_MDL / DT_CTRL)):
      car_state.vEgo = v_ego
      car_state.aEgo = a_ego
      car_state.standstill = v_ego < 0.01
      car_state.cruiseState.standstill = car_state.standstill
      commanded_accel = long_control.update(True, car_state, a_target, should_stop, (ACCEL_MIN, ACCEL_MAX))
      actuator_delay.append(commanded_accel)
      delayed_accel = actuator_delay.popleft()
      a_ego += np.clip(delayed_accel - a_ego, -4.0 * DT_CTRL, 4.0 * DT_CTRL)
      v_ego = max(0.0, v_ego + a_ego * DT_CTRL)
      distance += v_ego * DT_CTRL
    trace.append((distance, v_ego, a_ego, constraint))

  return stop_line, trace


@pytest.mark.parametrize(("initial_speed", "stop_line"), [(3.0, 15.0), (8.0, 35.0), (12.0, 60.0), (20.0, 150.0)])
def test_stationary_constraint_stops_and_holds_at_target(initial_speed, stop_line):
  stop_line, trace = run_stop_approach(initial_speed=initial_speed, stop_line=stop_line)
  active_constraints = [constraint for _, _, _, constraint in trace if constraint.active]

  assert active_constraints
  assert all(not constraint.hold for constraint in active_constraints if constraint.state == StopIntentState.approach)
  assert trace[-1][3].state == StopIntentState.hold
  assert trace[-1][3].hold
  assert trace[-1][1] < 0.02
  assert 0.0 <= stop_line - trace[-1][0] < 3.0
  assert min(a_ego for _, _, a_ego, _ in trace) >= -StopIntentController.MAX_REQUIRED_DECEL - 0.01

  first_active = next(idx for idx, (*_, constraint) in enumerate(trace) if constraint.active)
  active_speeds = [v_ego for _, v_ego, _, _ in trace[first_active:]]
  assert all(next_speed <= speed + 0.01 for speed, next_speed in zip(active_speeds, active_speeds[1:], strict=False))


def test_committed_stop_survives_model_dropout_without_reaccelerating():
  stop_line, trace = run_stop_approach(dropout=range(30, 38))
  dropout_trace = trace[30:38]

  assert all(constraint.active for _, _, _, constraint in dropout_trace)
  assert max(v_ego for _, v_ego, _, _ in dropout_trace) <= trace[29][1] + 0.05
  assert trace[-1][3].hold
  assert trace[-1][1] < 0.02
  assert 0.0 <= stop_line - trace[-1][0] < 3.0
