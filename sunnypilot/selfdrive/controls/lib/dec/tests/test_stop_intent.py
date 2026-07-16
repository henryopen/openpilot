import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.dec.stop_intent import StopIntentController, StopIntentState


class CarState:
  def __init__(self, v_ego=10.0, *, standstill=False, gas_pressed=False):
    self.vEgo = v_ego
    self.standstill = standstill
    self.gasPressed = gas_pressed


class Model:
  def __init__(
    self,
    distance=50.0,
    *,
    valid=True,
    terminal_speed=0.0,
    terminal_lateral_speed=0.0,
    should_stop=False,
    desired_accel=-1.0,
    curve=0.0,
  ):
    size = StopIntentController.TRAJECTORY_SIZE if valid else 0
    denominator = max(1, size - 1)
    self.position = type(
      'Position',
      (),
      {
        'x': [distance * i / denominator for i in range(size)],
        'y': [distance * curve * i / denominator for i in range(size)],
      },
    )()
    self.velocity = type(
      'Velocity',
      (),
      {
        'x': [terminal_speed] * size,
        'y': [terminal_lateral_speed] * size,
      },
    )()
    self.action = type('Action', (), {'shouldStop': should_stop, 'desiredAcceleration': desired_accel})()


def run_fixed_target(controller, car_state, start_distance=60.0, frames=20, **model_kwargs):
  constraint = None
  for frame in range(frames):
    distance = max(0.1, start_distance - car_state.vEgo * frame * DT_MDL)
    constraint = controller.update(Model(distance, **model_kwargs), car_state, enabled=True, lead_present=False)
  return constraint


def qualify(controller, car_state, start_distance=60.0):
  constraint = run_fixed_target(controller, car_state, start_distance)
  assert constraint.active
  return constraint


def test_fixed_world_endpoint_qualifies_and_sets_bounded_ceiling():
  controller = StopIntentController()
  car_state = CarState(v_ego=10.0)

  constraint = qualify(controller, car_state)

  control_distance = constraint.distance_m - controller.ENDPOINT_SETTLE_MARGIN - car_state.vEgo * controller.ACTUATION_LOOKAHEAD
  assert constraint.state == StopIntentState.approach
  assert constraint.accel_ceiling_mps2 == pytest.approx(-(car_state.vEgo**2) / (2.0 * control_distance))
  assert -controller.MAX_REQUIRED_DECEL <= constraint.accel_ceiling_mps2 < 0.0


def test_curved_path_uses_arc_length():
  controller = StopIntentController()
  constraint = controller.update(Model(10.0, curve=1.0), CarState(v_ego=1.0), enabled=True, lead_present=False)

  assert constraint.state == StopIntentState.qualifying
  assert constraint.distance_m == pytest.approx(math.sqrt(200.0))


@pytest.mark.parametrize('target_world_speed', [1.0, 2.0, 4.0])
def test_moving_world_endpoint_never_commits(target_world_speed):
  controller = StopIntentController()
  car_state = CarState(v_ego=10.0)

  for frame in range(40):
    distance = 60.0 - (car_state.vEgo - target_world_speed) * frame * DT_MDL
    constraint = controller.update(Model(distance), car_state, enabled=True, lead_present=False)

  assert not constraint.active


@pytest.mark.parametrize('v_ego', [0.0, 2.0, 4.0])
def test_rolling_constant_horizon_never_commits(v_ego):
  controller = StopIntentController()
  car_state = CarState(v_ego=v_ego, standstill=v_ego == 0.0)

  for _ in range(80):
    constraint = controller.update(Model(12.0), car_state, enabled=True, lead_present=False)

  assert not constraint.active


@pytest.mark.parametrize(('v_ego', 'terminal_speed'), [(10.0, 2.0), (20.0, 5.0)])
def test_speed_reduction_trajectory_is_not_promoted_to_stop(v_ego, terminal_speed):
  controller = StopIntentController()
  constraint = run_fixed_target(controller, CarState(v_ego=v_ego), 5.0 * v_ego, 30, terminal_speed=terminal_speed)

  assert constraint.state == StopIntentState.clear


def test_terminal_velocity_dip_and_turn_are_rejected():
  controller = StopIntentController()
  car_state = CarState()
  dip = Model(50.0, terminal_speed=5.0)
  dip.velocity.x[-1] = 0.0

  for _ in range(20):
    dip_constraint = controller.update(dip, car_state, enabled=True, lead_present=False)
  controller.reset()
  for _ in range(20):
    turn_constraint = controller.update(
      Model(50.0, terminal_lateral_speed=8.0),
      car_state,
      enabled=True,
      lead_present=False,
    )

  assert dip_constraint.state == StopIntentState.clear
  assert turn_constraint.state == StopIntentState.clear


def test_gentle_low_speed_stop_can_qualify():
  controller = StopIntentController()
  constraint = run_fixed_target(controller, CarState(v_ego=3.0), 15.0, 30, desired_accel=-0.35)

  assert constraint.state == StopIntentState.approach


def test_acquisition_reserves_qualification_and_actuator_distance():
  controller = StopIntentController()
  v_ego = 8.0
  reserve = max(v_ego * controller.QUALIFY_TIME, controller.WORLD_ERROR_MIN + controller.QUALIFY_TRAVEL_MARGIN)
  minimum = controller.ENDPOINT_SETTLE_MARGIN + v_ego * controller.ACTUATION_LOOKAHEAD + v_ego**2 / (2.0 * controller.MAX_REQUIRED_DECEL) + reserve

  assert controller._feasible(minimum + 1e-6, v_ego, reserve)
  assert not controller._feasible(minimum - 1e-6, v_ego, reserve)
  assert not controller._feasible(150.0, controller.MAX_APPROACH_SPEED + 0.1, reserve)


def test_lead_or_invalid_model_blocks_acquisition():
  controller = StopIntentController()
  car_state = CarState()

  for frame in range(30):
    distance = 60.0 - car_state.vEgo * frame * DT_MDL
    lead_constraint = controller.update(Model(distance), car_state, enabled=True, lead_present=True)
  controller.reset()
  for _ in range(30):
    invalid_constraint = controller.update(Model(valid=False), car_state, enabled=True, lead_present=False)

  assert lead_constraint.state == StopIntentState.clear
  assert invalid_constraint.state == StopIntentState.clear


def test_committed_target_survives_dropout_and_accepts_closer_revision():
  controller = StopIntentController()
  car_state = CarState()
  constraint = qualify(controller, car_state)
  distance_before_dropout = constraint.distance_m

  for _ in range(4):
    constraint = controller.update(Model(), car_state, enabled=True, lead_present=False, observation_valid=False)
  assert constraint.distance_m == pytest.approx(distance_before_dropout - 2.0)

  closer_distance = constraint.distance_m - 8.0
  constraint = controller.update(Model(closer_distance), car_state, enabled=True, lead_present=False)
  assert constraint.distance_m == pytest.approx(closer_distance)


def test_committed_target_cancels_only_on_sustained_clear_evidence():
  controller = StopIntentController()
  car_state = CarState()
  constraint = qualify(controller, car_state)

  for _ in range(9):
    constraint = controller.update(
      Model(max(0.1, constraint.distance_m - car_state.vEgo * DT_MDL), desired_accel=0.0),
      car_state,
      enabled=True,
      lead_present=False,
    )
  assert constraint.active

  constraint = controller.update(Model(constraint.distance_m, desired_accel=0.0), car_state, enabled=True, lead_present=False)
  assert constraint.state == StopIntentState.clear


def test_early_standstill_latches_hold_and_gap_does_not_release_it():
  controller = StopIntentController()
  car_state = CarState()
  constraint = qualify(controller, car_state)
  assert constraint.distance_m > controller.HOLD_DISTANCE

  car_state.vEgo = 0.0
  car_state.standstill = True
  constraint = controller.update(Model(constraint.distance_m), car_state, enabled=True, lead_present=True)
  assert constraint.state == StopIntentState.hold

  for _ in range(20):
    constraint = controller.update(Model(10.0, should_stop=True), car_state, enabled=True, lead_present=False)
  constraint = controller.update(Model(10.0, desired_accel=0.0), car_state, enabled=True, lead_present=False, dt=0.5)
  assert constraint.state == StopIntentState.hold


def test_gas_immediately_suppresses_committed_stop():
  controller = StopIntentController()
  car_state = CarState()
  qualify(controller, car_state)

  car_state.gasPressed = True
  constraint = controller.update(Model(), car_state, enabled=True, lead_present=False)

  assert constraint.state == StopIntentState.suppressed
  assert not constraint.active
