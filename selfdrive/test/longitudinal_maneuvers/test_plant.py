import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant


def test_full_lead_observation_is_independent_from_truth():
  callback_inputs = []

  def observe_lead(current_time, lead_name, truth):
    callback_inputs.append((current_time, lead_name, truth))
    if lead_name == "leadOne":
      return {
        "dRel": 12.5,
        "vRel": -4.0,
        "vLead": 6.0,
        "vLeadK": 5.5,
        "aLeadK": -1.25,
        "aLeadTau": 0.7,
        "status": True,
        "modelProb": 0.9,
        "radarTrackId": 42,
      }
    return None

  plant = Plant(lead_relevancy=True, speed=10.0, distance_lead=50.0, lead_observation_fn=observe_lead)
  result = plant.step(v_lead=8.0)

  assert [entry[1] for entry in callback_inputs] == ["leadOne", "leadTwo"]
  assert callback_inputs[0][2]["dRel"] == pytest.approx(50.0)
  assert result["truth_lead"]["dRel"] == pytest.approx(50.0)
  assert result["lead_one_observation"]["dRel"] == pytest.approx(12.5)
  assert result["lead_one_observation"]["radarTrackId"] == 42
  assert result["lead_two_observation"] is None
  assert result["distance_lead"] == pytest.approx(50.0 + 8.0 * DT_MDL)


def test_model_action_realized_acceleration_and_source_logging():
  def model_action(current_time, v_ego, a_ego):
    return -1.25, True

  plant = Plant(speed=10.0, e2e=True, force_decel=True, model_action_fn=model_action, actuator_lag=0.5)
  first = plant.step()
  second = plant.step()

  assert first["model_action"] == {"desiredAcceleration": -1.25, "shouldStop": True}
  assert first["published_a_ego"] == pytest.approx(0.0)
  assert second["published_a_ego"] == pytest.approx(first["realized_acceleration"])
  assert first["acceleration"] == first["realized_acceleration"]
  assert abs(first["realized_acceleration"]) < abs(first["actuator_command"])
  assert first["mpc_source"] is not None
  assert first["dec_mode"] in ("acc", "blended")
  assert "pace_cap" in first
  assert "raw_energy_cap" in first
  assert "live_filtered_cap" in first
  assert first["lead_one_observation"] is not None
  assert first["truth_lead"] == first["lead_one_observation"]


def test_configurable_transport_delay_and_first_order_lag():
  plant = Plant(speed=10.0, actuator_delay=2 * DT_MDL, actuator_lag=0.2)

  assert plant.planner.CP.longitudinalActuatorDelay == pytest.approx(2 * DT_MDL)
  delayed_commands = [plant._update_actuator(-1.0) for _ in range(3)]
  assert [command for command, _ in delayed_commands[:2]] == [0.0, 0.0]

  expected_acceleration = -(1.0 - math.exp(-DT_MDL / 0.2))
  assert delayed_commands[2][0] == -1.0
  assert delayed_commands[2][1] == pytest.approx(expected_acceleration)


@pytest.mark.parametrize(
  ("delay", "lag"),
  [(-0.1, 0.0), (float("nan"), 0.0), (float("inf"), 0.0), (None, -0.1), (None, float("nan")), (None, float("inf"))],
)
def test_invalid_actuator_dynamics(delay, lag):
  with pytest.raises(ValueError):
    Plant(actuator_delay=delay, actuator_lag=lag)
