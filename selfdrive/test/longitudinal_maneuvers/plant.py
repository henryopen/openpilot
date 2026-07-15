#!/usr/bin/env python3
from collections import deque
from collections.abc import Callable
import math
import time
from typing import Any

import numpy as np

from cereal import log
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper, DT_MDL
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU


LeadObservation = dict[str, Any]
LeadObservationFn = Callable[[float, str, LeadObservation], LeadObservation | None]
ModelActionFn = Callable[[float, float, float], tuple[float, bool]]


class Plant:
  messaging_initialized = False

  def __init__(
    self,
    lead_relevancy=False,
    speed=0.0,
    distance_lead=2.0,
    enabled=True,
    only_lead2=False,
    only_radar=False,
    e2e=False,
    personality=0,
    force_decel=False,
    lead_observation_fn: LeadObservationFn | None = None,
    model_action_fn: ModelActionFn | None = None,
    actuator_delay: float | None = None,
    actuator_lag: float = 0.0,
  ):
    """Closed-loop longitudinal planner plant.

    ``lead_observation_fn(time, lead_name, truth)`` may return a complete or partial
    observed LeadData mapping, or ``None`` for an absent lead. It is called separately
    for ``leadOne`` and ``leadTwo``. The supplied truth mapping is a copy, and observed
    values never affect the physical lead trajectory.

    ``model_action_fn(time, v_ego, a_ego)`` returns
    ``(desired_acceleration, should_stop)``.

    Passing ``actuator_delay`` both overrides ``CP.longitudinalActuatorDelay`` and
    adds the corresponding command transport delay to the plant. ``None`` keeps the
    historical Honda planner delay with instantaneous plant response. ``actuator_lag``
    is an optional first-order acceleration-response time constant. Both defaults keep
    historical plant dynamics unchanged.
    """
    if actuator_delay is not None and (not math.isfinite(actuator_delay) or actuator_delay < 0.0):
      raise ValueError("actuator_delay must be finite and non-negative")
    if not math.isfinite(actuator_lag) or actuator_lag < 0.0:
      raise ValueError("actuator_lag must be finite and non-negative")

    self.rate = 1.0 / DT_MDL

    if not Plant.messaging_initialized:
      Plant.radar = messaging.pub_sock('radarState')
      Plant.controls_state = messaging.pub_sock('controlsState')
      Plant.selfdrive_state = messaging.pub_sock('selfdriveState')
      Plant.car_state = messaging.pub_sock('carState')
      Plant.plan = messaging.sub_sock('longitudinalPlan')
      Plant.messaging_initialized = True

    self.v_lead_prev = 0.0

    self.distance = 0.0
    self.speed = speed
    self.should_stop = False
    self.acceleration = 0.0
    self.a_target = 0.0
    self.actuator_command = 0.0

    # lead car
    self.lead_relevancy = lead_relevancy
    self.distance_lead = distance_lead
    self.enabled = enabled
    self.only_lead2 = only_lead2
    self.only_radar = only_radar
    self.e2e = e2e
    self.personality = personality
    self.force_decel = force_decel
    self.lead_observation_fn = lead_observation_fn
    self.model_action_fn = model_action_fn
    self.actuator_delay = actuator_delay
    self.actuator_lag = actuator_lag
    self.publish_realized_a_ego = any((lead_observation_fn is not None, model_action_fn is not None, actuator_delay is not None, actuator_lag > 0.0))

    self.rk = Ratekeeper(self.rate, print_delay_threshold=100.0)
    self.ts = 1.0 / self.rate
    time.sleep(0.1)
    self.sm = messaging.SubMaster(['longitudinalPlan'])

    from opendbc.car.honda.values import CAR
    from opendbc.car.honda.interface import CarInterface

    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    if self.actuator_delay is not None:
      CP.longitudinalActuatorDelay = self.actuator_delay
    CP_SP = CarInterface.get_non_essential_params_sp(CP, CAR.HONDA_CIVIC)
    self.planner = LongitudinalPlanner(CP, CP_SP, init_v=self.speed)

    delay_steps = 0 if self.actuator_delay is None else round(self.actuator_delay / self.ts)
    self._actuator_delay_queue = deque([self.acceleration] * delay_steps)

  @property
  def current_time(self):
    return float(self.rk.frame) / self.rate

  @staticmethod
  def _lead_message(observation: LeadObservation):
    lead = log.RadarState.LeadData.new_message()
    for field, value in observation.items():
      setattr(lead, field, value)
    return lead

  def _observe_lead(self, lead_name: str, truth: LeadObservation, present_by_default: bool) -> LeadObservation | None:
    if self.lead_observation_fn is None:
      return dict(truth) if present_by_default else None

    observed = self.lead_observation_fn(self.current_time, lead_name, dict(truth))
    if observed is None:
      return None

    # Partial overrides are convenient for individual sensor glitches, while copying
    # from truth ensures every field written to cereal is deterministic.
    complete_observation = dict(truth)
    complete_observation.update(observed)
    return complete_observation

  def _update_actuator(self, command: float) -> tuple[float, float]:
    if self._actuator_delay_queue:
      self._actuator_delay_queue.append(command)
      delayed_command = self._actuator_delay_queue.popleft()
    else:
      delayed_command = command

    if self.actuator_lag > 0.0:
      alpha = 1.0 - math.exp(-self.ts / self.actuator_lag)
      self.acceleration += alpha * (delayed_command - self.acceleration)
    else:
      self.acceleration = delayed_command
    return delayed_command, self.acceleration

  def step(self, v_lead=0.0, prob_lead=1.0, v_cruise=50.0, pitch=0.0, prob_throttle=1.0):
    # ******** publish a fake model going straight and fake calibration ********
    # note that this is worst case for MPC, since model will delay long mpc by one time step
    radar = messaging.new_message('radarState')
    control = messaging.new_message('controlsState')
    ss = messaging.new_message('selfdriveState')
    car_state = messaging.new_message('carState')
    lp = messaging.new_message('liveParameters')
    car_control = messaging.new_message('carControl')
    model = messaging.new_message('modelV2')
    car_state_sp = messaging.new_message('carStateSP')
    live_map_data_sp = messaging.new_message('liveMapDataSP')
    gps_data = messaging.new_message('gpsLocation')
    a_lead = (v_lead - self.v_lead_prev) / self.ts
    self.v_lead_prev = v_lead

    if self.lead_relevancy:
      d_rel = np.maximum(0.0, self.distance_lead - self.distance)
      v_rel = v_lead - self.speed
      if self.only_radar:
        status = True
      elif prob_lead > 0.5:
        status = True
      else:
        status = False
    else:
      d_rel = 200.0
      v_rel = 0.0
      prob_lead = 0.0
      status = False

    truth_lead: LeadObservation = {
      "dRel": float(d_rel),
      "yRel": 0.0,
      "vRel": float(v_rel),
      "aRel": float(a_lead - self.acceleration),
      "vLead": float(v_lead),
      "dPath": 0.0,
      "vLat": 0.0,
      "vLeadK": float(v_lead),
      "aLeadK": float(a_lead),
      "fcw": False,
      "status": bool(status),
      # TODO use real radard logic for this
      "aLeadTau": float(_LEAD_ACCEL_TAU),
      "modelProb": float(prob_lead),
      "radar": bool(self.only_radar),
      "radarTrackId": -1,
    }
    lead_one_observation = self._observe_lead("leadOne", truth_lead, not self.only_lead2)
    lead_two_observation = self._observe_lead("leadTwo", truth_lead, True)
    if lead_one_observation is not None:
      radar.radarState.leadOne = self._lead_message(lead_one_observation)
    if lead_two_observation is not None:
      radar.radarState.leadTwo = self._lead_message(lead_two_observation)

    # Simulate model predicting slightly faster speed
    # this is to ensure lead policy is effective when model
    # does not predict slowdown in e2e mode
    position = log.XYZTData.new_message()
    position.x = [float(x) for x in (self.speed + 0.5) * np.array(ModelConstants.T_IDXS)]
    model.modelV2.position = position
    if self.model_action_fn is None:
      model_acceleration, model_should_stop = self.acceleration + 0.1, False
    else:
      model_acceleration, model_should_stop = self.model_action_fn(self.current_time, self.speed, self.acceleration)
    model.modelV2.action.desiredAcceleration = float(model_acceleration)
    model.modelV2.action.shouldStop = bool(model_should_stop)
    velocity = log.XYZTData.new_message()
    velocity.x = [float(x) for x in (self.speed + 0.5) * np.ones_like(ModelConstants.T_IDXS)]
    velocity.x[0] = float(self.speed)  # always start at current speed
    model.modelV2.velocity = velocity
    acceleration = log.XYZTData.new_message()
    acceleration.x = [float(x) for x in np.zeros_like(ModelConstants.T_IDXS)]
    model.modelV2.acceleration = acceleration
    model.modelV2.meta.disengagePredictions.gasPressProbs = [float(prob_throttle) for _ in range(6)]

    control.controlsState.longControlState = LongCtrlState.pid if self.enabled else LongCtrlState.off
    ss.selfdriveState.experimentalMode = self.e2e
    ss.selfdriveState.personality = self.personality
    control.controlsState.forceDecel = self.force_decel
    car_state.carState.vEgo = float(self.speed)
    published_a_ego = self.acceleration if self.publish_realized_a_ego else 0.0
    car_state.carState.aEgo = float(published_a_ego)
    car_state.carState.standstill = bool(self.speed < 0.01)
    car_state.carState.vCruise = float(v_cruise * 3.6)
    car_control.carControl.orientationNED = [0.0, float(pitch), 0.0]

    # ******** get controlsState messages for plotting ***
    sm = {
      'radarState': radar.radarState,
      'carState': car_state.carState,
      'carControl': car_control.carControl,
      'controlsState': control.controlsState,
      'selfdriveState': ss.selfdriveState,
      'liveParameters': lp.liveParameters,
      'modelV2': model.modelV2,
      'carStateSP': car_state_sp.carStateSP,
      'liveMapDataSP': live_map_data_sp.liveMapDataSP,
      'gpsLocation': gps_data.gpsLocation,
    }
    self.planner.update(sm)
    self.a_target = self.planner.output_a_target
    self.actuator_command = self.a_target
    if self.planner.output_should_stop:
      self.actuator_command = min(-0.5, self.actuator_command)
    delayed_actuator_command, _ = self._update_actuator(self.actuator_command)
    self.speed = self.speed + self.acceleration * self.ts
    self.should_stop = self.planner.output_should_stop
    fcw = self.planner.fcw
    self.distance_lead = self.distance_lead + v_lead * self.ts

    # ******** run the car ********
    # print(self.distance, speed)
    if self.speed <= 0:
      self.speed = 0
      self.acceleration = 0
    self.distance = self.distance + self.speed * self.ts

    # *** radar model ***
    if self.lead_relevancy:
      d_rel = np.maximum(0.0, self.distance_lead - self.distance)
      v_rel = v_lead - self.speed
    else:
      d_rel = 200.0
      v_rel = 0.0

    # print at 5hz
    # if (self.rk.frame % (self.rate // 5)) == 0:
    #   print("%2.2f sec   %6.2f m  %6.2f m/s  %6.2f m/s2   lead_rel: %6.2f m  %6.2f m/s"
    #         % (self.current_time, self.distance, self.speed, self.acceleration, d_rel, v_rel))

    # ******** update prevs ********
    self.rk.monitor_time()

    accel_controller_result = getattr(self.planner, "accel_controller_result", None)
    return {
      "distance": self.distance,
      "speed": self.speed,
      "acceleration": self.acceleration,
      "realized_acceleration": self.acceleration,
      "a_target": self.a_target,
      "actuator_command": self.actuator_command,
      "delayed_actuator_command": delayed_actuator_command,
      "published_a_ego": published_a_ego,
      "should_stop": self.should_stop,
      "distance_lead": self.distance_lead,
      "fcw": fcw,
      "mpc_source": self.planner.mpc.source,
      "dec_mode": self.planner.dec.mode(),
      "pace_cap": getattr(accel_controller_result, "target_speed", None),
      "base_target": getattr(accel_controller_result, "base_speed", None),
      "raw_energy_cap": getattr(accel_controller_result, "raw_energy_cap", None),
      "live_filtered_cap": getattr(accel_controller_result, "live_filtered_cap", None),
      "shadow_filtered_cap": getattr(accel_controller_result, "shadow_filtered_cap", None),
      "accel_controller_selected_lead": getattr(accel_controller_result, "selected_lead", None),
      "model_action": {
        "desiredAcceleration": float(model_acceleration),
        "shouldStop": bool(model_should_stop),
      },
      "truth_lead": dict(truth_lead),
      "lead_one_observation": None if lead_one_observation is None else dict(lead_one_observation),
      "lead_two_observation": None if lead_two_observation is None else dict(lead_two_observation),
    }


# simple engage in standalone mode
def plant_thread():
  plant = Plant()
  while 1:
    plant.step()


if __name__ == "__main__":
  plant_thread()
