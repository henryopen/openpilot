"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import numpy as np
import pytest

import cereal.messaging as messaging
from cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP, LongitudinalPlanSource
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import (
  _ACCEL_RELEASE_RATE, _ENTERING_PRED_LAT_ACC_TH, _RELIEF_CONFIRMATION_FRAMES, _TARGET_RELEASE_RATE, SmartCruiseControlVision,
)

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState


def _th_above_f32(th: float) -> float:
  """
  Return the next representable float32 *above* `th`.
  This avoids flaky comparisons around thresholds due to float32 rounding.
  """
  th32 = np.float32(th)
  above32 = np.nextafter(th32, np.float32(np.inf), dtype=np.float32)
  return float(above32)


def _build_single_spike_filtered(n: int, base: float = 1.0) -> np.ndarray:
  """
  Create an array where max() is >= threshold but p97 is < threshold.
  This demonstrates the behavior difference vs np.amax().

  Note: We intentionally construct using float32-representable values to match
  the data path through cereal/capnp.
  """
  th = float(_ENTERING_PRED_LAT_ACC_TH)
  th32 = float(np.float32(th))

  # numpy percentile default is linear interpolation: idx=(n-1)*p/100
  idx = (n - 1) * 0.97
  w = float(idx - np.floor(idx))

  base32 = float(np.float32(base))

  # Choose spike so that p97 = base + w*(spike-base) < th
  # -> spike < base + (th-base)/w. Use a margin (0.9) and ensure spike >= th.
  if w == 0.0:
    spike = th32 + 1.0
  else:
    spike = base32 + (th32 - base32) / w * 0.9
    spike = max(spike, th32 + 0.01)

  arr = np.full(n, base32, dtype=np.float32)
  arr[-1] = np.float32(spike)
  return arr


def generate_modelV2():
  model = messaging.new_message('modelV2')
  position = log.XYZTData.new_message()
  speed = 30
  position.x = [float(x) for x in (speed + 0.5) * np.array(ModelConstants.T_IDXS)]
  model.modelV2.position = position
  orientation = log.XYZTData.new_message()
  curvature = 0.05
  orientation.x = [float(curvature) for _ in ModelConstants.T_IDXS]
  orientation.y = [0.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.orientation = orientation
  orientationRate = log.XYZTData.new_message()
  orientationRate.z = [float(z) for z in ModelConstants.T_IDXS]
  model.modelV2.orientationRate = orientationRate
  velocity = log.XYZTData.new_message()
  velocity.x = [float(x) for x in (speed + 0.5) * np.ones_like(ModelConstants.T_IDXS)]
  velocity.x[0] = float(speed)  # always start at current speed
  model.modelV2.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = [float(x) for x in np.zeros_like(ModelConstants.T_IDXS)]
  acceleration.y = [float(y) for y in np.zeros_like(ModelConstants.T_IDXS)]
  model.modelV2.acceleration = acceleration

  return model


def generate_carState():
  car_state = messaging.new_message('carState')
  speed = 30
  v_cruise = 50
  car_state.carState.vEgo = float(speed)
  car_state.carState.standstill = False
  car_state.carState.vCruise = float(v_cruise * 3.6)

  return car_state


def generate_controlsState():
  controls_state = messaging.new_message('controlsState')
  controls_state.controlsState.curvature = 0.05

  return controls_state


class TestSmartCruiseControlVision:

  def setup_method(self):
    self.params = Params()
    self.reset_params()
    self.scc_v = SmartCruiseControlVision()

    mdl = generate_modelV2()
    cs = generate_carState()
    controls_state = generate_controlsState()
    self.sm = {'modelV2': mdl.modelV2, 'carState': cs.carState, 'controlsState': controls_state.controlsState}

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlVision", True, block=True)

  def set_lat_accels(self, current: float, predicted: float) -> None:
    v_ego = 20.
    self.sm['controlsState'].curvature = current / v_ego**2
    self.sm['modelV2'].velocity.x = [1.] * len(ModelConstants.T_IDXS)
    self.sm['modelV2'].orientationRate.z = [predicted] * len(ModelConstants.T_IDXS)

  def update_lat_accels(self, current: float, predicted: float, cruise: float = 30., a_ego: float = 0.) -> None:
    self.set_lat_accels(current, predicted)
    self.scc_v.update(self.sm, True, False, 20., a_ego, cruise)

  def enter_curve(self, predicted: float = 2.2) -> None:
    self.update_lat_accels(0.5, predicted)
    self.update_lat_accels(0.5, predicted)
    assert self.scc_v.state == VisionState.entering

  def test_initial_state(self):
    assert self.scc_v.state == VisionState.disabled
    assert not self.scc_v.is_active
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert self.scc_v.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlVision", False, block=True)
    self.scc_v.enabled = self.params.get_bool("SmartCruiseControlVision")

    for _ in range(int(10. / DT_MDL)):
      self.scc_v.update(self.sm, True, False, 0., 0., 0.)
    assert self.scc_v.state == VisionState.disabled
    assert not self.scc_v.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_v.update(self.sm, False, False, 0., 0., 0.)
    assert self.scc_v.state == VisionState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_v.update(self.sm, True, False, 0., 0., 0.)
    assert self.scc_v.state == VisionState.enabled

  def test_unconfirmed_leaving_and_reentry_never_request_propulsion(self):
    self.enter_curve()
    targets = [(self.scc_v.output_v_target, self.scc_v.output_a_target)]
    assert targets[-1][1] < 0.

    self.update_lat_accels(2., 2.2)
    assert self.scc_v.state == VisionState.turning
    targets.append((self.scc_v.output_v_target, self.scc_v.output_a_target))

    self.update_lat_accels(1.2, 1.2)
    assert self.scc_v.state == VisionState.leaving
    targets.append((self.scc_v.output_v_target, self.scc_v.output_a_target))

    self.update_lat_accels(1., 3.)
    assert self.scc_v.state == VisionState.entering
    targets.append((self.scc_v.output_v_target, self.scc_v.output_a_target))

    v_targets, a_targets = np.array(targets).T
    assert np.all(np.diff(v_targets[:-1]) <= 0.)
    assert v_targets[-1] < v_targets[-2]
    assert np.all(a_targets < 0.)
    assert np.all(np.diff(a_targets[:-1]) <= 0.)
    assert a_targets[-1] < a_targets[-2]

  def test_new_curve_interrupts_confirmed_release_immediately(self):
    self.enter_curve()
    for _ in range(_RELIEF_CONFIRMATION_FRAMES + 1):
      self.update_lat_accels(0.8, 0.8)
    releasing_v_target = self.scc_v.output_v_target
    releasing_a_target = self.scc_v.output_a_target
    assert self.scc_v.state == VisionState.leaving

    self.update_lat_accels(0.8, 3.)
    assert self.scc_v.state == VisionState.entering
    assert self.scc_v.output_v_target < releasing_v_target
    assert self.scc_v.output_a_target < releasing_a_target

  def test_jitter_requires_confirmed_relief_then_releases_smoothly(self):
    self.enter_curve()
    held_v_target = self.scc_v.output_v_target
    held_a_target = self.scc_v.output_a_target

    for frame in range(_RELIEF_CONFIRMATION_FRAMES * 2):
      self.update_lat_accels(1., 1.05 if frame % 2 == 0 else 1.15)
      assert self.scc_v.state == VisionState.entering
      assert self.scc_v.output_v_target == held_v_target
      assert self.scc_v.output_a_target == held_a_target

    for _ in range(_RELIEF_CONFIRMATION_FRAMES):
      self.update_lat_accels(1.15, 0.8)
      assert self.scc_v.state == VisionState.entering
      assert self.scc_v.output_v_target == held_v_target
      assert self.scc_v.output_a_target == held_a_target

    release_cruise = held_v_target + 2.5 * _TARGET_RELEASE_RATE * DT_MDL
    for _ in range(_RELIEF_CONFIRMATION_FRAMES - 1):
      self.update_lat_accels(0.8, 0.8, release_cruise)
      assert self.scc_v.state == VisionState.entering
      assert self.scc_v.output_v_target == held_v_target
      assert self.scc_v.output_a_target == held_a_target

    active_v_targets = [held_v_target]
    active_a_targets = [held_a_target]
    for _ in range(_RELIEF_CONFIRMATION_FRAMES + 10):
      self.update_lat_accels(0.8, 0.8, release_cruise)
      if not self.scc_v.is_active:
        break
      assert self.scc_v.state == VisionState.leaving
      assert self.scc_v.output_v_target != V_CRUISE_UNSET
      active_v_targets.append(self.scc_v.output_v_target)
      active_a_targets.append(self.scc_v.output_a_target)

    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert active_v_targets[-1] == pytest.approx(release_cruise)
    assert np.all((np.diff(active_v_targets) >= 0.) & (np.diff(active_v_targets) <= _TARGET_RELEASE_RATE * DT_MDL + 1e-9))
    assert np.all((np.diff(active_a_targets) >= 0.) & (np.diff(active_a_targets) <= _ACCEL_RELEASE_RATE * DT_MDL + 1e-9))
    emitted_a_targets = [*active_a_targets, self.scc_v.output_a_target]
    assert np.max(np.abs(np.diff(emitted_a_targets)) / DT_MDL) <= _ACCEL_RELEASE_RATE + 1e-9
    assert np.max(np.abs(np.diff(emitted_a_targets)) / DT_MDL) < 3.

  def test_negative_accel_handoff_is_continuous_through_planner_arbitration(self):
    car_control = messaging.new_message('carControl')
    car_control.carControl.enabled = True
    car_control.carControl.cruiseControl.override = False
    self.sm['carControl'] = car_control.carControl
    self.sm['carState'].vCruiseCluster = 108.

    planner = LongitudinalPlannerSP.__new__(LongitudinalPlannerSP)
    planner.scc = SimpleNamespace(
      vision=self.scc_v,
      map=SimpleNamespace(output_v_target=V_CRUISE_UNSET, output_a_target=0.),
      update=lambda sm, enabled, override, v_ego, a_ego, v_cruise: self.scc_v.update(
        sm, enabled, override, v_ego, a_ego, v_cruise),
    )
    planner.resolver = SimpleNamespace(
      speed_limit_valid=False, speed_limit_last_valid=False, speed_limit=0., speed_limit_final_last=0., distance=0.,
      update=lambda _v_ego, _sm: None,
    )
    planner.sla = SimpleNamespace(
      output_v_target=V_CRUISE_UNSET, output_a_target=0., update=lambda *_args: None,
    )
    planner.events_sp = SimpleNamespace()

    self.set_lat_accels(0.5, 2.2)
    planner.update_targets(self.sm, 20., 0., 30.)
    planner.update_targets(self.sm, 20., 0., 30.)
    assert planner.source == LongitudinalPlanSource.sccVision
    held_v_target = self.scc_v.output_v_target
    release_cruise = held_v_target + 0.5 * _TARGET_RELEASE_RATE * DT_MDL

    self.set_lat_accels(0.8, 0.8)
    for _ in range(_RELIEF_CONFIRMATION_FRAMES + 30):
      planner.update_targets(self.sm, 20., 0., release_cruise)
      if self.scc_v.relief_frames >= _RELIEF_CONFIRMATION_FRAMES and abs(self.scc_v.output_a_target) <= _ACCEL_RELEASE_RATE * DT_MDL:
        break

    assert self.scc_v.state == VisionState.leaving
    assert planner.source == LongitudinalPlanSource.sccVision
    assert self.scc_v.output_v_target < release_cruise

    prior_accel = planner.output_a_target
    assert prior_accel > -1.
    planner.update_targets(self.sm, 20., -1., release_cruise)
    assert self.scc_v.state == VisionState.leaving
    assert planner.source == LongitudinalPlanSource.sccVision
    assert planner.output_a_target == -1.
    assert planner.output_a_target < prior_accel
    assert self.scc_v.output_v_target < release_cruise

    active_accel = planner.output_a_target
    planner.update_targets(self.sm, 20., -1., release_cruise)
    assert self.scc_v.state == VisionState.leaving
    assert planner.source == LongitudinalPlanSource.cruise
    assert abs(planner.output_a_target - active_accel) / DT_MDL < 3.

    planner.update_targets(self.sm, 20., -1., release_cruise)
    assert self.scc_v.state == VisionState.enabled
    assert planner.source == LongitudinalPlanSource.cruise

  @pytest.mark.parametrize(
    "case, should_enter",
    [
      ("p97_just_above_threshold", True),
      ("single_spike_filtered", False),
      ("persistent_high_values", True),
    ],
    ids=[
      "p97>threshold_enters",
      "single_spike_max_large_but_p97_below_threshold",
      "high_values_persist_trigger_entering",
    ],
  )
  def test_max_pred_lat_acc_uses_p97_and_threshold(self, case, should_enter):
    n = len(ModelConstants.T_IDXS)
    th = float(_ENTERING_PRED_LAT_ACC_TH)

    if case == "p97_just_above_threshold":
      # Use the next representable float32 above threshold to avoid float32 rounding flakiness.
      val = _th_above_f32(th)
      pred_lat_accels = np.full(n, np.float32(val), dtype=np.float32)

    elif case == "single_spike_filtered":
      pred_lat_accels = _build_single_spike_filtered(n, base=1.0)

    elif case == "persistent_high_values":
      # Make enough "high" samples so p97 is driven by the persistent trend, not a single outlier.
      high_count = max(2, int(np.ceil(n * 0.03)) + 1)
      pred_lat_accels = np.full(n, np.float32(1.0), dtype=np.float32)
      pred_lat_accels[-high_count:] = np.float32(2.0)
      pred_lat_accels[-1] = np.float32(8.0)  # keep one big outlier too

    else:
      raise AssertionError(f"Unknown case: {case}")

    # Override model predictions so:
    # predicted_lat_accels = abs(orientationRate.z) * velocity.x == pred_lat_accels
    mdl = generate_modelV2()
    mdl.modelV2.velocity.x = [1.0 for _ in range(n)]
    mdl.modelV2.orientationRate.z = [float(x) for x in pred_lat_accels]
    self.sm["modelV2"] = mdl.modelV2

    v_ego = float(MIN_V + 5.0)

    # 1st update: disabled -> enabled
    self.scc_v.update(self.sm, True, False, v_ego, 0.0, 0.0)
    # 2nd update: evaluate entering condition from enabled state
    self.scc_v.update(self.sm, True, False, v_ego, 0.0, 0.0)

    # Controller does percentile on numpy float64 arrays (values already quantized by capnp),
    # so compute expected in float64 to match behavior and avoid interpolation/rounding deltas.
    expected_p97 = float(np.percentile(pred_lat_accels.astype(np.float64), 97))

    # allow tiny numeric differences due to float conversions/interpolation
    assert np.isclose(self.scc_v.max_pred_lat_acc, expected_p97, rtol=1e-6, atol=1e-5)

    if should_enter:
      # We assert entering primarily by state (this is the actual intended behavior).
      assert self.scc_v.state == VisionState.entering
      # Optional sanity: should be >= threshold with some margin (since we used nextafter above threshold).
      assert self.scc_v.max_pred_lat_acc > th

    else:
      # Difference vs np.amax(): max can be above threshold, but p97 stays below it.
      assert float(np.max(pred_lat_accels)) >= th
      assert self.scc_v.max_pred_lat_acc < th
      assert self.scc_v.state == VisionState.enabled

  # TODO-SP: mock modelV2 data to test other states
