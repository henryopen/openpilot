"""Slow down for a curve the model can see coming.

Ported from sunnypilot's smart cruise control (vision). The model's predicted yaw rate
over the path ahead gives the lateral acceleration we would pull at the current speed;
if that is more than is comfortable, ease off before the corner rather than braking in it.

The map-based half of that feature is deliberately not ported: it needs offline map
curvature and this car ran with it off.
"""
from enum import IntEnum

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

MIN_V = 20 * CV.KPH_TO_MS   # below this, do not start slowing for a corner at all
V_FLOOR = 15 * CV.KPH_TO_MS  # ...but once slowing, this is as far down as it goes
PARAMS_UPDATE_PERIOD = 3.   # seconds

_ENTERING_PRED_LAT_ACC_TH = 1.3        # predicted lat acc that starts the entering state
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # drop below this and the corner was a false alarm
_TURNING_LAT_ACC_TH = 1.6              # actual lat acc that means we are in the corner
_LEAVING_LAT_ACC_TH = 1.3              # falling below this means the corner is opening up
_FINISH_LAT_ACC_TH = 1.1               # and below this it is over
_A_LAT_REG_MAX = 2.                    # most lateral acceleration we are willing to pull
# ...but how much that is depends on the speed. 3 m/s2 is an ordinary turn into a junction
# and an unpleasant one on a sweeper, so every lateral-acceleration threshold below is
# scaled by this rather than fixed: 1.5x under 36 km/h, tapering to unchanged from 72 km/h
# up, so nothing about high speed cornering moves.
_LAT_TOL_BP = [0., 10., 20.]           # m/s
_LAT_TOL_V = [1.5, 1.25, 1.0]

# Smooth deceleration on the way in, by how sharp the corner ahead looks
_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]
# What to do mid corner, by how hard it is actually pulling
_TURNING_ACC_V = [0.5, 0., -0.4]
_TURNING_ACC_BP = [1.5, 2.3, 3.]
_LEAVING_ACC = 0.5                     # comfortable pull back up to speed on the way out


class CurveState(IntEnum):
  disabled = 0
  enabled = 1
  entering = 2
  turning = 3
  leaving = 4
  overriding = 5


ACTIVE_STATES = (CurveState.entering, CurveState.turning, CurveState.leaving)
ENABLED_STATES = (CurveState.enabled, CurveState.overriding, *ACTIVE_STATES)


class CurveSpeedControl:
  def __init__(self):
    self.params = Params()
    self.frame = -1
    self.enabled = self.params.get_bool("SmartCruiseControlVision")

    self.state = CurveState.disabled
    self.is_active = False
    self.long_enabled = False
    self.long_override = False
    self.v_ego = 0.
    self.a_ego = 0.
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.
    self.v_target = 0.
    self.a_target = 0.

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm) -> None:
    if not self.long_enabled:
      return

    rate_plan = np.abs(np.array(sm['modelV2'].orientationRate.z))
    vel_plan = np.array(sm['modelV2'].velocity.x)
    if not len(rate_plan) or not len(vel_plan):
      return

    self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

    # the worst of what the model says the path ahead will pull
    self.max_pred_lat_acc = np.percentile(rate_plan * vel_plan, 97)

    v_ego = max(self.v_ego, 0.1)
    max_curve = self.max_pred_lat_acc / (v_ego ** 2)
    if max_curve > 0:
      self.v_target = (_A_LAT_REG_MAX * self._lat_tol() / max_curve) ** 0.5

  def _lat_tol(self) -> float:
    """How much the lateral-acceleration thresholds are relaxed at this speed."""
    return float(np.interp(self.v_ego, _LAT_TOL_BP, _LAT_TOL_V))

  def _update_state_machine(self) -> bool:
    tol = self._lat_tol()
    if self.state != CurveState.disabled:
      # losing longitudinal control or the toggle always wins
      if not self.long_enabled or not self.enabled:
        self.state = CurveState.disabled
      elif self.long_override:
        self.state = CurveState.overriding

      elif self.state == CurveState.enabled:
        if self.v_ego > MIN_V and self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH * tol:
          self.state = CurveState.entering

      elif self.state == CurveState.overriding:
        if not self.long_override:
          self.state = CurveState.enabled

      elif self.state == CurveState.entering:
        if self.current_lat_acc >= _TURNING_LAT_ACC_TH * tol:
          self.state = CurveState.turning
        elif self.max_pred_lat_acc < _ABORT_ENTERING_PRED_LAT_ACC_TH * tol:
          self.state = CurveState.enabled

      elif self.state == CurveState.turning:
        if self.current_lat_acc <= _LEAVING_LAT_ACC_TH * tol:
          self.state = CurveState.leaving

      elif self.state == CurveState.leaving:
        if self.current_lat_acc >= _TURNING_LAT_ACC_TH * tol:
          self.state = CurveState.turning
        elif self.current_lat_acc < _FINISH_LAT_ACC_TH * tol:
          self.state = CurveState.enabled

    elif self.long_enabled and self.enabled:
      self.state = CurveState.overriding if self.long_override else CurveState.enabled

    return self.state in ACTIVE_STATES

  def _update_solution(self) -> float:
    tol = self._lat_tol()
    if self.state not in ACTIVE_STATES:
      return self.a_ego
    if self.state == CurveState.entering:
      return float(np.interp(self.max_pred_lat_acc / tol, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V))
    if self.state == CurveState.turning:
      return float(np.interp(self.current_lat_acc / tol, _TURNING_ACC_BP, _TURNING_ACC_V))
    return _LEAVING_ACC   # leaving

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego

    self._update_params()
    self._update_calculations(sm)
    self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    # sunnypilot's controller returns a speed and floors it: max(v_target, MIN_V). This
    # port returns an acceleration instead, and the floor did not come with it, so nothing
    # stopped it slowing all the way down - it only exits the corner when current_lat_acc
    # falls under _LEAVING_LAT_ACC_TH, and since that is v^2 * curvature, a junction had to
    # get down to about 10 km/h before it let go. The equivalent here is to stop asking for
    # deceleration at the floor. Kept separate from the entry threshold: starting to slow
    # and refusing to slow further are different questions, and tying them together would
    # add braking between 15 and 20 km/h that was never there.
    if self.v_ego <= V_FLOOR:
      self.a_target = max(self.a_target, 0.0)

    self.frame += 1
