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

# There is no speed floor. A 20 km/h one sat here and ruled out the junction turns this is
# for: replaying the 09-16 drive, removing it adds 461 frames of entering, 316 of them
# between 15 and 20 km/h at a median curvature of 0.0498 with the indicator on - signalled
# turns into junctions, predicted lateral acceleration 2.22 against a threshold of 1.79.
#
# Nothing is needed in its place. Whether a corner warrants slowing is already decided by
# v_target, which is what this corner allows at _A_LAT_REG_MAX, and below it update() returns
# zero. On the same replay that test alone removes every low-speed frame: of the frames that
# cross the entering threshold under 10 km/h, the number where v_ego actually exceeds
# v_target is zero - at 5 km/h the corner ahead allows 9.5, at 12 it allows 14.5. A speed
# floor is a second guess at a question v_target answers exactly.
V_FLOOR = 15 * CV.KPH_TO_MS  # ...but once slowing, this is as far down as it goes
PARAMS_UPDATE_PERIOD = 3.   # seconds

_ENTERING_PRED_LAT_ACC_TH = 1.3        # predicted lat acc that starts the entering state
_ABORT_ENTERING_PRED_LAT_ACC_TH = 1.1  # drop below this and the corner was a false alarm
_TURNING_LAT_ACC_TH = 1.6              # actual lat acc that means we are in the corner
_LEAVING_LAT_ACC_TH = 1.3              # falling below this means the corner is opening up
_FINISH_LAT_ACC_TH = 1.1               # and below this it is over
_A_LAT_REG_MAX = 2.                    # most lateral acceleration we are willing to pull
# ...but how much that is depends on the speed, and the 1.5x that sat here below 36 km/h was
# written from the guess that 3 m/s2 is an ordinary turn into a junction. Measured instead:
# over the 09-18 drives, at junctions taken with the indicator on and more than 45 degrees of
# steering, the driver pulls a median of 0.89 m/s2 and a 90th percentile of 1.10. Openpilot
# on the same corners pulls 1.20. The ceiling those thresholds were scaled to was 2.72 -
# three times what the driver actually uses - so v_target came out above the speed the car
# was already doing and the entering state handed back _LEAVING_ACC instead of braking. On
# 1451 frames of signalled junction turns, 54.6% entered and 0.3% asked for any deceleration:
# the car accelerates through the corner, which is what the driver reported.
# Scaled to the driver's own numbers a junction is allowed 1.2-1.5 m/s2, which puts v_target
# just under the speed these corners are actually taken at. Nothing above 72 km/h moves:
# there is no measurement of this driver on fast sweepers to justify touching it.
_LAT_TOL_BP = [0., 10., 20.]           # m/s
_LAT_TOL_V = [0.6, 0.75, 1.0]

# Smooth deceleration on the way in, by how sharp the corner ahead looks
_ENTERING_SMOOTH_DECEL_V = [-0.2, -1.]
_ENTERING_SMOOTH_DECEL_BP = [1.3, 3.]
_LEAVING_ACC = 0.5                     # comfortable pull back up to speed on the way out
_APPROACH_TC = 4.0                     # s: under v_target on the way in, close on it this gently

# v_target is shown on the HUD beside MAX, but only the entering state ever looked at it: turning
# took its acceleration from how hard the car was pulling alone, never lower than -0.4, and
# leaving always handed back +0.5. So the display could say 33 while the car held 51. On the
# 09-23 drive (route 39 seg 26, 13:37) a corner seen late was entered at 55 km/h: turning sat on
# -0.4 for seven seconds, the car pulled 3.5 m/s^2 of lateral acceleration, v_target went down to
# 33 and the car only to 46, and when the corner eased the table went positive while the wheel
# was still at -30 degrees - the driver had to take it, close to the outside of the bend.
# In every state now: over v_target by more than the margin, slow toward it, closing the gap in
# about _OVERSPEED_TC; over it at all, no acceleration. v_target itself is unchanged - it is
# already looser than this driver: 1.39 allowed at 15-30 km/h against his own median of 1.39,
# 1.52 at 30-45 against 1.23. Openpilot's own corners at 45-60 km/h pull 1.71 at the median and
# 2.80 at the 90th percentile against 1.73 allowed; it is that top end this reaches.
_OVERSPEED_MARGIN = 2 * CV.KPH_TO_MS
_OVERSPEED_TC = 2.0                    # s
_OVERSPEED_A_MIN = -1.5                # m/s^2


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
        if self.max_pred_lat_acc >= _ENTERING_PRED_LAT_ACC_TH * tol:
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
    a = self._state_solution()
    if self.state in ACTIVE_STATES and self.v_target > 0.:
      over = self.v_ego - self.v_target
      if over > 0.:
        a = min(a, 0.)                                        # never speed up over the target
      if over > _OVERSPEED_MARGIN:
        a = min(a, max(-over / _OVERSPEED_TC, _OVERSPEED_A_MIN))
    return float(a)

  def _state_solution(self) -> float:
    tol = self._lat_tol()
    if self.state not in ACTIVE_STATES:
      return self.a_ego
    if self.state == CurveState.entering:
      # v_target is the speed this corner allows at _A_LAT_REG_MAX, so at or below it the
      # corner is already inside what we are willing to pull and there is nothing left to
      # give up. It was being computed and then only shown on the display, never used to
      # call the braking off, so the entering state kept asking for deceleration until
      # max_pred_lat_acc fell under _ABORT_ENTERING_PRED_LAT_ACC_TH - barely half of
      # _A_LAT_REG_MAX. Measured over seven corners on 2026-09-09: braking stopped at a
      # median 74% of the lateral acceleration allowed, with the car pulling only 49% of
      # it, costing 2-12 km/h a corner. Hand back _LEAVING_ACC rather than zero: a_cruise
      # is 0.35-0.44 at these speeds, so zero would keep blocking ordinary acceleration
      # once the braking is done.
      # 09-23, the driver's rule for a corner: slow before it, neither gain nor lose speed in
      # it, pick up only on the way out. So under v_target on the way in, close on it gently
      # and stop gaining as it is reached, rather than a flat +0.5 into the bend.
      if self.v_ego <= self.v_target:
        return float(np.clip((self.v_target - self.v_ego) / _APPROACH_TC, 0.0, _LEAVING_ACC))
      return float(np.interp(self.max_pred_lat_acc / tol, _ENTERING_SMOOTH_DECEL_BP, _ENTERING_SMOOTH_DECEL_V))
    if self.state == CurveState.turning:
      # hold the speed. This used to come off a table of how hard the car was pulling, which
      # went positive as soon as the wheel came back a little (+0.5 under 1.5 m/s^2 x tol) -
      # the car sped up mid-corner. Over v_target, _update_solution still slows it.
      return 0.0
    return _LEAVING_ACC   # leaving: the corner is opening, pick the speed back up

  def update(self, sm, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego

    self._update_params()
    self._update_calculations(sm)
    self.is_active = self._update_state_machine()
    self.a_target = self._update_solution()

    # sunnypilot's controller returns a speed and floors it at a minimum. This
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
