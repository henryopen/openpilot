"""Slow down for a curve the model can see coming.

Ported from sunnypilot's smart cruise control (vision). The model's predicted yaw rate
over the path ahead gives the lateral acceleration we would pull at the current speed;
if that is more than is comfortable, ease off before the corner rather than braking in it.

The map-based half of that feature is deliberately not ported: it needs offline map
curvature and this car ran with it off.
"""
import os
from enum import IntEnum

import numpy as np

from openpilot.cereal import log
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

# ---- v2 (2026-09-27): take a corner the way people drive one, within what this car can steer.
# The driver: small corners too slow, big ones not made. Over 127 corners on 09-24..09-27 he
# pressed the accelerator in 26% of the 60-120 m ones, and in the ones he had to steer the
# controller was at 85% of its own request - the wheel could not wind in fast enough.
#
# How people drive a corner (Reymond et al. 2001, and the naturalistic curve studies): the lateral
# acceleration they accept falls with speed; they slow before the corner and stop slowing by the
# apex; from the apex, as it opens, they pick the speed back up. What this car can do, measured on
# the same drives: however fast the plan asks the lateral acceleration to rise, the car winds in at
# about 0.8 m/s^3 at most (0.77-0.82 when asked for 1-2).
#
# What was here does the opposite on three counts. Its allowance rises with speed (tol 0.6 -> 1.0,
# 1.2 at 0 km/h to 2.0 at 72). It reads the corner as the 97th percentile of the whole 10 s plan,
# which lands on a spike - 1.30x the curvature actually driven in 60-120 m corners. And it only
# ever looked at how sharp the corner is, never at how suddenly it arrives, which is what the
# steering cannot keep up with. It also brakes in pieces: a median of 5 on/off braking stretches a
# corner in a closed-loop replay, because the target jumps with every model frame.
#
# v2 reads the plan point by point: the curvature at each point ahead (3-point smoothed, K_CAL),
# the speed allowed there by the comfort curve and by the steering, and the deceleration needed
# to get down to it by then. It brakes once that is worth doing, at what it takes, and lets go as
# soon as it is not; holds through the corner to the apex; from there accelerates inside the g-g
# circle. Closed-loop replay of the 09-24..09-27 corners against the code above (46 road corners,
# 73 junction turns): apex speed in 60-120 m corners 44.7 -> 46.9 km/h, 120-300 m 58.8 -> 57.0;
# in the corners the driver took, lateral 1.96 -> 1.73 and the share asking the steering for more
# than 0.8 m/s^3 71% -> 43%; junction turns 16.8 -> 17.0 km/h at 1.56 -> 1.38 m/s^2; braking
# stretches per corner 5 -> 2. /data/curve_v2_off brings the code above back.
_V2_T_IDX = np.array([10.0 * (i / 32) ** 2 for i in range(33)])   # ModelConstants.T_IDXS
_V2_LAT_BP = [10., 20., 40., 60., 80., 100.]  # km/h
# falling with speed from 40 up, as people take corners; under 20 held down instead, because a
# tight turn is where the steering falls behind - the driver took the wheel on 95% of junction
# turns and takes them himself at about 1.0
_V2_LAT_V = [1.3, 1.5, 1.8, 1.7, 1.5, 1.3]   # m/s^2, felt (actual) lateral acceleration
_V2_K_CAL = 1.15            # plan curvature read this way vs driven, set so the apex lands on the comfort curve
_V2_J_MAX = 0.8             # m/s^3, how fast this car winds into a corner
_V2_J_PLAN_TO_DEMAND = 1 / 0.68   # the plan's own lateral jerk vs what the controller asks on arrival (median)
_V2_A_START = 1.0           # m/s^2 of needed deceleration before braking at all: late and firm, not long and light
_V2_A_STOP = 0.3            # braking, let go once less than this is needed
_V2_A_MAX = 1.6             # m/s^2, the most an ordinary driver uses for a corner
_V2_D_MARGIN_T = 0.8        # s: be down to speed this far before the point
_V2_A_TOTAL = 2.0           # m/s^2, longitudinal and lateral together on the way out
_V2_SLOW_TURN_V = 25 * CV.KPH_TO_MS   # below this, hold through the whole turn, not just to the apex
_V2_REQ_TAU_UP = 0.6        # s: the needed deceleration jumps up with the model frame - smooth that
_V2_REQ_TAU_DOWN = 0.15     # s: but let go quickly once it has been done
_V2_START_HOLD = 0.3        # s over _V2_A_START before braking
_V2_MIN_BRAKE = 1.0         # s: once braking, no acceleration for at least this long
_V2_OFF_FLAG = "/data/curve_v2_off"

# ---- the steering-rate limit reads the plan from 1.0 s out, not 0.5 (2026-10-01). The driver on 09-30,
# town and a mountain road: it keeps slowing in bends, down below what they need. On that drive 76% of
# the curve braking came from this limit, at a point a median 0.6 s / 7 m ahead - found as the car was
# already turning in, and braked for right there at the 1.6 cap, often to 10-25 km/h under what the
# comfort curve allowed; bends were taken at a median 72% of the comfort curve.
# Closed loop on the device (the whole planner; 28 bends with nothing in front, 09-26..09-30, openpilot
# in control throughout; the simulation puts the current code at 78% against 81% in the logs), apex
# lateral as a share of the comfort curve, median (P25-P75) / bends over it by more than 10% / braking
# a bend:  0.5 s 78% (72-96) / 2 / 2.5 s;  1.0 s 93% (77-103) / 3 / 2.2 s;  1.5 s 98% / 6;  2.0 s 108%
# (88-123) / 10 - a sharp bend only grows into the plan late (09-28), so not looking near at all misses
# it. _V2_J_MAX 0.8 -> 1.0 with 0.5 s did 89% / 3 / 2.4 s. The one bend 1.0 s adds over 110% is taken at
# the same speed as now (32.6 km/h); the difference is where the simulation puts the apex.
# Counting the speed the cruise law's hand-back still takes off before letting go was tried alongside
# and changed nothing measurable.
_V2_JERK_T_MIN = 1.0        # s

# ---- braking that builds up, the way a person brakes (2026-10-01). The driver, after 10-01: it brakes
# hard first and then lighter, and that is what is uncomfortable; it still feels like two things are in
# control. The command here went from nothing to the full need in one frame: over 10-01 a curve braking
# stretch reached its deepest a median 0.10 s in, the first 0.5 s at 0.99 of it, against 1.05 s and 0.59
# for the driver's own braking. So the braking may only deepen at _V2_ONSET_J, starts at _V2_A_START_RAMP
# instead of _V2_A_START, and the need is worked out with the road the build-up costs taken off (v * T/2,
# T = D/J). Letting go is unchanged - the cruise law hands it back at its own jerk.
# Closed loop on the device, 34 bends with nothing in front (09-26..10-01), now -> this: hard first 85%
# -> 6%, deepest at 0.05 -> 0.90 s, first 0.5 s at 0.99 -> 0.42 of the deepest; apex lateral at 89% ->
# 86% of the comfort curve and bends over it by >10% 4 -> 2; braking per bend 2.3 -> 2.6 s; changes of
# what is in control while braking 1.47 -> 1.06 a bend. Without counting the build-up's distance the ramp
# arrived faster (09-30 17:12 +4.5..7.7 km/h, 6-9 bends over); J 1.0 builds up slower still (deepest at
# 1.15 s) for the same apex; 2.0 is back to 12% hard-first.
_V2_ONSET_J = 1.5           # m/s^3; 0 turns the ramp off
_V2_A_START_RAMP = 0.6      # m/s^2 of needed deceleration before braking, with the ramp on

# ---- anticipation (2026-09-28): a sharp corner grows into the plan, so ease off before it is all there.
# 09-27 16:07, 66 km/h into an 86 m corner: at the apex the controller was at 87% of its request and the
# wheel at full torque for a second. The model had it 8 s out, but as a bend a fraction as sharp as it was.
# Over the 46 road corners of 09-24..09-27, what the plan puts at the apex's position, as a share of the
# curvature driven there, by seconds to the apex:
#                 8 s    6 s    5 s    4 s    3 s
#   R 60-120 m   0.35   0.62   0.76   0.84   0.96
#   R 120-300 m  0.53   0.84   0.91   0.94   0.95
# Not a bias in how far out the plan reads a bend (grouped by what it predicts at 60-160 m, what is driven
# there is 1.0-1.25x at the median) but how late a sharp one appears in it. So from 3 s out the curvature is
# taken as up to 1.8x what the plan shows, and all that may ask for is to stop gaining speed (needed
# deceleration over _FAR_LIFT) or a light one, _FAR_A_MAX at most. Braking harder stays with the near
# field above, which now goes to _V2_A_MAX_ANT once the corner is plainly there.
# Closed-loop replay of the same 46 corners, v2 -> this: over the comfort curve at the apex 28% -> 13%;
# asking the steering for more than 0.8 m/s^3 43% -> 35%; 16:07 apex lateral 1.93 -> 1.63 (comfort 1.79);
# apex speed R 60-120 48.8 -> 45.4 km/h (the rules before v2: 46.1), R 120-300 57.0 -> 54.8; 73 signalled
# junction turns 17.2 -> 16.7 km/h. Open loop over 158 min / 157 km of openpilot driving, the far field is
# stricter than v2 for 68 s with a corner within 8 s and 25 s without one, mostly 0.1-0.3 of lift for a
# second. /data/curve_far_off brings back v2 as it was on 09-27.
_FAR_T_BP = [3., 4., 5., 6., 7., 8.]      # s, time of the plan point
_FAR_GAIN = [1., 1.15, 1.25, 1.45, 1.6, 1.8]
_FAR_LIFT = 0.1           # m/s^2 of needed deceleration: stop gaining speed
_FAR_START = 0.3          # m/s^2: and past this, slow gently
_FAR_A_MAX = 0.5          # m/s^2
_V2_A_MAX_ANT = 2.0       # m/s^2, near-field braking with anticipation on (was _V2_A_MAX)
_FAR_OFF_FLAG = "/data/curve_far_off"

# ---- lane change (2026-09-28): a lane change is not a corner.
# While one is under way the plan carries the sideways move as an S of yaw rate, and that reads as a
# corner arriving fast - to v2 mostly through the steering-rate term. On the 09-28 drives 11 of 45 lane
# changes, 44 of them on a straight road, had this brake at the full 1.6 m/s^2 for 0.3-1.9 s: 10:58:56
# at 110 km/h lost 20 km/h moving right. So nothing here acts while the lane change is running or for
# _LC_HOLD after it; replayed over the same day that leaves 0 of 45 with any braking (1 s already does,
# 1 of 45 at 0 s) and every one of the 160535 frames away from a lane change unchanged.
_LC_HOLD = 2.0   # s
_LC_STATES = (log.LaneChangeState.laneChangeStarting, log.LaneChangeState.laneChangeFinishing)


class CurveV2:
  def __init__(self):
    self.anticipate = True
    self.reset()

  def reset(self) -> None:
    self.braking = False
    self.req_f = 0.
    self.over_t = 0.
    self.brake_t = 0.
    self.v_target = 0.
    self.a_cmd = 0.

  def update(self, x, k, vp, v_ego: float, lat_now: float, k_now: float) -> float:
    """x/k/vp: the plan's distance ahead, curvature and speed at each of its 33 points.
    Returns a ceiling on the longitudinal acceleration, np.inf where the corner has nothing to say."""
    kk = np.abs(k)
    ks = np.concatenate([kk[:1], (kk[:-2] + kk[1:-1] + kk[2:]) / 3, kk[-1:]]) / _V2_K_CAL
    kc = np.maximum(ks, 1e-4)
    # speed the comfort curve allows at each point; the allowance depends on the speed, so iterate
    vv = np.sqrt(np.interp(v_ego * CV.MS_TO_KPH, _V2_LAT_BP, _V2_LAT_V) / kc)
    for _ in range(2):
      vv = np.sqrt(np.interp(vv * CV.MS_TO_KPH, _V2_LAT_BP, _V2_LAT_V) / kc)
    v_lat = np.where(ks > 1e-4, vv, 99.)
    # speed the steering allows: the plan's lateral jerk where it is actually winding into a corner
    # (lateral >= 0.4 and rising, from _V2_JERK_T_MIN - see there), scaled to what will be asked on
    # arrival, and that falls with v^3
    alp = vp ** 2 * kk
    dj = np.zeros(33)
    dj[1:] = np.abs(np.diff(alp)) / np.maximum(np.diff(_V2_T_IDX), 0.05)
    rising = np.zeros(33, dtype=bool)
    rising[1:] = alp[1:] > alp[:-1]
    dj[~((alp >= 0.4) & rising & (_V2_T_IDX >= _V2_JERK_T_MIN))] = 0.
    j_dem = dj * _V2_J_PLAN_TO_DEMAND
    v_jerk = np.where(j_dem > 0.05, np.maximum(vp, 1.) * (_V2_J_MAX / np.maximum(j_dem, 1e-6)) ** (1 / 3), 99.)
    v_allow = np.maximum(np.minimum(v_lat, v_jerk), V_FLOOR)
    # the deceleration it takes to be down to that by each point
    d = np.maximum(x - _V2_D_MARGIN_T * v_ego, 0.5 * x)
    need = (v_ego ** 2 - v_allow ** 2) / (2 * np.maximum(d, 2.))
    if _V2_ONSET_J > 0.:
      # building up to a deceleration D at _V2_ONSET_J takes D / J seconds at half of it on average, which
      # costs about v * D / 2J of the distance - so ask as if that much less road were left
      t_ramp = np.clip(need, 0., _V2_A_MAX_ANT if self.anticipate else _V2_A_MAX) / _V2_ONSET_J
      need = (v_ego ** 2 - v_allow ** 2) / (2 * np.maximum(d - v_ego * t_ramp / 2., 2.))
    need[x < 2.] = 0.
    a_raw = float(need.max())
    tau = _V2_REQ_TAU_UP if a_raw > self.req_f else _V2_REQ_TAU_DOWN
    self.req_f += (a_raw - self.req_f) * min(DT_MDL / tau, 1.)
    a_req = max(self.req_f, 0.)
    ahead_pts = x > 0.5
    self.v_target = float(v_allow[ahead_pts].min()) if ahead_pts.any() else 99.
    # still tightening: sharper within the next 2 s than where we are now
    near = (_V2_T_IDX > 0.2) & (_V2_T_IDX <= 2.0)
    tightening = ks[near].max() > abs(k_now) * 1.05 and ks[near].max() > 0.004

    if self.braking:
      self.brake_t += DT_MDL
      self.braking = a_req > _V2_A_STOP or self.brake_t < _V2_MIN_BRAKE
    else:
      a_start = _V2_A_START_RAMP if _V2_ONSET_J > 0. else _V2_A_START
      self.over_t = self.over_t + DT_MDL if a_req >= a_start else 0.
      self.braking = self.over_t >= _V2_START_HOLD
      self.brake_t = 0.
      self.a_cmd = 0.

    if self.braking:
      target = min(a_req, _V2_A_MAX_ANT if self.anticipate else _V2_A_MAX)
      # deepen at most at _V2_ONSET_J; easing off is not held back
      self.a_cmd = min(target, self.a_cmd + _V2_ONSET_J * DT_MDL) if _V2_ONSET_J > 0. else target
      a = -self.a_cmd
    elif a_req > 0.25:
      a = 0.                     # a corner is coming that will need braking: stop gaining speed
    elif lat_now > 0.6 and (tightening or v_ego < _V2_SLOW_TURN_V):
      a = 0.                     # in the corner, apex still ahead: hold
    elif lat_now > 0.3:
      a = float(np.sqrt(max(_V2_A_TOTAL ** 2 - lat_now ** 2, 0.)))   # past the apex: out inside the g-g circle
    else:
      a = np.inf
    if self.anticipate:
      a = min(a, self._far(x, ks, v_ego))
    if v_ego <= V_FLOOR and a < 0.:
      a = 0.
    return a

  @staticmethod
  def _far(x, ks, v_ego: float) -> float:
    """From _FAR_T_BP[0] out: the corner taken as sharper than it looks, and only a lift or a light brake for it."""
    far = (_V2_T_IDX >= _FAR_T_BP[0]) & (ks > 1e-4)
    if not far.any():
      return np.inf
    kc = ks[far] * np.interp(_V2_T_IDX[far], _FAR_T_BP, _FAR_GAIN)
    vv = np.sqrt(np.interp(v_ego * CV.MS_TO_KPH, _V2_LAT_BP, _V2_LAT_V) / kc)
    for _ in range(2):
      vv = np.sqrt(np.interp(vv * CV.MS_TO_KPH, _V2_LAT_BP, _V2_LAT_V) / kc)
    vv = np.maximum(vv, V_FLOOR)
    d = np.maximum(x[far] - _V2_D_MARGIN_T * v_ego, 0.5 * x[far])
    need = float(((v_ego ** 2 - vv ** 2) / (2 * np.maximum(d, 2.))).max())
    if need > _FAR_START:
      return -min(need, _FAR_A_MAX)
    if need > _FAR_LIFT:
      return 0.
    return np.inf


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
    self.v2 = CurveV2()
    self.use_v2 = not os.path.exists(_V2_OFF_FLAG)
    self.lc_hold = 0.

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")
      use_v2 = not os.path.exists(_V2_OFF_FLAG)
      if use_v2 != self.use_v2:   # switched: start either one from scratch
        self.v2.reset()
        self.state = CurveState.disabled
      self.use_v2 = use_v2
      self.v2.anticipate = not os.path.exists(_FAR_OFF_FLAG)

  def _update_v2(self, sm) -> None:
    """v2: same outputs as the state machine - is_active, v_target, a_target."""
    self.is_active = False
    if not (self.enabled and self.long_enabled) or self.long_override:
      self.v2.reset()
      return
    m = sm['modelV2']
    x = np.array(m.position.x)
    vp = np.array(m.velocity.x)
    w = np.array(m.orientationRate.z)
    if len(x) != 33 or len(vp) != 33 or len(w) != 33:
      return
    k_now = float(sm['controlsState'].curvature)
    a = self.v2.update(x, w / np.maximum(vp, 1.), vp, self.v_ego, self.v_ego ** 2 * abs(k_now), k_now)
    if np.isfinite(a):
      self.is_active = True
      self.a_target = a
      self.v_target = self.v2.v_target

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
    if sm['modelV2'].meta.laneChangeState in _LC_STATES:
      self.lc_hold = _LC_HOLD
    else:
      self.lc_hold = max(self.lc_hold - DT_MDL, 0.)
    if self.lc_hold > 0.:
      self.v2.reset()
      if self.state in ACTIVE_STATES:
        self.state = CurveState.enabled
      self.is_active = False
      self.frame += 1
      return
    if self.use_v2:
      self._update_v2(sm)
      self.frame += 1
      return
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
