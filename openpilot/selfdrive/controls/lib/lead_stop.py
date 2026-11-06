"""Stop behind a car that is stopping, at a distance the driver asked for.

The lead MPC already brakes for a lead, so why this exists: its target gap is a soft cost,
divided by (v_ego + 10), against an A_CHANGE_COST of 200. At a standstill one metre of gap
is worth 0.03 against 2.0 for moving the accel command by 0.1 - so the solver will trade
8.2 m of gap rather than change what it is asking for. Measured over 8 routes and 151
engaged minutes on 2026-09-14, it stops at a median 3.39 m with 35% of stops inside 3 m,
against 4.61 m when the driver drives the same roads himself. The target it is working to
says 7.0. It is not converging on it, and neither STOP_DISTANCE (6.0 -> 7.0) nor
COMFORT_BRAKE (2.0 -> 1.5) moved it, because both only move that same soft target.

So this is not another target. It is the deceleration the remaining distance actually needs:

    a_need = -(v_ego^2 - v_lead^2) / (2 * (dRel - TARGET_GAP))

a law in the distance rather than in the speed, which is the same reason stop_for_lights
could not be built out of the cruise law - 36 of 36 approaches overshot the line, a median
8.6 m, because a speed law's authority fades exactly as the distance runs out. This one
grows as the distance runs out, so where it ends is arithmetic rather than a hope.

Three parts, and they are separate on purpose (2026-09-14, driver):
  1. deciding the car ahead is stopping - a detection question
  2. the curve that takes us to it - one smooth law, armed early so it stays gentle
  3. what happens when gentle is no longer enough - it brakes hard, because the
     alternative is hitting the car

On (1): the gate is the lead's speed, and how early we notice decides everything about how
hard we have to brake. Over the 22 stops the car made for itself that day, waiting for the
lead to be fully stopped (< 0.5 m/s) arms a median 6.1 s out and has asked for as much as
-4.21 by then; taking it at < 3.0 m/s arms 10.1 s out and the worst case falls to -2.74.
Four extra seconds is the difference between a firm stop and standing on the brake. Going
further to 4.0 buys one more stop out of 22 and starts reaching towards cars that are simply
driving slowly, so the gate sits at 3.0.

On (3): there is deliberately no comfort cap here. junction_handoff's floor caps at 2.5
because it is acting on a guess - the model's own idea that a junction is coming - and the
worst a wrong guess should do is slow the car firmly. This one is acting on a radar return
from a car that is nearly stopped, and if the arithmetic says it takes 3.2 m/s^2 to stop
behind it, then capping at 2.5 does not make the stop gentler, it makes it not happen. The
only bound is what the car can physically do.
"""
import numpy as np

from opendbc.car.interfaces import ACCEL_MIN
from openpilot.common.realtime import DT_MDL

# (1) Deciding the lead is stopping. Hysteresis because a single threshold on vLead chatters
# - replaying the first version of this, 13.7% of the frames it held down were leads that
# were actually moving, from vLead crossing back and forth over one line.
LEAD_STILL_ON = 3.0         # m/s, 10.8 km/h - arm here
LEAD_STILL_OFF = 4.5        # and let go here
LEAD_VLEAD_TAU = 0.3        # smooth vLead first; the radar's own value is noisy

# Only a radar lead. Vision distance is not good enough to divide by, and vision leads do not
# go through the Kalman filter, so their aLeadK reads ~0 even under real braking.
ARM_MAX_GAP = 50.0          # beyond this a stopped car is not yet our problem
ARM_MIN_SPEED = 1.0         # m/s

# (2) The curve. TARGET_GAP is where the car should come to rest, measured the way the radar
# measures it: dRel reads a systematic 0.85 m shorter than the real gap (verified against
# video on 2026-09-08 using number plates as a scale), so 5.0 here is about 5.8 m to the eye,
# which is what the driver asked for.
TARGET_GAP = 5.0
MIN_GEOMETRY_GAP = 0.5      # closer than this to the target, the division stops meaning anything

# Inside the target gap the question changes. Asking for the target is no longer the point -
# it is behind us - so the law re-aims at not touching the car, and the distance it divides
# by is the real one rather than the one to a target already passed. Without this the first
# version asked for ACCEL_MIN whenever the gap was inside 5.5 m and we were still rolling,
# which is a description of every traffic queue: replayed over 8 routes it was most of the
# 23.5% of held-down frames that had no stop behind them, and 4 of 15 stops hit -3.50.
HARD_MIN_GAP = 2.0

# Ramped so arming is not a step at the wheels. Two rates: the normal one matches
# junction_handoff's floor, and once the geometry is asking for real braking the ramp opens
# up, because a rate limit that smooths an emergency is a rate limit that causes one.
FLOOR_JERK = 4.0            # m/s3
FLOOR_JERK_URGENT = 8.0
URGENT_DECEL = 2.0          # above this, use the faster ramp


class LeadStop:
  def __init__(self):
    self.armed = False
    self.active = False
    self.a_floor = 0.0
    self.v_lead_filtered = 0.0
    self._init = False

  def reset(self):
    self.armed = False
    self.active = False
    self.a_floor = 0.0
    self._init = False

  def _update_lead_speed(self, v_lead):
    if not self._init:
      self.v_lead_filtered = v_lead
      self._init = True
    else:
      alpha = 1.0 - np.exp(-DT_MDL / LEAD_VLEAD_TAU)
      self.v_lead_filtered += alpha * (v_lead - self.v_lead_filtered)
    return self.v_lead_filtered

  def update(self, lead, v_ego, a_prev):
    """Returns the most positive acceleration allowed while closing on a stopping lead.

    a_prev is what the planner put out last frame. The ramp starts from there rather than
    from zero, because the caller takes min(this, its own plan) and a floor that appears at
    -0.20 while the plan is at +0.73 is a 19 m/s3 step at the wheels - measured, on 23 of
    the 36 worst frames of the first version. Starting from the output makes taking over a
    ramp rather than a step, and FLOOR_JERK then bounds it.
    """
    usable = lead is not None and bool(lead.present) and bool(lead.radar)
    if not usable or v_ego < ARM_MIN_SPEED:
      self.armed = False
      self._init = False
      self._ramp(0.0, a_prev, FLOOR_JERK)
      return self.a_floor

    v_lead = max(self._update_lead_speed(float(lead.vLead)), 0.0)
    gap = float(lead.dRel)

    # (1) detection, with hysteresis
    if self.armed:
      self.armed = v_lead < LEAD_STILL_OFF and gap < ARM_MAX_GAP
    else:
      self.armed = v_lead < LEAD_STILL_ON and gap < ARM_MAX_GAP

    # (2) the curve, and (3) no comfort cap on it
    want = 0.0
    if self.armed and v_ego > v_lead:
      d = gap - TARGET_GAP
      if d <= MIN_GEOMETRY_GAP:
        d = max(gap - HARD_MIN_GAP, MIN_GEOMETRY_GAP)   # past the target: aim at not hitting it
      want = -(v_ego ** 2 - v_lead ** 2) / (2.0 * d)
      want = float(np.clip(min(want, 0.0), ACCEL_MIN, 0.0))

    jerk = FLOOR_JERK_URGENT if (want < -URGENT_DECEL or self.a_floor < -URGENT_DECEL) \
        else FLOOR_JERK
    self._ramp(want, a_prev, jerk)
    return self.a_floor

  def _ramp(self, want, a_prev, jerk):
    # While not holding anything down the floor sits on the output, so the first frame it
    # does hold is a step of one jerk-limited increment rather than the whole gap between
    # the two curves. Letting go is the same in reverse.
    start = self.a_floor if self.active else float(a_prev)
    step = jerk * DT_MDL
    self.a_floor = float(np.clip(want, start - step, start + step))
    self.active = self.a_floor < float(a_prev) - 1e-6
