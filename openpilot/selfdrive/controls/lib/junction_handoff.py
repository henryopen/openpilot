"""Hand a junction to the driving model instead of braking for it here.

openpilot only lets the model's own longitudinal plan compete in experimental mode. Outside
it the plan is cruise and the lead MPC, and neither knows a junction is coming, which is why
the car holds its set speed into a red light. Rather than write a second stopping law beside
the model's - which is what stop_for_lights did - this arms experimental mode for the
approach and stands down after it, so there is one stopping law and the model owns it.

The trigger was the model's path running short, and that was wrong. A path runs short for a
stop, but it runs just as short for a turn, a bend, a crest, or nothing at all. Measured over
56 engaged minutes on 2026-09-06: of the ten armings with no car in front, four were real
stops and six were not, and three of the six were the driver's own indicator - a turn desire
halves the planned path, from a median 74 m to 33 m, which drops it straight onto the old
threshold.

What the model plans to *do* separates them where the path cannot. Approaching a red light
its planned speed collapses to nothing; taking a turn it dips to turning speed and carries
on. Over the same drive, arming on the planned speed reaching 7 km/h caught four real stops
and nothing else - no turns, no bends, no empty road.

The cost is that it arms later, because the plan only collapses once the model is committed:
42 to 63 m out at 44 to 52 km/h rather than 100 m at 55. That is why the floor below exists.
Left to itself the model brakes at about 64% of what stopping in the distance it has would
need, and the shortfall compounds - every metre it does not shed raises what the next metre
demands. On 2026-09-06 it was on an overshoot trajectory in every frame of all three real
approaches, and the driver took over each time. The floor asks for the deceleration the
remaining distance actually needs, capped so it can never become a slam, and ramped so it is
never a step.
"""
import time

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import LANE_TURN_SPEED_MAX, LANE_TURN_SPEED_MIN

# The model is planning a stop when its own speed plan reaches this; it leaves at the higher
# one so a plan hovering around the line cannot chatter the mode.
STOP_SPEED_ON = 2.0         # m/s, 7.2 km/h
STOP_SPEED_OFF = 3.5

FILTER_TIME = 0.35          # a plan has to hold for about one time constant
THRESHOLD = 1.0 - 1.0 / np.e
HOLD_TIME = 4.0             # keep the mode through a brief loss of the plan

# Above this the model's horizon is longer than any junction it could be reacting to, and a
# false arming at speed is the one that matters. Taiwan's fastest surface roads sit under it.
DISABLE_SPEED = 80 * CV.KPH_TO_MS

# A lead inside the stopping distance vetoes the handoff outright. Braking for a car in
# front is the lead MPC's job and it is already doing it; the model has nothing to add
# there, and everything it does add is felt. Measured on 2026-09-06 the mode was on for 21%
# of the drive with a lead there 93% of the time, and every one of the 50 armings that had a
# lead had a lead barely moving - traffic queues, not junctions.
LEAD_LOOKAHEAD = 7.0        # seconds of travel a lead has to be inside to count
LEAD_BLOCK_MARGIN = 15.0

# What the driver's indicator means to the model: desire_helper sends turnLeft/turnRight in
# this band, the model plans the corner, and the plan ends at the corner. That is not a stop.
# Bounds are imported so the two cannot drift apart.

# The floor. 2.5 m/s2 is firm braking and not an emergency stop; every real approach on
# 2026-09-06 needed between 1.59 and 2.01 at the moment the plan collapsed, so the cap is
# clear of what the trigger actually asks for while still bounding what a bad frame can do.
MAX_FLOOR_DECEL = 2.5
FLOOR_JERK = 4.0            # m/s3, so full floor is reached in about 0.6 s rather than at once
MIN_STOP_DISTANCE = 2.0     # below this the geometry is meaningless


class JunctionHandoff:
  def __init__(self):
    self.filter = FirstOrderFilter(0.0, FILTER_TIME, DT_MDL)
    self.lead_clear_filter = FirstOrderFilter(0.0, 0.6, DT_MDL)
    self.active = False
    self.model_detected = False
    self.hold_until = 0.0
    self.stop_x = None
    self.a_floor = 0.0

  def reset(self):
    self.filter.x = 0.0
    self.lead_clear_filter.x = 0.0
    self.active = False
    self.model_detected = False
    self.hold_until = 0.0
    self.stop_x = None
    self.a_floor = 0.0

  def _turning(self, car_state, v_ego):
    """The indicator is on and the model has been told to take a turn, so its plan ends at
    the corner rather than at anything in front of us."""
    return bool(car_state.leftBlinker or car_state.rightBlinker) and \
        LANE_TURN_SPEED_MIN <= v_ego < LANE_TURN_SPEED_MAX

  def _planned_stop(self, model):
    """How far ahead the model plans to be stopped, or None if it does not."""
    xs, vs = model.position.x, model.velocity.x
    n = min(len(xs), len(vs))
    if not n:
      return None
    limit = STOP_SPEED_OFF if self.model_detected else STOP_SPEED_ON
    for i in range(n):
      if vs[i] < limit:
        return float(xs[i])
    return None

  def update(self, model, car_state, v_ego, lead):
    if v_ego > DISABLE_SPEED or self._turning(car_state, v_ego):
      self.reset()
      return

    self.stop_x = self._planned_stop(model)
    self.model_detected = self.stop_x is not None

    threshold = max(v_ego * LEAD_LOOKAHEAD, 0.0)
    lead_relevant = lead is not None and bool(lead.present) and lead.dRel < threshold + LEAD_BLOCK_MARGIN
    self.lead_clear_filter.update(not lead_relevant)
    lead_cleared = self.lead_clear_filter.x >= THRESHOLD

    self.filter.update(self.model_detected and lead_cleared)
    armed = self.filter.x >= THRESHOLD

    now = time.monotonic()
    if armed and self.model_detected:
      self.hold_until = now + HOLD_TIME

    self.active = bool(armed or (not lead_relevant and now < self.hold_until))

    # Ask for what stopping in the distance the model says it has would take. Rate limited
    # in both directions: arming is a step in the trigger but must not be one at the wheels,
    # and letting go has to be as smooth as taking hold.
    want = 0.0
    if self.active and self.stop_x is not None and self.stop_x > MIN_STOP_DISTANCE and v_ego > 1.0:
      want = -min(v_ego ** 2 / (2.0 * self.stop_x), MAX_FLOOR_DECEL)
    step = FLOOR_JERK * DT_MDL
    self.a_floor = float(np.clip(want, self.a_floor - step, self.a_floor + step))
