"""Hand a junction to the driving model instead of braking for it here.

openpilot only lets the model's own longitudinal plan compete in experimental mode. Outside
it the plan is cruise and the lead MPC, and neither knows a junction is coming, which is why
the car holds its set speed into a red light. Rather than write a second stopping law beside
the model's - which is what stop_for_lights did - this arms experimental mode for the
approach and stands down after it, so there is one stopping law and the model owns it.

The trigger is the model's own path going short, which is the same signal StarPilot's
conditional experimental mode keys off. Measured on this car over 9886 moving frames from
three drives, the path runs a median 5.2 s of travel ahead when the car really does stop
within the next eight seconds, against 10.4 s when it does not:

    threshold   stops caught   non-stops tripped
        5 s         46.0%            4.9%
        7 s         76.5%           14.1%
        9 s         85.8%           26.9%

7 s is taken. A frame counted there as tripped is not a braking event: arming only adds the
model's plan to the candidates, and min() still takes the most conservative, so the model
has to ask for the deceleration itself before anything moves.

The raw signal is noisy - on one approach the path read 100 m, then 42, then 60, then 25 in
consecutive seconds - so it is filtered, and entering and leaving use different margins.
"""
import time

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

STOP_TIME = 7.0             # the threshold above, in seconds of travel
MAX_TIME = 9.0              # ceiling once the boost is applied at speed
ON_MARGIN = 2.5             # metres of hysteresis entering
OFF_MARGIN = 4.0            # and leaving, so the noise above cannot chatter the mode
HOLD_TIME = 4.0             # keep the mode through a brief loss of the prediction
STRONG_MARGIN = 10.0        # only a clear prediction earns that hold

# Above this the model's horizon is longer than any junction it could be reacting to, and a
# false arming at speed is the one that matters. Taiwan's fastest surface roads sit under it.
DISABLE_SPEED = 80 * CV.KPH_TO_MS

# Filtering: quick to react in town, slower where a false trigger costs more. The boost
# lengthens the threshold at speed, where the model plans further ahead anyway.
SPEED_BP = [0.0, 56.0 * CV.KPH_TO_MS, 72.0 * CV.KPH_TO_MS]
FILTER_TIMES = [0.35, 0.35, 0.8]
BOOSTS = [1.0, 1.0, 1.2]
CAPS = [0.0, 0.0, 1.0]
THRESHOLD = 1.0 - 1.0 / np.e    # a filtered condition has to hold for about one time constant

# A lead nearer than the stop threshold is the queue at the junction, not a reason to say
# there is no junction - that conflation is what kept the older forks from arming at a red
# light with a car in front. A lead that is itself stopped lifts that veto, and only that:
# the model still has to see the path end before anything arms. Letting the stopped lead
# arm on its own instead put the mode on for half of a town drive, most of it standing
# still behind a car with the path reading long - see the note on `armed` below.
LEAD_BLOCK_MARGIN = 15.0
LEAD_HANDOFF_SPEED = 2.0

# Mid-turn the path is short because the road bends, not because it ends.
TURN_VETO_SPEED = 24 * CV.KPH_TO_MS
TURN_VETO_ANGLE = 45.0


class JunctionHandoff:
  def __init__(self):
    self.filter = FirstOrderFilter(0.0, FILTER_TIMES[0], DT_MDL)
    self.lead_clear_filter = FirstOrderFilter(0.0, 0.6, DT_MDL)
    self.active = False
    self.model_detected = False
    self.hold_until = 0.0

  def reset(self):
    self.filter.x = 0.0
    self.lead_clear_filter.x = 0.0
    self.active = False
    self.model_detected = False
    self.hold_until = 0.0

  def _in_turn(self, car_state, v_ego):
    if car_state.standstill or v_ego > TURN_VETO_SPEED:
      return False
    if not (car_state.leftBlinker or car_state.rightBlinker):
      return False
    return abs(car_state.steeringAngleDeg) >= TURN_VETO_ANGLE

  def update(self, model, car_state, v_ego, lead):
    if v_ego > DISABLE_SPEED or self._in_turn(car_state, v_ego):
      self.reset()
      return

    xs = model.position.x
    if not len(xs):
      self.reset()
      return
    path = float(xs[-1])

    self.filter.update_alpha(float(np.interp(v_ego, SPEED_BP, FILTER_TIMES)))
    boost = float(np.interp(v_ego, SPEED_BP, BOOSTS))
    cap = float(np.interp(v_ego, SPEED_BP, CAPS))
    model_time = STOP_TIME * boost
    if cap > 0:
      model_time = min(model_time, MAX_TIME * cap + STOP_TIME * (1.0 - cap))
    threshold = max(v_ego * model_time, 0.0)

    # asymmetric, so leaving needs the path to grow further than entering needed it to shrink
    if self.model_detected:
      self.model_detected = path < threshold + OFF_MARGIN
    else:
      self.model_detected = path < max(threshold - ON_MARGIN, 0.0)

    lead_relevant = lead is not None and bool(lead.present) and lead.dRel < threshold + LEAD_BLOCK_MARGIN
    handoff = lead_relevant and lead.vLead < LEAD_HANDOFF_SPEED
    if handoff:
      lead_cleared = True
    else:
      self.lead_clear_filter.update(not lead_relevant)
      lead_cleared = self.lead_clear_filter.x >= THRESHOLD

    # The stopped lead reaches this only through lead_cleared. Arming on it directly held
    # the mode on for 49.9% of an engaged town drive on 2026-09-05, 87% of that from this
    # one path; 73% of those frames were standing still and 80% had the path longer than
    # the threshold, which is the model saying there is no junction. Replayed over the same
    # drive this line drops it to 31.6% and still catches all six real stops.
    self.filter.update(self.model_detected and lead_cleared)
    armed = self.filter.x >= THRESHOLD

    now = time.monotonic()
    if armed and path < max(threshold - STRONG_MARGIN, 0.0):
      self.hold_until = now + HOLD_TIME

    self.active = bool(armed or (not lead_relevant and now < self.hold_until))
