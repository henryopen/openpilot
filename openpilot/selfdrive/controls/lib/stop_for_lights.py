"""Stop for a junction the model can see, on an empty road, without experimental mode.

Only with nothing in front. A car to follow already answers both halves of a junction -
the planner brakes for it and pulls away with it - and over 2026-08-29 and 08-30 it did so
at every one of the stops with a lead. This is for the other kind, where the road ahead is
empty and nothing but the light says to stop.

The model does not know about traffic lights. What it publishes is a plan: how far it
intends to travel over the next ten seconds and how fast it expects to be going by then.
A junction it has decided to stop at shows up as a plan that ends near us and ends slow.
That is a hint, not a fact - over these drives it is wrong about four times an hour.

So the answer to a hint is a hint back, which is how the driver does it himself: take a
little speed off and see whether the road keeps agreeing. What separates a real stop from
a phantom in the seconds that follow is not how hard the model asks to brake - that reads
the same for both, -0.3 either way - but whether the stop point holds still. A real line
does not move: drive ten metres and the plan should end ten metres nearer. A phantom grows
back, by 8 m in the first second against a real stop's -1.4.

  arm       the plan ends within 45 m and below 2 m/s: take 5 km/h off the set speed
  confirm   the point has held still for a second and the plan now ends at a standstill
  commit    put a car at the line and let the MPC brake for it
  hand off  inside 6 m ask for the stop itself, and hold the car once it is stopped
  let go    the model says the road runs well past where we think it stops - give the speed
            back and forget it. So does the accelerator, which suppresses it outright.

Committing writes no deceleration of its own. Stopping is a question about where, and the
one part of this chain that is asked where is the MPC: its cost is the error against the
distance it wants to keep from an obstacle. Behind a lead that is what stops the car, at
every junction with a car at it, smoothly and in the right place. So this hands it the same
problem in the same terms - a stopped car at the line - rather than writing a second
deceleration curve beside it. Taking the set speed off instead was measured and does not
work: the cruise law only knows the speed error, so it eases off as the error closes and
36 of 36 stops ran past the line, 8.6 m past at the median.

What is given up by not writing the curve is the model's own braking request, which used to
be taken whenever it was firmer than the geometry needed. Over these drives it asks for
-0.3 either way, at a real stop and at a phantom alike, so there was little in it.

The distance is tracked by dead reckoning rather than read fresh each frame, because the
model's view flickers: it counts down with the wheels, is pulled in whenever the model sees
something nearer, and is never pushed out. Over these drives that lands within +2 m of
where the car really stopped, inside 40 m. Beyond that there is nothing to track - the plan
is only ten seconds long, so at town speeds its length is the horizon rather than a stop -
which is why arming waits for the plan to end near us.
"""
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

MIN_SPEED = 15 * CV.KPH_TO_MS      # below this the ordinary planner is already stopping us
ARM_LENGTH = 45.                   # the plan ends this near, so its length is a stop
ARM_END_SPEED = 2.0                # and ends this slow, so it is a stop and not a crawl
ARM_SPEED_DROP = 5 * CV.KPH_TO_MS  # what the hint costs while it proves itself
COMMIT_END_SPEED = 1.0             # committing wants the plan ending at a standstill
COMMIT_HOLD = 1.0                  # and the point holding still for this long
SLACK = 6.0                        # the model may say the road runs this much further
RECEDE_FOR = 0.7                   # for this long before we believe it and let go
HANDOFF = 6.0                      # inside this, ask for the stop rather than a speed
STOP_GAP = 4.0                     # where the car comes to rest short of the plan's end
MAX_DECEL = -1.5                   # the firmest the MPC may brake for a call that is a guess
COMMITTED = 15 * CV.KPH_TO_MS      # below this the stop is happening, see it through
STOPPED = 0.5                      # standing still
SETTLE = 1.5                       # the model needs a moment at a halt before it says so
CLEAR_FOR = 1.0                    # and has to keep saying it: at a halt the flag flickers
HOLD_MAX = 180.                    # no light is this long; let go rather than strand the car
NO_CAP = 1e9                       # no opinion about the set speed


class StopForLights:
  def __init__(self):
    self.params = Params()
    self.enabled = self.params.get_bool("StopForTrafficLights")
    self.frame = 0
    self.armed = False
    self.is_active = False          # committed to the stop, or holding one we made
    self.v_cruise_cap = NO_CAP
    self.stop_distance = 0.
    self.held_still_for = 0.
    self.receded_for = 0.
    self.held_for = 0.
    self.stopped_for = 0.
    self.clear_for = 0.

  def update_params(self) -> None:
    if self.frame % 50 == 0:
      self.enabled = self.params.get_bool("StopForTrafficLights")
    self.frame += 1

  def reset(self) -> None:
    self.armed = False
    self.is_active = False
    self.v_cruise_cap = NO_CAP
    self.stop_distance = 0.
    self.held_still_for = 0.
    self.receded_for = 0.
    self.held_for = 0.
    self.stopped_for = 0.
    self.clear_for = 0.

  @staticmethod
  def _plan(md):
    """How far the model plans to travel, and how fast it expects to be by then."""
    x, v = md.position.x, md.velocity.x
    if len(x) == 0 or len(v) == 0:
      return NO_CAP, NO_CAP
    return float(x[-1]), float(v[-1])

  def _track(self, plan_length: float, v_ego: float) -> None:
    """Where the stop is now, counting down with the wheels.

    Pulled in whenever the model sees something nearer and never pushed out, so a frame
    that loses sight of the junction cannot move the line away from us. The model saying
    the road runs well past it, and keeping that up, is what ends the call instead.
    """
    self.stop_distance = max(self.stop_distance - v_ego * DT_MDL, 0.)
    if plan_length < self.stop_distance:
      self.stop_distance = plan_length
      self.receded_for = 0.
      self.held_still_for += DT_MDL
    elif plan_length > self.stop_distance + SLACK:
      self.receded_for += DT_MDL
      self.held_still_for = 0.
    else:
      self.receded_for = 0.
      self.held_still_for += DT_MDL

  def _hold_at_standstill(self, md) -> None:
    """Stopped for this junction. Nothing else keeps us here - cruise wants the set speed
    back - so stay until the way is clear, which is also what lets us move off again.

    The model only calls a stop once we are at one, so give it a moment to catch up before
    reading anything into its silence. And once at a halt the flag flickers: anything
    moving nearby has the model briefly draw a path forward again, 442 times across these
    stops, half of them lasting a single frame, while genuinely clearing a junction holds
    for the best part of a second. So it has to keep saying go, not just say it once.
    """
    self.stopped_for += DT_MDL
    if md.action.shouldStop:
      self.clear_for = 0.
    else:
      self.clear_for += DT_MDL
      if self.stopped_for > SETTLE and self.clear_for > CLEAR_FOR:
        self.reset()

  def update(self, md, v_ego: float, v_cruise: float, gas_pressed: bool = False, lead=None) -> None:
    self.update_params()

    if not self.enabled or gas_pressed:
      self.reset()
      return

    plan_length, plan_end_speed = self._plan(md)
    has_lead = lead is not None and lead.present

    if self.is_active and v_ego < STOPPED:
      self.held_for += DT_MDL
      if self.held_for > HOLD_MAX:
        # something is wrong with the call: it has kept us here far longer than a light
        self.reset()
      else:
        self.v_cruise_cap = 0.
        self._hold_at_standstill(md)
      return

    if not self.armed:
      # a car to follow is the whole answer, and below MIN_SPEED the planner is already
      # stopping us. otherwise a plan that ends near us and ends slow is worth a look.
      if has_lead or v_ego < MIN_SPEED or plan_length >= ARM_LENGTH or plan_end_speed >= ARM_END_SPEED:
        self.reset()
        return
      self.armed = True
      self.stop_distance = plan_length
      self.held_still_for = 0.
      self.receded_for = 0.
      self.held_for = 0.
      self.v_cruise_cap = max(v_cruise - ARM_SPEED_DROP, 0.)
      return

    if has_lead:
      # something to follow turned up: it brakes harder and better than this does
      self.reset()
      return

    self._track(plan_length, v_ego)
    if self.receded_for > RECEDE_FOR:
      # the road runs on well past where we thought it stopped. give the speed back.
      self.reset()
      return

    if v_ego < COMMITTED:
      # slow enough that the stop is happening either way; see it through
      self.is_active = True
    elif not self.is_active:
      self.is_active = self.held_still_for > COMMIT_HOLD and plan_end_speed < COMMIT_END_SPEED

    if not self.is_active:
      # still only a hint: hold the 5 km/h off and keep watching
      self.v_cruise_cap = max(v_cruise - ARM_SPEED_DROP, 0.)
      return

    self.held_for = 0.
    # the approach is the MPC's from here, so cruise has no opinion to offer. Inside the
    # handoff the set speed goes to zero as well, so nothing tries to pull away from a
    # stop we are in the middle of making.
    self.v_cruise_cap = 0. if self.stop_distance <= HANDOFF else NO_CAP

  def obstacle_x(self, mpc_stop_distance: float) -> float:
    """Where to stand the imaginary car so we come to rest at the line.

    The MPC settles that much short of whatever obstacle it is given, and the car should
    end up STOP_GAP short of the tracked point, which is where it actually came to rest
    over these drives. So the obstacle goes the difference further out.
    """
    return self.stop_distance - STOP_GAP + mpc_stop_distance
