"""Let the model's own decision to stop reach the plan, without experimental mode.

The model already works out that the road ahead comes to a stop, and modeld publishes
that intent every frame. Master only feeds it to the planner in experimental mode, so
on this car it is thrown away and nothing brakes for a red light.

This watches the model's predicted speed profile rather than its stop flag: the flag
only goes true once we are already stopped, which is too late to be of any use. A
profile that falls to a standstill and stays there, with the standstill close enough
to be this junction rather than one further on, is what a red light looks like from
here.

The check is deliberately not trusted on its own. Even at its best it calls a stop
that never happens about one time in five, so what it enables is capped: the braking
it can ask for is firm at worst, never a stab at the brake.

How hard to brake is worked out from the model's own predicted stop distance rather
than taken from the acceleration it publishes. Over this car's logs the published
value is about -1.4 where stopping at the distance the model itself predicts needs
-2.3, and the planner resolves its candidates by min(), so that request loses to
cruise or the lead car and the junction arrives with nothing having happened. Asking
for what the geometry needs is what makes the difference: replayed over four trips it
takes the module from braking in 3 of the 15 stops the old 15 m gate let it see to 16 of
the 20 a reach that grows with speed sees, and from 8 km/h with 10 m left to 16 km/h with
12 m, which is a lift and a long gentle wind-down rather than a late shove.

Stopping in time is not what it is for. Held to a gentle deceleration it comes to rest
short of the line about four times in five and rolls slowly past the rest, which is the
right way round: the driver would sooner finish a stop himself than be braked hard by a
call that is wrong one time in five. Being early matters more than being firm for another
reason too. The model reads a junction far better while the car is slowing: over these
logs, 30-45 m from a stop it never once had the standstill in its profile while holding
speed, and had it a third of the time while decelerating. Easing off early is what earns
the better prediction that follows.
"""
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

MIN_SPEED = 15 * CV.KPH_TO_MS   # below this the ordinary planner is already stopping us
VEL_STOPPED = 0.5               # predicted speed at or under this is a standstill
VEL_TAIL = 1.0                  # and it has to stay stopped, not dip and recover
STOP_WITHIN = 20.               # nothing nearer than this is worth reaching further for
FILTER_TIME = 1.0               # a red light does not appear and vanish inside a second
THRESHOLD = 1 - 1 / np.e        # ~0.63, the filter's value after one time constant
MAX_DECEL = -1.5                # gentle: arriving slowly beats being braked hard
STOP_GAP = 4.0                  # where the car comes to rest short of the predicted point
COMMITTED = 15 * CV.KPH_TO_MS   # below this the stop is happening, see it through
STOPPED = 0.5                   # standing still
SETTLE = 1.5                    # the model needs a moment at a halt before it says so
CLEAR_FOR = 1.0                 # and has to keep saying it: at a halt the flag flickers
HOLD_MAX = 180.                 # no light is this long; let go rather than strand the car


class StopForLights:
  def __init__(self):
    self.params = Params()
    self.enabled = self.params.get_bool("StopForTrafficLights")
    self.frame = 0
    self.filter = FirstOrderFilter(0., FILTER_TIME, DT_MDL)
    self.is_active = False
    self.held_for = 0.
    self.stopped_for = 0.
    self.clear_for = 0.
    self.stop_distance = 0.

  def update_params(self) -> None:
    if self.frame % 50 == 0:
      self.enabled = self.params.get_bool("StopForTrafficLights")
    self.frame += 1

  def _predicts_a_stop(self, md, v_ego: float) -> bool:
    """Does the model's speed profile come to a standstill, near enough to be this junction?

    Near enough means within braking distance from the speed we are doing, so the reach
    grows with speed rather than being one number that suits one speed. It was 15 m, which
    over this car's logs threw away every stop the model saw: it first has the standstill
    in its profile at 25 m typically and as far out as 66 m, always further than 15. What
    it cannot do is see a stop it is not yet close to in time, its profile only running ten
    seconds ahead, and its distant calls flicker, so reaching past braking distance buys
    nothing and only brakes for junctions we drive through.

    Leaves the distance to that standstill behind for limit() to brake against, and clears
    it when there is none, so nothing brakes against a distance the model has moved on from.
    """
    self.stop_distance = 0.
    v, x = md.velocity.x, md.position.x
    if len(v) == 0 or len(x) != len(v):
      return False

    idx = next((i for i, u in enumerate(v) if u < VEL_STOPPED), None)
    if idx is None:
      return False
    # capnp lists index but do not slice, so walk it
    if any(v[i] > VEL_TAIL for i in range(idx, len(v))):   # dips and recovers: traffic, not a light
      return False

    self.stop_distance = float(x[idx])
    return self.stop_distance < max(STOP_WITHIN, v_ego * v_ego / (2. * -MAX_DECEL) + STOP_GAP)

  def _reset(self) -> None:
    self.filter.x = 0.
    self.is_active = False
    self.held_for = 0.
    self.stopped_for = 0.
    self.clear_for = 0.
    self.stop_distance = 0.

  def update(self, md, v_ego: float, gas_pressed: bool = False) -> None:
    self.update_params()

    if not self.enabled or gas_pressed:
      self._reset()
      return

    if self.is_active:
      self.held_for += DT_MDL
      if self.held_for > HOLD_MAX:
        # something is wrong with the call: it has kept us here far longer than a light
        self._reset()
      elif v_ego < STOPPED:
        # Stopped for this junction. Nothing else keeps us here - the e2e branch is what
        # holds standstill, and cruise wants the set speed back - so stay until the model
        # says the way is clear. That is also what lets us move off when it changes.
        # The model only calls a stop once we are actually at one, so give it a moment
        # to catch up before reading anything into its silence.
        #
        # And once at a halt the flag flickers - anything moving nearby is enough to have
        # the model briefly draw a path forward again. Over this car's logs that happens
        # 442 times across the stops, half of them lasting a single frame, while the model
        # genuinely clearing a junction holds for the best part of a second. So the flag
        # has to keep saying go, not just say it once.
        self.stopped_for += DT_MDL
        if md.action.shouldStop:
          self.clear_for = 0.
          self.is_active = True
        else:
          self.clear_for += DT_MDL
          if self.stopped_for > SETTLE and self.clear_for > CLEAR_FOR:
            self.is_active = False
      elif v_ego < COMMITTED:
        # slow enough that the stop is happening; see it through. the distance still has
        # to be read every frame, since it is what the braking is worked out against.
        self._predicts_a_stop(md, v_ego)
        self.stopped_for = 0.
        self.clear_for = 0.
        self.is_active = True
      else:
        # still quick: let the model take it back if it no longer sees a stop
        self.filter.update(1. if self._predicts_a_stop(md, v_ego) else 0.)
        self.is_active = self.filter.x > THRESHOLD
      return

    if v_ego < MIN_SPEED:
      self.filter.x = 0.
      return

    self.filter.update(1. if self._predicts_a_stop(md, v_ego) else 0.)
    self.is_active = self.filter.x > THRESHOLD
    if self.is_active:
      self.held_for = 0.

  def limit(self, a_target_e2e: float, v_ego: float) -> float:
    """What it takes to stop where the model says the road does, within what is allowed.

    Constant deceleration to a standstill a little short of the predicted point, since
    that is where the car actually comes to rest. Never above what the model asked for -
    if it wants to brake harder than the geometry needs, it has a reason to.
    """
    a_target = float(a_target_e2e)
    if self.stop_distance > 0.:
      a_target = min(a_target, -v_ego * v_ego / (2. * max(self.stop_distance - STOP_GAP, 1.)))
    return max(a_target, MAX_DECEL)
