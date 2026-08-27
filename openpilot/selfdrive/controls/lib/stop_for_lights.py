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
that never happens about one time in five, so what it enables is capped: the model
may ask for gentle braking and nothing more. A false call costs a light lift off the
accelerator, which is the worst it is allowed to cost.
"""
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL

MIN_SPEED = 15 * CV.KPH_TO_MS   # below this the ordinary planner is already stopping us
VEL_STOPPED = 0.5               # predicted speed at or under this is a standstill
VEL_TAIL = 1.0                  # and it has to stay stopped, not dip and recover
STOP_WITHIN = 15.               # the standstill has to be this junction, not a later one
FILTER_TIME = 1.0               # a red light does not appear and vanish inside a second
THRESHOLD = 1 - 1 / np.e        # ~0.63, the filter's value after one time constant
MAX_DECEL = -1.5                # the most a false call is allowed to cost


class StopForLights:
  def __init__(self):
    self.params = Params()
    self.enabled = self.params.get_bool("StopForTrafficLights")
    self.frame = 0
    self.filter = FirstOrderFilter(0., FILTER_TIME, DT_MDL)
    self.is_active = False
    self.stop_distance = 0.

  def update_params(self) -> None:
    if self.frame % 50 == 0:
      self.enabled = self.params.get_bool("StopForTrafficLights")
    self.frame += 1

  def _predicts_a_stop(self, md) -> bool:
    """Does the model's speed profile come to a standstill, near enough to be this junction?"""
    v, x = md.velocity.x, md.position.x
    if len(v) == 0 or len(x) != len(v):
      return False

    idx = next((i for i, u in enumerate(v) if u < VEL_STOPPED), None)
    if idx is None:
      return False
    if any(u > VEL_TAIL for u in v[idx:]):   # dips and recovers: traffic, not a light
      return False

    self.stop_distance = float(x[idx])
    return self.stop_distance < STOP_WITHIN

  def update(self, md, v_ego: float) -> None:
    self.update_params()

    if not self.enabled or v_ego < MIN_SPEED:
      self.filter.x = 0.
      self.is_active = False
      self.stop_distance = 0.
      return

    self.filter.update(1. if self._predicts_a_stop(md) else 0.)
    self.is_active = self.filter.x > THRESHOLD

  def limit(self, a_target_e2e: float) -> float:
    """What the model is allowed to ask for. Gentle braking, never a stab at the brake."""
    return max(float(a_target_e2e), MAX_DECEL)
