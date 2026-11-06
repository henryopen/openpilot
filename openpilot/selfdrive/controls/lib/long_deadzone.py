"""A band where the car neither gains nor loses speed, the way a driver holds a steady foot.

Asked for on 09-23: "driving is not only accelerating and braking; there should be stretches
of neither, like a deadzone - not the lead speeding up and slowing down and us doing the same".
On the 09-23 drives, following above 20 km/h (19.2 min), the plan's acceleration crossed from
above +0.2 to below -0.2 or back 6.5 times a minute - once every nine seconds - and within 3 s
of each crossing the lead's own speed had moved a median 8.9 km/h. Half of all following frames
asked for less than 0.22 m/s^2 either way: small corrections chasing the lead.

So a small request, held for ENTER_S, puts the plan into a hold: the output eases to zero and
the speed stays where it is, letting the gap breathe inside the band the MPC tolerates. Anything
past the exit thresholds leaves the hold on that frame with the plan's own value - braking
first, at a lower magnitude than accelerating, so a closing lead is never waited on.

Off below MIN_SPEED (stop-and-go wants every correction), when a stop is planned, and whenever
something other than cruise or a lead set the value - a corner, a junction or the model's stop
are all deliberate and go straight through.
"""
import os

MIN_SPEED = 20 / 3.6          # m/s
ENTER_LO, ENTER_HI = -0.25, 0.20
EXIT_LO, EXIT_HI = -0.45, 0.35
ENTER_S = 1.0                 # s the request has to stay small before holding
RAMP = 1.0                    # m/s^3: how quickly the output eases to zero on entering the hold
OFF_FLAG = '/data/long_deadzone_off'   # the HUD's switch: present = deadzone off
RECHECK_S = 1.0


class LongDeadzone:
  def __init__(self, dt):
    self.dt = dt
    self.holding = False
    self.small_s = 0.0
    self.out = 0.0
    self.enabled = not os.path.isfile(OFF_FLAG)
    self._recheck = 0.0

  def _reset(self, a):
    self.holding = False
    self.small_s = 0.0
    self.out = a

  def update(self, a, v_ego, eligible):
    """a: the plan's acceleration; eligible: set by cruise or a lead, no stop planned."""
    self._recheck -= self.dt
    if self._recheck <= 0.0:
      self._recheck = RECHECK_S
      self.enabled = not os.path.isfile(OFF_FLAG)

    if not self.enabled or not eligible or v_ego < MIN_SPEED:
      self._reset(a)
      return a

    if self.holding:
      if a < EXIT_LO or a > EXIT_HI:
        self._reset(a)
        return a
      step = RAMP * self.dt
      self.out = max(0.0, self.out - step) if self.out > 0 else min(0.0, self.out + step)
      return self.out

    if ENTER_LO < a < ENTER_HI:
      self.small_s += self.dt
      if self.small_s >= ENTER_S:
        self.holding = True
    else:
      self.small_s = 0.0
    self.out = a
    return a
