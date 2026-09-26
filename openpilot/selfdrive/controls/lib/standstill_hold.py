"""Stopped behind a car, stay stopped until that car has actually gone.

The gate this replaces (STANDSTILL_CREEP_*, 2026-09-06 / 09-20) was stateless: every frame it
held the accel at <= 0 while the radar's vRel said the lead was not leaving. The radar's vRel
on a stopped car jitters - 0.17-0.41 m/s in the 09-20 false starts - so one frame over the line
let a positive aTarget through, and nothing kept the controller in its stopping state
(shouldStop stays false at a standstill when the MPC wants to close in), so that one frame was
enough to release the brake. On 2026-09-25 the car moved off 8 times behind a car that had not
gone: 6 of them with the old gate's condition true at the moment of release, aTarget +0.19 to
+0.40 in the half second before, the car rolling 0.3-2.7 m and stopping again.

So this latches. Once we are stopped behind a radar lead that is itself stopped, it remembers
where the lead was, and releases only when the lead has moved away from that point by
RELEASE_M on its own - measured as dRel plus the distance we have covered, so it reads the same
whether we moved or not. While it holds, the accel is capped at 0 and shouldStop is set, so the
controller stays in its stopping state instead of sitting in PID at zero.

RELEASE_M is the driver's 1 m (2026-09-26). This class replayed over 09-16..09-25 on the stops
where it arms, sorted the 09-20 way by how far the car went once released (under 10 m = it
should not have gone): it is holding at the moment of all 16 false starts, and releases the 44
real ones a median 0.22 s later than the car went, p90 1.22 s, worst 2.11 s, none stuck. Where
the lead does move a metre or more - traffic crawling - it lets go and the car follows, which is
right: 14 of the shorter "false" starts in the wider set had a lead that moved 1.1-9.3 m.
"""
from openpilot.common.filter_simple import FirstOrderFilter

ARM_SPEED = 0.3        # m/s - where should_stop takes over anyway; the approach is left to the MPC
ARM_MAX_GAP = 9.0      # m - same reach as the gate this replaces
ARM_LEAD_STILL = 0.5   # m/s - the lead has to be stopped too; a crawling lead is followed, not held
RELEASE_M = 1.0        # m the lead moves away on its own before we go (driver, 2026-09-26)
LOST_GRACE = 1.0       # s - the radar drops a lead 2.6-2.9 times a minute; do not let go on a blink
DREL_TAU = 0.3         # s - smooth dRel before differencing; the radar's range on a stopped car jitters


class StandstillHold:
  def __init__(self, dt: float):
    self.dt = dt
    self.d_filter = FirstOrderFilter(0.0, DREL_TAU, dt)
    # After a release, not again until we have actually moved off: a lead creeping away at
    # under ARM_LEAD_STILL would otherwise re-arm it on the very next frame, from a new
    # reference, and the car would never get going.
    self.need_move = False
    self.reset()

  def _release(self) -> None:
    self.reset()
    self.need_move = True

  def reset(self) -> None:
    self.active = False
    self.odo = 0.0
    self.ref_d = 0.0
    self.ref_odo = 0.0
    self.lost_t = 0.0
    self.moved = 0.0

  def update(self, enabled: bool, v_ego: float, lead, gas_pressed: bool) -> bool:
    """lead is radarState.leadOne. Returns True while the car should be held stopped."""
    self.odo += v_ego * self.dt
    radar_lead = bool(lead.present and lead.radar)

    if not enabled or gas_pressed:
      self.reset()
      self.need_move = False
      return False

    if v_ego >= ARM_SPEED:
      self.need_move = False

    if not self.active:
      if (not self.need_move and radar_lead and v_ego < ARM_SPEED and lead.dRel < ARM_MAX_GAP
          and lead.vLead < ARM_LEAD_STILL):
        self.active = True
        self.d_filter.x = float(lead.dRel)
        self.ref_d = float(lead.dRel)
        self.ref_odo = self.odo
        self.lost_t = 0.0
        self.moved = 0.0
      return self.active

    if radar_lead:
      self.lost_t = 0.0
      d = self.d_filter.update(float(lead.dRel))
      self.moved = d + (self.odo - self.ref_odo) - self.ref_d
      if self.moved >= RELEASE_M:
        self._release()
    else:
      self.lost_t += self.dt
      if self.lost_t >= LOST_GRACE:
        self._release()
    return self.active
