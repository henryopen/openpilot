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
# ...but smoothing does not survive a jump. 2026-09-29 07:26:31, held 3.9 m behind a car: one frame read
# 43.7 m (some other return) and the next 3.96 again. Through DREL_TAU that one frame moved the filter
# 6.6 m, over RELEASE_M, and the hold let go. A car does not cover JUMP_M in a frame (that is 40 m/s),
# so a reading that far from the filter counts as the lead missing, and only LOST_GRACE of that lets go.
JUMP_M = 2.0           # m

# Creeping up on it (2026-09-28). should_stop only comes at 0.3 m/s, and this car does not act on the
# small decelerations the MPC asks for below walking pace: at 17:31:50 it rolled at 1.5-2.1 km/h with
# -0.16 to -0.25 asked for and aEgo about 0, from 5.8 m to 1.15 m behind a car that had all but stopped,
# until the driver braked. So closer than CRAWL_GAP to a lead that is not going, at under CRAWL_SPEED,
# hold as if already stopped - should_stop takes the controller into its stopping state. Normal stops
# end at a median 4.3 m (P10 3.1) on the radar, so this is short of where the MPC puts the car anyway.
CRAWL_SPEED = 1.0      # m/s
CRAWL_GAP = 3.5        # m

# The pull away after a release. While held, the MPC keeps planning from zero and keeps wanting to
# close in, so on release its plan went straight out: +0.47-0.72 in the first frames against 0.00 on
# 09-25, before the hold. The car is about 0.9 s behind and overshoots 3-4x at walking pace - aEgo
# 2.2 at 1 s on 09-28 against 1.2 on 09-25, reaching 4.8 km/h in a second at 17:31:48. So the accel is
# ramped from LAUNCH_A0 at LAUNCH_JERK, which puts it where the 09-25 launches were (0.18 / 0.25 / 0.31
# at 0.25 / 0.5 / 0.75 s), until LAUNCH_TIME or LAUNCH_SPEED.
LAUNCH_A0 = 0.1        # m/s^2
LAUNCH_JERK = 0.3      # m/s^3
LAUNCH_TIME = 2.5      # s
LAUNCH_SPEED = 3.0     # m/s


class StandstillHold:
  def __init__(self, dt: float):
    self.dt = dt
    self.d_filter = FirstOrderFilter(0.0, DREL_TAU, dt)
    # After a release, not again until we have actually moved off: a lead creeping away at
    # under ARM_LEAD_STILL would otherwise re-arm it on the very next frame, from a new
    # reference, and the car would never get going.
    self.need_move = False
    self.launch_t = None
    self.reset()

  def _release(self) -> None:
    self.reset()
    self.need_move = True
    self.launch_t = 0.0

  def launch_cap(self, v_ego: float) -> float:
    """Ceiling on the accel while pulling away after a release; inf otherwise."""
    if self.launch_t is None:
      return float('inf')
    if self.launch_t >= LAUNCH_TIME or v_ego >= LAUNCH_SPEED:
      self.launch_t = None
      return float('inf')
    return LAUNCH_A0 + LAUNCH_JERK * self.launch_t

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
      self.launch_t = None
      return False

    if self.launch_t is not None:
      self.launch_t += self.dt

    if v_ego >= ARM_SPEED:
      self.need_move = False

    if not self.active:
      stopped = not self.need_move and v_ego < ARM_SPEED and lead.dRel < ARM_MAX_GAP
      crawling_in = v_ego < CRAWL_SPEED and lead.dRel < CRAWL_GAP
      if radar_lead and lead.vLead < ARM_LEAD_STILL and (stopped or crawling_in):
        self.launch_t = None
        self.active = True
        self.d_filter.x = float(lead.dRel)
        self.ref_d = float(lead.dRel)
        self.ref_odo = self.odo
        self.lost_t = 0.0
        self.moved = 0.0
      return self.active

    if radar_lead and abs(float(lead.dRel) - self.d_filter.x) <= JUMP_M:
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
