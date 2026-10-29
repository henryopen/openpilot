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
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE

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

# ...and faster than that (2026-10-14 17:10:02). Released behind a car that had crept 1 m in 5.5 s, the
# car surged (+1.87 for +0.17 asked), the lead stopped again, and from 6.2 m to 2.5 m at 4-5.7 km/h the
# plan asked -0.3..-0.6, which this car ignores at that pace - shouldStop never came, the crawl arm needs
# < 1.0 m/s, and the driver braked at 1.9 m. So also hold when, under CLOSE_SPEED, a lead that is stopped
# (|vLead| - an oncoming or crossing return has a large negative one) would still be closer than CLOSE_GAP
# after a gentle CLOSE_DECEL stop from here. Replayed over 10/08-10/14 against the shipped hold: 30 more
# holds - 12 where the driver braked within 3 s and the gap got to 1.2-3.3 m, 12 that come earlier than
# the crawl arm did (ending 1.5-3.6 m), 4 blips of 0.05-0.25 s, and 2 that would stop the car where it
# rolled on 2.4-2.7 m past a near-still car.
CLOSE_SPEED = 2.0      # m/s
CLOSE_DECEL = 0.8      # m/s^2
CLOSE_GAP = STOP_DISTANCE   # m - was 3.0, see "The stop distance is the line" below

# The stop distance is the line (2026-10-30). The driver: "跟前車的距離是最後底線，不管什麼都不該在前進",
# and "stop_distance 設定裡面就不能在前進". On 10/29's four drives the car rolled to within 4 m of a lead
# 23 times, down to 0.9 / 1.6 / 1.8 / 2.3 m, the same way each time: what went on CAN (sendcan SCC12, the
# car's TCS13) shows that once should_stop takes the controller into its stopping state the SCC12 carries
# StopReq, and from then on the car stops on its own at about -0.2 whatever aReq says - 17:45:32 asked
# -2.01 for 2 s and got -0.13..-0.27, rolling from 3.4 m to 0.9 m. Before StopReq (PID, the same aReq
# without it) it follows: 19:06:12-13 asked -0.62..-0.93 at 3.7-5.5 km/h and got -0.45..-0.51. The arms
# above were built on should_stop stopping the car at about CLOSE_DECEL; this car does not do that, so
# the arm put it into its slowest stop earlier. And the release let go on 1 m of the lead's movement
# alone: 17:45:28 at 4.8 m, 18:36:24 at 5.9, 19:03:44 at 5.4, after which the car surged (+0.25 asked,
# +1.63 done), the lead stopped again and the car rolled in.
# So, under GAP_SPEED with a lead that is not pulling away (radarState leadOne, the planner's lead):
#  * no release while the lead is within STOP_DISTANCE (the MPC's stop distance, the driver's 5.0);
#  * within STOP_DISTANCE no throttle, and while still rolling brake as a request in PID - at least
#    GAP_BRAKE_MIN (the car does not act on less at this pace), more if stopping short of GAP_STOP_AT
#    needs it, up to GAP_BRAKE_MAX;
#  * held by an arm above while still rolling: the same braking, aimed at stopping at STOP_DISTANCE,
#    and should_stop (StopReq) only once under ARM_SPEED, where the car's own stop has nothing left to
#    cover (0.3 m/s at -0.2 is 0.2 m);
#  * outside the line, behind a lead that is not going: brake early if stopping behind it at
#    STOP_DISTANCE, were it to stop from here, would take more than GAP_BRAKE_MIN;
#  * from a standstill behind a crawling lead, no throttle until GAP_LAUNCH_MARGIN past the line.
GAP_SPEED = 3.0        # m/s - stop-and-go pace; above it a lead this close is a cut-in, the MPC's
GAP_LEAVING = 0.3      # m/s - lead faster than us by this: it is going, no braking (still no throttle)
GAP_BRAKE_MIN = 0.8    # m/s^2
GAP_BRAKE_MAX = 2.0    # m/s^2 - the controller's stopping decel (CP.stopAccel)
GAP_STOP_AT = 2.0      # m - inside the line: stop before this, as soon as the car can
GAP_LEAD_DECEL = 1.0   # m/s^2 - how hard a lead at this pace is taken to stop, at least
# Pulling away, the car surges for about a second whatever is asked (10/29: +0.13..+0.44 asked, +0.88 /
# +1.63 / +1.72 done), 0.5-1 m more than asked. Behind a lead only crawling (19:03:44, 4 km/h) that took a
# start at 5.4-6.0 m to 1.8 m. So from a standstill, behind a lead slower than GAP_LAUNCH_LEAD_V, wait
# for STOP_DISTANCE + GAP_LAUNCH_MARGIN; a lead actually driving off opens that within a second anyway.
GAP_LAUNCH_MARGIN = 1.5    # m
GAP_LAUNCH_LEAD_V = 2.0    # m/s

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
    self.gap_wait = False    # kept stopped by gap_limit last frame
    self.reset()

  def _release(self) -> None:
    self.reset()
    self.need_move = True
    self.launch_t = 0.0

  def launch(self) -> None:
    """Pull away on the same ramp as a release - the keypad's GO, at a light with nothing ahead."""
    self._release()

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
      closing_in = (v_ego < CLOSE_SPEED and abs(lead.vLead) < ARM_LEAD_STILL
                    and lead.dRel - v_ego ** 2 / (2. * CLOSE_DECEL) < CLOSE_GAP)
      if radar_lead and lead.vLead < ARM_LEAD_STILL and (stopped or crawling_in or closing_in):
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
      if self.moved >= RELEASE_M and d >= STOP_DISTANCE:
        self._release()
    else:
      self.lost_t += self.dt
      if self.lost_t >= LOST_GRACE:
        self._release()
    return self.active

  def gap_limit(self, enabled: bool, v_ego: float, lead, held: bool) -> float:
    """Ceiling on the accel from the stop distance - see "The stop distance is the line". inf when it
    has nothing to say, 0 for no throttle, negative for a brake request (meant to be sent without
    should_stop, which the caller only sets under ARM_SPEED). Kept stopped by it and then let go, the
    car pulls away on the release ramp (launch_cap): the MPC wants the whole gap at once by then
    (+1.0 behind a lead already at 8-10 km/h, which this car makes 2-3 at walking pace)."""
    a = self._gap_limit(enabled, v_ego, lead, held)
    waiting = enabled and v_ego < ARM_SPEED and a <= 0.
    if self.gap_wait and not waiting and enabled and v_ego < ARM_SPEED and not held:
      self.launch_t = 0.0
    self.gap_wait = waiting
    return a

  @staticmethod
  def _gap_limit(enabled: bool, v_ego: float, lead, held: bool) -> float:
    if not enabled or not lead.present or v_ego >= GAP_SPEED:
      return float('inf')
    d = float(lead.dRel)
    leaving = float(lead.vLead) - v_ego > GAP_LEAVING
    if v_ego < ARM_SPEED and d < STOP_DISTANCE + GAP_LAUNCH_MARGIN and float(lead.vLead) < GAP_LAUNCH_LEAD_V:
      return 0.     # not pulling away into the car's surge behind a lead that is only crawling
    if d < STOP_DISTANCE:
      if v_ego < ARM_SPEED or (leaving and not held):
        return 0.
      room = max(d - GAP_STOP_AT, 0.5)
    elif held and v_ego >= ARM_SPEED:
      room = max(d - STOP_DISTANCE, 0.5)
    elif not leaving and v_ego >= ARM_SPEED:
      # outside the line, a lead that is not going: if it stopped from here (at its own decel, at least
      # GAP_LEAD_DECEL), would stopping behind it at STOP_DISTANCE take more than GAP_BRAKE_MIN? Then
      # brake now - the car answers about 0.4 s late, and the MPC asked -0.4 for the first half second
      # of 10/29 19:06:35 (8 km/h, 7 m, the lead stopping in 1.3 s), which put the car at 2.5 m.
      vl = max(float(lead.vLead), 0.)
      lead_stop = vl ** 2 / (2. * max(-float(lead.aLeadK), GAP_LEAD_DECEL))
      room = max(d + lead_stop - STOP_DISTANCE, 0.5)
      if v_ego ** 2 / (2. * room) < GAP_BRAKE_MIN:
        return float('inf')
    else:
      return float('inf')
    return -min(max(GAP_BRAKE_MIN, v_ego ** 2 / (2. * room)), GAP_BRAKE_MAX)
