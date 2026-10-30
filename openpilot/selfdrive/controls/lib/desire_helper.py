import json
import os
import time

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.selfdrive.mapd.turn_ahead import TurnAheadReader

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# 20 km/h (2026-10-15, the driver's call), with the turn desire back on below it. History: 30 (stock),
# 40 while a turn desire ran under it, then 30 again with no turn desire (09-28). The turn band below
# is tied to it, so a blinker under 20 is a turn and over 20 is a lane change; junction_handoff's
# blinker veto and the HUD's "turning" intent end at 20 too.
LANE_CHANGE_SPEED_MIN = 20 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# Turning at a junction, as opposed to changing lanes. Below the lane change threshold a
# blinker does not mean "move over", it means "we are about to turn", and the model drives
# a junction very differently once it is told so. Tied to the lane change threshold rather
# than given its own number, the way sunnypilot's LaneTurnController caps itself: the two
# are complementary, so a blinker under this speed is a turn and over it is a lane change,
# with no band in between where neither applies.
LANE_TURN_SPEED_MAX = LANE_CHANGE_SPEED_MIN
# Below this a blinker is more likely to be pulling over, parking, or standing still than
# taking a junction, and handing the model a turn desire there would be inventing one.
#
# Measured over 37297 frames of this car actually turning - blinker on and more than 45
# degrees of wheel, across four drives on 09-16 - the median speed is 9.9 km/h and the 25th
# percentile is 1.2. The 5 km/h this was set to on 09-09, from 2831 frames, keeps 64.6% of
# them rather than the 89.4% that smaller sample suggested; it was cutting 131.9 seconds of
# turning at a median of 205 degrees of wheel.
#
# 2 km/h keeps 73.3%. What it still excludes is the part that should be excluded: 74 of
# those 131.9 seconds are under 1.1 km/h, which is the driver working the wheel while
# stopped or shuffling in a car park, and where standstill takes lateral away regardless -
# the threshold now sits just above that rather than well inside the turns.
LANE_TURN_SPEED_MIN = 2 * CV.KPH_TO_MS

# A blinker that is a junction turn, not a lane change, and when the model is told so (2026-10-27, the driver's
# calls). That day five of nine junction turns were signalled at 24-40 km/h and each became a lane change (15:53:19:
# the wheel pulled to -14 deg a second after the blinker); and under 20 km/h every blinker was a turn, lane changes
# and pull-overs too. On the 19 kept drives (10/12-10/27: 112 turns, 110 lane changes / pull-overs, every blinker
# >= 3 km/h; selfdrive/mapd/turn_ahead.py's OSM junctions, mapd_bridge twice a second; tools in E:/Temp/turnahead):
#  * The driver signals a median 39 m / 8.5 s before the turn (the heading 30 deg round), is under 20 km/h from a
#    median 18 m out and turns the wheel past 15 deg a median 3.5 s / 14 m before; OSM's junction node sits a median
#    7 m past that point, and at the blinker OSM's nearest junction that way was an earlier one in 11 of 93.
#  * Turn desire (TURN_*): when OSM has a junction that way and either we are under TURN_SLOW_SPEED within
#    TURN_NEAR_M of it, or under TURN_TT_SPEED within TURN_TT_S of it (distance / speed); held once given until
#    the blinker goes off. Against "under 20 km/h": turns given it 98 -> 91 / 112 (OSM lacks some roads), lane
#    changes / pull-overs given it 17 -> 14 / 110, given at the right junction 109 -> 86, its lead before the turn
#    point P10 2.4 -> 3.2 s (median 5.8 -> 5.7). With the route (NavDash via the PiBar) the route's distance is used
#    instead. With no OSM (no fix, no map) it falls back to "under 20 km/h".
#  * Lane change hold (A, HOLD_*): 20-50 km/h, OSM has a junction that way within HOLD_OSM_M and the car slowed
#    HOLD_SLOWING in the first HOLD_WATCH_S of the blinker (the lane change waits those seconds to see), or the route
#    turns that way within HOLD_NAV_M. Replayed on the logged drives: turns that became a lane change 41/42 -> 16/42;
#    lane changes started 38/41 -> 31/41 (5 held, 12 later by a median 1.9 s).
#  * A rear-window caption QUIET_CAPTIONS (臨時停車, 路況不熟 - keypad or C3 voice, via the PiBar) showing: a blinker
#    is neither a turn nor a lane change.
# OSM_OFF_FLAG puts the OSM parts back to the old rules (the route and the captions still count).
HOLD_MIN_SPEED = LANE_CHANGE_SPEED_MIN
HOLD_MAX_SPEED = 50 * CV.KPH_TO_MS
HOLD_NAV_M = 150.
HOLD_OSM_M = 100.
HOLD_WATCH_S = 2.0
HOLD_SLOWING = 1 * CV.KPH_TO_MS
TURN_SLOW_SPEED = 20 * CV.KPH_TO_MS
TURN_NEAR_M = 60.
# 10/28 (the driver: "轉彎門檻可以放寬"): 25 -> 30 km/h. That day's turns taken at 25-35 km/h got the desire 1.2-2.7 s
# before the turn point or never (13:19:59 at 25.6 km/h became a lane change first). On 21 drives (10/12-10/28, 132
# turns, 120 lane changes, 27 pull-overs): turns given it under 3.5 s before the turn point 18 -> 8, given 109 -> 110,
# lane changes wrongly given it 8 -> 9, pull-overs 17 -> 16 - with TURN_MIN_S: the lane changes it wrongly caught at
# 25-30 km/h were all signalled 9-28 m before a junction, a turn there no car takes at that speed.
TURN_TT_SPEED = 30 * CV.KPH_TO_MS
TURN_TT_S = 6.0
TURN_MIN_S = 2.0                                 # at TURN_SLOW_SPEED and over, a junction closer in time is passed
QUIET_CAPTIONS = ('m_temp_stop', 'm_unfamiliar')
OSM_OFF_FLAG = '/data/turn_osm_off'
TURN_HOLD_FILE = '/dev/shm/turn_hold.json'     # for the HUD: turn / hold / quiet and why, rewritten when it changes


def _turn_due(d, v_ego):
  """A turn desire is due at d metres to the turn (OSM junction or route step) at v_ego."""
  if d is None or v_ego < LANE_TURN_SPEED_MIN:
    return False
  if v_ego < TURN_SLOW_SPEED and d <= TURN_NEAR_M:
    return True
  tt = d / max(v_ego, 1.0)
  return v_ego < TURN_TT_SPEED and tt <= TURN_TT_S and (v_ego < TURN_SLOW_SPEED or tt >= TURN_MIN_S)


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.turn_ahead = TurnAheadReader()
    self.turn_go = None             # the turn desire is on, and from what: 'nav' / 'osm' / 'speed'
    self.turn_hold = None           # no lane change, and why: 'nav' / 'osm'
    self.quiet = False              # a quiet caption: neither
    self.blink_prev = False
    self.blink_t = 0.0
    self.blink_v0 = 0.0
    self.osm_off = False
    self.osm_check = 0

  def _write_hold(self, side):
    try:
      tmp = TURN_HOLD_FILE + '.tmp'
      with open(tmp, 'w') as f:
        json.dump({'go': self.turn_go, 'hold': self.turn_hold, 'quiet': self.quiet, 'side': side, 't': time.monotonic()}, f)
      os.replace(tmp, TURN_HOLD_FILE)
    except OSError:
      pass

  def update_turn_hold(self, carstate, one_blinker, below_lane_change_speed):
    """turn_go / turn_hold / quiet above, and whether a lane change should wait to see (HOLD_WATCH_S)."""
    if self.osm_check % 100 == 0:
      self.osm_off = os.path.exists(OSM_OFF_FLAG)
    self.osm_check += 1
    v_ego = carstate.vEgo
    side = 'L' if carstate.leftBlinker else 'R'
    was = (self.turn_go, self.turn_hold, self.quiet)
    nav, d_osm = None, None
    if one_blinker and not self.blink_prev:
      self.blink_t, self.blink_v0 = 0.0, v_ego
      self.turn_go, self.turn_hold, self.quiet = None, None, False
    self.blink_prev = one_blinker
    wait = False
    if not one_blinker:
      self.turn_go, self.turn_hold, self.quiet = None, None, False
    else:
      self.blink_t += DT_MDL
      ta = self.turn_ahead.get()
      self.quiet = ta.get('caption') in QUIET_CAPTIONS
      nav = ta.get('nav')
      d_nav = nav[1] if nav and nav[0] == side else None
      osm = bool(ta.get('osm')) and not self.osm_off
      d_osm = ta.get(side) if osm else None
      if self.quiet:
        self.turn_go = None
      elif self.turn_go is None:
        if _turn_due(d_nav, v_ego):
          self.turn_go = 'nav'
        elif osm:
          if _turn_due(d_osm, v_ego):
            self.turn_go = 'osm'
        elif LANE_TURN_SPEED_MIN <= v_ego < LANE_TURN_SPEED_MAX:
          self.turn_go = 'speed'           # no OSM to ask: the old rule
      if self.turn_hold is None and not self.quiet and HOLD_MIN_SPEED <= v_ego < HOLD_MAX_SPEED:
        if d_nav is not None and d_nav <= HOLD_NAV_M:
          self.turn_hold = 'nav'
        elif d_osm is not None and d_osm <= HOLD_OSM_M:
          if self.blink_t < HOLD_WATCH_S:
            wait = True
          elif self.blink_t < HOLD_WATCH_S + 2 * DT_MDL and self.blink_v0 - v_ego >= HOLD_SLOWING:
            self.turn_hold = 'osm'
    if (self.turn_go, self.turn_hold, self.quiet) != was:
      self._write_hold(side)
      # in the log too (2026-10-30: a turn taken from the route could not be told apart afterwards - the route's
      # step was nowhere in the log)
      cloudlog.event("turn_decision", go=self.turn_go, hold=self.turn_hold, quiet=self.quiet, side=side if one_blinker else None,
                     v=round(v_ego * 3.6, 1), nav=nav, osm=d_osm)
    return wait

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob,
             left_edge_detected=False, right_edge_detected=False):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN
    turn_wait = self.update_turn_hold(carstate, one_blinker, below_lane_change_speed)
    # a turn, a held turn or a quiet caption is no lane change, the same as under the lane change speed
    below_lane_change_speed = below_lane_change_speed or self.turn_hold is not None or self.turn_go is not None or self.quiet

    self.alc.update_params()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or        self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
    else:
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        # a road edge too close to move into counts the same as an occupied blind spot
        blindspot_detected = (((carstate.leftBlindspot or left_edge_detected) and self.lane_change_direction == LaneChangeDirection.left) or
                              ((carstate.rightBlindspot or right_edge_detected) and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif (torque_applied or self.alc.auto_lane_change_allowed) and not blindspot_detected and not turn_wait:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if self.turn_hold is not None or self.turn_go is not None or self.quiet:   # it turned out to be a turn
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
        elif lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          self.lane_change_timer = 0.0
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

    self.alc.update_state()

    self.prev_one_blinker = one_blinker and lateral_active

    self.desire = log.Desire.none
    if self.lane_change_state == LaneChangeState.laneChangeStarting:
      if self.lane_change_direction == LaneChangeDirection.left:
        self.desire = log.Desire.laneChangeLeft
      elif self.lane_change_direction == LaneChangeDirection.right:
        self.desire = log.Desire.laneChangeRight
    elif lateral_active and one_blinker and self.turn_go is not None:
      # The turn desire (2026-10-15, the driver's call, for the big junction turns), given when turn_go says the
      # turn is due (10-27, see TURN_* above). A blind spot here is a car alongside in the junction, so hold the
      # desire back the same way a lane change would be held back
      if carstate.leftBlinker and not (carstate.leftBlindspot or left_edge_detected):
        self.desire = log.Desire.turnLeft
      elif carstate.rightBlinker and not (carstate.rightBlindspot or right_edge_detected):
        self.desire = log.Desire.turnRight
