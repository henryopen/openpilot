import json
import os
import time

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
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

# A blinker over 20 km/h that is a junction turn, not a lane change (2026-10-27, the driver's call "A+B"): that day
# five of nine junction turns were signalled at 24-40 km/h and each became a lane change (15:53:19: the wheel pulled
# to -14 deg a second after the blinker). Below TURN_CHECK_MAX a blinker is held off a lane change ("turn hold") when
#   B  the phone's route (NavDash, forwarded by the PiBar) turns that way within NAV_TURN_M, or
#   A  OSM has a junction with a road leaving that way within OSM_TURN_M and the car slowed OSM_SLOWING within the
#      first OSM_WATCH_S of the blinker (the lane change waits those seconds to see). OSM alone could not tell them
#      apart: over 110 blinkers at 20-50 km/h on 19 drives (10/12-10/27), a junction within 100 m caught 80% of the
#      turns but also 50% of the lane changes; with the slowing, 32/50 turns and 6/44 lane changes. Replayed through
#      this file on those drives (logged modelV2 / carState, lateral on, 20-50 km/h): turns that became a lane change
#      41/42 -> 16/42; lane changes 38/41 started -> 31/41 (5 held, 12 later by a median 1.9 s). Only with
#      OSM_FLAG, until the driver has seen those numbers.
# A held turn gets no turn desire above LANE_TURN_SPEED_MAX (at 30-40 km/h it could take the car into a side road
# early); below it the turn desire runs as before. mapd_bridge works out both, see selfdrive/mapd/turn_ahead.py.
TURN_CHECK_MAX = 50 * CV.KPH_TO_MS
NAV_TURN_M = 150.
OSM_TURN_M = 100.
OSM_WATCH_S = 2.0
OSM_SLOWING = 1 * CV.KPH_TO_MS
OSM_FLAG = '/data/turn_osm_on'
TURN_HOLD_FILE = '/dev/shm/turn_hold.json'     # for the HUD: the hold and why, rewritten when it changes


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.turn_ahead = TurnAheadReader()
    self.turn_hold = None           # None, or why: 'nav' / 'osm'
    self.blink_prev = False
    self.blink_t = 0.0
    self.blink_v0 = 0.0
    self.osm_on = False
    self.osm_check = 0

  def _write_hold(self, side):
    try:
      tmp = TURN_HOLD_FILE + '.tmp'
      with open(tmp, 'w') as f:
        json.dump({'hold': self.turn_hold, 'side': side, 't': time.monotonic()}, f)
      os.replace(tmp, TURN_HOLD_FILE)
    except OSError:
      pass

  def update_turn_hold(self, carstate, one_blinker, below_lane_change_speed):
    """The turn hold above, and whether a lane change should wait to see (OSM's first seconds)."""
    if self.osm_check % 100 == 0:
      self.osm_on = os.path.exists(OSM_FLAG)
    self.osm_check += 1
    v_ego = carstate.vEgo
    side = 'L' if carstate.leftBlinker else 'R'
    was = self.turn_hold
    if one_blinker and not self.blink_prev:
      self.blink_t, self.blink_v0, self.turn_hold = 0.0, v_ego, None
    self.blink_prev = one_blinker
    wait = False
    if not one_blinker:
      self.turn_hold = None
    else:
      self.blink_t += DT_MDL
      if self.turn_hold is None and not below_lane_change_speed and v_ego < TURN_CHECK_MAX:
        ta = self.turn_ahead.get()
        nav = ta.get('nav')
        if nav and nav[0] == side and nav[1] <= NAV_TURN_M:
          self.turn_hold = 'nav'
        elif self.osm_on and ta.get(side) is not None and ta[side] <= OSM_TURN_M:
          if self.blink_t < OSM_WATCH_S:
            wait = True
          elif self.blink_t < OSM_WATCH_S + 2 * DT_MDL and self.blink_v0 - v_ego >= OSM_SLOWING:
            self.turn_hold = 'osm'
    if self.turn_hold != was:
      self._write_hold(side)
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
    # a held turn is no lane change, the same as under the lane change speed
    below_lane_change_speed = below_lane_change_speed or self.turn_hold is not None

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

        if self.turn_hold is not None:      # the route turned up that way after the lane change began
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
    elif lateral_active and one_blinker and LANE_TURN_SPEED_MIN <= v_ego < LANE_TURN_SPEED_MAX:
      # Turn desire back on (2026-10-15, the driver's call, for the big junction turns), as it ran
      # before 09-28. A blind spot here is a car alongside in the junction, so hold the desire back
      # the same way a lane change would be held back
      if carstate.leftBlinker and not (carstate.leftBlindspot or left_edge_detected):
        self.desire = log.Desire.turnLeft
      elif carstate.rightBlinker and not (carstate.rightBlindspot or right_edge_detected):
        self.desire = log.Desire.turnRight
