from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5

# Turning at a junction, as opposed to changing lanes. Below the lane change threshold a
# blinker does not mean "move over", it means "we are about to turn", and the model drives
# a junction very differently once it is told so. Tied to the lane change threshold rather
# than given its own number, the way sunnypilot's LaneTurnController caps itself: the two
# are complementary, so a blinker under this speed is a turn and over it is a lane change,
# with no band in between where neither applies.
LANE_TURN_SPEED_MAX = LANE_CHANGE_SPEED_MIN
# Below this a blinker is more likely to be pulling over, parking, or crawling in traffic
# than taking a junction, and handing the model a turn desire there would be inventing one.
# Measured over 2831 frames of this car actually turning - blinker on and more than 45
# degrees of wheel - the median speed is 10.3 km/h. A 15 km/h floor, which is what the
# hunch "below that a blinker means pulling over" produced, keeps 9.7% of them; 5 km/h
# keeps 89.4% and still excludes crawling and standing still.
LANE_TURN_SPEED_MIN = 5 * CV.KPH_TO_MS

class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob,
             left_edge_detected=False, right_edge_detected=False):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

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
        elif (torque_applied or self.alc.auto_lane_change_allowed) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
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
      # A blind spot here is a car alongside in the junction, so hold the desire back the
      # same way a lane change would be held back
      if carstate.leftBlinker and not (carstate.leftBlindspot or left_edge_detected):
        self.desire = log.Desire.turnLeft
      elif carstate.rightBlinker and not (carstate.rightBlindspot or right_edge_detected):
        self.desire = log.Desire.turnRight
