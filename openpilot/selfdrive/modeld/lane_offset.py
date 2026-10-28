"""Keep a little left of centre in the rightmost lane, by shifting the camera the model sees (2026-10-29).

The driver: "我說偏右是在一般道路上 路邊停著一堆摩托車 都快碰上了 我必須用手修正". Over 10/23-10/28 (9 drives), on
roads with both lane lines clear he corrected the steering 120 times: 92 in the rightmost lane, 72 of them (78%) to
the left, against 8 of 28 in the other lanes. openpilot holds every lane dead centre (within 0.05 m), which in the
rightmost lane leaves 0.48-0.68 m from the car's right side to the line, with parked scooters, cars, walls and
guardrails just beyond it. Driving himself he sits 0.10 m left of centre there (centre in the other lanes), and each
correction took the car a median 0.16 m left.

How, from sunnypilot's CameraOffset (modeld_v2/camera_offset_helper.py): the transform from the model's input to
the camera image is sheared so that the camera appears moved sideways - a pixel (y - cy) below the horizon is
sampled offset / height * (y - cy) further over, which for a point on the road is exactly a camera displaced by
`offset`. The model then drives its own path from where it believes the car is, so nothing downstream fights it:
+ offset reads as a camera moved right, the model steers back left, and the car settles `offset` left of where it
would have been. The model's lane lines and road edges come out in the shifted frame (the car looks centred).

Only while all of these hold (from the model's previous frame), else back to 0:
  both lane lines next to the car seen (prob > LINE_P), the lane LANE_W wide;
  rightmost: no lane line beyond the right one (prob < FAR_P) or the road edge within EDGE_GAP of it;
  straight (|desiredCurvature| < STRAIGHT_CURV), over MIN_SPEED, no blinker, no lane change.
On for ON_S and off for OFF_S before it changes its mind; the offset moves at RATE. FLAG_OFF turns it off.
"""
import json
import os
import time

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog

OFFSET = 0.15            # m to the left in the rightmost lane
RATE = 0.05              # m/s the offset moves at (0.15 m in 3 s)
LINE_P = 0.6
FAR_P = 0.3
EDGE_GAP = 1.5           # m from the right lane line to the road edge: nothing more than a shoulder beyond it
LANE_W = (2.4, 4.5)      # m
STRAIGHT_CURV = 0.003    # 1/m
MIN_SPEED = 20 / 3.6     # m/s
ON_S = 1.0
OFF_S = 1.0
FLAG_OFF = '/data/lane_offset_off'
OUT = '/dev/shm/lane_offset.json'   # for the HUD: written on a change and once a second
DT = 0.05


class LaneOffset:
  def __init__(self):
    self.offset = 0.
    self.want = False
    self.cond_t = 0.         # s the condition has held in its current state, against `want`
    self.disabled = os.path.exists(FLAG_OFF)
    self.frame = 0
    self._out_t = 0.
    self._out_last = None

  @staticmethod
  def rightmost_straight(md, car_state, lane_change_state, v_ego):
    if v_ego < MIN_SPEED or car_state.leftBlinker or car_state.rightBlinker:
      return False
    if lane_change_state != log.LaneChangeState.off:
      return False
    if len(md.laneLines) < 4 or len(md.laneLineProbs) < 4 or len(md.roadEdges) < 2:
      return False
    p = md.laneLineProbs
    if p[1] < LINE_P or p[2] < LINE_P:
      return False
    yl, yr = md.laneLines[1].y[0], md.laneLines[2].y[0]
    if not (LANE_W[0] < yr - yl < LANE_W[1]):
      return False
    if not (p[3] < FAR_P or md.roadEdges[1].y[0] - yr < EDGE_GAP):
      return False
    return abs(md.action.desiredCurvature) < STRAIGHT_CURV

  def update(self, md, car_state, lane_change_state, v_ego):
    """md: the modelV2 just published (the model's view one frame back). Returns the offset for the next frame."""
    self.frame += 1
    if self.frame % 20 == 0:
      self.disabled = os.path.exists(FLAG_OFF)
    cond = not self.disabled and self.rightmost_straight(md, car_state, lane_change_state, v_ego)
    if cond != self.want:
      self.cond_t += DT
      if self.cond_t >= (ON_S if cond else OFF_S):
        self.want, self.cond_t = cond, 0.
        cloudlog.event("lane_offset", on=bool(cond), offset=round(self.offset, 3))
    else:
      self.cond_t = 0.
    target = OFFSET if self.want else 0.
    step = RATE * DT
    self.offset = min(self.offset + step, target) if self.offset < target else max(self.offset - step, target)
    self._publish()
    return self.offset

  def _publish(self):
    now = time.monotonic()
    state = (self.want, round(self.offset, 2), self.disabled)
    if state == self._out_last and now - self._out_t < 1.0:
      return
    self._out_last, self._out_t = state, now
    try:
      tmp = OUT + '.tmp'
      with open(tmp, 'w') as f:
        json.dump({'t': now, 'on': self.want, 'offset': round(self.offset, 3), 'disabled': self.disabled}, f)
      os.replace(tmp, OUT)
    except OSError:
      pass


def apply_offset(model_transform, intrinsics, height, offset):
  """sunnypilot's shear: the camera seen as moved `offset` m sideways (+ = the model keeps the car left)."""
  if offset == 0.:
    return model_transform
  import numpy as np
  cy = intrinsics[1, 2]
  shear = np.eye(3, dtype=np.float32)
  shear[0, 1] = offset / height
  shear[0, 2] = -offset / height * cy
  return (shear @ model_transform).astype(np.float32)
