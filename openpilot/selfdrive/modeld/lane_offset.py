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

Bends (2026-10-31): the model cuts to the inside of a bend - on 10-31 left bends 0.15-0.32 m left of centre (towards
the oncoming lane), right bends 0.10-0.45 m right. compensation.py learns, from each drive, how far it cuts for each
band of its own desiredCurvature; the opposite of that is added here (the same shear), BEND_MAX at most, moving at
BEND_RATE, while there is no blinker or lane change and the car is over BEND_SPEED_MIN. BEND_FLAG_OFF turns that part
off. What it applied is logged ('bend_offset', twice a second while not zero) so it can be checked against the drive.
"""
import json
import os
import time

import numpy as np

from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.mapd import compensation

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
BEND_MAX = 0.4           # m
BEND_RATE = 0.15         # m/s
BEND_SPEED_MIN = 15 / 3.6
BEND_TAU = 0.5           # s, smoothing the model's curvature (it flickers frame to frame)
BEND_RELOAD_S = 60.      # s between reads of the learned table
BEND_FLAG_OFF = '/data/bend_offset_off'


class LaneOffset:
  def __init__(self):
    self.offset = 0.
    self.want = False
    self.cond_t = 0.         # s the condition has held in its current state, against `want`
    self.disabled = os.path.exists(FLAG_OFF)
    self.frame = 0
    self._out_t = 0.
    self._out_last = None
    self.straight = 0.       # the rightmost-lane part
    self.bend = 0.           # the bend part
    self.k_f = 0.
    self.bend_k, self.bend_o = None, None
    self.bend_disabled = os.path.exists(BEND_FLAG_OFF)
    self._bend_load_t = -1e9
    self._bend_log_t = 0.

  def _load_bend(self):
    """The learned table: desiredCurvature band centre -> how far the model sits off centre there (+ = left)."""
    p = compensation.load()
    k, o = [0.], [0.]
    if p and p.get('bend_offset'):
      for kc, oc in zip(p['bend_k'], p['bend_offset'], strict=True):
        if oc is not None and abs(kc) > 1e-4:
          k.append(kc)
          o.append(oc)
    order = np.argsort(k)
    self.bend_k, self.bend_o = np.array(k)[order], np.array(o)[order]

  def bend_target(self, md, car_state, lane_change_state, v_ego):
    k = float(md.action.desiredCurvature)
    self.k_f += (k - self.k_f) * min(DT / BEND_TAU, 1.)
    if (self.bend_disabled or self.bend_k is None or len(self.bend_k) < 2 or v_ego < BEND_SPEED_MIN
        or car_state.leftBlinker or car_state.rightBlinker or lane_change_state != log.LaneChangeState.off):
      return 0.
    cut = float(np.interp(self.k_f, self.bend_k, self.bend_o))
    return float(np.clip(-cut, -BEND_MAX, BEND_MAX))

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
      self.bend_disabled = os.path.exists(BEND_FLAG_OFF)
    now = time.monotonic()
    if now - self._bend_load_t > BEND_RELOAD_S:
      self._bend_load_t = now
      try:
        self._load_bend()
      except Exception:
        cloudlog.exception("lane_offset: bend table")
        self.bend_k = None
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
    self.straight = min(self.straight + step, target) if self.straight < target else max(self.straight - step, target)
    bt = self.bend_target(md, car_state, lane_change_state, v_ego)
    step = BEND_RATE * DT
    self.bend = min(self.bend + step, bt) if self.bend < bt else max(self.bend - step, bt)
    self.offset = self.straight + self.bend
    if abs(self.bend) > 0.02 and now - self._bend_log_t >= 0.5:
      self._bend_log_t = now
      cloudlog.event("bend_offset", bend=round(self.bend, 3), k=round(self.k_f, 5), offset=round(self.offset, 3))
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
        json.dump({'t': now, 'on': self.want, 'offset': round(self.offset, 3), 'disabled': self.disabled,
                   'bend': round(self.bend, 3)}, f)
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
