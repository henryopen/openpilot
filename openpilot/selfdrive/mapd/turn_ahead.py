"""Is a turn coming on the blinker's side? For desire_helper, which otherwise calls every blinker over 20 km/h a lane
change - on 10-27 five of the day's nine junction turns were signalled at 24-40 km/h and openpilot started a lane
change on each (15:53:19: one second after the blinker, the wheel pulled to -14 deg before the driver took it).

Two sources, the driver's call (10-27, "A+B"):
  OSM   a junction ahead on our road with a road we may drive into leaving on that side (junctions_tw.npz, built on
        .201 from the Taiwan OSM extract by openpilot/tools/build_junctions.py: 574k junctions, their arms' bearings, classes
        and whether a car may drive out along them)
  nav   the phone's NavDash route (via the PiBar, hud server /nav): the next step is a turn / ramp / fork that way
mapd_bridge works out both twice a second and writes TURN_FILE; desire_helper reads it.
"""
import json
import math
import os
import time

import numpy as np

JUNCTIONS = '/data/media/0/osm/junctions_tw.npz'
TURN_FILE = '/dev/shm/turn_ahead.json'
NAV_FILE = '/dev/shm/nav_c4.json'          # written by hud/server.py from what the PiBar forwards
CELL = 0.005                               # deg, ~550 m: the grid the junctions are looked up in
LAT_M = 14.                                # a junction this far off our line is on another road
BEHIND_M = -8.                             # just passed it (GPS lag, a wide junction) still counts
# deg off our heading: a road leaving to that side. 30-160 (10/27): 40-140 missed a back-right fork (144 deg) and
# shallow slip roads; on 19 drives the wider window found 2 more turns and no more lane changes.
ARM_MIN, ARM_MAX = 30, 160
# Only junctions on our road (2026-10-30). The driver: "OSM 跟導航都沒路口 OP 為什麼要下 turn desire" - 10/29 17:46:02,
# right blinker at 19.5 km/h going up the ramp onto the viaduct: OSM's nearest junction with a road to the right
# was 47 m ahead and 7 m to the side on the road below (its arms 255 / 76 / 61 deg, none the way we drive, 151), and
# the next one was on the other carriageway of the divided road. So a junction counts only when one of its arms
# leads back the way we came (ON_ROAD_DEG): one-way into the junction, or two-way with the way ahead open or no way
# ahead (a T). A back arm one may drive out along while the way ahead is closed is the other carriageway: a left
# turn crosses the median to it (Taiwan drives on the right) - 10/29 13:55:01's left turn onto the ramp is mapped
# there - a right turn never leaves from it. Every blinker with lateral on, 21 routes 10/12-10/29
# (E:/Temp/d1030/onroad_eval.py): turns given the desire 105 -> 105 of 124, lane changes given it 29 -> 26 of 156,
# unclear 14 -> 12 of 61, 17:46:02 no longer.
ON_ROAD_DEG = 35.
MAJOR = ('motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified', 'motorway_link', 'trunk_link',
         'primary_link', 'secondary_link', 'tertiary_link')
# Ramps and forks are no turn (2026-10-30, the driver: lane changes on the motorway taken as turns). Leaving the
# motorway is a lane change to the right and then the ramp, whose bend the model follows by itself (10/29 17:46:51:
# it drew the viaduct ramp's 84 deg right bend with the desire straight). As turns, a right blinker for the exit lane
# held the lane change back (17:59:45, 46 km/h, OSM had no junction: the wheel pushed right to 306 and nothing).
NAV_MANEUVERS = ('turn', 'roundabout')
NAV_STALE_S = 5.0
FILE_STALE_S = 2.0
REACH_M = 150.                             # looked this far; desire_helper decides how near counts
FIX_FRESH_S = 2.0                          # an older GPS fix places us nowhere


class JunctionMap:
  def __init__(self, path=JUNCTIONS):
    z = np.load(path)
    self.lat = z['lat'].astype(np.float64) * 1e-6
    self.lon = z['lon'].astype(np.float64) * 1e-6
    self.first, self.count = z['first'], z['count'].astype(np.int64)
    self.brg, self.cls, self.out = z['brg'].astype(np.int64), z['cls'], z['out'].astype(bool)
    self.classes = [str(c) for c in z['classes']]
    key = self._key(self.lat, self.lon)
    self.order = np.argsort(key, kind='stable')
    self.keys = key[self.order]

  @staticmethod
  def _key(lat, lon):
    return np.floor(np.asarray(lat) / CELL).astype(np.int64) * 100000 + np.floor(np.asarray(lon) / CELL).astype(np.int64)

  def class_mask(self, names):
    return np.isin(np.array(self.classes), list(names))

  def _near(self, lat, lon):
    ci, cj = math.floor(lat / CELL), math.floor(lon / CELL)
    idx = []
    for di in (-1, 0, 1):
      for dj in (-1, 0, 1):
        k = (ci + di) * 100000 + (cj + dj)
        a, b = np.searchsorted(self.keys, [k, k + 1])
        idx.append(self.order[a:b])
    return np.concatenate(idx)

  def turn_ahead(self, lat, lon, brg, reach, classes=None):
    """Metres to the nearest junction ahead with a drivable road leaving to the left / right (None if none within
    reach). classes: a boolean mask over self.classes, or None for every class."""
    idx = self._near(lat, lon)
    if not len(idx):
      return None, None
    dy = (self.lat[idx] - lat) * 111320.
    dx = (self.lon[idx] - lon) * 111320. * math.cos(math.radians(lat))
    b = math.radians(brg)
    along = dx * math.sin(b) + dy * math.cos(b)
    cross = dx * math.cos(b) - dy * math.sin(b)
    keep = (along > BEHIND_M) & (along < reach) & (np.abs(cross) < LAT_M)
    best = {'L': None, 'R': None}
    for k, d in zip(idx[keep], along[keep], strict=True):
      f = self.first[k]
      sl = slice(f, f + self.count[k])
      ok = self.out[sl]
      arms = self.brg[sl]
      # is the junction on our road? (see ON_ROAD_DEG)
      back = np.abs((arms - (brg + 180.) + 540) % 360 - 180) <= ON_ROAD_DEG
      fwd = np.abs((arms - brg + 540) % 360 - 180) <= ON_ROAD_DEG
      ours = back.any() and ((~ok[back]).any() or not fwd.any() or ok[fwd].any())
      other = back.any() and not ours
      if not (ours or other):
        continue
      if classes is not None:
        ok = ok & classes[self.cls[sl]]
      rel = (arms - brg + 540) % 360 - 180          # + = to the right
      for side, m in (('R', (rel >= ARM_MIN) & (rel <= ARM_MAX)), ('L', (rel <= -ARM_MIN) & (rel >= -ARM_MAX))):
        if other and side == 'R':
          continue
        if (m & ok).any() and (best[side] is None or d < best[side]):
          best[side] = float(d)
    return best['L'], best['R']


def nav_turn(nav, now):
  """(side 'L'/'R', metres, maneuver) of the next route step if it turns, from NAV_FILE's content."""
  try:
    if nav is None or now - float(nav.get('mono', 0)) > NAV_STALE_S:
      return None
    n = nav.get('nav') or {}
    step = n.get('step') or {}
    if n.get('state') != 'navigating' or step.get('maneuver') not in NAV_MANEUVERS:
      return None
    mod = str(step.get('modifier') or '')
    side = 'L' if 'left' in mod else ('R' if 'right' in mod else None)
    if side is None:
      return None
    return side, max(0., float(step.get('dist') or 0)), str(step.get('maneuver'))
  except (TypeError, ValueError, AttributeError):
    return None


def caption_now(nav, now):
  """The rear-window caption showing (keypad / C3 voice, forwarded with the route), or None."""
  try:
    if nav is None or now - float(nav.get('mono', 0)) > NAV_STALE_S:
      return None
    c = nav.get('caption')
    return c if isinstance(c, str) else None
  except (TypeError, ValueError, AttributeError):
    return None


def read_nav():
  try:
    with open(NAV_FILE, encoding='utf-8') as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def write(state):
  tmp = TURN_FILE + '.tmp'
  with open(tmp, 'w', encoding='utf-8') as f:
    json.dump(state, f)
  os.replace(tmp, TURN_FILE)


class TurnAheadReader:
  """desire_helper's side: TURN_FILE at most twice a second; a stale or missing file means no turn known."""
  def __init__(self):
    self.state, self.read_t = {}, 0.

  def get(self):
    now = time.monotonic()
    if now - self.read_t >= 0.5:
      self.read_t = now
      try:
        with open(TURN_FILE, encoding='utf-8') as f:
          self.state = json.load(f)
      except (OSError, ValueError):
        self.state = {}
    if now - float(self.state.get('t', 0)) > FILE_STALE_S:
      return {}
    return self.state
