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
MAJOR = ('motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified', 'motorway_link', 'trunk_link',
         'primary_link', 'secondary_link', 'tertiary_link')
NAV_MANEUVERS = ('turn', 'off_ramp', 'fork', 'roundabout', 'on_ramp')
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
      if classes is not None:
        ok = ok & classes[self.cls[sl]]
      rel = (self.brg[sl] - brg + 540) % 360 - 180          # + = to the right
      for side, m in (('R', (rel >= ARM_MIN) & (rel <= ARM_MAX)), ('L', (rel <= -ARM_MIN) & (rel >= -ARM_MAX))):
        if (m & ok).any() and (best[side] is None or d < best[side]):
          best[side] = float(d)
    return best['L'], best['R']


def nav_turn(nav, now):
  """(side 'L'/'R', metres) of the next route step if it turns / leaves the road, from NAV_FILE's content."""
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
    return side, max(0., float(step.get('dist') or 0))
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
