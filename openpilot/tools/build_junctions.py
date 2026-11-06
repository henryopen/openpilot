"""On .201: every road junction in Taiwan for C4's turn-ahead check (mapd_bridge).

A junction is a node two or more drivable ways share (a way passing through counts once, a branch adds one).
For each, the roads leaving it: compass bearing to the next node along each way, the way's class, and whether a
car may drive out of the junction along it (oneway ways only one way). Written as one npz:
  lat, lon        int32, 1e-6 deg
  first, count    int32 / uint8, the junction's arms in the arm arrays
  brg             uint16, deg; cls  uint8 (CLASSES index); out  uint8 (1 = may drive out along it)
  ~/speedlimit/venv/bin/python build_junctions.py ~/speedlimit/taiwan-latest.osm.pbf junctions_tw.npz
"""
import math
import sys
from collections import defaultdict

import numpy as np
import osmium

CLASSES = ['motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified', 'residential', 'living_street',
           'motorway_link', 'trunk_link', 'primary_link', 'secondary_link', 'tertiary_link', 'service', 'road']
CI = {c: i for i, c in enumerate(CLASSES)}


class Ways(osmium.SimpleHandler):
  def __init__(self):
    super().__init__()
    self.ways = []                  # (cls, oneway, [(id, lat, lon)])
    self.use = defaultdict(int)

  def way(self, w):
    hw = w.tags.get('highway')
    if hw not in CI:
      return
    if hw == 'service' and w.tags.get('service') in ('parking_aisle', 'drive-through', 'driveway'):
      return
    if w.tags.get('access') in ('no', 'private') or w.tags.get('area') == 'yes':
      return
    ow = w.tags.get('oneway', 'no')
    if hw in ('motorway', 'motorway_link') and ow == 'no':
      ow = 'yes'
    oneway = 1 if ow in ('yes', 'true', '1') else (-1 if ow == '-1' else 0)
    pts = [(n.ref, n.location.lat, n.location.lon) for n in w.nodes if n.location.valid()]
    if len(pts) < 2:
      return
    self.ways.append((CI[hw], oneway, pts))
    for ref, _, _ in pts:
      self.use[ref] += 1


def bearing(a, b):
  dy = (b[0] - a[0]) * 111320.
  dx = (b[1] - a[1]) * 111320. * math.cos(math.radians(a[0]))
  return int(round(math.degrees(math.atan2(dx, dy)))) % 360


h = Ways()
h.apply_file(sys.argv[1], locations=True, idx='flex_mem')
print('ways', len(h.ways), flush=True)
arms = defaultdict(list)            # node -> [(brg, cls, out)]
loc = {}
for cls, oneway, pts in h.ways:
  for k, (ref, la, lo) in enumerate(pts):
    if h.use[ref] < 2:
      continue
    loc[ref] = (la, lo)
    if k + 1 < len(pts):            # along the way
      arms[ref].append((bearing((la, lo), pts[k + 1][1:]), cls, int(oneway >= 0)))
    if k > 0:                       # back along it
      arms[ref].append((bearing((la, lo), pts[k - 1][1:]), cls, int(oneway <= 0)))
refs = [r for r in arms if len(arms[r]) >= 3]     # a plain bend between two ways is not a junction
print('junctions', len(refs), flush=True)
lat = np.array([round(loc[r][0] * 1e6) for r in refs], dtype=np.int32)
lon = np.array([round(loc[r][1] * 1e6) for r in refs], dtype=np.int32)
count = np.array([min(len(arms[r]), 255) for r in refs], dtype=np.uint8)
first = np.zeros(len(refs), dtype=np.int32)
first[1:] = np.cumsum(count.astype(np.int64))[:-1]
flat = [a for r in refs for a in arms[r][:255]]
np.savez_compressed(sys.argv[2], lat=lat, lon=lon, first=first, count=count,
                    brg=np.array([a[0] for a in flat], dtype=np.uint16), cls=np.array([a[1] for a in flat], dtype=np.uint8),
                    out=np.array([a[2] for a in flat], dtype=np.uint8), classes=np.array(CLASSES))
print('arms', len(flat))
