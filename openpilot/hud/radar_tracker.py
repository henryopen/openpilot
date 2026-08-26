"""Read the Custin's front radar track list off bus 1, for display only.

radard is handed a single target so that a guardrail cannot be mistaken for the car
ahead, which means the cars either side never reach the display. The full track list is
on the bus regardless, so the HUD reads it here. Nothing in this file controls the car.

bus 1, 0x238-0x255, 33 Hz, 8 bytes:
  FLAG      bit 0-1
  LONG_DIST bit 2-11    x 0.1 m
  AZIMUTH   bit 18-28   signed, x 0.0388 deg
  V_ABS     bit 34-44   signed, the target's own speed rather than a relative one

The list carries no relative speed, so it comes from a least squares fit of range over a
short window, which measured to 0.33 m/s against the stock ACC's own target.
"""
import math
from collections import deque

RADAR_ADDRS = list(range(0x238, 0x256))
AZ_SCALE = 0.0388        # deg per LSB, from video geometry
V_SCALE = 0.02526        # m/s per LSB
V_OFFSET = 2.587         # m/s. Both calibrated against the stock ACC, r=0.949
WIN = 12                 # ~0.36 s at 33 Hz
MIN_N = 6

LANE_HALF = 5.5          # keep this lane and the two beside it; roadside sits outside
STATIC_TH = 2.5          # |vRel + vEgo| under this is something standing still
MIN_RANGE = 2.0          # closer than this is bumper clutter


def _ext(d, off, ln, sg=False):
  v = int.from_bytes(d, 'big')
  sh = 64 - off - ln
  r = (v >> sh) & ((1 << ln) - 1)
  if sg and r >= (1 << (ln - 1)):
    r -= (1 << ln)
  return r


def parse_track(d):
  rng = _ext(d, 2, 10) * 0.1
  if rng <= 0.5:
    return None
  az = math.radians(_ext(d, 18, 11, True) * AZ_SCALE)
  v_abs = _ext(d, 34, 11, True) * V_SCALE + V_OFFSET
  return {"rng": rng, "dRel": math.cos(az) * rng, "yRel": -math.sin(az) * rng, "vAbs": v_abs}


class Slot:
  __slots__ = ('buf', 'y')

  def __init__(self):
    self.buf = deque(maxlen=WIN)
    self.y = 0.0

  def reset(self):
    self.buf.clear()

  def update(self, m, t):
    b = self.buf
    if b and (t - b[-1][0] > 0.2 or abs(m['dRel'] - b[-1][1]) > 3.0):
      b.clear()
    b.append((t, m['dRel']))
    self.y = m['yRel'] if len(b) == 1 else 0.7 * self.y + 0.3 * m['yRel']

  def solve(self):
    """Range and its rate over the window, or None if there is not enough to fit."""
    b = self.buf
    if len(b) < MIN_N:
      return None
    if len({round(r, 1) for _, r in b}) == 1:   # a range that never moves is a stale slot
      return None
    ts = [p[0] for p in b]
    rs = [p[1] for p in b]
    n = len(b)
    mt = sum(ts) / n
    mr = sum(rs) / n
    sxx = sum((t - mt) ** 2 for t in ts)
    if sxx <= 0:
      return None
    v = sum((t - mt) * (r - mr) for t, r in zip(ts, rs, strict=True)) / sxx
    return mr + v * (ts[-1] - mt), v


class RadarTracker:
  def __init__(self):
    self.slots = {a: Slot() for a in RADAR_ADDRS}

  def on_can(self, addr, data, t):
    m = parse_track(data)
    if m is None:
      self.slots[addr].reset()
    else:
      self.slots[addr].update(m, t)

  def points(self, v_ego):
    """[(addr, dRel, yRel, vRel, is_static)]"""
    out = []
    for a, sl in self.slots.items():
      s = sl.solve()
      if s is None:
        continue
      d, v = s
      out.append((a, d, sl.y, v, abs(v + v_ego) < STATIC_TH))
    return out


def cluster(pts, d_tol=4.0, y_tol=2.2, v_tol=6.0):
  """One car lights up several slots, so merge the ones sitting on top of each other."""
  groups = []
  for p in sorted(pts, key=lambda q: q[1]):
    for g in groups:
      if abs(g['dRel'] - p[1]) < d_tol and abs(g['yRel'] - p[2]) < y_tol and abs(g['vRel'] - p[3]) < v_tol:
        g['members'].append(p)
        n = len(g['members'])
        g['dRel'] = sum(m[1] for m in g['members']) / n
        g['yRel'] = sum(m[2] for m in g['members']) / n
        g['vRel'] = sum(m[3] for m in g['members']) / n
        break
    else:
      groups.append({"dRel": p[1], "yRel": p[2], "vRel": p[3], "members": [p]})
  return groups


def lane_of(y_rel):
  if y_rel > 5.4:
    return '左左'
  if y_rel > 1.8:
    return '左'
  if y_rel < -5.4:
    return '右右'
  if y_rel < -1.8:
    return '右'
  return '前'


def relevant(groups, v_ego):
  """What is worth drawing: this lane and the two beside it, moving, not the bumper."""
  out = []
  for g in groups:
    if abs(g['yRel']) > LANE_HALF:
      continue
    if abs(g['vRel'] + v_ego) < STATIC_TH:   # standing still, so scenery
      continue
    if g['dRel'] < MIN_RANGE:
      continue
    g['lane'] = lane_of(g['yRel'])
    g['oncoming'] = g['vRel'] < -(v_ego * 0.6)
    out.append(g)
  return sorted(out, key=lambda g: g['dRel'])
