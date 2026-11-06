"""Stopped at a red: pin the head that stopped us and read only that head (2026-11-03, overnight tl-overnight P1/P2).

The driver (11/02): "如果因為紅燈而停下來 停止的時候車子沒動畫面也沒動 只要釘住同一地方紅轉綠 就走 這樣綠燈會不會更準 也不會被其他的影響".
Before, tlwatchd's GO at a wait was "a red was seen in this wait, then a green ball anywhere within 12 deg and no red" -
any head: a pedestrian green, the next junction, another road's head. Now, while the car stands:

  find  the first FIND_S s of readings: a red VEHICLE head (the lamp model says vehicle - every red box gets a lamp read
        here, see LampCache; without it a red sign was pinned on 1029_182857 and GO'd on red) within AZ_MAX of straight
        ahead and above the horizon, at the same place in >= FIND_SHARE of the readings -> pinned (the biggest / most
        central). Its region = the head's boxes side by side, padded; only the red lamp boxed -> the rest of a
        landscape head to its right (Taiwan: red leftmost).
        Every other red box at a fixed place then (any kind - a low turn head reads as a pedestrian one) is kept too:
        'others', reported per side of the pinned head (tlwatchd uses the turn side with the blinker).
  read  the pinned red lamp lit = a red box on it, or its pixels >= 40% of their lit redness and >= 30% of the lit
        pixel count (night: the surroundings keep ~80% of the redness after it went dark). Green lit in the region =
        a green box (lightcls green -> ball, lamp model straight / left / right) or, with green pixels but no box,
        the lamp model on the region itself. -> state 'red' / 'go' (red dark and some green) / 'dark'.
        Dark REPIN_DARK_S on end: the pin was wrong or the head hidden - find again.

Offline (tl-overnight work/eval_stops.py, 27 judged C3 waits of 10/29 + 10/31, frames every 0.5 s, the live band, lamp
reads as live + every red box): GO on red 0 (the rule on the car: 1), missed 5 of 26 (13), median 0.5 s after the
green. The blinker is C4's: c3pin reports, tlwatchd decides (Wait.watch_c3_pin).
"""
import math
import time

import numpy as np

FOCAL = 2648.                     # px, full res (tici road camera)
FIND_S = 3.0                      # s of readings a head has to be red in before it is pinned (tlwatchd FIND_S)
FIND_SHARE = 0.5                  # ... in at least this share of them (a passing scooter's tail light is not)
REPIN_DARK_S = 12.0               # pinned head neither red nor go this long: find again
AZ_MAX = 12.0                     # deg: our head (C3_AZ_DEG on C4)
OTHER_AZ_MAX = 25.0               # deg: other fixed reds kept (a turn head stands off to the side)
LAMP_TTL = 1.0                    # s a lamp read of a box is reused (stopped: the box does not move)
CLS = ['green', 'green_arrow', 'off', 'pedestrian', 'red', 'yellow']   # c3lights CLS_NAMES


class Det:
  __slots__ = ('box', 'u', 'v', 'w', 'h', 'cls', 'lamp', 'az', 'el')

  def __init__(self, box, prob, lamp, ahead_u, horizon_v):
    self.box = box
    x1, y1, x2, y2 = box
    self.u, self.v, self.w, self.h = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
    self.cls = {k: float(p) for k, p in zip(CLS, prob, strict=True)}
    self.lamp = lamp or {}
    self.az = math.degrees(math.atan((self.u - ahead_u) / FOCAL))
    self.el = math.degrees(math.atan((horizon_v - self.v) / FOCAL))

  def p_red(self):
    return max(self.lamp.get('red', 0.), self.cls['red'])

  def p_ped(self):
    return max(self.lamp.get('pedestrian', 0.), self.cls['pedestrian'])

  def p_vehicle(self):
    return self.lamp.get('vehicle', 0.)

  def p_go(self):
    return max([self.lamp.get(k, 0.) for k in ('green', 'straight', 'left', 'right')] + [self.cls['green'], self.cls['green_arrow']])


def iou(a, b):
  ix = max(0., min(a[2], b[2]) - max(a[0], b[0]))
  iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
  inter = ix * iy
  return inter / max((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter, 1e-6)


def dedupe(dets, thr=0.5):
  """The finder is NMS-free: keep the first (highest score - find() sorts) of boxes overlapping >= thr."""
  out = []
  for d in dets:
    if all(iou(d.box, o.box) < thr for o in out):
      out.append(d)
  return out


def crop(rgb, r0, box):
  """Pixels of a full-res box out of the rows we have (rgb starts at full-res row r0), int16."""
  x1, y1, x2, y2 = (int(round(v)) for v in box)
  y1, y2 = max(y1 - r0, 0), min(y2 - r0, rgb.shape[0])
  x1, x2 = max(x1, 0), min(x2, rgb.shape[1])
  if y2 <= y1 or x2 <= x1:
    return None
  return rgb[y1:y2, x1:x2].astype(np.int16)


def redness(rgb, r0, box):
  """90th percentile of r - max(g, b) in the box (tlwatchd.redness)."""
  p = crop(rgb, r0, box)
  if p is None:
    return 0.
  return float(np.percentile(np.clip(p[..., 0] - np.maximum(p[..., 1], p[..., 2]), 0, None), 90))


def hot_count(rgb, r0, box):
  """Pixels of a lit red lamp: bright saturated red, or the white core of an over-exposed red (night)."""
  p = crop(rgb, r0, box)
  if p is None:
    return 0
  r, g, b = p[..., 0], p[..., 1], p[..., 2]
  return int(((r > 150) & (r - np.maximum(g, b) > 70)).sum() + ((r > 235) & (g > 190) & (b > 150)).sum())


def greenness(rgb, r0, box):
  """Signal green (Taiwan's LEDs are blue-green): bright pixels with g, b well above r."""
  p = crop(rgb, r0, box)
  if p is None:
    return 0
  r, g, b = p[..., 0], p[..., 1], p[..., 2]
  return int(((g > 130) & (g - r > 60) & (b > 0.55 * g)).sum())


def head_group(dets, seed, gap=1.6):
  """The boxes of the seed's head housing: same row, side by side with gaps below `gap` heights."""
  grp, changed = [seed], True
  while changed:
    changed = False
    for d in dets:
      if d in grp:
        continue
      for g in grp:
        hh = max(g.h, d.h)
        if abs(d.v - g.v) < 0.7 * hh and (max(d.box[0], g.box[0]) - min(d.box[2], g.box[2])) < gap * hh:
          grp.append(d)
          changed = True
          break
  return grp


def near(d, o):
  """Box d is at other head o's place (an upright head's green sits a cell lower than its red)."""
  return abs(d.u - o.u) < max(o.w, 8) and abs(d.v - o.v) < max(o.h, 8)


class LampCache:
  """The lamp model on single boxes, each place read at most once per LAMP_TTL (the car stands, boxes do not move)."""
  def __init__(self, fn):
    self.fn, self.cache = fn, {}

  def get(self, rgb, r0, box, now):
    key = tuple(int(v) // 6 for v in box)
    hit = self.cache.get(key)
    if hit is not None and now - hit[0] < LAMP_TTL:
      return hit[1]
    x1, y1, x2, y2 = box
    out = self.fn(rgb, [(x1, y1 - r0, x2, y2 - r0)])
    lm = out[0] if out else None
    self.cache = {k: v for k, v in self.cache.items() if now - v[0] < LAMP_TTL}
    self.cache[key] = (now, lm)
    return lm


class StopPin:
  """update() once per reading while stopped -> the 'pin' entry of det.json (or None while moving)."""
  def __init__(self, lamp_fn):
    self.lamps = LampCache(lamp_fn) if lamp_fn is not None else None
    self.reset()

  def reset(self):
    self.t0, self.seen, self.pinned, self.others, self.dark_since = None, [], None, [], None
    self.ms = 0.

  def update(self, now, stopped, boxes, probs, lamps, rgb, r0, horizon_v, ahead_u):
    """boxes full res, probs lightcls vectors, lamps c3lights.lamps() (None for boxes it did not read), rgb the rows
    the boxes were read from (starting at full-res row r0), horizon_v / ahead_u full res."""
    if not stopped:
      if self.t0 is not None:
        self.reset()
      return None
    t_start = time.perf_counter()
    if self.t0 is None:
      self.t0 = now
    dets = dedupe([Det(b, p, lm, ahead_u, horizon_v) for b, p, lm in zip(boxes, probs, lamps or [None] * len(boxes), strict=True)])
    if self.pinned is None:
      out = self._find(now, dets, rgb, r0)
    else:
      out = self._read(now, dets, rgb, r0)
    self.ms = (time.perf_counter() - t_start) * 1000
    out.update(since=round(now - self.t0, 1), ms=round(self.ms, 1))
    return out

  def _lamp(self, d, rgb, r0, now):
    if not d.lamp and self.lamps is not None:
      d.lamp = self.lamps.get(rgb, r0, d.box, now) or {}

  def _find(self, now, dets, rgb, r0):
    for d in dets:                       # every red box gets the lamp model: vehicle or not decides the pin
      if d.cls['red'] >= 0.5 or d.p_red() >= 0.5:
        self._lamp(d, rgb, r0, now)
    self.seen = [x for x in self.seen if x[0] > now - FIND_S - 1e-6] + [(now, dets)]
    if self.seen[-1][0] - self.seen[0][0] < FIND_S - 0.6:
      return {'state': 'finding'}
    cl = []                              # [biggest Det, count]
    for _, ds in self.seen:
      for d in ds:
        if d.p_red() < 0.5 or d.p_ped() >= 0.5 or d.p_vehicle() < 0.5 or abs(d.az) > AZ_MAX or d.el < 0:
          continue
        for c in cl:
          if abs(c[0].u - d.u) < max(20., c[0].w) and abs(c[0].v - d.v) < max(20., c[0].h):
            c[1] += 1
            c[0] = d if d.h >= c[0].h else c[0]
            break
        else:
          cl.append([d, 1])
    cl = [c for c in cl if c[1] >= FIND_SHARE * len(self.seen)]
    if not cl:
      return {'state': 'none'}
    seed = max(cl, key=lambda c: c[1] * c[0].h / (1. + abs(c[0].az) / 6.))[0]
    here = [d for d in dets if near(d, seed) and d.p_red() >= 0.5]     # the seed as boxed in this reading, if it is
    if here:
      seed = max(here, key=lambda d: d.h)
    grp = head_group(dets if here else dets + [seed], seed)
    x1, y1 = min(d.box[0] for d in grp), min(d.box[1] for d in grp)
    x2, y2 = max(d.box[2] for d in grp), max(d.box[3] for d in grp)
    ph = 0.3 * (y2 - y1)
    region = [x1 - ph, y1 - ph, x2 + ph, y2 + ph]
    if (region[2] - region[0]) < 3.5 * (region[3] - region[1]) and seed.w < 2.5 * seed.h:
      region[2] += 3.5 * seed.h          # only the red lamp boxed: the rest of the landscape head is to its right
    self.pinned = {'region': region, 'red_box': list(seed.box), 'u': seed.u, 'v': seed.v, 'az': seed.az, 'el': seed.el,
                   'red_ref': redness(rgb, r0, seed.box), 'hot_ref': hot_count(rgb, r0, seed.box), 'n_ref': 1}
    self.others = []                     # every other fixed red (any kind) in the window, outside our head
    ocl = []
    for _, ds in self.seen:
      for d in ds:
        if d.p_red() < 0.5 or d.el < 0 or abs(d.az) > OTHER_AZ_MAX:
          continue
        for c in ocl:
          if abs(c[0].u - d.u) < max(20., c[0].w) and abs(c[0].v - d.v) < max(20., c[0].h):
            c[1] += 1
            c[0] = d if d.h >= c[0].h else c[0]
            break
        else:
          ocl.append([d, 1])
    for c in ocl:
      o = c[0]
      if c[1] >= FIND_SHARE * len(self.seen) and not (region[0] <= o.u <= region[2] and region[1] <= o.v <= region[3]):
        self.others.append(o)
    self.dark_since = None
    return self._read(now, dets, rgb, r0)

  def _read(self, now, dets, rgb, r0):
    P = self.pinned
    x1, y1, x2, y2 = P['region']
    inside = [d for d in dets if x1 <= d.u <= x2 and y1 <= d.v <= y2]
    red_px = redness(rgb, r0, P['red_box'])
    hot = hot_count(rgb, r0, P['red_box'])
    red_box = any(d.p_red() >= 0.5 and abs(d.u - P['u']) < max(d.w, 8) and abs(d.v - P['v']) < max(d.h, 8) for d in inside)
    red_lit = red_box or (P['red_ref'] > 0 and red_px >= 0.4 * P['red_ref'] and hot >= 0.3 * max(P['hot_ref'], 1))
    if red_box and red_px > P['red_ref']:          # learn the lit level while it is plainly red
      P['red_ref'] = (P['red_ref'] * P['n_ref'] + red_px) / (P['n_ref'] + 1)
      P['n_ref'] += 1
    kinds = set()
    for d in inside:
      if d.p_red() >= 0.5:
        continue
      if d.cls['green'] >= 0.5 or d.cls['green_arrow'] >= 0.5 or d.p_go() >= 0.5:
        self._lamp(d, rgb, r0, now)
      if d.cls['green'] >= 0.5 or (d.lamp.get('green', 0.) >= 0.5 and max(d.lamp.get(k, 0.) for k in ('left', 'straight', 'right')) < 0.5):
        kinds.add('ball')
      for k in ('straight', 'left', 'right'):
        if d.lamp.get(k, 0.) >= 0.5:
          kinds.add(k)
      if not d.lamp and d.cls['green_arrow'] >= 0.5:
        kinds.add('arrow?')
    green_px = greenness(rgb, r0, P['region'])
    if not kinds and green_px >= 6 and self.lamps is not None:
      lm = self.lamps.get(rgb, r0, P['region'], now)    # green lit but no box on it: the lamp model on the head itself
      if lm:
        if lm.get('green', 0.) >= 0.5 and max(lm.get(k, 0.) for k in ('left', 'straight', 'right')) < 0.5:
          kinds.add('ball')
        kinds.update(k for k in ('straight', 'left', 'right') if lm.get(k, 0.) >= 0.5)
    state = 'red' if red_lit else 'go' if kinds else 'dark'
    side = {'left': {'red': False, 'go': False}, 'right': {'red': False, 'go': False}}
    for o in self.others:
      s = side['left' if o.u < P['u'] else 'right']
      o_red = any(d.p_red() >= 0.5 and near(d, o) for d in dets)
      s['red'] = s['red'] or o_red
      if not o_red:
        for d in dets:
          if near(d, o) and d.p_go() >= 0.5:
            self._lamp(d, rgb, r0, now)
            if d.p_go() >= 0.5 and d.p_vehicle() >= 0.5 and d.p_ped() < 0.5:
              s['go'] = True
    self.dark_since = None if state != 'dark' else (self.dark_since if self.dark_since is not None else now)
    out = {'state': state, 'red_lit': bool(red_lit), 'kinds': sorted(kinds), 'side': side,
           'u': round(P['u'] / 2, 1), 'v': round(P['v'] / 2, 1), 'az': round(P['az'], 1), 'el': round(P['el'], 1),
           'others': len(self.others)}
    if self.dark_since is not None and now - self.dark_since >= REPIN_DARK_S:
      self.seen, self.pinned, self.others, self.dark_since = [], None, [], None    # a new pin must be red first
    return out
