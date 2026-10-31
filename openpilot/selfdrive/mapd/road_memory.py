#!/usr/bin/env python3
"""Road memory: how fast this car has been driven here before, as a cap on cruise ahead (2026-10-30, the driver's
call on 10-29: "依照我們開過的路加減速...來修正模型的不足"; 10-30: "路段記憶要做"; "你做記憶要考慮到GPS跟圖資的準度").

Bends only, and only the driver's own speed (2026-10-30, the driver: "我開車都不會超速 所以速度記憶主要還是要用在轉彎
每天路況都不同速度學習跟本沒用"; "過彎速度你就看我接手有操作的時候速度是多少"): a point caps cruise only near a bend - the
wheel turned past BEND_ANG there (BEND_NEAR points either side), nowhere within BLINK_TURN_S of a blinker - and
only where the speed was the driver's - driving it themselves, on the throttle, or in the AFTER_BRAKE_S after
letting the brake go (what they braked it down to). A bend openpilot took without the driver touching anything has
no memory and no cap. The first try took openpilot's own free speed in the bends: replaying 10/28's mountain road
against itself, of 37 driver brakes near a bend it would have slowed for 3 - the brakes are where openpilot was too
fast, and those seconds were left out. 10-31 the bends came from the GPS heading over +-15 m: on the day's
drives 9 of 23 "would slow" were on a straight road (GPS heading jitter at low speed) and the rest mostly junction
turns. The driver: "很多直路都會顯示彎道 ... GPS跟地圖的判斷只用來作為換道或轉彎的判斷而且一定要打方向燈". So the wheel says
where the bends are, and junction turns (a blinker near) are not bends.

Blinkers (BlinkMemory, same traces): where each blinker came on, which side, and whether the car then turned or
changed lanes; desire_helper takes what the driver did there before (turn_ahead's 'mem').

The cases it is for: the mountain road's chain of bends, where cruise winds back up to the set speed between two
bends and arrives at the next too fast (10/28: SCC to 1.3-2.0 m/s^2 between bends, the driver braking), and any
bend or slope the model and curve_speed do not slow for the way the driver does.

Learning (road_memd, offroad, after a drive): every logged drive becomes a trace, one point every STEP_M along it
(GPS position, altitude, heading, speed, and what was going on: the lead, the last / next stop, the blinker, the
pedals, whether openpilot drove), plus every blinker and what followed it. Which points count is worked out when the
memory is loaded (driver_mask, bend_mask), so the rules can change without reading the logs again: the driver's
speed (above) near a bend, and the road's - no slower car holding us (within LEAD_FREE_S and not pulling away by
LEAD_AWAY), not slowing to a stop (PRE_STOP_S) or pulling away from one (POST_STOP_S), no blinker near (BLINK_S).

Using it (mapd_bridge, twice a second): the traces that pass within MATCH_M of us, heading within MATCH_DEG and
altitude within ALT_M (the GPS cannot tell lanes apart at 1.3-1.8 m, nor a viaduct from the road below it without
the altitude), followed AHEAD_M forward; per BIN_M of road ahead the fastest counted speed of each day, and of those
the fastest (the driver later taking a bend faster - on the throttle - lifts its cap), where at least MIN_DAYS days
agree on where that bin is (a turn-off one day does not cap the road ahead). The speed allowed now is the lowest over the bins
of what still slows to that bin's cap (plus MARGIN) at A_COMFORT, never under V_FLOOR. Written to OUT_FILE for
the planner (road_memory_limit) and the HUD.

Shadow until /data/road_mem_on exists (road_memory_limit.ON_FLAG).
"""
import datetime
import glob
import json
import math
import os
import sys
import time

import numpy as np

ROOT = '/data/media/0/osm/road_memory'
TRACE_DIR = os.path.join(ROOT, 'traces')
DONE_FILE = os.path.join(ROOT, 'done.json')
LOG_ROOT = '/data/media/0/realdata'
OUT_FILE = '/dev/shm/road_mem.json'

KPH = 1 / 3.6
STEP_M = 5.0
MIN_SPEED = 3.0                 # m/s: slower is a car park or a queue, no trace
MAX_H_ACC = 5.0                 # m: a looser fix places us nowhere
MAX_V_ACC = 8.0                 # m: a looser altitude cannot tell a viaduct from the road below
LEAD_FREE_S = 2.0               # s: a lead nearer than this ...
LEAD_AWAY = 1.0                 # m/s: ... and not this much faster than us is holding us
PRE_STOP_S = 6.
POST_STOP_S = 4.
BLINK_S = 6.
PRE_BRAKE_S = 4.
AFTER_BRAKE_S = 2.0             # s after the driver lets the brake go: the speed they braked it down to
BLINK_MIN_S = 1.0
BLINK_LOOK_S = 25.
BLINK_TURN_DEG = 50.
BLINK_LANE_DEG = 20.
STOP_V = 1.0                    # m/s
BREAK_M = 30.                   # a gap this long in a trace (no fix, stopped) ends the forward walk

MATCH_M = 10.
MATCH_DEG = 25.
ALT_M = 6.
AHEAD_M = 200.
BIN_M = 10.
MIN_DAYS = 1                    # one drive's own correction in a bend is the driver's call there
AGREE_M = 15.                   # m: two days' points for the same bin ahead are on the same road
A_COMFORT = 1.0                 # m/s^2
# s: the planner follows the allowed speed with a lag (road_memory_limit TAU 1.5 s, plus the twice-a-second lookup), so the
# slowdown for a bin starts this much earlier - without it the car reaches the bin ~7 km/h over
LEAD_S = 3.0
MARGIN = 2 * KPH
V_FLOOR = 15 * KPH
CELL = 20.                      # m, the lookup grid
# A bend: tighter than this radius. On the logged drives (free points) the speed follows the radius only there -
# medians 38 / 34 / 29 / 22 / 18 km/h at 200-500 / 100-200 / 50-100 / 25-50 / <25 m - while on the straights (83.5%
# of the road) it spreads 33-103 km/h with the traffic.
BEND_ANG = 15.                  # deg of wheel: about a 200 m radius at this car's steer ratio; lane changes stay under
BLINK_TURN_S = 15.              # s: a blinker this near makes it a junction turn, not a bend
BEND_NEAR = 6                   # points: the braking for a bend starts on the way in
# A bend is a bend every time through; one drive's wheel is not. With the wheel rule, 4 of the 6 'would slow' on 10-31's
# drives were still on a straight road: each from a single other day where the driver had the wheel at 16-25 deg there
# with no blinker (pulling out at the start of a drive, going round something), while every other pass went straight.
# So where other drives passed the same place (within AGREE_M, heading within MATCH_DEG, driven by anyone), a point is
# near a bend only if most of the drives there were.
BEND_AGREE = 0.5                # share of the drives through a place that must have been near a bend there
BLINK_MATCH_M = 50.             # a blinker this near, heading within MATCH_DEG, same side, is "here"
TW = datetime.timezone(datetime.timedelta(hours=8))


def _xy(lat, lon, lat0=25.0):
  return np.asarray(lon) * 111320. * math.cos(math.radians(lat0)), np.asarray(lat) * 111320.


def _angdiff(a, b):
  return np.abs((np.asarray(a) - b + 540.) % 360. - 180.)


# ---------------------------------------------------------------- learning (offroad)

def extract_route(route, log_root=LOG_ROOT):
  """One drive -> dict of arrays, one point every STEP_M while moving with a good fix, or None."""
  from openpilot.tools.lib.logreader import LogReader
  segs = sorted(int(d.rsplit('--', 1)[1]) for d in os.listdir(log_root) if d.startswith(route + '--'))
  cols = {k: [] for k in ('t', 'lat', 'lon', 'alt', 'brg', 'hacc', 'vacc', 'gspeed', 'v', 'gas', 'brake', 'en', 'lead_d',
                          'lead_v', 'bl', 'br', 'ang', 'unix')}
  st = {'v': 0., 'gas': False, 'brake': False, 'en': False, 'lead_d': 1e3, 'lead_v': 0., 'bl': False, 'br': False, 'ang': 0.}
  for s in segs:
    p = os.path.join(log_root, f'{route}--{s}', 'rlog.zst')
    if not os.path.exists(p):
      p = os.path.join(log_root, f'{route}--{s}', 'rlog')
      if not os.path.exists(p):
        continue
    try:
      for m in LogReader(p):
        w = m.which()
        if w == 'carState':
          c = m.carState
          st.update(v=c.vEgo, gas=c.gasPressed, brake=c.brakePressed, bl=c.leftBlinker, br=c.rightBlinker,
                    ang=c.steeringAngleDeg)
        elif w == 'selfdriveState':
          st['en'] = m.selfdriveState.enabled
        elif w == 'radarState':
          ld = m.radarState.leadOne
          st['lead_d'], st['lead_v'] = (float(ld.dRel), float(ld.vLead)) if ld.present else (1e3, 0.)
        elif w == 'gpsLocationExternal':
          g = m.gpsLocationExternal
          if not g.hasFix or (g.latitude == 0 and g.longitude == 0):
            continue
          cols['t'].append(m.logMonoTime * 1e-9)
          cols['lat'].append(g.latitude)
          cols['lon'].append(g.longitude)
          cols['alt'].append(g.altitude)
          cols['brg'].append(g.bearingDeg)
          cols['hacc'].append(g.horizontalAccuracy)
          cols['vacc'].append(g.verticalAccuracy)
          cols['gspeed'].append(g.speed)
          cols['unix'].append(g.unixTimestampMillis * 1e-3)
          for k in ('v', 'gas', 'brake', 'en', 'lead_d', 'lead_v', 'bl', 'br', 'ang'):
            cols[k].append(st[k])
    except Exception:  # a corrupt segment loses that minute, not the drive
      continue
  if len(cols['t']) < 20:
    return None
  a = {k: np.asarray(v, dtype=np.float64) for k, v in cols.items()}
  o = np.argsort(a['t'])
  a = {k: v[o] for k, v in a.items()}
  t, v = a['t'], a['v']

  def near(mask, before, after):
    """True where a sample with mask lies within [t - before, t + after]."""
    tm = t[mask.astype(bool)]
    if not len(tm):
      return np.zeros(len(t), bool)
    i = np.searchsorted(tm, t - before)
    j = np.searchsorted(tm, t + after, side='right')
    return j > i

  def since_until(mask):
    """Seconds since the last and until the next sample with mask (99 if none)."""
    tm = t[mask.astype(bool)]
    if not len(tm):
      return np.full(len(t), 99.), np.full(len(t), 99.)
    i = np.searchsorted(tm, t, side='right')
    since = np.where(i > 0, t - tm[np.maximum(i - 1, 0)], 99.)
    j = np.searchsorted(tm, t, side='left')
    until = np.where(j < len(tm), tm[np.minimum(j, len(tm) - 1)] - t, 99.)
    return np.minimum(since, 99.), np.minimum(until, 99.)

  stopped = v < STOP_V
  op_brake = (a['brake'] > 0.5) & (a['en'] > 0.5)
  stop_since, stop_until = since_until(stopped)
  blink = (a['bl'] > 0.5) | (a['br'] > 0.5)
  blink_since, blink_until = since_until(blink)
  brake = a['brake'] > 0.5
  brake_since, _ = since_until(brake)
  good = (v >= MIN_SPEED) & (a['hacc'] <= MAX_H_ACC)
  # one point every STEP_M along the road
  x, y = _xy(a['lat'], a['lon'])
  idx = np.flatnonzero(good)
  if len(idx) < 2:
    return None
  keep, s_keep, last_xy, s = [], [], None, 0.
  for i in idx:
    if last_xy is not None:
      d = math.hypot(x[i] - last_xy[0], y[i] - last_xy[1])
      if d < STEP_M:
        continue
      s += d
    keep.append(i)
    s_keep.append(s)
    last_xy = (x[i], y[i])
  keep = np.asarray(keep)
  out = {k: a[k][keep].astype(np.float32 if k not in ('lat', 'lon', 'unix') else np.float64)
         for k in ('lat', 'lon', 'alt', 'brg', 'vacc', 'v', 'unix')}
  out['s'] = np.asarray(s_keep, np.float32)
  out['lead_d'] = a['lead_d'][keep].astype(np.float32)
  out['lead_v'] = a['lead_v'][keep].astype(np.float32)
  out['pedal'] = ((a['gas'] > 0.5) | brake)[keep]
  out['ang'] = a['ang'][keep].astype(np.float32)
  out['gas'] = (a['gas'] > 0.5)[keep]
  out['after_brk'] = (~brake & (brake_since <= AFTER_BRAKE_S))[keep]   # the speed the driver braked it down to
  out['stop_since'] = stop_since[keep].astype(np.float32)
  out['stop_until'] = stop_until[keep].astype(np.float32)
  out['blink_near'] = np.minimum(blink_since, blink_until)[keep].astype(np.float32)
  out['brk_soon'] = near(op_brake, 0., PRE_BRAKE_S)[keep]
  out['en'] = a['en'][keep] > 0.5            # not used to drive: for replaying a drive against the other days
  out.update(blink_events(a))
  day = datetime.datetime.fromtimestamp(float(np.median(a['unix'])), TW).strftime('%Y-%m-%d')
  return out, day


def blink_events(a):
  """Every blinker of a drive (one side on for >= BLINK_MIN_S, gaps under 2 s are one), where it came on and what the
  car then did: the heading within BLINK_LOOK_S turned past BLINK_TURN_DEG that way = a turn (1), stayed within
  BLINK_LANE_DEG = a lane change (0), else unclear (-1, not used)."""
  t = a['t']
  ev = {k: [] for k in ('blk_lat', 'blk_lon', 'blk_brg', 'blk_side', 'blk_out')}
  for side, key in ((0, 'bl'), (1, 'br')):
    on = np.flatnonzero(a[key] > 0.5)
    if not len(on):
      continue
    for run in np.split(on, np.flatnonzero(np.diff(t[on]) > 2.0) + 1):
      if t[run[-1]] - t[run[0]] < BLINK_MIN_S:
        continue
      i0 = run[0]
      w = np.flatnonzero((t >= t[i0] - 1) & (t <= t[i0] + BLINK_LOOK_S) & (a['gspeed'] > 1.5))
      if len(w) < 5 or a['hacc'][i0] > MAX_H_ACC or a['gspeed'][i0] < 1.0:
        continue
      h = np.degrees(np.unwrap(np.radians(a['brg'][w])) - math.radians(a['brg'][w[0]]))
      peak = h[np.argmax(np.abs(h))]
      if abs(peak) > BLINK_TURN_DEG:
        out = 1 if (peak > 0) == (side == 1) else -1
      else:
        out = 0 if abs(peak) < BLINK_LANE_DEG else -1
      ev['blk_lat'].append(a['lat'][i0])
      ev['blk_lon'].append(a['lon'][i0])
      ev['blk_brg'].append(a['brg'][i0])
      ev['blk_side'].append(side)
      ev['blk_out'].append(out)
  return {k: np.asarray(v, np.float64 if k in ('blk_lat', 'blk_lon') else np.float32) for k, v in ev.items()}


def learn(log_root=LOG_ROOT, trace_dir=TRACE_DIR, done_file=DONE_FILE, only=None):
  """Every finished drive not yet learned -> trace_dir/<route>.npz. Returns how many were added."""
  os.makedirs(trace_dir, exist_ok=True)
  try:
    with open(done_file) as f:
      done = set(json.load(f))
  except (OSError, ValueError):
    done = set()
  routes = sorted({d.rsplit('--', 1)[0] for d in os.listdir(log_root) if d.count('--') == 2})
  added = 0
  for r in routes:
    if r in done or (only and r not in only):
      continue
    try:
      res = extract_route(r, log_root)
    except Exception as e:
      print(f'road_memory: {r} failed: {e}', file=sys.stderr)
      res = None
    if res is not None:
      tr, day = res
      np.savez_compressed(os.path.join(trace_dir, r + '.npz'), day=day, **tr)
      added += 1
    done.add(r)
    tmp = done_file + '.tmp'
    with open(tmp, 'w') as f:
      json.dump(sorted(done), f)
    os.replace(tmp, done_file)
  return added


def driver_mask(z):
  """Where the speed was the driver's (see the module doc) and the road's - no lead holding, no stop, no blinker."""
  v = z['v']
  held = (z['lead_d'] < LEAD_FREE_S * v) & (z['lead_v'] < v + LEAD_AWAY)
  drv = ~z['en'].astype(bool) | z['gas'].astype(bool) | z['after_brk'].astype(bool)
  return (drv & ~held & (z['stop_since'] > POST_STOP_S) & (z['stop_until'] > PRE_STOP_S) & (z['blink_near'] > BLINK_S))


def bend_mask(z):
  """Near a bend: the wheel past BEND_ANG (BEND_NEAR points either side, the braking for it starts on the way in), and
  no blinker within BLINK_TURN_S (that is a junction turn)."""
  bend = (np.abs(z['ang']) >= BEND_ANG) & (z['blink_near'] > BLINK_TURN_S)
  return np.convolve(bend.astype(float), np.ones(2 * BEND_NEAR + 1), 'same') > 0


# ---------------------------------------------------------------- using it (onroad)

class RoadMemory:
  def __init__(self, trace_dir=TRACE_DIR, exclude_day=None):
    parts = []
    for f in sorted(glob.glob(os.path.join(trace_dir, '*.npz'))):
      try:
        z = np.load(f)
        if 'ang' not in z:                       # a trace from before 10-31 20:00 lacks the wheel angle
          continue
        day = str(z['day'])
        if day == exclude_day:
          continue
        parts.append((z, day))
      except (OSError, ValueError, KeyError):
        continue
    self.n_traces = len(parts)
    if not parts:
      self.lat = np.zeros(0)
      return
    self.days = sorted({d for _, d in parts})
    day_id = {d: i for i, d in enumerate(self.days)}
    cat = lambda k: np.concatenate([z[k] for z, _ in parts])  # noqa: E731
    self.lat, self.lon, self.alt, self.brg = cat('lat'), cat('lon'), cat('alt'), cat('brg')
    self.vacc, self.v, self.s = cat('vacc'), cat('v'), cat('s')
    bend = np.concatenate([bend_mask(z) for z, _ in parts])
    self.free = np.concatenate([driver_mask(z) for z, _ in parts]) & bend   # the driver's speed near bends
    self.trace = np.concatenate([np.full(len(z['s']), i, np.int32) for i, (z, _) in enumerate(parts)])
    self.day = np.concatenate([np.full(len(z['s']), day_id[d], np.int32) for z, d in parts])
    self.x, self.y = _xy(self.lat, self.lon)
    keys = np.floor(self.x / CELL).astype(np.int64) * 1000003 + np.floor(self.y / CELL).astype(np.int64)
    self.order = np.argsort(keys, kind='stable')
    self.keys = keys[self.order]
    for k in np.flatnonzero(self.free):                  # see BEND_AGREE
      idx = self._near(float(self.x[k]), float(self.y[k]))
      idx = idx[(np.hypot(self.x[idx] - self.x[k], self.y[idx] - self.y[k]) < AGREE_M) &
                (_angdiff(self.brg[idx], self.brg[k]) < MATCH_DEG)]
      trs = np.unique(self.trace[idx])
      if len(trs) > 1:
        n_bend = sum(bool(bend[idx[self.trace[idx] == t]].any()) for t in trs)
        self.free[k] = n_bend > BEND_AGREE * len(trs)

  def __len__(self):
    return len(self.lat)

  def _near(self, x, y):
    ci, cj = math.floor(x / CELL), math.floor(y / CELL)
    out = []
    for di in (-1, 0, 1):
      for dj in (-1, 0, 1):
        k = (ci + di) * 1000003 + (cj + dj)
        a, b = np.searchsorted(self.keys, [k, k + 1])
        out.append(self.order[a:b])
    return np.concatenate(out)

  def lookup(self, lat, lon, alt, brg, vacc, v_now=0.):
    """{'v_allow', 'cap', 'd', 'days'} for the binding bin ahead, or None (no memory here)."""
    if not len(self) or vacc is None or vacc > MAX_V_ACC:
      return None
    x, y = _xy(lat, lon)
    idx = self._near(float(x), float(y))
    if not len(idx):
      return None
    d = np.hypot(self.x[idx] - x, self.y[idx] - y)
    ok = (d < MATCH_M) & (_angdiff(self.brg[idx], brg) < MATCH_DEG) & (np.abs(self.alt[idx] - alt) < ALT_M) & \
         (self.vacc[idx] <= MAX_V_ACC)
    idx, d = idx[ok], d[ok]
    if not len(idx):
      return None
    nb = int(AHEAD_M // BIN_M) + 1
    best = {}                                   # day -> per-bin fastest free speed
    for tr in np.unique(self.trace[idx]):
      m = self.trace[idx] == tr
      i0 = int(idx[m][np.argmin(d[m])])
      # walk forward along this trace (the traces sit one after another, so its end is where its id stops)
      j1 = int(np.searchsorted(self.trace, tr, side='right'))
      ss = self.s[i0:j1] - self.s[i0]
      gaps = np.flatnonzero(np.diff(ss) > BREAK_M)
      stop = len(ss) if not len(gaps) else gaps[0] + 1
      ss = ss[:stop]
      within = np.flatnonzero(ss <= AHEAD_M)
      if not len(within):
        continue
      j = i0 + within
      fr = self.free[j]
      if not fr.any():
        continue
      bins = (ss[within][fr] // BIN_M).astype(int)
      vv = self.v[j][fr]
      arr, px, py = best.setdefault(int(self.day[i0]), (np.full(nb, -1.), np.full(nb, np.nan), np.full(nb, np.nan)))
      for b, v, k in zip(bins, vv, j[fr], strict=True):
        if v > arr[b]:
          arr[b], px[b], py[b] = v, self.x[k], self.y[k]
    if len(best) < MIN_DAYS:
      return None
    stack = np.stack([b[0] for b in best.values()])
    bx = np.stack([b[1] for b in best.values()])
    by = np.stack([b[2] for b in best.values()])
    # a bin counts only where days agree on where it is: one day turned off at a junction ahead and another went on,
    # and the turn's side street must not cap the road ahead
    dd = np.hypot(bx[:, None, :] - bx[None, :, :], by[:, None, :] - by[None, :, :])
    agree = np.nan_to_num(dd, nan=1e9) < AGREE_M
    agree &= (stack >= 0)[:, None, :] & (stack >= 0)[None, :, :]
    keep = agree.sum(1) >= MIN_DAYS                      # day x bin: at least MIN_DAYS days (itself too) here
    stack = np.where(keep, stack, -1.)
    n_days = keep.sum(0)
    cap = stack.max(0)
    use = n_days >= MIN_DAYS
    if not use.any():
      return None
    dist = np.arange(nb) * BIN_M
    v_ok = np.sqrt((cap[use] + MARGIN) ** 2 + 2 * A_COMFORT * np.maximum(dist[use] - max(v_now, 0.) * LEAD_S, 0.))
    k = int(np.argmin(v_ok))
    return {'v_allow': float(max(v_ok[k], V_FLOOR)), 'cap': float(cap[use][k]), 'd': float(dist[use][k]),
            'days': int(n_days[use][k])}


class BlinkMemory:
  """What the driver did after a blinker here before: {'L': 'turn' / 'lane' / None, 'R': ...} by majority of the
  blinkers within BLINK_MATCH_M, heading within MATCH_DEG, that side (unclear ones left out; a tie is None)."""
  def __init__(self, trace_dir=TRACE_DIR, exclude_day=None):
    cols = {k: [] for k in ('blk_lat', 'blk_lon', 'blk_brg', 'blk_side', 'blk_out')}
    for f in sorted(glob.glob(os.path.join(trace_dir, '*.npz'))):
      try:
        z = np.load(f)
        if 'blk_out' not in z or str(z['day']) == exclude_day:
          continue
        for k in cols:
          cols[k].append(z[k])
      except (OSError, ValueError, KeyError):
        continue
    a = {k: np.concatenate(v) if v else np.zeros(0) for k, v in cols.items()}
    keep = a['blk_out'] >= 0
    self.lat, self.lon, self.brg = a['blk_lat'][keep], a['blk_lon'][keep], a['blk_brg'][keep]
    self.side, self.out = a['blk_side'][keep].astype(int), a['blk_out'][keep].astype(int)
    self.x, self.y = _xy(self.lat, self.lon)

  def __len__(self):
    return len(self.lat)

  def lookup(self, lat, lon, brg):
    res = {'L': None, 'R': None}
    if not len(self):
      return res
    x, y = _xy(lat, lon)
    near = (np.hypot(self.x - x, self.y - y) < BLINK_MATCH_M) & (_angdiff(self.brg, brg) < MATCH_DEG)
    for side, key in ((0, 'L'), (1, 'R')):
      o = self.out[near & (self.side == side)]
      if len(o):
        n_turn = int(o.sum())
        if 2 * n_turn > len(o):
          res[key] = 'turn'
        elif 2 * n_turn < len(o):
          res[key] = 'lane'
    return res


def write_out(state):
  tmp = OUT_FILE + '.tmp'
  with open(tmp, 'w') as f:
    json.dump(state, f)
  os.replace(tmp, OUT_FILE)


def main():
  """road_memd: offroad, learn every finished drive; a scan every 10 minutes."""
  try:
    os.nice(15)
  except OSError:
    pass
  from openpilot.common.swaglog import cloudlog
  while True:
    try:
      t0 = time.monotonic()
      n = learn()
      if n:
        cloudlog.event("road_memory learned", routes=n, s=round(time.monotonic() - t0, 1))
    except Exception:
      cloudlog.exception("road_memory learn failed")
    time.sleep(600)


if __name__ == "__main__":
  if len(sys.argv) > 1 and sys.argv[1] == 'learn':
    print('learned', learn(only=set(sys.argv[2:]) or None))
  else:
    main()
