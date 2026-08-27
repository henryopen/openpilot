"""Read the Custin's front radar track list off bus 1, for display only.

radard is handed a single target so that a guardrail cannot be mistaken for the car
ahead, which means the cars either side never reach the display. The full track list is
on the bus regardless, so the HUD reads it here. Nothing in this file controls the car.

The 30 addresses are 10 targets of three messages each, not 30 separate tracks. Only the
first of each three carries the position: its FLAG byte stays 0 and its azimuth stays inside
about a quarter of the field, while the other two swing across the full +-1024 and their
range column sits in a narrow band. Sampled over 20 s of driving, the valid-frame counts of
0x238 and 0x239 match exactly (664/664) and the pattern repeats every three addresses, which
is what gives the grouping away. Reading all 30 produced two fake targets for every real one,
which is what made the display look nothing like traffic.

bus 1, 0x238 + 3n (n = 0..9), 33 Hz, 8 bytes:
  FLAG      bit 0-1
  LONG_DIST bit 2-11    x 0.1 m
  AZIMUTH   bit 18-28   signed, x 0.0388 deg
  V_ABS     bit 34-44   signed, the target's own speed rather than a relative one

On 2026-08-27 this field was briefly re-read as a lateral offset rather than an angle,
because targets appeared to drift toward the centre of the screen as they approached and
the offset model made that go away. It was wrong. Replaying the logs and comparing the
radar's own lateral figure against the vision lead on the same car puts the angle within
0.01 m over 1170 frames in town, where the offset model sits 0.43 m out and is the closer
of the two in only 12% of frames.

The three camera-measured targets used to justify the change could not tell the two apart:
their slant ranges were 24.0, 33.5 and 30.4 m, and over so narrow a span sin(raw*s)*rng and
raw*k have the same shape. The one frame that could have separated them, the lead at
-0.44 deg, was set aside as a measurement error. It was the only real evidence there.

The list carries no relative speed, so it comes from a least squares fit of range over a
short window, which measured to 0.33 m/s against the stock ACC's own target.
"""
import math
from collections import deque

RADAR_ADDRS = list(range(0x238, 0x256, 3))   # 10 targets, three addresses each
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
    sl = self.slots.get(addr)      # the two companion addresses of each target are not tracks
    if sl is None:
      return
    m = parse_track(data)
    if m is None:
      sl.reset()
    else:
      sl.update(m, t)

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


LANE_NAME = {-2: '右右', -1: '右', 0: '前', 1: '左', 2: '左左'}


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


def lane_index(y_rel):
  return max(-2, min(2, int(round(y_rel / LANE_W))))


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
    g['oncoming'] = (g['vRel'] + v_ego) < ONCOMING_V   # 目標自己在倒著走 = 對向車
    out.append(g)
  return sorted(out, key=lambda g: g['dRel'])

# ── 顯示用的目標生命週期管理 ───────────────────────────────────────────────
# 每一格單獨算出來的 group 沒有身分：雷達 slot 會抖，同一台車在相鄰兩格可能落在不同
# slot，靠近 STATIC_TH 的目標也會在「靜止/移動」之間進出，所以直接畫出來會一直閃。
# 量測 12 秒的輸出：25 個目標裡有 11 個活不到 0.5 秒。這裡做的是量產雷達那三件事:
# 關聯給 id、連續命中才確認、掉格先滑行保持。
CONFIRM_HITS = 3         # 連續出現這麼多格才上畫面
COAST_MISS = 3           # 掉格容忍格數，10 Hz 下約 0.3 秒
ASSOC_D = 4.5            # 關聯門檻，公尺
ASSOC_Y = 2.5            # 關聯門檻，橫向公尺
V_SMOOTH = 0.35          # vRel 的低通，擬合窗口剛換時會噴尖峰
Y_SMOOTH = 0.2           # yRel 的低通。角度量到的橫向在 30 m 外約有 1.5 m 的雜訊，
                         # 壓得比距離兇一點才不會讓目標在車道之間游移
LANE_W = 3.5             # m, 一般車道寬
LANE_HYST = 0.7          # m, 要越過中線這麼多才改判車道，免得騎在線上的目標一直跳
ONCOMING_V = -2.0        # m/s, 絕對速度低於此 = 逆向而來。用絕對速度判，不是相對速度：
                         # 對向車的 vRel 跟「前方有車在急煞」一模一樣，只看相對速度會混淆
COAST_SHIFT = 2.0        # 掉格期間最多外推這麼多公尺。橫向沒有速度可推，只推縱向的話
                         # 目標會沿著固定的橫向位置直直逼近本車，對向車尤其明顯


class Target:
  __slots__ = ('id', 'dRel', 'yRel', 'vRel', 'lane', 'lane_n', 'oncoming', 'hits', 'misses', 'shift')

  def __init__(self, tid, g):
    self.id = tid
    self.dRel = g['dRel']
    self.yRel = g['yRel']
    self.vRel = g['vRel']
    self.lane = g.get('lane', '前')
    self.lane_n = lane_index(self.yRel)
    self.oncoming = g.get('oncoming', False)
    self.hits = 1
    self.misses = 0
    self.shift = 0.0

  def absorb(self, g):
    self.dRel = g['dRel']
    self.yRel = (1 - Y_SMOOTH) * self.yRel + Y_SMOOTH * g['yRel']
    self.vRel = (1 - V_SMOOTH) * self.vRel + V_SMOOTH * g['vRel']
    want = lane_index(self.yRel)
    if want != self.lane_n and abs(self.yRel - self.lane_n * LANE_W) > LANE_W / 2 + LANE_HYST:
      self.lane_n = want                 # 只有真的越過中線一段距離才改判
    self.lane = LANE_NAME[self.lane_n]
    self.oncoming = g.get('oncoming', False)
    self.hits += 1
    self.misses = 0
    self.shift = 0.0


class TargetTracker:
  def __init__(self):
    self.targets = []
    self._next_id = 1

  def update(self, groups, dt):
    for t in self.targets:                       # 掉格的先用相對速度往前推
      step = t.vRel * dt
      if t.misses > 0:                           # 已經在掉格：外推要封頂，否則會被推著撞過來
        room = COAST_SHIFT - abs(t.shift)
        step = 0.0 if room <= 0 else max(-room, min(room, step))
        t.shift += step
      t.dRel += step

    used = set()
    for g in groups:
      best, score = None, 1e9
      for t in self.targets:
        if id(t) in used:
          continue
        dd, dy = abs(t.dRel - g['dRel']), abs(t.yRel - g['yRel'])
        if dd > ASSOC_D or dy > ASSOC_Y:
          continue
        if dd + dy * 2 < score:
          score, best = dd + dy * 2, t
      if best is None:
        best = Target(self._next_id, g)
        self._next_id += 1
        self.targets.append(best)
      else:
        best.absorb(g)
      used.add(id(best))

    for t in self.targets:
      if id(t) not in used:
        t.misses += 1

    self.targets = [t for t in self.targets if t.misses <= COAST_MISS]
    return [t for t in self.targets if t.hits >= CONFIRM_HITS]

  def reset(self):
    self.targets = []
