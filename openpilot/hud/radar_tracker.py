"""Pick out and steady the radar's other targets, for display only.

radard is handed a single target so that a guardrail cannot be mistaken for the car ahead,
which means the cars either side never reach the display. The full track list is on the bus
regardless, so this reads it here.

The decode is not repeated: the dbc, the slot handling and the thresholds all come from
opendbc, the same ones that feed the car, and only the choice of which targets to show is
made here. Nothing in this file controls the car.
"""
import math

from opendbc.can import CANParser
from openpilot.selfdrive.pandad.pandad_api_impl import can_capnp_to_list
from opendbc.car.hyundai.radar_interface import (CUSTIN_MIN_HITS, CUSTIN_MIN_RANGE,
                                                 CUSTIN_MIN_SCORE, CUSTIN_RADAR_ADDRS,
                                                 CustinSlot)

LANE_HALF = 5.5          # keep this lane and the two beside it; roadside sits outside
STATIC_TH = 2.5          # |vRel + vEgo| under this is something standing still
MIN_RANGE = 2.0          # closer than this is bumper clutter

LANE_NAME = {-2: '右右', -1: '右', 0: '前', 1: '左', 2: '左左'}


class RadarReader:
  """Every track on the bus, decoded with opendbc's own dbc and thresholds."""

  def __init__(self):
    self.rcp = CANParser("custin_radar", [(f"RADAR_TRACK_{a:x}", 33) for a in CUSTIN_RADAR_ADDRS], 1)
    self.slots = {a: CustinSlot() for a in CUSTIN_RADAR_ADDRS}
    self.hits = dict.fromkeys(CUSTIN_RADAR_ADDRS, 0)

  def update(self, can_strings, t):
    if not can_strings:
      return []
    self.rcp.update(can_capnp_to_list(can_strings))
    out = []
    for a in CUSTIN_RADAR_ADDRS:
      msg = self.rcp.vl[f"RADAR_TRACK_{a:x}"]
      rng = msg["LONG_DIST"]
      az = math.radians(msg["AZIMUTH"])
      if rng > CUSTIN_MIN_RANGE and msg["SCORE"] >= CUSTIN_MIN_SCORE:
        self.slots[a].update(t, math.cos(az) * rng)
        self.hits[a] = min(self.hits[a] + 1, CUSTIN_MIN_HITS)
      else:
        self.slots[a].reset()
        self.hits[a] = 0
      fit = self.slots[a].solve()
      if fit is not None and self.hits[a] >= CUSTIN_MIN_HITS:
        out.append({"dRel": fit[0], "yRel": -math.sin(az) * rng, "vRel": fit[1]})
    return out


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
