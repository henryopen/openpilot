#!/usr/bin/env python3
"""drive_report: what the longitudinal control and the turns actually did on a drive.

Answers three questions off the recorded logs rather than off impressions:
  - does the car overshoot the set speed, and does it hunt around it
  - how hard is it accelerating, versus what the profile allows
  - what happens to speed through a sharp turn

  ./drive_report.py <route-prefix>        e.g. 00000008--35b5e794c4
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.append('/data/openpilot')
from openpilot.tools.lib.logreader import LogReader

CV = 3.6  # m/s -> km/h


def collect(segs):
  """One row per carState, carrying the most recent plan/state seen alongside it."""
  rows = []
  geom = {}
  cur = {'aTarget': None, 'reason': None, 'sp': None, 'enabled': False, 'lead': False}
  for seg in segs:
    f = seg / 'rlog.zst'
    if not f.exists():
      continue
    try:
      for msg in LogReader(str(f)):
        w = msg.which()
        if w == 'carParams' and not geom:
          geom['steerRatio'] = float(msg.carParams.steerRatio)
          geom['wheelbase'] = float(msg.carParams.wheelbase)
        elif w == 'longitudinalPlan':
          p = msg.longitudinalPlan
          cur['aTarget'] = float(p.aTarget) if hasattr(p, 'aTarget') else None
          for name in ('longitudinalPlanSource', 'source'):
            if hasattr(p, name):
              cur['reason'] = str(getattr(p, name))
              break
        elif w == 'longitudinalPlanSP':
          # this branch's own reason enum, which unlike the stock source names the curve
          cur['sp'] = str(msg.longitudinalPlanSP.reason)
        elif w == 'radarState':
          cur['lead'] = bool(msg.radarState.leadOne.present)
        elif w == 'selfdriveState':
          cur['enabled'] = bool(msg.selfdriveState.enabled)
        elif w == 'carState':
          cs = msg.carState
          v_cruise = None
          for name in ('vCruise', 'vCruiseCluster'):
            if hasattr(cs, name):
              v = float(getattr(cs, name))
              if 0 < v < 200:
                v_cruise = v
                break
          if v_cruise is None and hasattr(cs, 'cruiseState'):
            v_cruise = float(cs.cruiseState.speed) * CV
          # what the driver sees is the cluster pair, not the control pair: the dash reads
          # vEgoCluster against vCruiseCluster, and the cluster speed is required to never
          # read low, so it sits a few percent above vEgo. Comparing the control pair
          # answers a different question from the one being asked.
          v_disp = float(getattr(cs, 'vEgoCluster', 0.0) or cs.vEgo) * CV
          vset_disp = float(getattr(cs, 'vCruiseCluster', 0.0) or 0.0) or v_cruise
          rows.append({
            't': msg.logMonoTime / 1e9,
            'v': float(cs.vEgo) * CV,
            'v_disp': v_disp,
            'vset_disp': vset_disp,
            'a': float(cs.aEgo),
            'angle': float(cs.steeringAngleDeg),
            'gas': bool(cs.gasPressed),
            'brake': bool(cs.brakePressed),
            'vset': v_cruise,
            **cur,
          })
    except Exception as e:
      print(f'  {seg.name}: {e}', file=sys.stderr)
  return rows, geom


# what longitudinal_planner.get_cruise_accel does to the acceleration budget in a corner
A_TOTAL_MAX_V, A_TOTAL_MAX_BP = [1.7, 3.2], [20., 40.]


def budget(v_ms, angle_deg, steer_ratio, wheelbase):
  """-> (lateral accel being pulled, longitudinal accel still allowed)."""
  a_total = float(np.interp(v_ms, A_TOTAL_MAX_BP, A_TOTAL_MAX_V))
  a_y = v_ms ** 2 * abs(angle_deg) * np.pi / 180 / (steer_ratio * wheelbase)
  return a_y, float(np.sqrt(max(a_total ** 2 - a_y ** 2, 0.)))


def budget_section(rows, geom):
  sr, wb = geom.get('steerRatio'), geom.get('wheelbase')
  print('\n=== 轉彎時還剩多少加速餘裕（摩擦圓限制）===')
  if not sr:
    print('  log 裡沒有 carParams，跳過')
    return
  print(f'  steerRatio {sr:.2f}  wheelbase {wb:.3f} m')
  eng = [r for r in rows if r['enabled']]
  if not eng:
    print('  這趟沒有 engaged 的資料')
    return
  pairs = [(abs(r['angle']), *budget(r['v'] / CV, r['angle'], sr, wb)) for r in eng]
  zero = [p for p in pairs if p[2] <= 0.01]
  print(f'  engaged {len(eng)} 幀中，縱向加速餘裕被歸零的有 {len(zero)} 幀 ({100 * len(zero) / len(eng):.2f}%)')
  if zero:
    print(f'    發生時方向盤角度：中位 {np.median([p[0] for p in zero]):.0f}°  最小 {min(p[0] for p in zero):.0f}°')
  print(f'  {"方向盤":>7} {"車速":>6} {"橫向a":>7} {"可用縱向a":>9}')
  for ang in (90, 180, 270, 360, 450, 540):
    for v_kph in (10, 20):
      a_y, a_x = budget(v_kph / CV, ang, sr, wb)
      print(f'  {ang:6.0f}° {v_kph:5.0f} {a_y:7.2f} {a_x:9.2f}' + ('   <-- 不能加速' if a_x <= 0.01 else ''))


def cruise_section(rows):
  """Speed holding: engaged, no lead, no pedals, set speed steady."""
  ok = [r for r in rows if r['enabled'] and not r['lead'] and not r['gas'] and not r['brake']
        and r['vset'] and r['v'] > 20]
  print(f'\n=== 定速追蹤（engaged、無前車、無踩踏板、>20 km/h）：{len(ok)} 幀 ===')
  if not ok:
    print('  沒有符合的片段')
    return
  err = np.array([r['v'] - r['vset'] for r in ok])
  print(f'  車速 − 設定速度：中位 {np.median(err):+.1f}  平均 {err.mean():+.1f} km/h')
  print(f'    p5 {np.percentile(err, 5):+.1f}   p95 {np.percentile(err, 95):+.1f}   最大 {err.max():+.1f}')
  for lim in (1, 2, 3, 5):
    print(f'    超過設定 {lim} km/h 的比例：{100 * (err > lim).mean():5.1f}%')

  # hunting: how often the acceleration command changes sign while holding speed
  at = np.array([r['aTarget'] for r in ok if r['aTarget'] is not None])
  if len(at) > 10:
    flips = int((np.diff(np.sign(at)) != 0).sum())
    span = ok[-1]['t'] - ok[0]['t']
    print(f'  aTarget 正負翻轉 {flips} 次 / {span:.0f}s  ({flips / max(span, 1) * 60:.1f} 次/分)')
    print(f'  aTarget 範圍 {at.min():+.2f} .. {at.max():+.2f} m/s²  中位 |a| {np.median(np.abs(at)):.3f}')


def accel_section(rows):
  print('\n=== 加速表現（起步／再加速，無前車）===')
  ok = [r for r in rows if r['enabled'] and not r['lead'] and not r['brake']]
  if not ok:
    print('  沒有符合的片段')
    return
  for lo, hi in [(0, 20), (20, 40), (40, 60), (60, 90)]:
    band = [r for r in ok if lo <= r['v'] < hi and r['vset'] and r['v'] < r['vset'] - 3]
    if len(band) < 5:
      continue
    a = np.array([r['a'] for r in band])
    at = np.array([r['aTarget'] for r in band if r['aTarget'] is not None])
    tgt = f'  指令中位 {np.median(at):+.2f}' if len(at) else ''
    print(f'  {lo:>2}-{hi:<3} km/h  n={len(band):<5} 實際加速中位 {np.median(a):+.2f}  p90 {np.percentile(a, 90):+.2f} m/s²{tgt}')


def turn_section(rows, angle_thresh=90.0):
  print(f'\n=== 轉彎（|方向盤| > {angle_thresh:.0f}°）===')
  runs, cur = [], []
  for r in rows:
    if abs(r['angle']) > angle_thresh:
      cur.append(r)
    else:
      if len(cur) > 20:
        runs.append(cur)
      cur = []
  if len(cur) > 20:
    runs.append(cur)
  if not runs:
    print('  這趟沒有大角度轉彎')
    return
  print(f'  共 {len(runs)} 次')
  print(f'  {"時長":>5} {"最大角":>7} {"進彎":>6} {"最低":>6} {"出彎":>6} {"aTarget最低":>10}  engaged')
  for run in runs[:15]:
    v = [r['v'] for r in run]
    at = [r['aTarget'] for r in run if r['aTarget'] is not None]
    eng = sum(r['enabled'] for r in run) / len(run)
    dt, ang = run[-1]['t'] - run[0]['t'], max(abs(r['angle']) for r in run)
    amin = min(at) if at else float('nan')
    print(f'  {dt:5.1f}s {ang:7.0f}° {v[0]:6.1f} {min(v):6.1f} {v[-1]:6.1f} {amin:10.2f}  {eng:.0%}')
  slow = [min(r['v'] for r in run) for run in runs]
  print(f'  彎中最低車速：中位 {np.median(slow):.1f} km/h，最慢 {min(slow):.1f} km/h')
  stalled = [r for r in runs if min(x['v'] for x in r) < 5]
  print(f'  幾乎停住（<5 km/h）的轉彎：{len(stalled)} / {len(runs)}')


def slowdown_section(rows):
  """Who is asking for the brakes, and what was steering doing at the time.

  The turn table keys off steering angle, which is already too late: by the time the wheel
  is turned the car has finished slowing. This keys off the deceleration itself.
  """
  print('\n=== engaged 時的減速事件（車速掉到 15 km/h 以下）===')
  events, i = [], 0
  while i < len(rows):
    r = rows[i]
    if r['enabled'] and r['v'] < 15 and not r['brake']:
      j = i
      while j > 0 and rows[j]['v'] < 25 and r['t'] - rows[j]['t'] < 12:
        j -= 1
      if rows[j]['v'] - r['v'] > 12:      # a real slowdown, not idling in traffic
        window = rows[j:i + 1]
        events.append(window)
        while i < len(rows) and rows[i]['v'] < 20:
          i += 1
    i += 1
  if not events:
    print('  沒有符合的減速事件')
    return
  print(f'  {len(events)} 次。誰下的指令（longitudinalPlan 的來源）：')
  from collections import Counter
  c = Counter()
  for w in events:
    for r in w:
      if r['reason'] and r['v'] < 30:
        c[r['reason']] += 1
  for k, v in c.most_common(6):
    print(f'    {k:<28} {100 * v / max(sum(c.values()), 1):5.1f}%')
  print(f'\n  {"從":>6} {"到":>6} {"耗時":>5} {"最大減速":>8} {"最大方向盤":>10}')
  for w in events[:12]:
    ats = [r['aTarget'] for r in w if r['aTarget'] is not None]
    amin = min(ats) if ats else float('nan')
    ang = max(abs(r['angle']) for r in w)
    dt = w[-1]['t'] - w[0]['t']
    print(f'  {w[0]["v"]:6.1f} {w[-1]["v"]:6.1f} {dt:5.1f}s {amin:8.2f} {ang:10.0f}°')


def disengage_section(rows):
  """What was happening in the seconds before the driver took over.

  The complaint is that it crawls through junctions, and the takeover is the evidence -
  by the time the wheel is turned openpilot is already off, so the cause has to be looked
  for before the handover, not during the turn.
  """
  from collections import Counter
  print('\n=== 接管前 8 秒發生什麼 ===')
  drops = [i for i in range(1, len(rows)) if rows[i - 1]['enabled'] and not rows[i]['enabled']]
  print(f'  這趟脫離 {len(drops)} 次')
  slow_takeovers = []
  for i in drops:
    j = i
    while j > 0 and rows[i]['t'] - rows[j]['t'] < 8:
      j -= 1
    w = rows[j:i + 1]
    if len(w) < 20:
      continue
    v0, vmin = w[0]['v'], min(r['v'] for r in w)
    if v0 - vmin > 5 and vmin < 25:      # it slowed down noticeably before the takeover
      slow_takeovers.append(w)
  print(f'  其中「脫離前明顯減速」的有 {len(slow_takeovers)} 次\n')
  if not slow_takeovers:
    return
  print(f'  {"進入":>6} {"最低":>6} {"脫離時":>7} {"最大減速":>8} {"方向盤":>7}  減速期間的 reason')
  for w in slow_takeovers[:14]:
    v0, vmin, vend = w[0]['v'], min(r['v'] for r in w), w[-1]['v']
    ats = [r['aTarget'] for r in w if r['aTarget'] is not None]
    amin = min(ats) if ats else float('nan')
    ang = max(abs(r['angle']) for r in w)
    c = Counter(r['sp'] or r['reason'] or '?' for r in w if r['a'] < -0.1)
    why = ' '.join(f'{k}:{100 * v / max(sum(c.values()), 1):.0f}%' for k, v in c.most_common(3))
    print(f'  {v0:6.1f} {vmin:6.1f} {vend:7.1f} {amin:8.2f} {ang:7.0f}°  {why}')

  allc = Counter()
  for w in slow_takeovers:
    for r in w:
      if r['a'] < -0.1:
        allc[r['sp'] or r['reason'] or '?'] += 1
  total = max(sum(allc.values()), 1)
  print('\n  合計：減速中的指令來源')
  for k, v in allc.most_common(6):
    print(f'    {k:<24} {100 * v / total:5.1f}%')


def override_section(rows):
  """Where the driver added throttle while still engaged.

  Pressing the gas does not disengage, so these moments never show up as takeovers - but
  they are the most direct evidence of "it is too slow here", which is exactly the
  complaint about junctions.
  """
  from collections import Counter
  print('\n=== engaged 中駕駛踩油門補油的時刻 ===')
  events, i = [], 0
  while i < len(rows):
    if rows[i]['enabled'] and rows[i]['gas']:
      j = i
      while j < len(rows) and rows[j]['gas']:
        j += 1
      k = i
      while k > 0 and rows[i]['t'] - rows[k]['t'] < 6:
        k -= 1
      if j - i > 5:
        events.append((rows[k:i + 1], rows[i:j]))
      i = j
    i += 1
  print(f'  {len(events)} 次')
  if not events:
    return
  print(f'\n  {"踩下時速":>8} {"前6秒最高":>9} {"方向盤":>7} {"前6秒最大減速":>12}  踩下前的 reason')
  big_angle = 0
  for before, during in events[:16]:
    v_at = during[0]['v']
    v_before = max(r['v'] for r in before) if before else v_at
    ang = max(abs(r['angle']) for r in before + during)
    ats = [r['aTarget'] for r in before if r['aTarget'] is not None]
    amin = min(ats) if ats else float('nan')
    c = Counter(r['sp'] or r['reason'] or '?' for r in before)
    why = ' '.join(f'{k}:{100 * v / max(len(before), 1):.0f}%' for k, v in c.most_common(2))
    if ang > 90:
      big_angle += 1
    print(f'  {v_at:8.1f} {v_before:9.1f} {ang:7.0f}° {amin:12.2f}  {why}')
  print(f'\n  其中方向盤轉超過 90°（路口／大轉彎）的：{big_angle} / {len(events[:16])}')

  allc = Counter()
  for before, _ in events:
    for r in before:
      allc[r['sp'] or r['reason'] or '?'] += 1
  total = max(sum(allc.values()), 1)
  print('  踩油門前 6 秒，速度是誰在決定：')
  for k, v in allc.most_common(6):
    print(f'    {k:<24} {100 * v / total:5.1f}%')
  slow = [d[0]['v'] for _, d in events]
  print(f'  踩下油門當下的車速：中位 {np.median(slow):.1f} km/h，最低 {min(slow):.1f} km/h')


def hunting_section(rows, min_seconds=25):
  """The waveform while holding a steady set speed - overshoot, undershoot, and period.

  The earlier summary counted how often the car was more than 1 km/h over, which is exactly
  the wrong question for a  50 -> 51 -> 48 -> accelerate  cycle: the threshold sits inside
  the swing, so it reads as "hardly ever over" while the car is visibly hunting.
  """
  print('\n=== 定速時的振盪波形 ===')
  runs, cur = [], []
  for r in rows:
    ok = (r['enabled'] and not r['lead'] and not r['gas'] and not r['brake']
          and r['vset'] and r['v'] > 25)
    if ok and (not cur or abs(r['vset'] - cur[0]['vset']) < 0.6):
      cur.append(r)
    else:
      if cur and cur[-1]['t'] - cur[0]['t'] > min_seconds:
        runs.append(cur)
      cur = [r] if ok else []
  if cur and cur[-1]['t'] - cur[0]['t'] > min_seconds:
    runs.append(cur)
  if not runs:
    print(f'  沒有連續 {min_seconds}s 以上、設定速度不變的定速區段')
    return
  print(f'  {len(runs)} 個穩定區段')

  for run in sorted(runs, key=len, reverse=True)[:4]:
    vset = run[0]['vset']
    err = np.array([r['v'] - vset for r in run])
    t = np.array([r['t'] - run[0]['t'] for r in run])
    print(f'\n  --- 設定 {vset:.0f} km/h，持續 {t[-1]:.0f}s ---')
    lo, hi = vset + err.min(), vset + err.max()
    print(f'      實際範圍 {lo:.1f} .. {hi:.1f} km/h   (誤差 {err.min():+.1f} .. {err.max():+.1f})')

    # zero crossings of the speed error give the half-period of the hunt
    sign = np.sign(err)
    cross = np.where(np.diff(sign) != 0)[0]
    if len(cross) > 2:
      periods = 2 * np.diff(t[cross])
      keep = periods[(periods > 2) & (periods < 90)]
      if len(keep):
        print(f'      穿越設定速度 {len(cross)} 次，振盪週期中位 {np.median(keep):.0f}s')

    # one sample a second, so the shape is visible without dumping 100 Hz
    step = max(1, len(run) // 40)
    line = ''.join('+' if e > 0.4 else ('-' if e < -0.4 else '=') for e in err[::step])
    print(f'      波形(每~{t[-1] / max(len(err[::step]), 1):.1f}s一格, +高於 =在上 -低於)：{line[:60]}')
    at = [r['aTarget'] for r in run if r['aTarget'] is not None]
    if at:
      a = np.array(at)
      flips = int((np.diff(np.sign(a)) != 0).sum())
      print(f'      aTarget {a.min():+.2f} .. {a.max():+.2f}，中位 {np.median(a):+.3f}，正負翻轉 {flips} 次')


def setspeed_section(rows):
  """What the set speed itself is doing.

  Every excursion above it in this drive happened while it was moving, which makes "over
  the max" ambiguous: a target that steps down under you looks identical to a controller
  that overshoots. SpeedLimitMode is on, so the assist is a candidate for moving it.
  """
  print('\n=== 設定速度本身穩不穩 ===')
  eng = [r for r in rows if r['enabled'] and r['vset']]
  if len(eng) < 50:
    print('  資料不足')
    return
  vs = np.array([r['vset'] for r in eng])
  span = eng[-1]['t'] - eng[0]['t']
  steps = int((np.abs(np.diff(vs)) > 0.4).sum())
  vals, counts = np.unique(np.round(vs), return_counts=True)
  top = sorted(zip(vals, counts, strict=False), key=lambda kv: -kv[1])[:8]
  print(f'  engaged 期間 {span / 60:.1f} 分鐘，設定速度變動 {steps} 次 ({steps / max(span / 60, 1):.1f} 次/分)')
  print(f'  出現過的值：{", ".join(f"{v:.0f}km/h({100 * c / len(vs):.0f}%)" for v, c in top)}')
  runs, cur = [], 1
  for a, b in zip(vs[:-1], vs[1:], strict=False):
    if abs(a - b) > 0.4:
      runs.append(cur)
      cur = 1
    else:
      cur += 1
  runs.append(cur)
  hold = np.array(runs) / max(len(vs) / span, 1)
  print(f'  同一個設定速度平均維持 {np.mean(hold):.1f}s，中位 {np.median(hold):.1f}s，最長 {hold.max():.0f}s')


def approach_section(rows):
  """Arriving at the set speed: the overshoot and the swing back.

  This is the event to look at, not steady cruising. The complaint is  50 -> 51 -> 48 ->
  accelerate again , which happens while catching the set speed, and a drive that never
  settles for long has plenty of these even though it has almost no steady cruising.
  """
  print('\n=== 追到設定速度時的超調與回擺 ===')
  def usable(r):
    return (r['enabled'] and not r['lead'] and not r['gas'] and not r['brake']
            and r['vset_disp'] and r['vset_disp'] > 25)

  gap = np.median([r['v_disp'] - r['v'] for r in rows if r['v'] > 20])
  print(f'  （儀表速度比實際高 {gap:+.1f} km/h，以下用儀表值，也就是你在畫面上看到的）')

  # every excursion above the set speed is an overshoot; take what follows it too
  events, i = [], 0
  while i < len(rows):
    r = rows[i]
    if usable(r) and r['v_disp'] > r['vset_disp'] + 0.2:
      j = i
      while j < len(rows) and rows[j]['t'] - r['t'] < 12:
        j += 1
      back = i
      while back > 0 and r['t'] - rows[back]['t'] < 3:
        back -= 1
      w = rows[back:j]
      # flag rather than drop: a target that stepped down under us looks the same as an
      # overshoot, and which one this is worth seeing
      steady = all(x['vset_disp'] and abs(x['vset_disp'] - r['vset_disp']) < 1.0 for x in w)
      events.append((r['vset_disp'], w, steady))
      i = j
    else:
      i += 1

  if not events:
    print('  這趟沒有「追上設定速度」的事件')
    return
  steady_n = sum(1 for e in events if e[2])
  print(f'  {len(events)} 次，其中設定速度全程未變的 {steady_n} 次\n')
  print(f'  {"設定":>5} {"最高":>6} {"超調":>6} {"12s內最低":>9} {"擺幅":>6}  穿越  設定穩定')
  overs, unders = [], []
  for vset, w, steady in events[:16]:
    v = np.array([r['v_disp'] for r in w])
    peak = v.max()
    after = v[int(np.argmax(v)):]
    trough = after.min() if len(after) > 1 else peak
    crossings = int((np.diff(np.sign(v - vset)) != 0).sum())
    if steady:
      overs.append(peak - vset)
      unders.append(trough - vset)
    mark = 'v' if steady else '-'
    print(f'  {vset:5.0f} {peak:6.1f} {peak - vset:+6.1f} {trough:9.1f} {peak - trough:6.1f}  {crossings:2d}    {mark}')
  if overs:
    band = np.median(np.array(overs) - np.array(unders))
    print(f'\n  （只算設定穩定的）超調中位 {np.median(overs):+.1f}，回落中位 {np.median(unders):+.1f}，擺幅中位 {band:.1f} km/h')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('route')
  ap.add_argument('--root', default='/data/media/0/realdata')
  args = ap.parse_args()

  segs = sorted(p for p in Path(args.root).iterdir() if p.name.startswith(args.route))
  if not segs:
    sys.exit(f'no segments matching {args.route}')
  print(f'{len(segs)} 段：{segs[0].name} .. {segs[-1].name}')
  rows, geom = collect(segs)
  if not rows:
    sys.exit('no carState in these logs')
  rows.sort(key=lambda r: r['t'])
  span = rows[-1]['t'] - rows[0]['t']
  eng = sum(r['enabled'] for r in rows) / len(rows)
  print(f'{len(rows)} 幀 / {span / 60:.1f} 分鐘，engaged {eng:.0%}，最高 {max(r["v"] for r in rows):.0f} km/h')

  cruise_section(rows)
  hunting_section(rows)
  setspeed_section(rows)
  approach_section(rows)
  accel_section(rows)
  turn_section(rows)
  budget_section(rows, geom)
  slowdown_section(rows)
  disengage_section(rows)
  override_section(rows)


if __name__ == '__main__':
  main()
