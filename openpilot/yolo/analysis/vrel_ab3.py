#!/usr/bin/env python3
"""A/B the Custin radar's relative speed on the one target that reaches control.

Truth is a centred +-1.5 s least squares slope of the range: non-causal, so the car can
never have it, but it is the quietest account of what the range was really doing. Only
slot 0x238 is measured because CUSTIN_PRIMARY_ONLY means that is the only one radard is
handed, and nothing is filtered out for being noisy - noisy is where the harm happens.

The failure that matters is calling a car 'opening' while it is not, so the tails are
reported by sign rather than rolled into one average.

  ./vrel_ab3.py <route>:<seg> [<route>:<seg> ...]
"""
import statistics
import sys

from opendbc.can import CANParser
from opendbc.car.can_definitions import CanData
from openpilot.tools.lib.logreader import LogReader

ADDRS = tuple(range(0x238, 0x238 + 30, 3))
PRIMARY = 0x238
MIN_S, MAX_GAP, MAX_JUMP, MIN_SCORE, MIN_RANGE = 6, 0.2, 3.0, 30, 2.0
TRUTH_HALF = 1.5


def slope(ts, ds):
  n = len(ts)
  mt = sum(ts) / n
  md = sum(ds) / n
  vt = sum((t - mt) ** 2 for t in ts)
  return None if vt <= 0 else sum((t - mt) * (d - md) for t, d in zip(ts, ds, strict=True)) / vt


res = []
for arg in sys.argv[1:]:
  route, seg = arg.split(':')
  cp = CANParser("custin_radar", [(f"RADAR_TRACK_{a:x}", 33) for a in ADDRS], 1)
  sp = CANParser("hyundai_can_generated", [("WHL_SPD11", 50)], 0)
  run = []
  rows = []
  for m in LogReader(f"/data/media/0/realdata/{route}--{seg}/rlog.zst"):
    if m.which() != 'can':
      continue
    pkt = [(m.logMonoTime, [CanData(c.address, c.dat, c.src) for c in m.can])]
    upd = cp.update(pkt)
    sp.update(pkt)
    if PRIMARY not in upd:
      continue
    whl = sp.vl["WHL_SPD11"]
    v_ego = (whl["WHL_SPD_FL"] + whl["WHL_SPD_FR"] + whl["WHL_SPD_RL"] + whl["WHL_SPD_RR"]) / 4. / 3.6
    t = m.logMonoTime / 1e9
    v = cp.vl[f"RADAR_TRACK_{PRIMARY:x}"]
    d, sc, va = v['LONG_DIST'], v['SCORE'], v['V_ABS']
    if d <= MIN_RANGE or sc < MIN_SCORE:
      run = []
      continue
    if run and (t - run[-1][0] > MAX_GAP or abs(d - run[-1][1]) > MAX_JUMP):
      run = []
    run.append((t, d, va, v_ego))
    if len(run) < MIN_S:
      continue
    a_old = slope([p[0] for p in run[-12:]], [p[1] for p in run[-12:]])          # today: 0.36 s
    h30 = run[-30:]
    a_long = slope([p[0] for p in h30], [p[1] for p in h30]) if len(h30) >= 20 else None   # 0.9 s
    a_new = statistics.median([p[2] for p in run[-12:]]) - v_ego
    rows.append((t, d, v_ego, a_old, a_long, a_new, run))
  for t, d, v_ego, a_old, a_long, a_new, run in rows:
    win = [p for p in run if abs(p[0] - t) <= TRUTH_HALF]
    if len(win) < 60:
      continue
    if not (win[0][0] < t - TRUTH_HALF * 0.8 and win[-1][0] > t + TRUTH_HALF * 0.8):
      continue                             # the run starts or ends here, so it is not centred
    tr = slope([p[0] for p in win], [p[1] for p in win])
    if tr is None or a_old is None:
      continue
    res.append((tr, a_old, a_long, a_new, v_ego, d))


def rep(name, errs):
  e = [x for x in errs if x is not None]
  n = len(e)
  if not n:
    print(f"  {name}: n=0")
    return
  ab = sorted(abs(x) for x in e)
  opening = sum(1 for x in e if x > 2) / n * 100
  closing = sum(1 for x in e if x < -2) / n * 100
  head = f"  {name:22s} n={n:5d} |err| median={statistics.median(ab):5.2f} p90={ab[int(.9 * n)]:5.2f}"
  print(f"{head} | falsely opening >2: {opening:5.1f}%  falsely closing <-2: {closing:5.1f}%")


print(f"0x238 samples {len(res)} (nothing filtered out for noise)")
print("\nall")
rep("range 0.36 s", [o - t for t, o, _l, _n, _v, _d in res])
rep("range 0.9 s", [None if l is None else l - t for t, _o, l, _n, _v, _d in res])
rep("V_ABS", [n - t for t, _o, _l, n, _v, _d in res])
slow = [r for r in res if r[0] + r[4] < 2.0 and r[4] > 1.5]
print(f"\nslow or stopped car ahead (target under 2 m/s, ego over 5.4 km/h) n={len(slow)}")
rep("range 0.36 s", [o - t for t, o, _l, _n, _v, _d in slow])
rep("range 0.9 s", [None if l is None else l - t for t, _o, l, _n, _v, _d in slow])
rep("V_ABS", [n - t for t, _o, _l, n, _v, _d in slow])
