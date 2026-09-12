#!/usr/bin/env python3
"""Should curve_speed.py adopt StarPilot's rewritten curve speed controller?

StarPilot rewrote CSC on 2026-09-07 (2ab6195d). The three parts worth testing here,
against our own logs rather than their claims:

  lookahead   every point of the model path gets an allowed speed, and the speed we
              may do now is sqrt(v_point^2 + 2*a*d) - so distance, which ours has no
              notion of, decides when to ease off.
  farfield    they multiply curvature by 1.23 beyond 30 m, on the grounds that the
              model under-reads distant bends.
  blinker     falls out of the first test: a blinker makes the model draw a lane
              change as a bend, and the two formulations disagree about it.
              Ours reads rate * velocity (lateral accel, small when the model also
              predicts slowing down), theirs reads rate / velocity (curvature, which
              blows up for the same reason).

Run:
  verify_csc_lookahead.py lookahead <rlog.zst> [...]
  verify_csc_lookahead.py farfield  <rlog.zst> [...]
  verify_csc_lookahead.py blinker   <rlog.zst> [...]

Measured 2026-09-13 on the sixteen 9/9 rlogs (19200 model frames, 8 corners):
  lookahead fired twice in 9803 engaged frames over 20 kph. Once for a real corner
  (one frame, 2.7 kph), once for 4.9 s demanding 40 -> 22 kph at a lane change that
  never bent (peak lateral accel 0.69). Our corners peak at 1.5-2.2 and the comfort
  ceiling is 2.0 * tol, so there is nothing for a lookahead to find.
  farfield  ratio of actual to predicted curvature: 0.97 at 30 m, 1.05 at 40 m,
  1.22 at 60 m. The gain is real but starts twice as far out as their threshold.
  blinker   the current limiter fires on a blinker 24% of the time against 8.8%
  exposure, but those are real junction turns (lateral accel p50 1.22, median
  episode 1.0 s, 0.0 kph lost). It is not fooled; the lookahead form is.

Conclusion: do not port it. Kept so the next person does not measure it again.
"""
import glob
import os
import re
import sys

import numpy as np

try:
  import capnp
  import zstandard
except ImportError:
  raise SystemExit("needs pycapnp and zstandard")

SCHEMA = os.environ.get("OP_SCHEMA", r"F:/c4sunny/schema_hcop")
KPH = 3.6
MIN_V = 20 / KPH
V_FLOOR = 15 / KPH
A_LAT_REG_MAX = 2.0          # curve_speed.py
ENTER_TH = 1.3
LAT_TOL_BP = [0., 10., 20.]
LAT_TOL_V = [1.5, 1.25, 1.0]
APPROACH_DECEL = 0.3         # StarPilot CSC_APPROACH_DECEL
MAX_LAT_HARD = 4.0           # CSC_MAX_LATERAL_ACCEL
PROFILE_MIN_SPEED = 3.0
PROFILE_MAX_CURVATURE = 0.1
FARFIELD_TARGETS = [10., 20., 30., 40., 60.]
FARFIELD_MIN_K = 0.004


def load_schema():
  capnp.remove_import_hook()
  os.chdir(SCHEMA)
  return capnp.load("log.capnp", imports=[SCHEMA])


def events(log, path):
  data = zstandard.ZstdDecompressor().stream_reader(open(path, "rb")).read()
  pump = iter(log.Event.read_multiple_bytes(data))
  t0 = None
  while True:
    try:
      e = next(pump)
    except StopIteration:
      return
    except Exception:
      return                 # the last segment of a route is always truncated
    try:
      w = e.which()
    except Exception:
      continue               # message types this schema does not know
    if t0 is None:
      t0 = e.logMonoTime
    yield w, e, (e.logMonoTime - t0) / 1e9


def tol_of(v):
  return float(np.interp(v, LAT_TOL_BP, LAT_TOL_V))


def lookahead_target(rate, vel, dist, ceiling):
  """StarPilot's: an allowed speed per point, the tightest one wins."""
  k = rate / np.clip(vel, PROFILE_MIN_SPEED, None)
  k = np.where(vel < PROFILE_MIN_SPEED, 0.0, np.minimum(k, PROFILE_MAX_CURVATURE))
  kf = np.maximum(k, 1e-4)
  pt = np.sqrt(ceiling / kf)
  pt = np.minimum(np.maximum(pt, V_FLOOR), np.sqrt(MAX_LAT_HARD / kf))
  allowed = np.sqrt(pt ** 2 + 2.0 * APPROACH_DECEL * np.maximum(dist, 0.0))
  i = int(np.argmin(allowed))
  return float(allowed[i]), float(dist[i])


def scan(log, paths):
  """One pass, everything the three reports need."""
  out = []
  for path in paths:
    seg = int(re.search(r"(\d+)", os.path.basename(path)).group(1))
    v = a = k = 0.0
    ena = bl = False
    blink_since = -99.0
    ts, vs, ks = [], [], []
    for w, e, t in events(log, path):
      if w == "carState":
        v, a = e.carState.vEgo, e.carState.aEgo
        bl = bool(e.carState.leftBlinker or e.carState.rightBlinker)
        if bl:
          blink_since = t
      elif w == "controlsState":
        k = abs(e.controlsState.curvature)
        ts.append(t); vs.append(v); ks.append(k)
      elif w == "selfdriveState":
        ena = e.selfdriveState.enabled
      elif w == "modelV2":
        m = e.modelV2
        try:
          rate = np.abs(np.array(m.orientationRate.z))
          vel = np.array(m.velocity.x)
          dist = np.array(m.position.x)
        except Exception:
          continue
        if not (len(rate) == len(vel) == len(dist) == 33):
          continue
        tol = tol_of(v)
        ceiling = A_LAT_REG_MAX * tol
        prod = rate * vel
        max_pred = float(np.percentile(prod, 97))   # note: on 33 points this is ~the max
        v_old = float(np.sqrt(ceiling / (max_pred / max(v, 0.1) ** 2))) if max_pred > 0 else 999.
        v_new, bind_d = lookahead_target(rate, vel, dist, ceiling)
        out.append(dict(seg=seg, t=t, v=v, a=a, k=k, ena=float(ena), lat=v * v * k,
                        bl=float(bl), since=t - blink_since, max_pred=max_pred,
                        v_old=v_old, armed_old=float(v > MIN_V and max_pred >= ENTER_TH * tol),
                        v_new=v_new, bind=bind_d,
                        rate=rate, vel=vel, dist=dist,
                        ts=ts, vs=vs, ks=ks))
  return out


def col(rows, name):
  return np.array([r[name] for r in rows], dtype=float)


def report_lookahead(rows):
  sel = (col(rows, "ena") > 0.5) & (col(rows, "v") > MIN_V)
  v, v_old, v_new = col(rows, "v")[sel], col(rows, "v_old")[sel], col(rows, "v_new")[sel]
  armed_old, lat, bind = col(rows, "armed_old")[sel], col(rows, "lat")[sel], col(rows, "bind")[sel]
  cur = (armed_old > 0.5) & (v_old < v - 0.5)
  new = v_new < v - 0.5
  print("engaged frames over 20 kph: %d" % sel.sum())
  print("  current   asks to slow: %5d (%.2f%%)" % (cur.sum(), 100 * cur.mean()))
  print("  lookahead asks to slow: %5d (%.2f%%)" % (new.sum(), 100 * new.mean()))
  print("  overlap %d, current only %d, lookahead only %d" %
        ((cur & new).sum(), (cur & ~new).sum(), (new & ~cur).sum()))
  for name, m, tgt in (("current", cur, v_old), ("lookahead", new, v_new)):
    if m.sum():
      print("  %-9s v_ego p50 %5.1f, target p50 %5.1f, gives up %4.1f kph, "
            "actual lat accel p50 %.2f" %
            (name, np.median(v[m]) * KPH, np.median(tgt[m]) * KPH,
             np.median(v[m] - tgt[m]) * KPH, np.median(lat[m])))
  if new.sum():
    print("  lookahead binding distance p50 %.0f m p90 %.0f m" %
          (np.median(bind[new]), np.percentile(bind[new], 90)))


def report_farfield(rows):
  pairs = {d: [] for d in FARFIELD_TARGETS}
  by_seg = {}
  for r in rows:
    by_seg.setdefault(r["seg"], []).append(r)
  for seg, rs in by_seg.items():
    ts = np.array(rs[-1]["ts"]); vs = np.array(rs[-1]["vs"]); ks = np.array(rs[-1]["ks"])
    if len(ts) < 50:
      continue
    cum = np.cumsum(vs * np.diff(ts, prepend=ts[0]))
    for r in rs:
      if r["ena"] < 0.5 or r["v"] < MIN_V or r["bl"] > 0.5:
        continue
      kpath = r["rate"] / np.clip(r["vel"], PROFILE_MIN_SPEED, None)
      i0 = int(np.searchsorted(ts, r["t"]))
      if i0 >= len(ts) - 5:
        continue
      for tgt in FARFIELD_TARGETS:
        if r["dist"][-1] < tgt:
          continue
        kp = float(np.interp(tgt, r["dist"], kpath))
        j = int(np.searchsorted(cum, cum[i0] + tgt))
        if j < len(ts):
          pairs[tgt].append((kp, float(ks[j])))
  print("%-5s %-7s %-9s %-9s %-9s" % ("D m", "n", "k_pred", "k_actual", "ratio p50"))
  for tgt in FARFIELD_TARGETS:
    p = np.array(pairs[tgt])
    if len(p) < 20:
      continue
    q = p[(p[:, 0] >= FARFIELD_MIN_K) | (p[:, 1] >= FARFIELD_MIN_K)]
    if len(q) < 20:
      continue
    print("%-5.0f %-7d %-9.4f %-9.4f %-9.2f" %
          (tgt, len(q), np.median(q[:, 0]), np.median(q[:, 1]),
           np.median(q[:, 1] / np.maximum(q[:, 0], 1e-5))))


def report_blinker(rows):
  sel = (col(rows, "ena") > 0.5) & (col(rows, "v") > MIN_V)
  since, lat, v = col(rows, "since")[sel], col(rows, "lat")[sel], col(rows, "v")[sel]
  v_new, v_ego = col(rows, "v_new")[sel], col(rows, "v")[sel]
  print("baseline: blinker on or within 6 s = %.1f%% of engaged frames over 20 kph"
        % (100 * (since < 6.0).mean()))
  fired = v_new < v_ego - 0.5
  if fired.sum():
    print("lookahead fires: %d frames, %.1f%% of them on a blinker, "
          "actual lat accel there p50 %.2f" %
          (fired.sum(), 100 * (since[fired] < 6.0).mean(), np.median(lat[fired])))
    worst = int(np.argmax((v_ego - v_new)[fired]))
    idx = np.where(fired)[0][worst]
    print("  worst demand: %.0f -> %.0f kph while pulling %.2f lateral" %
          (v[idx] * KPH, v_new[idx] * KPH, lat[idx]))


if __name__ == "__main__":
  if len(sys.argv) < 3:
    raise SystemExit(__doc__)
  mode, args = sys.argv[1], sys.argv[2:]
  files = []
  for a in args:
    files += sorted(glob.glob(a)) if any(c in a for c in "*?") else [a]
  log = load_schema()
  data = scan(log, files)
  print("%d model frames from %d files\n" % (len(data), len(files)))
  {"lookahead": report_lookahead,
   "farfield": report_farfield,
   "blinker": report_blinker}[mode](data)
