#!/usr/bin/env python3
"""Compensation learned from the car's own drives (2026-10-31): what the control layer has to add because the model
only plans the path and does not adapt to this car (the driver, 10-29: "模型只負責規劃路徑 他不參與控制 ... 依照自動
學習 ... 來修正模型的不足"; "像他那樣調整 camera 的偏差"; 10-31: "要會自動調整參數 ... 轉向角度要跟車道偏移(往左偏
0.15 這個做得很好) 因為現在模型沒法自己動 總是看要怎樣可以補償"). No GPS: camera and car signals only.

1 bend offset (lane_offset applies it): in a bend the model cuts to the inside. 10-31's five drives, openpilot steering,
no blinker, both lines seen: left bends sat 0.15 / 0.29 / 0.32 m left of the lane centre at R 286 / 133 / 50 m, right
bends 0.10 / 0.34 / 0.45 m right - the left ones towards the oncoming lane. Per drive, the median offset in each band of
the model's desiredCurvature (+ = right on this car); the table is the sample-weighted mean of the last N_ROUTES drives.
The offset is read in the frame the model sees, which lane_offset shears, so it is the model's own cut whether or not
a compensation was applied that drive - the table does not chase its own correction.

2 curve brake release (curve_speed uses it): when the bend's braking lets go, the cruise law hands the deceleration
back at its own jerk and the brakes take ~0.5 s to come off, so the car goes on slowing. 10-31: 83 curve braking
stretches, a further 5.0 km/h (median) lost in the 1.5 s after - the bend taken ~5 km/h under the formula's speed.
Per drive, that further loss per m/s^2 of deceleration at the hand-back; the parameter is the median over N_ROUTES.

road_memd calls learn() after each scan; OUT holds the current parameters for the two consumers.
"""
import glob
import json
import os
import sys

import numpy as np

ROOT = '/data/media/0/osm/road_memory'
COMP_DIR = os.path.join(ROOT, 'comp')
DONE_FILE = os.path.join(ROOT, 'comp_done.json')
OUT = os.path.join(ROOT, 'compensation.json')
LOG_ROOT = '/data/media/0/realdata'

N_ROUTES = 10
# desiredCurvature band edges (1/m, + = right): R 500 / 250 / 125 / 70 / 40 m
BEND_EDGES = [-0.025, -0.014, -0.008, -0.004, -0.002, 0.002, 0.004, 0.008, 0.014, 0.025]
BEND_MIN_S = 3.0          # s of samples in a band, one drive, before it counts
BEND_MIN_TOTAL_S = 15.0   # s over the drives combined, before a band is used
SPEED_MIN = 15 / 3.6
TQ_MAX = 60               # driver torque: hands off
LINE_P = 0.5
DT = 0.1                  # s, the extraction rate

BRAKE_CMD = -0.3          # m/s^2: a curve braking stretch
BRAKE_MIN_S = 0.5
AFTER_S = 1.5
RELEASE_MIN_EVENTS = 3


def extract_route(route, log_root=LOG_ROOT):
  """-> per-drive statistics for both compensations, or None (nothing usable)."""
  from openpilot.tools.lib.logreader import LogReader
  segs = sorted((int(d.rsplit('--', 1)[1]), d) for d in os.listdir(log_root) if d.startswith(route + '--'))
  st = {'v': 0., 'en': False, 'lat': False, 'tq': 0., 'bl': False, 'lcs': 0, 'l1': 0., 'l2': 0., 'q1': 0., 'q2': 0., 'k': 0.,
        'reason': '', 'acc': 0.}
  rows = []
  last = -1.
  for _, d in segs:
    p = os.path.join(log_root, d, 'rlog.zst')
    if not os.path.exists(p):
      continue
    try:
      for m in LogReader(p):
        w = m.which()
        if w == 'carState':
          c = m.carState
          st.update(v=c.vEgo, tq=c.steeringTorque, bl=c.leftBlinker or c.rightBlinker)
          t = m.logMonoTime / 1e9
          if t - last >= DT:
            last = t
            rows.append((t, st['v'], st['en'], st['lat'], st['tq'], st['bl'], st['lcs'], st['l1'], st['l2'], st['q1'], st['q2'],
                         st['k'], st['reason'] == 'curve', st['acc'], c.gasPressed or c.brakePressed))
        elif w == 'selfdriveState':
          st['en'] = m.selfdriveState.enabled
        elif w == 'carControl':
          st['lat'] = m.carControl.latActive
          st['acc'] = m.carControl.actuators.accel
        elif w == 'longitudinalPlanSP':
          st['reason'] = str(m.longitudinalPlanSP.reason)
        elif w == 'modelV2':
          md = m.modelV2
          st['lcs'] = int(md.meta.laneChangeState.raw)
          st['k'] = float(md.action.desiredCurvature)
          if len(md.laneLines) >= 4 and len(md.laneLineProbs) >= 4:
            st.update(l1=md.laneLines[1].y[0], l2=md.laneLines[2].y[0], q1=md.laneLineProbs[1], q2=md.laneLineProbs[2])
    except Exception as e:  # a corrupt segment loses that segment, not the drive
      print(f'compensation: {d}: {e}', file=sys.stderr)
  if len(rows) < 100:
    return None
  a = np.array(rows, dtype=float)
  t, v, en, lat, tq, bl, lcs, l1, l2, q1, q2, k, curve, acc, ped = a.T
  near_bl = np.convolve(bl, np.ones(int(20 / DT) + 1), 'same') > 0       # +-10 s
  ok = (en > .5) & (lat > .5) & ~near_bl & (lcs == 0) & (v > SPEED_MIN) & (np.abs(tq) < TQ_MAX) & (q1 > LINE_P) & \
       (q2 > LINE_P) & (ped < .5)
  centre = (l1 + l2) / 2                     # > 0: the car left of the lane centre (model y + = right)
  bands = []
  for lo, hi in zip(BEND_EDGES[:-1], BEND_EDGES[1:], strict=True):
    m = ok & (k >= lo) & (k < hi)
    n = int(m.sum())
    bands.append([float(np.median(centre[m])) if n * DT >= BEND_MIN_S else None, round(n * DT, 1)])
  # curve brake release: stretches of curve braking, openpilot in control, no pedal
  rel = []
  br = (en > .5) & (ped < .5) & (curve > .5) & (acc < BRAKE_CMD)
  d = np.diff(np.concatenate([[0], br.astype(int), [0]]))
  n_after = int(AFTER_S / DT)
  for s0, e0 in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1), strict=True):
    if (e0 - s0) * DT < BRAKE_MIN_S or e0 + n_after >= len(v) or not ((en[e0:e0 + n_after] > .5).all() and (ped[e0:e0 + n_after] < .5).all()):
      continue
    a_end = float(-acc[e0 - 1])
    extra = float(v[e0] - v[e0:e0 + n_after].min())
    rel.append([round(a_end, 3), round(extra, 3)])
  return {'route': route, 'bands': bands, 'release': rel}


def combine(stats):
  """Per-drive statistics (newest last) -> the parameters."""
  stats = stats[-N_ROUTES:]
  table = []
  for i in range(len(BEND_EDGES) - 1):
    vals = [(s['bands'][i][0], s['bands'][i][1]) for s in stats if s['bands'][i][0] is not None]
    tot = sum(w for _, w in vals)
    table.append(round(sum(o * w for o, w in vals) / tot, 3) if tot >= BEND_MIN_TOTAL_S else None)
  ev = [r for s in stats for r in s['release'] if r[0] > 0.3]
  loss = float(np.median([e / a for a, e in ev])) if len(ev) >= RELEASE_MIN_EVENTS else None
  centres = [(lo + hi) / 2 for lo, hi in zip(BEND_EDGES[:-1], BEND_EDGES[1:], strict=True)]
  return {'bend_k': [round(c, 4) for c in centres], 'bend_offset': table, 'release_loss': None if loss is None else round(loss, 3),
          'release_events': len(ev), 'routes': [s['route'] for s in stats]}


def learn(log_root=LOG_ROOT, comp_dir=COMP_DIR, done_file=DONE_FILE, out=OUT):
  """Every finished drive not yet seen -> comp_dir/<route>.json, then OUT from the newest N_ROUTES. Returns how many."""
  os.makedirs(comp_dir, exist_ok=True)
  try:
    with open(done_file) as f:
      done = set(json.load(f))
  except (OSError, ValueError):
    done = set()
  routes = sorted({d.rsplit('--', 1)[0] for d in os.listdir(log_root) if d.count('--') == 2})
  added = 0
  for r in routes:
    if r in done:
      continue
    try:
      res = extract_route(r, log_root)
    except Exception as e:
      print(f'compensation: {r} failed: {e}', file=sys.stderr)
      res = None
    if res is not None:
      _write(os.path.join(comp_dir, r + '.json'), res)
      added += 1
    done.add(r)
    _write(done_file, sorted(done))
  stats = []
  for f in sorted(glob.glob(os.path.join(comp_dir, '*.json'))):     # route names sort by time
    try:
      with open(f) as fh:
        stats.append(json.load(fh))
    except (OSError, ValueError):
      continue
  if stats and (added or not os.path.exists(out)):
    _write(out, combine(stats))
  return added


def load(path=None):
  try:
    with open(path or OUT) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def _write(path, obj):
  tmp = path + '.tmp'
  with open(tmp, 'w') as f:
    json.dump(obj, f)
  os.replace(tmp, path)


if __name__ == '__main__':
  print('learned', learn())
  print(json.dumps(load(), indent=1))
