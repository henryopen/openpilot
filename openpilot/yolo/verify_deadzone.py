#!/usr/bin/env python3
"""verify_deadzone: does the cruise deadzone actually stop the hunting, on real data.

Replays the recorded (v_cruise, v_ego) traces through both the old and the new control law
and compares them. This is not a simulation of the car - the speeds are the ones that
really happened - so it cannot prove the resulting speed is smoother. What it can show is
whether the command still flips sign continuously while holding a set speed, which is the
mechanism being blamed, and that nothing changes when there is a real gap to close.

  ./verify_deadzone.py <route-prefix> [--deadzone 0.25]
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.append('/data/openpilot')
from openpilot.tools.lib.logreader import LogReader

CV = 3.6
J_CRUISE_COMFORT = 0.16
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.2, 1.1, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
DT = 0.05


def law(v_cruise, v_ego, prev, deadzone):
  """The cruise candidate, with an optional deadzone. Mirrors get_cruise_accel."""
  err = v_cruise - v_ego
  if deadzone:
    if abs(err) <= deadzone:
      err = 0.0
    else:
      err -= math.copysign(deadzone, err)
  comfort = min(abs(err), math.sqrt(2. * J_CRUISE_COMFORT * abs(err)))
  max_accel = float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))
  target = float(np.clip(math.copysign(comfort, err), A_CRUISE_MIN, max_accel))
  j = float(np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS))
  return float(np.clip(target, prev - j * DT, prev + j * DT))


def run(trace, deadzone):
  out, prev = [], 0.0
  for v_cruise, v_ego in trace:
    prev = law(v_cruise, v_ego, prev, deadzone)
    out.append(prev)
  return np.array(out)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('route')
  ap.add_argument('--deadzone', type=float, default=0.25)
  ap.add_argument('--root', default='/data/media/0/realdata')
  args = ap.parse_args()

  segs = sorted(p for p in Path(args.root).iterdir() if p.name.startswith(args.route))
  rows = []
  enabled = False
  lead = False
  for seg in segs:
    f = seg / 'rlog.zst'
    if not f.exists():
      continue
    try:
      for msg in LogReader(str(f)):
        w = msg.which()
        if w == 'selfdriveState':
          enabled = bool(msg.selfdriveState.enabled)
        elif w == 'radarState':
          lead = bool(msg.radarState.leadOne.present)
        elif w == 'carState' and enabled and not lead:
          cs = msg.carState
          vc = float(getattr(cs, 'vCruise', 0.0))
          if 0 < vc < 200 and not cs.gasPressed and not cs.brakePressed:
            rows.append((vc / CV, float(cs.vEgo)))
    except Exception as e:
      print(f'  {seg.name}: {e}', file=sys.stderr)
  if len(rows) < 100:
    sys.exit('not enough engaged, lead-free data')

  old = run(rows, 0.0)
  new = run(rows, args.deadzone)
  err = np.array([vc - ve for vc, ve in rows])

  def flips(a):
    nz = a[np.abs(a) > 1e-6]
    return int((np.diff(np.sign(nz)) != 0).sum()) if len(nz) > 1 else 0

  dz_kph = args.deadzone * CV
  print(f'{len(rows)} 幀（engaged、無前車、無踏板），deadzone {args.deadzone} m/s = {dz_kph:.2f} km/h\n')
  print(f'{"":22} {"原本":>10} {"加 deadzone":>12}')
  print(f'{"指令正負翻轉次數":22} {flips(old):10d} {flips(new):12d}')
  print(f'{"指令為零的比例":22} {100 * (np.abs(old) < 1e-6).mean():9.1f}% {100 * (np.abs(new) < 1e-6).mean():11.1f}%')
  print(f'{"指令中位絕對值":22} {np.median(np.abs(old)):10.3f} {np.median(np.abs(new)):12.3f}')

  # the band that matters: already at the set speed, where the hunting happens
  near = np.abs(err) < 1.0
  if near.sum() > 20:
    print(f'\n速度差在 1 m/s (3.6 km/h) 以內的 {near.sum()} 幀 —— 也就是「已經追到了」的時候：')
    print(f'{"  指令正負翻轉":22} {flips(old[near]):10d} {flips(new[near]):12d}')
    print(f'{"  指令中位絕對值":22} {np.median(np.abs(old[near])):10.3f} {np.median(np.abs(new[near])):12.3f}')

  far = np.abs(err) > 2.0
  if far.sum() > 20:
    d = np.abs(old[far] - new[far])
    print(f'\n速度差大於 2 m/s 的 {far.sum()} 幀 —— 也就是「還在追」的時候，應該幾乎不變：')
    print(f'  指令差異 中位 {np.median(d):.3f}，最大 {d.max():.3f} m/s²')

  def steady(err, deadzone):
    """The law with the jerk limiter left out, so the shape itself is visible."""
    if deadzone:
      if abs(err) <= deadzone:
        err = 0.0
      else:
        err -= math.copysign(deadzone, err)
    return math.copysign(min(abs(err), math.sqrt(2. * J_CRUISE_COMFORT * abs(err))), err)

  print('\n控制律本身（速度差 -> 指令，不含 jerk 限制）：')
  print(f'  {"速度差":>10} {"原本":>9} {"加deadzone":>12}')
  for kph in (0.2, 0.5, 0.9, 1.2, 1.5, 3.0, 6.0, 12.0):
    e = kph / CV
    print(f'  {kph:6.1f}km/h {steady(e, 0.0):9.3f} {steady(e, args.deadzone):12.3f}')


if __name__ == '__main__':
  main()
