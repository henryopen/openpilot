#!/usr/bin/env python3
"""Check the dash/true speed conversion against a recorded route: residual of the current
constants, and what each set speed lands on.

Run on the device:

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_dash_speed.py 00000026'
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, '/data/openpilot')

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.longitudinal_planner import (DASH_GAIN, DASH_OFFSET_KPH,
                                                                   dash_to_true, true_to_dash)
from openpilot.tools.lib.logreader import LogReader

KPH = CV.MS_TO_KPH


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('route')
  ap.add_argument('--segs', default='')
  ap.add_argument('--realdata', default='/data/media/0/realdata')
  args = ap.parse_args()

  paths = sorted(glob.glob(os.path.join(args.realdata, f'{args.route}--*')))
  if args.segs:
    want = {int(s) for s in args.segs.split(',')}
    paths = [p for p in paths if int(p.rsplit('--', 1)[-1]) in want]
  if not paths:
    sys.exit(f'no segments matched {args.route}')

  print(f'DASH_GAIN = {DASH_GAIN}, DASH_OFFSET_KPH = {DASH_OFFSET_KPH}')

  rows = []
  for p in paths:
    rlog = os.path.join(p, 'rlog.zst')
    if not os.path.exists(rlog):
      rlog = os.path.join(p, 'rlog')
    if not os.path.exists(rlog):
      continue
    for m in LogReader(rlog):
      if m.which() != 'carState':
        continue
      cs = m.carState
      if cs.standstill or cs.vEgo < 0.5 or cs.vEgoCluster <= 0:
        continue
      rows.append((cs.vEgo * KPH, cs.vEgoCluster * KPH))

  a = np.array(rows)
  true, dash = a.T
  print(f'moving samples: {len(a)}')

  pred = DASH_GAIN * true + DASH_OFFSET_KPH
  err = dash - pred
  p95 = np.percentile(np.abs(err), 95)
  print(f'\nresidual (dash - model) whole route: p50 {np.median(err):+.2f}  |p95| {p95:.2f} km/h')
  hi = true >= 40
  ehi = err[hi]
  p95hi = np.percentile(np.abs(ehi), 95)
  print(f'above 40 km/h ({int(hi.sum())} samples): p50 {np.median(ehi):+.2f}  |p95| {p95hi:.2f} km/h')

  print(f"\n{'band(true)':<12} {'dash_p50':>10} {'model':>10} {'error':>10}")
  for lo, hi_ in [(20, 40), (40, 60), (60, 80), (80, 90), (90, 100), (100, 115)]:
    b = (true >= lo) & (true < hi_)
    if b.sum() < 100:
      continue
    t50, d50 = np.median(true[b]), np.median(dash[b])
    mdl = DASH_GAIN * t50 + DASH_OFFSET_KPH
    band = f'{lo}-{hi_}'
    print(f'{band:<12} {d50:>10.1f} {mdl:>10.1f} {d50 - mdl:>+10.2f}')

  print(f"\n{'set (dash)':<12} {'true target':>14} {'dash it should hit':>20}")
  for ds in [40., 50., 60., 70., 80., 90., 100., 110., 120.]:
    t = dash_to_true(ds * CV.KPH_TO_MS) * KPH
    back = true_to_dash(t * CV.KPH_TO_MS) * KPH
    print(f'{ds:<12.0f} {t:>14.1f} {back:>20.1f}')

  worst = max(abs(true_to_dash(dash_to_true(d * CV.KPH_TO_MS)) * KPH - d) for d in range(20, 130))
  print(f'\nround trip must be exact: max error {worst:.4f} km/h')


if __name__ == '__main__':
  main()
