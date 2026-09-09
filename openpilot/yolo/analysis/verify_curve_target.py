#!/usr/bin/env python3
"""Replay the curve speed limiter over a recorded route and show what the v_target cutoff changes.

Run on the device (it needs the real cereal schema and the real Params):

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_curve_target.py 00000026 --segs 23,41,57,77'

It reports, per frame where the limiter was in the entering state:
  - how often v_ego was already at or under v_target (the new branch fires)
  - what deceleration the old code would still have been asking for there
  - the speed that would have been kept
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, '/data/openpilot')

from openpilot.selfdrive.controls.lib.curve_speed import CurveSpeedControl, CurveState, _LEAVING_ACC
from openpilot.selfdrive.controls.lib import curve_speed as cs_mod
from openpilot.tools.lib.logreader import LogReader

KPH = 3.6


class Frame(dict):
  """Minimal stand-in for SubMaster: the limiter only indexes modelV2 and controlsState."""


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('route', help='route prefix, e.g. 00000026')
  ap.add_argument('--segs', default='', help='comma separated segment numbers; default all')
  ap.add_argument('--realdata', default='/data/media/0/realdata')
  args = ap.parse_args()

  paths = sorted(glob.glob(os.path.join(args.realdata, f'{args.route}--*')))
  if args.segs:
    want = {int(s) for s in args.segs.split(',')}
    paths = [p for p in paths if int(p.rsplit('--', 1)[-1]) in want]
  if not paths:
    sys.exit(f'no segments matched {args.route}')

  ctl = CurveSpeedControl()
  print(f'SmartCruiseControlVision = {ctl.enabled}')
  print(f'_A_LAT_REG_MAX {cs_mod._A_LAT_REG_MAX}, enter {cs_mod._ENTERING_PRED_LAT_ACC_TH}, abort {cs_mod._ABORT_ENTERING_PRED_LAT_ACC_TH}')
  if not ctl.enabled:
    print('WARNING: the toggle is off on this device; the replay still exercises the code path')
    ctl.enabled = True

  rows = []
  exceptions = 0
  for p in paths:
    rlog = os.path.join(p, 'rlog.zst')
    if not os.path.exists(rlog):
      rlog = os.path.join(p, 'rlog')
    if not os.path.exists(rlog):
      continue
    model = None
    controls = None
    v_ego = 0.0
    a_ego = 0.0
    long_en = False
    override = False
    for m in LogReader(rlog):
      w = m.which()
      if w == 'modelV2':
        model = m.modelV2
      elif w == 'controlsState':
        controls = m.controlsState
      elif w == 'carState':
        v_ego = m.carState.vEgo
        a_ego = m.carState.aEgo
        override = m.carState.gasPressed
      elif w == 'carControl':
        long_en = m.carControl.enabled
      elif w == 'longitudinalPlan':
        if model is None or controls is None:
          continue
        sm = Frame(modelV2=model, controlsState=controls)
        try:
          ctl.update(sm, long_en, override, v_ego, a_ego)
        except Exception as e:
          exceptions += 1
          if exceptions < 4:
            print(f'EXCEPTION {type(e).__name__}: {e}')
          continue
        if ctl.state == CurveState.entering:
          old = float(np.interp(ctl.max_pred_lat_acc / ctl._lat_tol(),
                                cs_mod._ENTERING_SMOOTH_DECEL_BP, cs_mod._ENTERING_SMOOTH_DECEL_V))
          rows.append((v_ego, ctl.v_target, ctl.a_target, old, ctl.max_pred_lat_acc,
                       cs_mod._A_LAT_REG_MAX * ctl._lat_tol()))

  print(f'\ncapnp exceptions: {exceptions}   (must be 0)')
  if not rows:
    print('no entering-state frames in this selection')
    return

  a = np.array(rows)
  v, vt, new, old, pred, cap = a.T
  fires = new != old
  print(f'entering-state frames: {len(a)}')
  print(f'  new branch fires (v_ego <= v_target): {fires.sum()}  ({100 * fires.mean():.0f}%)')
  if fires.any():
    print(f'  old code there: median {np.median(old[fires]):+.2f} m/s2 (worst {old[fires].min():+.2f}), new asks {_LEAVING_ACC:+.2f}')
    print(f'  speed there: median {np.median(v[fires]) * KPH:.1f} km/h, v_target median {np.median(vt[fires]) * KPH:.1f} km/h')
    print(f'  predicted lat acc: median {np.median(pred[fires]):.2f} of {np.median(cap[fires]):.2f} allowed '
          + f'({100 * np.median(pred[fires] / cap[fires]):.0f}%)')
  print(f'  frames still braking: {(~fires).sum()}, median demand {np.median(old[~fires]):+.2f} m/s2')


if __name__ == '__main__':
  main()
