#!/usr/bin/env python3
"""Replay JunctionHandoff over a route with and without the committed-stop rule, and report
exactly which frames change and what the car was doing there.

Run on the device:

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_committed_stop.py 00000026'
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, '/data/openpilot')

from openpilot.selfdrive.controls.lib import junction_handoff as jh
from openpilot.tools.lib.logreader import LogReader

KPH = 3.6


def replay(paths, committed_speed):
  """Run the handoff over the segments with COMMITTED_SPEED patched, return per-frame state."""
  saved = jh.COMMITTED_SPEED
  jh.COMMITTED_SPEED = committed_speed
  try:
    ctl = jh.JunctionHandoff()
    out = []
    for p in paths:
      rlog = os.path.join(p, 'rlog.zst')
      if not os.path.exists(rlog):
        rlog = os.path.join(p, 'rlog')
      if not os.path.exists(rlog):
        continue
      model = None
      lead = None
      cs = None
      for m in LogReader(rlog):
        w = m.which()
        if w == 'modelV2':
          model = m.modelV2
        elif w == 'radarState':
          lead = m.radarState.leadOne
        elif w == 'carState':
          cs = m.carState
          if model is None:
            continue
          ctl.update(model, cs, cs.vEgo, lead)
          out.append((cs.vEgo, float(ctl.active), ctl.a_floor,
                      lead.dRel if (lead is not None and lead.present) else -1.0,
                      float(cs.brakePressed), float(cs.gasPressed)))
    return np.array(out)
  finally:
    jh.COMMITTED_SPEED = saved


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
  print(f'segments: {len(paths)}, COMMITTED_SPEED = {jh.COMMITTED_SPEED} m/s = {jh.COMMITTED_SPEED * KPH:.1f} km/h')

  old = replay(paths, -1.0)      # veto always applies: the behaviour before this change
  new = replay(paths, jh.COMMITTED_SPEED)
  n = min(len(old), len(new))
  old, new = old[:n], new[:n]
  print(f'frames replayed: {n}')

  v, a_old, f_old, drel, br, gas = old.T
  _, a_new, f_new, _, _, _ = new.T

  print(f'\nhandoff active: old {a_old.mean() * 100:.1f}%  new {a_new.mean() * 100:.1f}%')
  chg = a_new != a_old
  print(f'frames where active differs: {int(chg.sum())} ({chg.mean() * 100:.2f}%)')
  gained = (a_new > a_old)
  lost = (a_new < a_old)
  print(f'  held on where it used to drop out: {int(gained.sum())}')
  print(f'  dropped out where it used to hold: {int(lost.sum())}  (must be 0)')

  if gained.any():
    gv = v[gained] * KPH
    print(f'\nspeed at the frames it now holds: p50 {np.median(gv):.1f}, max {gv.max():.1f} km/h')
    print(f'  all under COMMITTED_SPEED? {bool((v[gained] < jh.COMMITTED_SPEED).all())}')
    gd = drel[gained]
    seen = gd > 0
    if seen.any():
      print(f'  lead present in {100 * seen.mean():.0f}% of them, dRel p50 {np.median(gd[seen]):.1f} m')
    print(f'  the floor it now asks for: p50 {np.median(f_new[gained]):.2f}, worst {f_new[gained].min():.2f} m/s2')

  # the point of the change: fewer frames where the car was speeding up while nearly stopped
  slow = v < jh.COMMITTED_SPEED
  print(f'\nframes below {jh.COMMITTED_SPEED * KPH:.1f} km/h: {int(slow.sum())}')
  print(f'  handoff active there: old {100 * a_old[slow].mean():.0f}%  new {100 * a_new[slow].mean():.0f}%')

  print('\nsanity: the rule must never let go, only hold')
  print(f'  any frame where new is off and old was on: {int(lost.sum())}')


if __name__ == '__main__':
  main()
