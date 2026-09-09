#!/usr/bin/env python3
"""Replay the lead gate over a recorded route: how often the road counts as clear, and what
the raised ceiling would have been worth there.

Run on the device (needs the real cereal schema):

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_free_accel.py 00000026'
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, '/data/openpilot')

from openpilot.selfdrive.controls.lib.longitudinal_planner import (A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS,
                                                                   A_CRUISE_MAX_VALS_FREE, FREE_LEAD_MARGIN,
                                                                   get_max_accel, lead_is_far)
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW
from openpilot.tools.lib.logreader import LogReader

KPH = 3.6


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

  print(f'FREE_LEAD_MARGIN = {FREE_LEAD_MARGIN}')
  print(f'normal ceiling = {A_CRUISE_MAX_VALS}')
  print(f'free   ceiling = {A_CRUISE_MAX_VALS_FREE}')
  print(f'breakpoints    = {A_CRUISE_MAX_BP}')

  rows = []
  exceptions = 0
  for p in paths:
    rlog = os.path.join(p, 'rlog.zst')
    if not os.path.exists(rlog):
      rlog = os.path.join(p, 'rlog')
    if not os.path.exists(rlog):
      continue
    lead = None
    active = False
    t_follow = 1.5
    atgt = 0.0
    for m in LogReader(rlog):
      w = m.which()
      if w == 'radarState':
        lead = m.radarState.leadOne
      elif w == 'selfdriveState':
        active = m.selfdriveState.active
        try:
          t_follow = get_T_FOLLOW(m.selfdriveState.personality)
        except Exception:
          pass
      elif w == 'longitudinalPlan':
        atgt = m.longitudinalPlan.aTarget
      elif w == 'carState':
        if lead is None:
          continue
        v = m.carState.vEgo
        try:
          free = lead_is_far(lead, v, t_follow)
          cap_now = get_max_accel(v, False)
          cap_free = get_max_accel(v, True)
        except Exception as e:
          exceptions += 1
          if exceptions < 4:
            print(f'EXCEPTION {type(e).__name__}: {e}')
          continue
        rows.append((v, float(free), cap_now, cap_free, float(active), atgt,
                     lead.dRel if lead.present else 999.0, float(lead.present)))

  print(f'\ncapnp exceptions: {exceptions}   (must be 0)')
  a = np.array(rows)
  v, free, cap_now, cap_free, act, atgt, drel, present = a.T
  eng = act > 0
  print(f'frames: {len(a)}, engaged {eng.sum()}')
  print(f'road counts as clear: {100 * free[eng].mean():.0f}% of engaged frames')
  print(f'  of those, no lead at all: {100 * (present[eng & (free > 0)] < 0.5).mean():.0f}%')

  hdr = ('band', 'frames', 'clear%', 'cap_now', 'cap_free', 'gain', 'at_ceiling%')
  print(f'\n{hdr[0]:<14} {hdr[1]:>8} {hdr[2]:>8} {hdr[3]:>9} {hdr[4]:>9} {hdr[5]:>9} {hdr[6]:>11}')
  for lo, hi in [(15, 25), (25, 36), (36, 45), (45, 54), (54, 72), (72, 90), (90, 110)]:
    m = eng & (v * KPH >= lo) & (v * KPH < hi)
    if m.sum() < 60:
      continue
    mf = m & (free > 0)
    at_ceil = (atgt[mf] > 0.92 * cap_now[mf]).mean() if mf.sum() else 0.0
    cn, cf = np.median(cap_now[m]), np.median(cap_free[m])
    band = f'{lo}-{hi} km/h'
    print(f'{band:<14} {m.sum():>8d} {100 * free[m].mean():>7.0f}% {cn:>9.2f} {cf:>9.2f} {cf - cn:>9.2f} {100 * at_ceil:>10.0f}%')

  print('\nsanity: a lead this close must never read as clear')
  for d, vv, vl in [(5.0, 10.0, 0.0), (20.0, 15.0, 0.0), (50.0, 20.0, 0.0), (50.0, 8.0, 8.0)]:
    class L:
      present = True
      dRel = d
      vLead = vl
    got = lead_is_far(L(), vv, 1.5)
    print(f'  lead {d:5.1f} m, ego {vv * KPH:4.1f} km/h, lead {vl * KPH:4.1f} km/h -> clear={got}')


if __name__ == '__main__':
  main()
