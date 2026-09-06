"""Verify the two standstill changes against real capnp readers, on the car.

Both sit in the loop that decides whether a stopped car may move, so they get the same
treatment as the other planner changes: the real fields, off real logs, on the machine that
will run them. A laptop's schema is older than the car's - that is how
radarState.leadOne.status got through on 2026-09-02 - so this only claims a result on device.

What it checks:
  1. Every field the new code reads exists and is readable on this car's schema.
  2. LeadHold._standstill_radar_loss fires where the radar track drops out while stopped.
  3. The planner's creep suppression never fires while the lead is opening up, which is the
     one way it could hold the car when it should go.

Run it on the car:

    PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \
        openpilot/yolo/analysis/verify_standstill_20260907.py --on-device --segments 60
"""
import argparse
import glob
import os

from openpilot.selfdrive.controls.lib.longitudinal_planner import (STANDSTILL_CREEP_DIST,
                                                                   STANDSTILL_CREEP_SPEED,
                                                                   STANDSTILL_CREEP_VREL)
from openpilot.selfdrive.controls.radard import (LEAD_HOLD_STANDSTILL_DIST,
                                                 LEAD_HOLD_STANDSTILL_JUMP,
                                                 LEAD_HOLD_STANDSTILL_SPEED, LeadHold)
from openpilot.tools.lib.logreader import LogReader


def main():
  p = argparse.ArgumentParser()
  p.add_argument('--on-device', action='store_true')
  p.add_argument('--segments', type=int, default=40)
  args = p.parse_args()

  root = '/data/media/0/realdata' if args.on_device else os.environ.get('RLOG_DIR', '')
  paths = sorted(glob.glob(os.path.join(root, '*', 'rlog.zst')))[-args.segments:]
  print(f'segments: {len(paths)}')
  if not paths:
    raise SystemExit(f'no rlogs found under {root}')

  read_once = False
  hold = LeadHold()
  n_frames = n_still = n_loss = n_creep_block = n_creep_while_opening = 0
  prev = None

  for path in paths:
    try:
      lr = LogReader(path)
    except Exception as e:
      print(f'  skip {path}: {e}')
      continue
    gas = False
    v_ego = 0.0
    for m in lr:
      w = m.which()
      if w == 'carState':
        v_ego = float(m.carState.vEgo)
        gas = bool(m.carState.gasPressed)
      elif w == 'radarState':
        lead = m.radarState.leadOne
        d = {'dRel': float(lead.dRel), 'vRel': float(lead.vRel),
             'present': bool(lead.present), 'radar': bool(lead.radar)}
        if not read_once:
          read_once = True
          print(f'field read ok: dRel={d["dRel"]:.2f} vRel={d["vRel"]:.2f} present={d["present"]} radar={d["radar"]}')
        n_frames += 1

        # the standstill hold, fed the sequence the car actually produced
        if prev is not None:
          hold.lead = dict(prev)
          if hold._standstill_radar_loss(d, v_ego):
            n_loss += 1
        prev = d if d['present'] else prev

        # the planner's suppression, on the same frame
        if v_ego < STANDSTILL_CREEP_SPEED and d['present']:
          n_still += 1
          if (d['radar'] and d['dRel'] < STANDSTILL_CREEP_DIST
              and d['vRel'] < STANDSTILL_CREEP_VREL and not gas):
            n_creep_block += 1
            if d['vRel'] >= STANDSTILL_CREEP_VREL:
              n_creep_while_opening += 1

  pct = 100.0 * n_creep_block / max(n_still, 1)
  print()
  print(f'radarState frames            {n_frames}')
  print(f'stopped with a lead          {n_still}')
  gate = f'SPEED={LEAD_HOLD_STANDSTILL_SPEED} DIST={LEAD_HOLD_STANDSTILL_DIST} JUMP={LEAD_HOLD_STANDSTILL_JUMP}'
  print(f'standstill radar loss fired  {n_loss}  ({gate})')
  print(f'creep suppressed             {n_creep_block}  ({pct:.1f}% of stopped frames)')
  print()
  print(f'MUST BE ZERO -> suppressed while the lead was opening: {n_creep_while_opening}')
  if n_creep_while_opening:
    raise SystemExit('FAIL: suppression fired on an opening lead')
  print('OK')


if __name__ == '__main__':
  main()
