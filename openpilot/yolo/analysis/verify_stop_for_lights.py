#!/usr/bin/env python3
"""Replay StopForLights over a route now that the planner drives it again, and report
every stop it would commit for, every stop it leaves alone, and when the hold lets go.

Run on the device, which is the only place the capnp readers are the real ones:

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_stop_for_lights.py 00000026'

Measured off-device on the 9/9 route (36 segments covering all 31 stops, 34361 engaged
model frames) before wiring it back in:

  armed 2.69% of frames, committed 2.37%
  six stops with a lead present for under half the approach - what this is for - and it
    committed for five of them
  eighteen stops with a lead: committed for none, the follow MPC keeps them
  six commit episodes: four ran into a real red light (checked on video), two lasted
    under a second with a lead in front and cleared themselves
  the standstill hold released 0.5-1.2 s BEFORE the driver pulled away at every stop it
    would have held, so it does not strand the car at a green

A commit episode that does not end in a stop, or a release later than the departure, is
what would make this unusable. Both are reported below.
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, '/data/openpilot')

from openpilot.selfdrive.controls.lib.stop_for_lights import StopForLights
from openpilot.tools.lib.logreader import LogReader

KPH = 3.6
MPC_STOP_DISTANCE = 7.0     # what the planner passes to obstacle_x
HELD = 2.0                  # a stop is a halt this long
ROLLING = 15 / KPH          # ...that we rolled into
MOVING = 1.0                # m/s, pulled away


def replay(paths):
  rows = []
  for p in paths:
    rlog = os.path.join(p, 'rlog.zst')
    if not os.path.exists(rlog):
      rlog = os.path.join(p, 'rlog')
    if not os.path.exists(rlog):
      continue
    seg = int(p.rstrip('/').split('--')[-1])
    sfl = StopForLights()
    v = vc = 0.0
    gas = ena = False
    lead = None
    t0 = None
    for m in LogReader(rlog):
      w = m.which()
      if t0 is None:
        t0 = m.logMonoTime
      t = (m.logMonoTime - t0) / 1e9
      if w == 'carState':
        v = m.carState.vEgo
        gas = m.carState.gasPressed
        vc = m.carState.vCruise / KPH
      elif w == 'radarState':
        lead = m.radarState.leadOne
      elif w == 'selfdriveState':
        ena = m.selfdriveState.enabled
      elif w == 'modelV2':
        if not ena:
          sfl.reset()
          rows.append((seg, t, v, -1., 0., 0., 0., 0.))
          continue
        sfl.update(m.modelV2, v, vc, gas, lead)
        rows.append((seg, t, v,
                     lead.dRel if (lead is not None and lead.present) else -1.,
                     float(sfl.armed), float(sfl.is_active), 1.,
                     float(m.modelV2.action.shouldStop)))
  return np.array(rows) if rows else np.empty((0, 8))


SEG, T, V, LEAD, ARMED, ACTIVE, ENA, SHOULD_STOP = range(8)


def report(a):
  if not len(a):
    print('no frames')
    return
  eng = a[:, ENA] > 0.5
  print('engaged model frames %d: armed %d (%.2f%%), committed %d (%.2f%%)' %
        (eng.sum(), (a[eng, ARMED] > 0.5).sum(), 100 * (a[eng, ARMED] > 0.5).mean(),
         (a[eng, ACTIVE] > 0.5).sum(), 100 * (a[eng, ACTIVE] > 0.5).mean()))

  print('\nstops:')
  print('%-5s %-9s %-7s %-8s %-8s %-10s %s' %
        ('seg', 't', 'held s', 'lead %', 'engaged', 'committed', 'hold releases'))
  ours = hit = theirs = wrong = 0
  for seg in sorted(set(a[:, SEG])):
    s = a[a[:, SEG] == seg]
    halted = s[:, V] < 0.5
    i = 0
    while i < len(s):
      if halted[i]:
        j = i
        while j < len(s) and halted[j]:
          j += 1
        held = s[j - 1, T] - s[i, T]
        appr = s[max(0, i - 240):i]
        if held >= HELD and len(appr) and appr[:, V].max() > ROLLING:
          lead_f = float((appr[:, LEAD] >= 0).mean())
          eng_f = float((appr[:, ENA] > 0.5).mean())
          got = bool(appr[:, ACTIVE].max() or s[i:j, ACTIVE].max())
          # when the hold would let go, against when the car moved
          rel = mv = float('nan')
          stopped_for = clear_for = 0.
          for k in range(i, len(s)):
            stopped_for += 0.05
            if s[k, SHOULD_STOP] > 0.5:
              clear_for = 0.
            else:
              clear_for += 0.05
              if stopped_for > 1.5 and clear_for > 1.0:
                rel = s[k, T] - s[i, T]
                break
          for k in range(j, len(s)):
            if s[k, V] > MOVING:
              mv = s[k, T] - s[i, T]
              break
          note = ''
          if rel == rel and mv == mv:
            note = '%.1f s vs departure %.1f s%s' % (rel, mv, '  <-- LATE' if rel > mv + 0.5 else '')
          elif rel != rel:
            note = 'not within the segment'
          if eng_f > 0.5:
            if lead_f < 0.5:
              ours += 1
              hit += got
            else:
              theirs += 1
              wrong += got
          print('%-5d %-9.1f %-7.1f %-8.0f %-8.0f %-10s %s' %
                (seg, s[i, T], held, 100 * lead_f, 100 * eng_f, 'YES' if got else '-', note))
        i = j
      i += 1

  print('\nengaged stops with a lead under half the approach: %d, committed for %d' % (ours, hit))
  print('engaged stops with a lead                         : %d, committed for %d%s' %
        (theirs, wrong, '   <-- should be 0' if wrong else ''))

  print('\ncommit episodes:')
  for seg in sorted(set(a[:, SEG])):
    s = a[a[:, SEG] == seg]
    act = np.where(s[:, ACTIVE] > 0.5)[0]
    if not len(act):
      continue
    runs = np.split(act, np.where(np.diff(act) > 20)[0] + 1)
    for r in runs:
      dur = s[r[-1], T] - s[r[0], T]
      vmin = s[r, V].min() * KPH
      print('  seg %-4d t=%-8.1f %5.1f s  v %3.0f -> %3.0f kph%s' %
            (seg, s[r[0], T], dur, s[r[0], V] * KPH, vmin,
             '   <-- no slowing, check this' if vmin > 15 and dur > 2 else ''))


if __name__ == '__main__':
  p = argparse.ArgumentParser()
  p.add_argument('route', help='route prefix, e.g. 00000026')
  p.add_argument('--root', default='/data/media/0/realdata')
  p.add_argument('--segments', default='', help='comma separated, default all')
  args = p.parse_args()

  paths = sorted(glob.glob(os.path.join(args.root, args.route + '*')),
                 key=lambda x: int(x.rstrip('/').split('--')[-1]))
  if args.segments:
    want = {int(x) for x in args.segments.split(',')}
    paths = [x for x in paths if int(x.rstrip('/').split('--')[-1]) in want]
  print('%d segments' % len(paths))
  report(replay(paths))
