#!/usr/bin/env python3
"""The keypad's commands (selfdrive/keypad.py) against real messages, on the device.

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    python openpilot/yolo/analysis/verify_keypad.py <route> [--segments 3,4,5]'

1. GO in the planner. At every stop where stop_for_lights or the junction handoff is holding
   the car with no lead, a GO is injected 2 s into the stop: both have to let go, and the accel
   has to come up on standstill_hold's launch ramp. At every stop behind a held lead it has to
   be refused. The whole replay must run without an exception, and with nothing injected the
   output must be what it was (keypad.poll finds no file, so this is the same code path).
2. ENGAGE / DISENGAGE in CarEvents, on the recorded carState / carControl / pandaStates.
3. The -10 / +10 step in VCruiseHelper.

Open loop: the car in the log did not move when the GO went in, so the 3 s GO_WAIT runs out
and the junction is allowed back - that part is the timeout working, not a failure.
"""
import argparse
import glob
import os
import sys
import traceback
from types import SimpleNamespace

sys.path.insert(0, '/data/openpilot')

from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, GO_WAIT
from openpilot.selfdrive.controls.lib.stop_for_lights import STOPPED
from openpilot.common.realtime import DT_MDL
from openpilot.tools.lib.logreader import LogReader

NEEDED = ('carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState',
          'modelV2', 'selfdriveState')
INJECT_AFTER = 2.0     # s into a stop


def take(pending):
  out = pending[:]
  pending.clear()
  return out


def rlogs(paths):
  for p in paths:
    for name in ('rlog.zst', 'rlog'):
      if os.path.exists(os.path.join(p, name)):
        yield os.path.join(p, name)
        break


def check_planner(paths, cp):
  planner = LongitudinalPlanner(cp)
  pending = []
  planner.keypad.poll = lambda: take(pending)
  sm = {}
  frames = errors = 0
  first_error = None
  stopped_for = 0.
  injected = False
  watch = None
  results = []
  for rl in rlogs(paths):
    for m in LogReader(rl):
      w = m.which()
      if w in NEEDED:
        sm[w] = getattr(m, w)
      if w != 'modelV2' or len(sm) < len(NEEDED):
        continue
      v = float(sm['carState'].vEgo)
      enabled = bool(sm['selfdriveState'].enabled)
      stopped_for = stopped_for + DT_MDL if (v < STOPPED and enabled) else 0.
      if stopped_for == 0.:
        injected = False
      held_light = planner.stop_for_lights.is_active or planner.junction.active
      held_lead = planner.standstill_hold.active
      if not injected and stopped_for >= INJECT_AFTER and (held_light or held_lead) \
         and not sm['carState'].brakePressed:
        pending.append('go')
        injected = True
        watch = {'kind': 'lead' if held_lead else 'light', 't': 0., 'a': [], 'sfl': [], 'junction': [],
                 'accepted': None}
      try:
        planner.update(sm)
        frames += 1
      except Exception:
        errors += 1
        if first_error is None:
          first_error = traceback.format_exc()
        continue
      if watch is not None:
        if watch['accepted'] is None:
          watch['accepted'] = planner.go_left > 0.
        watch['a'].append(float(planner.output_a_target))
        watch['sfl'].append(bool(planner.stop_for_lights.is_active))
        watch['junction'].append(bool(planner.junction.active))
        watch['t'] += DT_MDL
        if watch['t'] >= GO_WAIT - DT_MDL:
          results.append(watch)
          watch = None

  print(f'planner frames {frames}, exceptions {errors}')
  if first_error:
    print(first_error)
  ok = errors == 0
  light = [r for r in results if r['kind'] == 'light']
  lead = [r for r in results if r['kind'] == 'lead']
  print(f'stops at a light, GO injected: {len(light)}')
  for r in light:
    a = r['a']
    released = not any(r['sfl']) and not any(r['junction'][:int(GO_WAIT / DT_MDL) - 2])
    rising = a[int(1.0 / DT_MDL)] > 0. and a[-2] >= a[int(1.0 / DT_MDL)]
    good = r['accepted'] and released and rising
    ok &= good
    at = [a[int(t / DT_MDL)] for t in (0.5, 1.0, 2.0)] + [a[-2]]
    print(f"  {'PASS' if good else 'FAIL'} accepted {r['accepted']}  released {released}  " +
          f"a_target @0.5s {at[0]:.2f} @1s {at[1]:.2f} @2s {at[2]:.2f} @2.9s {at[3]:.2f}")
  print(f'stops behind a held lead, GO injected: {len(lead)}')
  for r in lead:
    good = not r['accepted'] and max(r['a']) <= 0.
    ok &= good
    print(f"  {'PASS' if good else 'FAIL'} refused {not r['accepted']}  max a_target {max(r['a']):.2f}")
  if not light:
    print('  (no light stop in these segments - pick others)')
    ok = False
  return ok


def check_events(paths, cp):
  ce = CarEvents(cp)
  pending = []
  ce.keypad.poll = lambda: take(pending)
  sm = {}
  prev_cs = None
  seen = {'cancel': [0, 0], 'enable': [0, 0], 'refused': [0, 0]}   # [cases, right]
  last_kind = None
  for rl in rlogs(paths):
    for m in LogReader(rl):
      w = m.which()
      if w in ('carControl', 'pandaStates'):
        sm[w] = getattr(m, w)
      if w != 'carState' or len(sm) < 2:
        continue
      cs = m.carState
      if prev_cs is None:
        prev_cs = cs
        continue
      cc = sm['carControl']
      panda_allowed = any(ps.controlsAllowed for ps in sm['pandaStates'])
      can_engage = panda_allowed and cs.cruiseState.available and str(cs.gearShifter) == 'drive' and not cs.brakePressed
      if cc.enabled:
        kind, cmd = 'cancel', 'disengage'
      elif can_engage:
        kind, cmd = 'enable', 'engage'
      else:
        kind, cmd = 'refused', 'engage'
      # one injection per change of situation, so the cases are spread over the drive
      if kind != last_kind:
        pending.append(cmd)
        names = [str(e.name) for e in ce.update(cs, prev_cs, cc, panda_allowed).to_msg()]
        seen[kind][0] += 1
        if kind == 'cancel':
          seen[kind][1] += 'buttonCancel' in names
        elif kind == 'enable':
          seen[kind][1] += 'buttonEnable' in names
        else:
          seen[kind][1] += 'buttonEnable' not in names
        last_kind = kind
      prev_cs = cs
  ok = True
  for kind, (n, right) in seen.items():
    print(f'  {kind:<8} {n:3d} cases, {right:3d} right')
    ok &= n > 0 and n == right
  return ok


def check_speed():
  cp = SimpleNamespace(pcmCruise=False)
  h = VCruiseHelper(cp)
  cs = SimpleNamespace(gasPressed=False, vEgo=0.)
  ok = True
  for start, enabled, sign, want in [(47, True, +1, 50), (50, True, +1, 60), (60, True, -1, 50), (47, True, -1, 40),
                                     (50, False, +1, 50), (140, True, +1, 145), (10, True, -1, 8)]:
    h.v_cruise_kph = start
    h.keypad_step(cs, enabled, sign)
    good = h.v_cruise_kph == want
    ok &= good
    print(f"  {'PASS' if good else 'FAIL'} {start:3d} {'+' if sign > 0 else '-'}{'' if enabled else ' (not engaged)'}" +
          f" -> {h.v_cruise_kph} (want {want})")
  return ok


if __name__ == '__main__':
  ap = argparse.ArgumentParser()
  ap.add_argument('route')
  ap.add_argument('--root', default='/data/media/0/realdata')
  ap.add_argument('--segments', default='')
  args = ap.parse_args()
  paths = sorted(glob.glob(os.path.join(args.root, args.route + '*')), key=lambda x: int(x.rstrip('/').split('--')[-1]))
  if args.segments:
    want = {int(x) for x in args.segments.split(',')}
    paths = [x for x in paths if int(x.rstrip('/').split('--')[-1]) in want]
  cp = None
  for rl in rlogs(paths):
    for m in LogReader(rl):
      if m.which() == 'carParams':
        cp = m.carParams
        break
    if cp is not None:
      break
  if cp is None:
    raise SystemExit('no carParams in these segments')
  print(f'{len(paths)} segments')
  print('== speed step')
  ok_speed = check_speed()
  print('== engage / disengage')
  ok_events = check_events(paths, cp)
  print('== go')
  ok_plan = check_planner(paths, cp)
  print('ALL PASS' if ok_speed and ok_events and ok_plan else 'FAILED')
  sys.exit(0 if ok_speed and ok_events and ok_plan else 1)
