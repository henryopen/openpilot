#!/usr/bin/env python3
"""Run the whole LongitudinalPlanner - MPC and all - over a recorded route on the device.

ruff and an import test do not exercise the capnp readers or the solver, and this line has
broken on the car twice for exactly that reason (a slice on a capnp list in 08-27, a field
name that existed only in the laptop's schema in 09-02). So the check that matters is the
real planner, the real messages, and the real acados solve, frame by frame.

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_planner_replay.py 00000026 \
      --segments 28,35,49,50'

Reports any exception, and what the plan reason was over the replay - stopLight appearing
is the junction stop reaching the MPC, which is the thing being verified.
"""
import argparse
import glob
import os
import sys
import traceback
from collections import Counter

sys.path.insert(0, '/data/openpilot')

from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.tools.lib.logreader import LogReader

NEEDED = ('carControl', 'carState', 'controlsState', 'vehicleParameters', 'radarState',
          'modelV2', 'selfdriveState')


class FakeSubMaster(dict):
  """The planner only ever indexes sm, so a dict of the latest message of each type is
  the whole interface. Keeping the capnp readers themselves is the point."""


def replay(paths, cp):
  planner = LongitudinalPlanner(cp)
  sm = FakeSubMaster()
  reasons = Counter()
  frames = errors = 0
  first_error = None
  a_targets = []
  for p in paths:
    rlog = os.path.join(p, 'rlog.zst')
    if not os.path.exists(rlog):
      rlog = os.path.join(p, 'rlog')
    if not os.path.exists(rlog):
      continue
    for m in LogReader(rlog):
      w = m.which()
      if w in NEEDED:
        sm[w] = getattr(m, w)
      if w != 'modelV2' or len(sm) < len(NEEDED):
        continue
      try:
        planner.update(sm)
        frames += 1
        reasons[str(planner.plan_reason)] += 1
        a_targets.append((float(planner.output_a_target),
                          float(planner.stop_for_lights.is_active),
                          float(sm['carState'].vEgo)))
      except Exception:
        errors += 1
        if first_error is None:
          first_error = traceback.format_exc()
  return frames, errors, first_error, reasons, a_targets


if __name__ == '__main__':
  ap = argparse.ArgumentParser()
  ap.add_argument('route')
  ap.add_argument('--root', default='/data/media/0/realdata')
  ap.add_argument('--segments', default='')
  args = ap.parse_args()

  paths = sorted(glob.glob(os.path.join(args.root, args.route + '*')),
                 key=lambda x: int(x.rstrip('/').split('--')[-1]))
  if args.segments:
    want = {int(x) for x in args.segments.split(',')}
    paths = [x for x in paths if int(x.rstrip('/').split('--')[-1]) in want]

  # CarParams comes out of the route itself, not the params directory: that key only
  # exists while the car is on, and this runs parked. The recorded one is also the
  # configuration the drive actually had, which is the one worth replaying against.
  cp = None
  for p in paths:
    rl = os.path.join(p, 'rlog.zst')
    if not os.path.exists(rl):
      rl = os.path.join(p, 'rlog')
    if not os.path.exists(rl):
      continue
    for m in LogReader(rl):
      if m.which() == 'carParams':
        cp = m.carParams
        break
    if cp is not None:
      break
  if cp is None:
    raise SystemExit('no carParams in these segments')
  print('car %s, openpilotLongitudinalControl %s' % (cp.carFingerprint, cp.openpilotLongitudinalControl))
  print('%d segments' % len(paths))

  frames, errors, err, reasons, a_targets = replay(paths, cp)
  print('planner frames %d, exceptions %d' % (frames, errors))
  if err:
    print(err)
  total = max(sum(reasons.values()), 1)
  for name, n in reasons.most_common():
    print('  %-28s %6d (%5.2f%%)' % (name, n, 100 * n / total))
  if a_targets:
    import numpy as np
    from openpilot.selfdrive.controls.lib.stop_for_lights import MAX_DECEL
    arr = np.array(a_targets)
    a, held, v = arr[:, 0], arr[:, 1] > 0.5, arr[:, 2]
    print('a_target overall     : p01 %.2f p50 %.2f p99 %.2f, min %.2f' %
          (np.percentile(a, 1), np.median(a), np.percentile(a, 99), a.min()))
    if held.any():
      # the junction stop is a guess, so MAX_DECEL is the worst a wrong one may ask for.
      # Anything below it came from somewhere else and is worth knowing about.
      print('a_target while committed: n %d, p50 %.2f, min %.2f (floor %.2f)%s' %
            (held.sum(), np.median(a[held]), a[held].min(), MAX_DECEL,
             '   <-- BELOW THE FLOOR' if a[held].min() < MAX_DECEL - 0.05 else ''))
      print('  speed while committed : p50 %.1f kph, max %.1f kph' %
            (np.median(v[held]) * KPH, v[held].max() * KPH))
    print('a_target elsewhere   : min %.2f' % a[~held].min() if (~held).any() else '')
  sys.exit(1 if errors else 0)
