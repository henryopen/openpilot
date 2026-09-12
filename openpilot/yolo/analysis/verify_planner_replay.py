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
        # both the junction stop and the handoff's floor report as the mpc's e2e slot, so
        # the reason field alone cannot say which one set the accel. Keep them apart here.
        lead1 = sm['radarState'].leadOne
        a_targets.append((float(planner.output_a_target),
                          float(planner.stop_for_lights.is_active),
                          float(sm['carState'].vEgo),
                          float(planner.junction.a_floor),
                          float(int(planner.mpc.source)),
                          float(lead1.dRel) if lead1.present else -1.,
                          float(planner.stop_for_lights.stop_distance)))
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
    a, held, v, floor = arr[:, 0], arr[:, 1] > 0.5, arr[:, 2], arr[:, 3]
    print('a_target overall        : p01 %.2f p50 %.2f p99 %.2f, min %.2f' %
          (np.percentile(a, 1), np.median(a), np.percentile(a, 99), a.min()))
    if held.any():
      # The junction stop is a guess, so MAX_DECEL is the worst a wrong one may ask the
      # solver for, and that bound is the argument for committing at all. The handoff's
      # floor is a separate thing that can go firmer, and it overwrites a_target outright -
      # so only frames where the floor did NOT bite say anything about this bound.
      # the floor overwrites a_target outright, so after it bites the two are equal. A
      # frame where they match to the float and the floor is asking for braking is the
      # floor's, not the stop's - comparing with >= hands those to the wrong one.
      overridden = (np.abs(floor - a) < 1e-6) & (floor < 0)
      own = held & ~overridden
      print('committed frames        : %d (floor also biting on %d)' % (held.sum(), (held & ~own).sum()))
      if own.any():
        print('  stop point alone      : p50 %.2f, min %.2f (bound %.2f)%s' %
              (np.median(a[own]), a[own].min(), MAX_DECEL,
               '   <-- BELOW THE BOUND' if a[own].min() < MAX_DECEL - 0.05 else '   ok'))
        # who asked for it: the solve is bounded at MAX_DECEL only for the stop obstacle,
        # so a lead that is nearer is free to brake harder and rightly does
        below = own & (a < MAX_DECEL - 0.05)
        if below.any():
          src, lead_d, stop_d = arr[:, 4], arr[:, 5], arr[:, 6]
          names = {0: 'cruise', 1: 'lead0', 2: 'lead1', 3: 'e2e/stop'}
          print('  frames under the bound: %d' % below.sum())
          for s in sorted(set(src[below])):
            m2 = below & (src == s)
            print('    source %-9s n %4d  a p50 %.2f min %.2f  lead %s  stop point %.0f m' %
                  (names.get(int(s), int(s)), m2.sum(), np.median(a[m2]), a[m2].min(),
                   ('%.0f m' % np.median(lead_d[m2])) if np.median(lead_d[m2]) >= 0 else 'none',
                   np.median(stop_d[m2])))
          # A replay is open loop: the car in the log never braked for this stop, so the
          # speed stays up while the tracked point counts down with the recorded wheels,
          # and the solver is asked for what that geometry needs. If the demand matches
          # v^2 / 2d at those frames, the bound is not being broken by the controller -
          # the replay is handing it an approach that never happened.
          need = v[below] ** 2 / (2 * np.maximum(stop_d[below], 1.0))
          print('    speed there p50 %.1f kph; v^2/2d p50 %.2f vs demand p50 %.2f' %
                (np.median(v[below]) * 3.6, np.median(need), -np.median(a[below])))
      if (held & ~own).any():
        print('  handoff floor won     : p50 %.2f, min %.2f  (its own limit, not this one)' %
              (np.median(a[held & ~own]), a[held & ~own].min()))
      print('  speed while committed : p50 %.1f kph, max %.1f kph' %
            (np.median(v[held]) * 3.6, v[held].max() * 3.6))
    if (~held).any():
      print('a_target when not committed: min %.2f' % a[~held].min())
  sys.exit(1 if errors else 0)
