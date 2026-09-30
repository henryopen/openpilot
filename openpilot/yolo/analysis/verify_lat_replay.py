#!/usr/bin/env python3
"""Run the torque lateral controller over recorded routes on the device, the first feedforward model
and the second side by side, open loop.

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_lat_replay.py 0000004c 0000004d'

Each carState frame gets the latest vehicleParameters, lateralDelay, lateralTorqueParameters, modelV2,
carControl.latActive and controlsState.desiredCurvature from the log, the way controlsd hands them
over. The first model is checked against what the car actually sent (actuators.torque) - if that does
not line up the replay itself is wrong and nothing else here means anything. Then what the second
changes: where, how much, and whether it runs at all.
"""
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, '/data/openpilot')
from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import latcontrol_torque as LT
from openpilot.selfdrive.controls.lib.nn_feedforward import NNFeedforward, MODEL_DIR
from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
from openpilot.tools.lib.logreader import LogReader

ROOT = '/data/media/0/realdata'


def make(CP, CI, which):
  c = LT.LatControlTorque(CP, CI, DT_CTRL)
  fp = CP.carFingerprint
  path = os.path.join(MODEL_DIR, f'{fp}.json' if which == 'v1' else f'{fp}_v2.json')
  c.nn_model = NNFeedforward(path)
  c.nn = c.nn_model
  c.nn_v2 = which == 'v2'
  return c


def main(prefixes):
  for pre in prefixes:
    routes = sorted({os.path.basename(p).rsplit('--', 1)[0] for p in glob.glob(f'{ROOT}/{pre}*--*')})
    for route in routes:
      segs = sorted(glob.glob(f'{ROOT}/{route}--*/rlog.zst'), key=lambda p: int(p.split('--')[-1].split('/')[0]))
      CP = None
      for m in LogReader(segs[0]):
        if m.which() == 'carParams':
          CP = m.carParams
          break
      CI = interfaces[CP.carFingerprint](CP)
      VM = VehicleModel(CP)
      ctl = {'v1': make(CP, CI, 'v1'), 'v2': make(CP, CI, 'v2')}
      last = {}
      rec = []
      exc = {'v1': 0, 'v2': 0}
      for p in segs:
        try:
          for m in LogReader(p):
            w = m.which()
            if w in ('vehicleParameters', 'lateralDelay', 'lateralTorqueParameters', 'modelV2', 'carControl', 'controlsState'):
              last[w] = getattr(m, w)
              if w == 'lateralTorqueParameters' and last[w].useParams:
                for c in ctl.values():
                  c.update_torque_parameters(last[w].latAccelFactorFiltered, last[w].latAccelOffsetFiltered,
                                             last[w].frictionCoefficientFiltered)
              continue
            if w != 'carState' or len(last) < 6:
              continue
            CS = m.carState
            lp = last['vehicleParameters']
            VM.update_params(max(lp.stiffnessFactor, 0.1), max(lp.steerRatio, 0.1))
            active = bool(last['carControl'].latActive)
            dc = float(last['controlsState'].desiredCurvature)
            lat_delay = last['lateralDelay'].lateralDelay + LAT_SMOOTH_SECONDS
            out = {}
            for k, c in ctl.items():
              c.plan = last['modelV2']
              try:
                steer, _, lac = c.update(active, CS, VM, lp, False, dc, False, lat_delay)
                out[k] = (float(steer), float(lac.f), float(lac.p))
              except Exception as e:
                exc[k] += 1
                if exc[k] <= 3:
                  print('EXC', k, repr(e))
                out[k] = (np.nan, np.nan, np.nan)
            v = float(CS.vEgo)
            curv = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg), v, lp.roll)
            rec.append((active and not CS.steeringPressed, v, dc * v * v, curv * v * v, float(last['carControl'].actuators.torque),
                        out['v1'][0], out['v2'][0]))
        except Exception as e:
          print('read error', p, repr(e)[:120])
      R = np.array(rec, dtype=float)
      on = R[:, 0] > 0.5
      logged, v1, v2 = R[:, 4], R[:, 5], R[:, 6]
      kph = R[:, 1] * 3.6
      print(f'== {route}: {len(R)} carState frames, lateral on hands-off {on.sum()}, exceptions v1 {exc["v1"]} v2 {exc["v2"]}')
      ok = on & np.isfinite(v1)
      corr = np.corrcoef(v1[ok], logged[ok])[0, 1]
      rmse = np.sqrt(np.mean((v1[ok] - logged[ok]) ** 2))
      print(f'  replay check, first model vs what the car sent: corr {corr:.3f}, RMSE {rmse:.4f}, '
            + f'median |diff| {np.median(np.abs(v1[ok] - logged[ok])):.4f}')
      ok2 = on & np.isfinite(v2)
      print(f'  second model: NaN {int(np.sum(on & ~np.isfinite(v2)))}, corr with the first {np.corrcoef(v1[ok2], v2[ok2])[0, 1]:.3f}')
      des = np.abs(R[:, 2])
      for name, m in (('straight (|req|<0.15)', des < 0.15), ('blend (0.15-0.5)', (des >= 0.15) & (des < 0.5)), ('turn (>=0.5)', des >= 0.5)):
        s = ok2 & m
        if s.sum() < 10:
          continue
        d = v2[s] - v1[s]
        ad = np.abs(d)
        print(f'  {name:22s} {s.sum():7d} frames: v2-v1 median {np.median(d):+.4f} |d| p90 {np.percentile(ad, 90):.4f} '
              + f'p99 {np.percentile(ad, 99):.4f}  |out| median v1 {np.median(np.abs(v1[s])):.3f} v2 {np.median(np.abs(v2[s])):.3f}')
      st = ok2 & (des < 0.1) & (kph > 30)
      idx = np.flatnonzero(st[1:] & st[:-1]) + 1
      for k, x in (('v1', v1), ('v2', v2), ('logged', logged)):
        print(f'  straights >30 km/h frame-to-frame |diff| p99 {k}: {np.percentile(np.abs(x[idx] - x[idx - 1]), 99):.4f}')
      for lo, hi in ((5, 15), (15, 25), (25, 40), (40, 70)):
        s = ok2 & (des >= 0.5) & (kph >= lo) & (kph < hi)
        if s.sum() > 50:
          print(f'  turns {lo}-{hi} km/h: |out| v1 {np.median(np.abs(v1[s])):.3f} v2 {np.median(np.abs(v2[s])):.3f} logged {np.median(np.abs(logged[s])):.3f}')


if __name__ == '__main__':
  main(sys.argv[1:] or ['0000004c', '0000004d'])
