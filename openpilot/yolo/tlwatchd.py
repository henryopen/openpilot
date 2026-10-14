#!/usr/bin/env python3
"""tlwatchd: at a standstill with nothing in front, watch the light and press GO when it turns green.

A bystander like yolod: it reads the road camera through VisionIPC and the car's state through
SubMaster, and its only way back into openpilot is the keypad's GO (openpilot.selfdrive.keypad.post), the
same thing the driver's key 7 does. So every guard the planner already puts on GO still holds -
engaged, not experimental, not braking, not held behind a stopped car (standstill_hold), spent
after GO_WAIT if the car has not moved - and nothing here can make the car do more than the
driver could with that key. Nothing in the driving stack waits on this process.

How it decides (2026-10-14, see stop-for-lights-findings in memory for the numbers):
  find   the first FIND_S seconds of a wait run the band detector (c4band1) and keep the places
         it saw red in at least half of its runs. The car is still, so a lamp keeps its pixels.
         No red lamp found -> do nothing for this wait.
  watch  every cycle: the light classifier (tlcls, none/red/go over the whole band, trained on
         the driver's own waits with the light's real green onset as the label) and, for each
         tracked lamp, how red its pixels still are. GO needs P(go) >= GO_P on GO_RUN cycles in a
         row AND every tracked lamp dark. The second part is what stops a pedestrian signal going
         green from releasing the car while ours is still red (it did fool the classifier once).
  Offline on 35 held-out waits (10/11-10/13): 0 releases before the green, 34 after it, median
  0.5 s after the green at 4 frames a second; one wait it never released.

ENABLE: without /data/tlwatch/ENABLE it only logs what it would have done (shadow mode).
DISABLE: /data/tlwatch/DISABLE stops it looking at all.

  tlwatchd.py                      live
  tlwatchd.py --replay <route dir glob>   replay recorded segments (fcamera.hevc + rlog), timed as live
"""
import argparse
import glob
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, '/data/yolo/pylibs')
sys.path.append('/data/openpilot')

import numpy as np

import yolo_core as yc

BASE = Path('/data/tlwatch')
ENABLE = BASE / 'ENABLE'
DISABLE = BASE / 'DISABLE'
MODEL_DET = '/data/yolo/c4band1_1344x352.int8v2.onnx'
MODEL_CLS = '/data/yolo/tlcls_v2_1344x352.onnx'

STOPPED = 0.3        # m/s
SETTLE = 1.5         # s stopped before looking: the car rocks on its springs as it stops
LEAD_NEAR = 15.0     # m: a car this close in front is what we are waiting for, not the light
FIND_S = 3.0         # s of detector runs to find the lamps (about 4 runs at 0.7 s)
FIND_MIN = 3         # detector runs needed before trusting the lamp list
DET_CONF = 0.15
LAMP_SHARE = 0.5     # a lamp has to be seen red in this share of the find runs
RED_FRAC = 0.5       # a lamp is still lit while its redness is above this share of its lit value
RED_MIN = 3.0        # a lamp whose lit redness is below this cannot be judged; it does not block
GO_P = 0.7
GO_RUN = 2
MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)


def redness(band, box):
  x1, y1, x2, y2 = box
  p = band[max(y1, 0):min(y2, band.shape[0]), max(x1, 0):min(x2, band.shape[1])].astype(np.int16)
  if p.size == 0:
    return 0.
  return float(np.percentile(np.clip(p[..., 0] - np.maximum(p[..., 1], p[..., 2]), 0, None), 90))


def snap(out, band, name):
  """Keep the band at the moments that matter, so each decision can be checked by eye afterwards."""
  try:
    from PIL import Image
    Image.fromarray(band).save(out / f"{name.replace(':', '').replace('+', '_')}.jpg", quality=85)
  except Exception:
    pass


def classify(sess, band):
  x = (band.astype(np.float32) / 255.).transpose(2, 0, 1)
  x = ((x - MEAN) / STD)[None]
  return sess.run(None, {'img': x})[0][0]   # none, red, go


class Wait:
  """One standstill: find the lamps, then watch them."""

  def __init__(self, t):
    self.t0 = t
    self.det_runs = 0
    self.reds = []          # (cx, cy, x1, y1, x2, y2, redness) from the find runs
    self.lamps = None       # [{'box', 'ref'}] once found; [] = no light here
    self.run = 0
    self.fired = None
    self.rows = []

  def find(self, det_sess, band, t):
    dets = yc.detect_band(det_sess, band, conf_thres=DET_CONF)
    self.det_runs += 1
    for d in dets:
      if d['light'] == 'red':
        x1, y1, x2, y2 = [int(v) for v in d['box']]
        self.reds.append(((x1 + x2) / 2, (y1 + y2) / 2, x1, y1, x2, y2, redness(band, (x1 - 1, y1 - 1, x2 + 2, y2 + 2))))
    if t - self.t0 >= FIND_S and self.det_runs >= FIND_MIN:
      clusters = []
      for r in self.reds:
        for c in clusters:
          if abs(c[0][0] - r[0]) < 20 and abs(c[0][1] - r[1]) < 20:
            c.append(r)
            break
        else:
          clusters.append([r])
      self.lamps = []
      for c in clusters:
        if len(c) >= LAMP_SHARE * self.det_runs:
          a = np.median(np.array([r[2:6] for r in c]), 0)
          self.lamps.append({'box': [int(a[0]) - 1, int(a[1]) - 1, int(a[2]) + 2, int(a[3]) + 2],
                             'ref': float(np.median([r[6] for r in c]))})

  def watch(self, cls_sess, band, t):
    p = classify(cls_sess, band)
    lit = [redness(band, L['box']) > RED_FRAC * L['ref'] for L in self.lamps if L['ref'] > RED_MIN]
    ok = p[2] >= GO_P and not any(lit)
    self.run = self.run + 1 if ok else 0
    self.rows.append({'t': round(t - self.t0, 2), 'p': [round(float(v), 3) for v in p], 'lit': lit})
    return self.run >= GO_RUN


def replay_source(pattern, bands=None):
  """(t, band, state) from recorded segments at 20 Hz, in order. state as the live loop builds it.

  The device has no video decoder, so with bands= the frames come from band JPEGs cut off the
  fcamera elsewhere (<bands>/<segment>/<frame:04d>.jpg, any subset of the 20 Hz frames)."""
  import subprocess
  from openpilot.tools.lib.logreader import LogReader
  if bands:
    from PIL import Image
  segs = sorted(glob.glob(pattern), key=lambda s: int(s.rsplit('--', 1)[1]))
  for seg in segs:
    st = {'v': 0., 'en': False, 'brake': False, 'lead': 999.}
    ev = []
    for m in LogReader(os.path.join(seg, 'rlog.zst')):
      w = m.which()
      if w == 'carState':
        ev.append((m.logMonoTime, 'cs', m.carState.vEgo, m.carState.brakePressed))
      elif w == 'selfdriveState':
        ev.append((m.logMonoTime, 'sd', m.selfdriveState.enabled))
      elif w == 'radarState':
        l1 = m.radarState.leadOne
        ev.append((m.logMonoTime, 'rs', l1.dRel if l1.present else 999.))
    ev.sort(key=lambda e: e[0])
    t_start = ev[0][0] if ev else 0
    t_seg = 60.0 * int(seg.rsplit('--', 1)[1])   # one timeline across consecutive segments
    if bands:
      def frames(seg=seg):
        for p in sorted(glob.glob(os.path.join(bands, os.path.basename(seg), '*.jpg'))):
          yield int(os.path.basename(p)[:-4]), np.asarray(Image.open(p).convert('RGB'))
    else:
      def frames(seg=seg):
        cmd = ['ffmpeg', '-v', 'error', '-r', '20', '-i', os.path.join(seg, 'fcamera.hevc'), '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-']
        pr = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        n, size = 0, 1344 * 760 * 3
        try:
          while True:
            raw = pr.stdout.read(size)
            if len(raw) < size:
              break
            yield n, np.frombuffer(raw, np.uint8).reshape(760, 1344, 3)[32:384]
            n += 1
        finally:
          pr.kill()
          pr.wait()
    k = 0
    for n, band in frames():
      t_ns = t_start + int(n * 50e6)
      while k < len(ev) and ev[k][0] <= t_ns:
        e = ev[k]
        if e[1] == 'cs':
          st['v'], st['brake'] = e[2], e[3]
        elif e[1] == 'sd':
          st['en'] = e[2]
        else:
          st['lead'] = e[2]
        k += 1
      yield f'{os.path.basename(seg)}+{n / 20:.2f}', t_seg + n / 20.0, band, dict(st)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--replay', default=None)
  ap.add_argument('--any-state', action='store_true', help='replay only: watch waits even where openpilot was not engaged')
  ap.add_argument('--bands', default=None, help='replay only: dir of band JPEGs per segment (the device has no decoder)')
  a = ap.parse_args()
  try:
    os.nice(19)
  except OSError:
    pass
  BASE.mkdir(parents=True, exist_ok=True)
  det = yc.make_session(MODEL_DET, threads=2)
  cls = yc.make_session(MODEL_CLS, threads=3)   # yolod measured 3 fastest onroad on these cores
  runs = BASE / ('replay' if a.replay else 'runs')
  name = datetime.now().strftime('%Y%m%d_%H%M%S')
  out = runs / name
  i = 1
  while out.exists():
    out = runs / f'{name}_{i}'
    i += 1
  out.mkdir(parents=True)
  log = (out / 'waits.jsonl').open('a', buffering=1)
  print(f'tlwatchd: {"replay " + a.replay if a.replay else "live"}, logging to {out}', flush=True)

  if a.replay:
    src = replay_source(a.replay, a.bands)
  else:
    from openpilot.cereal import log as clog, messaging
    from openpilot.cereal.visionipc import VisionStreamType
    from msgq.visionipc import VisionIpcClient
    from openpilot.selfdrive import keypad
    sm = messaging.SubMaster(['carState', 'selfdriveState', 'radarState', 'deviceState'])
    Thermal = clog.DeviceState.ThermalStatus
    vipc = None

  wait = None
  stopped_since = None
  t_busy_until = 0.
  while True:
    try:
      if a.replay:
        try:
          tag, t, band, st = next(src)
        except StopIteration:
          break
        if t < t_busy_until and wait is not None:     # still "computing" the previous cycle
          continue
        if a.any_state:
          st['en'] = True
        busy0 = time.monotonic()
      else:
        if DISABLE.exists():
          time.sleep(5)
          continue
        sm.update(100)
        cs = sm['carState']
        st = {'v': cs.vEgo, 'en': sm['selfdriveState'].enabled, 'brake': cs.brakePressed,
              'lead': sm['radarState'].leadOne.dRel if sm['radarState'].leadOne.present else 999.}
        t = time.monotonic()
        tag = datetime.now().strftime('%H:%M:%S')
        thermal = sm['deviceState'].thermalStatus
        if thermal == Thermal.critical:
          wait = None
          time.sleep(2)
          continue

      quiet = st['v'] < STOPPED and st['en'] and not st['brake'] and st['lead'] > LEAD_NEAR
      if not quiet:
        if wait is not None:
          log.write(json.dumps({'end': tag, 't0': wait.t0, 'dur': round(t - wait.t0, 1), 'det_runs': wait.det_runs,
                                'lamps': wait.lamps, 'fired': wait.fired, 'rows': wait.rows}) + '\n')
        wait = None
        stopped_since = None
        if not a.replay:
          time.sleep(0.2)
        continue
      if stopped_since is None:
        stopped_since = t
      if t - stopped_since < SETTLE:
        if not a.replay:
          time.sleep(0.1)
        continue
      if wait is None:
        wait = Wait(t)
      if wait.fired is not None or wait.lamps == []:
        # done with this wait (pressed, or there is no light to watch): no frames until the car moves
        if not a.replay:
          time.sleep(0.2)
        continue

      if not a.replay:
        if vipc is None or not vipc.is_connected():
          vipc = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_NARROW_ROAD, conflate=True)
          if not vipc.connect(False):
            vipc = None
            time.sleep(1)
            continue
        buf = vipc.recv()
        if buf is None:
          time.sleep(0.05)
          continue
        band = yc.nv12_to_band(buf)
        busy0 = time.monotonic()

      if wait.lamps is None:
        wait.find(det, band, t)
        if wait.lamps:
          snap(out, band, f'{tag}_found')
      elif wait.watch(cls, band, t):
        if not a.replay:
          # a cycle takes most of a second; the car must still be the car we decided about
          sm.update(0)
          cs = sm['carState']
          lead = sm['radarState'].leadOne
          if (cs.vEgo >= STOPPED or cs.brakePressed or not sm['selfdriveState'].enabled
              or (lead.present and lead.dRel <= LEAD_NEAR)):
            wait.run = 0
            continue
        wait.fired = round(t - wait.t0, 2)
        snap(out, band, f'{tag}_go')
        enabled = ENABLE.exists()
        if not a.replay and enabled:
          keypad.post('go')
        what = 'GO sent' if enabled and not a.replay else 'would GO (shadow)'
        print(f'tlwatchd: {tag} green, {what} after {wait.fired} s stopped', flush=True)
      cost = time.monotonic() - busy0
      if a.replay:
        t_busy_until = t + cost
      elif not a.replay and sm['deviceState'].thermalStatus == Thermal.overheated:
        time.sleep(3 * cost)      # back off: thermal throttling would slow the driving stack too
    except Exception as ex:      # never let one bad frame end the watch; systemd restarts on a crash anyway
      print('tlwatchd: error', repr(ex), flush=True)
      wait = None
      time.sleep(1)
  print('tlwatchd: replay done', flush=True)


if __name__ == '__main__':
  main()
