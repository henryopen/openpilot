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
         No red lamp found -> watch on the classifier alone (below), which never presses GO.
  watch  every cycle: the light classifier (tlcls, none/red/go over the whole band, trained on
         the driver's own waits with the light's real green onset as the label) and, for each
         tracked lamp, how red its pixels still are. GO needs P(go) >= GO_P on GO_RUN cycles in a
         row AND every tracked lamp dark. The second part is what stops a pedestrian signal going
         green from releasing the car while ours is still red (it did fool the classifier once).
  Offline on 35 held-out waits (10/11-10/13): 0 releases before the green, 34 after it, median
  0.5 s after the green at 4 frames a second; one wait it never released.
  classifier alone (no lamp found; 10/14: the detector missed a plain red lamp at the one no-lead
         wait of that day): GO once P(red) >= 0.7 was seen on CLS_ONLY_RED cycles and then
         P(go) >= CLS_ONLY_P on CLS_ONLY_RUN cycles in a row. On the same 35 waits at the car's ~0.75 s
         cycle that is still 1 release 2 s before the green (the pedestrian signal), so it is shadow
         only: it shows and logs its GO and never presses, ENABLE or not.

Every other standstill (a car in front, the brake held, not engaged) the classifier still reads the
light each cycle, only for the HUD and the stop log, so every stop of a drive can be checked against
the video - not just the few with nothing in front.

Moving (2026-10-14): from GPS and the OSM signal list (/data/media/0/osm/tw_signals.npy, every
highway=traffic_signals and crossing=traffic_signals node in Taiwan) it works out the nearest signal
ahead - within AHEAD_RANGE m along the heading and CROSS_M m either side of it. Inside APPROACH_M of
one it runs the classifier every cycle and calls the light red or go once two cycles in a row agree
at LIGHT_P; that call is held LIGHT_HOLD s. Nothing is pressed while moving: it only reports.

Everything it knows is written to STATE (/dev/shm/tlwatch.json) a few times a second, for the HUD
and, once that is wired, the planner. A reader must treat a file older than a second as absent.

ENABLE: without /data/tlwatch/ENABLE it only logs what it would have done (shadow mode).
DISABLE: /data/tlwatch/DISABLE stops it looking at all.

C3 does the looking (2026-10-25, the driver: "C4就不要花效能去辨識燈號 由C3處理 當然最後要C4去綜合判斷"). C3 runs its own
finder + colour on its camera (c3link/c3lights.py, 88% of the lit frames found on the Custin night/rain set) and sends
the lights it has seen twice, as car-frame directions, through c3_relay to /dev/shm/c3lights.json. While that file is
fresh (C3_FRESH_S) this process grabs no frame and runs neither model: the light is C3's lights within C3_AZ_DEG of
straight ahead - any red there is red; a green ball and no red is go (a green arrow alone is not: C3 cannot tell its
direction). The OSM signal, the approach call (LightCall), the HUD state and every guard on GO stay as they are. At a
wait, GO on C3 needs a red seen earlier in this wait and then go held C3_GO_HOLD_S; it is pressed only with
/data/tlwatch/C3_GO as well as ENABLE - without it shadow (logged, the HUD says observe) until C3's calls are checked
against real drives. No fresh C3 (cable out, C3 down) -> C4's own models as before. 'src' in the logs and the HUD
state says which one decided.

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
C3_GO = BASE / 'C3_GO'          # GO on C3's call is pressed only with this too (see the top)
C3_FILE = '/dev/shm/c3lights.json'
C3_FRESH_S = 1.5                # c3_relay rewrites it twice a second while C3 sends
C3_AZ_DEG = 12.0                # the HUD's "in front" for C3's lights
C3_GO_HOLD_S = 1.5              # go (green ball, no red) held this long before a GO on C3's call
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
CLS_ONLY_RED = 2     # no lamp found: cycles of P(red) >= LIGHT_P needed before a go counts
CLS_ONLY_P = 0.85   # 0.85 and 0.9 score the same offline (1 early / 29 released of 35); 0.9 missed 10/14 18:39 by 0.007
CLS_ONLY_RUN = 3
GO_SHOW = 3.0        # s the HUD shows a GO for
SIGNALS = '/data/media/0/osm/tw_signals.npy'
STATE = '/dev/shm/tlwatch.json'
AHEAD_RANGE = 250.0  # m: how far ahead to look for a signal
CROSS_M = 20.0       # m either side of the heading a node may sit and still be on our road
APPROACH_M = 160.0   # m: inside this of a signal ahead, read the light while moving
MOVING = 3.0         # m/s
LIGHT_P = 0.7
LIGHT_HOLD = 2.0     # s a red / go call is held after the last cycle that agreed
GPS_MAX_ACC = 25.0   # m: worse than this and the signal list is not trusted
NEXT_SIGNAL_JUMP = 25.0  # m: the distance ahead growing by more than this = we passed one, a new one is ahead
MEAN = np.array([0.485, 0.456, 0.406], np.float32).reshape(3, 1, 1)
STD = np.array([0.229, 0.224, 0.225], np.float32).reshape(3, 1, 1)


def redness(band, box):
  x1, y1, x2, y2 = box
  p = band[max(y1, 0):min(y2, band.shape[0]), max(x1, 0):min(x2, band.shape[1])].astype(np.int16)
  if p.size == 0:
    return 0.
  return float(np.percentile(np.clip(p[..., 0] - np.maximum(p[..., 1], p[..., 2]), 0, None), 90))


# A yellow lamp lit on one of our heads: no GO (2026-10-16). The classifier has no yellow - it read the
# yellow after a red + right-arrow phase as go (0.90-0.95) with the red lamp already dark, GO went out,
# and the car pulled into the junction on the red countdown that followed. Amber pixels in the head:
# that yellow showed 65-87 (rgb ~150,86,40), a red ball's overexposed core shows as many - but then the
# red is still lit, which already blocks - and a green ball or arrow none. Vehicle heads only (landscape,
# or a tall vertical one): a pedestrian signal's orange figure and countdown read 13-21 at its green.
# Offline over the 35 held-out waits: releases, timing and false releases unchanged, 0 frames vetoed.
AMBER_MIN = 12


def amber(band, box, pad=2):
  x1, y1, x2, y2 = box
  p = band[max(y1 - pad, 0):min(y2 + pad, band.shape[0]), max(x1 - pad, 0):min(x2 + pad, band.shape[1])].astype(np.float32)
  if p.size == 0:
    return 0
  r, g, b = p[..., 0], p[..., 1], p[..., 2]
  return int(((r > 120) & (g > 0.35 * r) & (g < 0.80 * r) & (b < 0.45 * r) & (r - b > 70)).sum())


def vehicle_head(box):
  w, h = box[2] - box[0], box[3] - box[1]
  return w >= 1.5 * h or h >= 2.2 * w


class Signals:
  """The OSM traffic signals of Taiwan, and which one is ahead of us."""

  def __init__(self, path=SIGNALS):
    try:
      a = np.load(path)
      self.lat, self.lon, self.kind = a[:, 0], a[:, 1], a[:, 2].astype(int)
    except Exception:
      self.lat = self.lon = self.kind = np.zeros(0)

  def ahead(self, lat, lon, bearing_deg):
    """-> (metres along the heading to the nearest signal in front, kind 0 junction / 1 crossing) or (None, None)."""
    if len(self.lat) == 0:
      return None, None
    box = (np.abs(self.lat - lat) < 0.003) & (np.abs(self.lon - lon) < 0.003)
    if not box.any():
      return None, None
    kx = 111320.0 * np.cos(np.radians(lat))
    dx = (self.lon[box] - lon) * kx            # east
    dy = (self.lat[box] - lat) * 110540.0      # north
    b = np.radians(bearing_deg)
    along = dx * np.sin(b) + dy * np.cos(b)
    cross = dx * np.cos(b) - dy * np.sin(b)
    ok = (along > 0) & (along < AHEAD_RANGE) & (np.abs(cross) < CROSS_M)
    if not ok.any():
      return None, None
    i = np.argmin(np.where(ok, along, np.inf))
    return float(along[i]), int(self.kind[box][i])


class LightCall:
  """red / go once two cycles in a row agree, held LIGHT_HOLD s; otherwise unknown."""

  def __init__(self):
    self.last = None
    self.call, self.until = 'unknown', 0.

  def update(self, p, t):
    c = 'red' if p[1] >= LIGHT_P else ('go' if p[2] >= LIGHT_P else None)
    if c is not None and c == self.last:
      self.call, self.until = c, t + LIGHT_HOLD
    self.last = c
    if t > self.until:
      self.call = 'unknown'
    return self.call

  def reset(self):
    self.__init__()


def write_state(d):
  try:
    tmp = STATE + '.tmp'
    with open(tmp, 'w') as f:
      json.dump(d, f)
    os.replace(tmp, STATE)
  except Exception:
    pass


def snap(out, band, name):
  """Keep the band at the moments that matter, so each decision can be checked by eye afterwards."""
  try:
    from PIL import Image
    Image.fromarray(band).save(out / f"{name.replace(':', '').replace('+', '_')}.jpg", quality=85)
  except Exception:
    pass


def c3_read(now):
  """C3's call on the light ahead as [none, red, go] like the classifier's, or None when C3 is not there / not reading.
  red = the most confident red within C3_AZ_DEG; go = the most confident green ball there, 0 while any red is there."""
  try:
    with open(C3_FILE) as f:
      j = json.load(f)
  except (OSError, ValueError):
    return None
  if now - float(j.get('t', 0)) > C3_FRESH_S or j.get('idle') or not j.get('using'):
    return None
  front = [lt for lt in (j.get('lights') or []) if abs(float(lt.get('az', 99))) <= C3_AZ_DEG]
  red = max([float(lt.get('conf', 0)) for lt in front if lt.get('cls') == 'light_red'], default=0.)
  go = max([float(lt.get('conf', 0)) for lt in front if lt.get('cls') == 'light_green'], default=0.)
  if red > 0:
    go = 0.
  return [round(1. - max(red, go), 3), round(red, 3), round(go, 3)]


def classify(sess, band):
  x = (band.astype(np.float32) / 255.).transpose(2, 0, 1)
  x = ((x - MEAN) / STD)[None]
  return sess.run(None, {'img': x})[0][0]   # none, red, go


class Wait:
  """One standstill: find the lamps, then watch them."""

  def __init__(self, t, tag=''):
    self.t0 = t
    self.tag = tag          # wall clock (live) or segment+seconds (replay) at the start, to find it in the video
    self.det_runs = 0
    self.reds = []          # (cx, cy, x1, y1, x2, y2, redness) from the find runs
    self.lamps = None       # [{'box', 'ref'}] once found; [] = no light here
    self.run = 0
    self.fired = None
    self.rows = []
    self.reds_seen = 0      # classifier-only watch: cycles it has called red so far
    self.src = None         # 'c3' while C3 does the looking for this wait, else C4's models
    self.c3_red_t = None    # last time C3 saw a red in front during this wait
    self.c3_go_since = None

  def watch_c3(self, p, t):
    """C3's call (c3_read) at a wait: GO once a red was seen in this wait and go has held C3_GO_HOLD_S."""
    if p[1] > 0:
      self.c3_red_t = t
    ok = self.c3_red_t is not None and p[1] == 0 and p[2] >= GO_P
    self.c3_go_since = (self.c3_go_since or t) if ok else None
    self.rows.append({'t': round(t - self.t0, 2), 'p': p, 'lit': None, 'src': 'c3'})
    return ok and t - self.c3_go_since >= C3_GO_HOLD_S

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
    amb = [amber(band, L['box']) for L in self.lamps if vehicle_head(L['box'])]
    ok = p[2] >= GO_P and not any(lit) and not any(a >= AMBER_MIN for a in amb)
    self.run = self.run + 1 if ok else 0
    self.rows.append({'t': round(t - self.t0, 2), 'p': [round(float(v), 3) for v in p], 'lit': lit, 'amber': amb})
    return self.run >= GO_RUN

  def watch_cls(self, cls_sess, band, t):
    """No lamp to track: the classifier alone, stricter. Shadow only - the caller never presses on it."""
    p = classify(cls_sess, band)
    if p[1] >= LIGHT_P:
      self.reds_seen += 1
    ok = self.reds_seen >= CLS_ONLY_RED and p[2] >= CLS_ONLY_P
    self.run = self.run + 1 if ok else 0
    self.rows.append({'t': round(t - self.t0, 2), 'p': [round(float(v), 3) for v in p], 'lit': None})
    return self.run >= CLS_ONLY_RUN


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
  ap.add_argument('--state', action='store_true',
                  help='replay only: play in real time and write the HUD state file, to check the HUD end to end; '
                  + 'stop the tlwatchd service first, it writes the same file')
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
  appr_log = (out / 'approach.jsonl').open('a', buffering=1)
  print(f'tlwatchd: {"replay " + a.replay if a.replay else "live"}, logging to {out}', flush=True)

  if a.replay:
    src = replay_source(a.replay, a.bands)
  else:
    from openpilot.cereal import log as clog, messaging
    from openpilot.cereal.visionipc import VisionStreamType
    from msgq.visionipc import VisionIpcClient
    from openpilot.selfdrive import keypad
    sm = messaging.SubMaster(['carState', 'selfdriveState', 'radarState', 'deviceState', 'gpsLocationExternal'])
    Thermal = clog.DeviceState.ThermalStatus
    vipc = None
    signals = Signals()
    print(f'tlwatchd: {len(signals.lat)} OSM signals loaded', flush=True)

    def grab():
      """The latest road frame's light band, or None (camerad not up yet)."""
      nonlocal vipc
      if vipc is None or not vipc.is_connected():
        vipc = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_NARROW_ROAD, conflate=True)
        if not vipc.connect(False):
          vipc = None
          time.sleep(1)
          return None
      buf = vipc.recv()
      if buf is None:
        time.sleep(0.05)
        return None
      return yc.nv12_to_band(buf)

  look_log = (out / 'looks.jsonl').open('a', buffering=1)
  wait = None
  paused = None           # the wait of this standstill while something else (a car in front, the brake) has it
  stopped_since = None
  look = None            # a standstill it will not act on (car in front / brake / not engaged): read the light anyway
  t_busy_until = 0.
  replay_t0 = None
  light = LightCall()
  osm_m, osm_kind, osm_t, bearing = None, None, 0., 0.
  p_last, state_t = None, 0.
  # the last GO (when, pressed, classifier only): the HUD keeps it up GO_SHOW s even if the car moves
  # off in the same instant - the driver has to see the moment it called, not just that it did
  go_shown = (-1e9, False, False)
  c3p = None                # C3's call this cycle (live only; replays are C4's models)
  while True:
    try:
      if a.replay:
        try:
          tag, t, band, st = next(src)
        except StopIteration:
          break
        if a.state:
          # played at the recording's own pace, so the HUD sees it as it happened
          if replay_t0 is None:
            replay_t0 = time.monotonic() - t
          lag = replay_t0 + t - time.monotonic()
          if lag > 0:
            time.sleep(lag)
        if t < t_busy_until and (wait is not None or look is not None):     # still "computing" the previous cycle
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
        c3p = c3_read(t)          # C3's call on the light, or None -> C4's own models (see the top)
        thermal = sm['deviceState'].thermalStatus
        if thermal == Thermal.critical:
          wait = None
          time.sleep(2)
          continue
        # the signal ahead, twice a second, off GPS; the heading is held while stopped
        gps = sm['gpsLocationExternal']
        if t - osm_t > 0.5:
          osm_t = t
          if (sm.alive['gpsLocationExternal'] and gps.horizontalAccuracy < GPS_MAX_ACC
              and not (gps.latitude == 0 and gps.longitude == 0)):
            if gps.speed > 2.0:
              bearing = float(gps.bearingDeg)
            osm_prev = osm_m
            osm_m, osm_kind = signals.ahead(float(gps.latitude), float(gps.longitude), bearing)
            if osm_m is not None and osm_prev is not None and osm_m > osm_prev + NEXT_SIGNAL_JUMP:
              # passed that signal: the next one is another light. The held call belonged to the old one -
              # on 10/14 a (wrong) red was carried 2 s into the next approach and slowed for it at 142 m.
              light.reset()
              p_last = None
          else:
            osm_m, osm_kind = None, None

      if not a.replay or a.state:
        now = time.monotonic()
        if now - state_t > 0.25:
          state_t = now
          ws = None
          if wait is not None:
            phase = ('fired' if wait.fired is not None else 'finding' if wait.lamps is None
                     else 'watching_cls' if not wait.lamps else 'watching')
            ws = {'phase': phase, 'lamps': len(wait.lamps or []), 'fired': wait.fired, 'cls_only': wait.lamps == [],
                  'p': wait.rows[-1]['p'] if wait.rows else None, 'lit': wait.rows[-1]['lit'] if wait.rows else None}
          mode = ('standstill' if wait is not None else 'stopped' if look is not None
                  else 'approach' if osm_m is not None and osm_m <= APPROACH_M and st['v'] >= MOVING else 'idle')
          write_state({'t': now, 'mode': mode, 'osm_m': None if osm_m is None else round(osm_m, 1), 'osm_kind': osm_kind,
                       'light': light.call, 'p': p_last, 'wait': ws, 'why': look['why'] if look is not None else None,
                       'go': ({'age': round(now - go_shown[0], 1), 'sent': go_shown[1], 'cls_only': go_shown[2]}
                              if now - go_shown[0] < GO_SHOW else None),
                       # whether a GO from whoever is looking now would be pressed (C3's needs C3_GO too)
                       'enable': ENABLE.exists() and (c3p is None or C3_GO.exists()),
                       'src': 'c3' if c3p is not None else 'c4'})

      stopped = st['v'] < STOPPED
      quiet = stopped and st['en'] and not st['brake'] and st['lead'] > LEAD_NEAR
      if not quiet:
        # A wait broken while the car stays put (a scooter edging into LEAD_NEAR and out again, a dab of
        # the brake) is paused, not dropped: the lamps cannot have moved. 10/14 18:39 was cut in three
        # that way, each piece spent its first 3 s finding again, and the light went green inside that.
        if wait is not None:
          wait.run = 0
          paused = wait
          wait = None
        if paused is not None and not stopped:
          log.write(json.dumps({'start': paused.tag, 'end': tag, 't0': paused.t0, 'dur': round(t - paused.t0, 1),
                                'det_runs': paused.det_runs, 'lamps': paused.lamps, 'fired': paused.fired,
                                'cls_only': paused.lamps == [], 'src': paused.src, 'rows': paused.rows}) + '\n')
          paused = None
        if stopped:
          # A standstill it will not act on. Read the light anyway, for the HUD and the stop log, so the
          # driver can see at every stop whether it calls the light right.
          if stopped_since is None:
            stopped_since = t
          if t - stopped_since < SETTLE:
            if not a.replay:
              time.sleep(0.1)
            continue
          why = 'lead' if st['lead'] <= LEAD_NEAR else 'brake' if st['brake'] else 'off'
          if look is None:
            look = {'start': tag, 't0': t, 'why': why, 'rows': []}
          look['why'] = why
          if c3p is not None:        # C3 is looking: no frame, no model here
            p_last = c3p
            light.update(p_last, t)
            look['rows'].append({'t': round(t - look['t0'], 2), 'p': p_last, 'why': why, 'src': 'c3'})
            time.sleep(0.5)
            continue
          if not a.replay:
            band = grab()
            if band is None:
              continue
            busy0 = time.monotonic()
          p_last = [round(float(x), 3) for x in classify(cls, band)]
          light.update(p_last, t)
          look['rows'].append({'t': round(t - look['t0'], 2), 'p': p_last, 'why': why})
          cost = time.monotonic() - busy0
          if a.replay:
            t_busy_until = t + cost
          elif sm['deviceState'].thermalStatus == Thermal.overheated:
            time.sleep(3 * cost)
          continue
        if look is not None:
          look_log.write(json.dumps({'start': look['start'], 'end': tag, 'dur': round(t - look['t0'], 1),
                                     'rows': look['rows']}) + '\n')
          look = None
        stopped_since = None
        if not a.replay:
          if st['v'] >= MOVING and osm_m is not None and osm_m <= APPROACH_M and c3p is not None:
            # coming up to a signal, C3 looking: its call, report only
            p_last = c3p
            light.update(p_last, t)
            appr_log.write(json.dumps({'t': round(t, 2), 'wall': tag, 'v': round(float(st['v']), 2),
                                       'osm_m': round(osm_m, 1), 'kind': osm_kind, 'p': p_last, 'call': light.call,
                                       'lead': round(float(st['lead']), 1), 'en': bool(st['en']), 'src': 'c3'}) + '\n')
            time.sleep(0.5)
          elif st['v'] >= MOVING and osm_m is not None and osm_m <= APPROACH_M:
            # coming up to a signal: read the light, report only
            band = grab()
            if band is not None:
              b0 = time.monotonic()
              p_last = [round(float(x), 3) for x in classify(cls, band)]
              light.update(p_last, t)
              # the approach as it happened, so the planner's use of it can be replayed afterwards
              appr_log.write(json.dumps({'t': round(t, 2), 'wall': tag, 'v': round(float(st['v']), 2),
                                         'osm_m': round(osm_m, 1), 'kind': osm_kind, 'p': p_last, 'call': light.call,
                                         'lead': round(float(st['lead']), 1), 'en': bool(st['en'])}) + '\n')
              if sm['deviceState'].thermalStatus == Thermal.overheated:
                time.sleep(3 * (time.monotonic() - b0))
          else:
            if st['v'] >= MOVING:
              light.reset()
              p_last = None
            time.sleep(0.2)
        continue
      if stopped_since is None:
        stopped_since = t
      if t - stopped_since < SETTLE:
        if not a.replay:
          time.sleep(0.1)
        continue
      if look is not None:
        look_log.write(json.dumps({'start': look['start'], 'end': tag, 'dur': round(t - look['t0'], 1),
                                   'rows': look['rows']}) + '\n')
        look = None
      if wait is None:
        wait = paused if paused is not None else Wait(t, tag)
        paused = None
      if wait.fired is not None:
        # done with this wait: no frames until the car moves
        if not a.replay:
          time.sleep(0.2)
        continue

      if c3p is not None:
        # C3 is looking (see the top): its call instead of finding / watching lamps on C4's frames
        if wait.src != 'c3':
          wait.src, wait.run, wait.c3_go_since = 'c3', 0, None
          wait.lamps = [{'box': None, 'ref': 0., 'src': 'c3'}]   # the HUD's "watching", not "classifier only"
        if wait.watch_c3(c3p, t):
          sm.update(0)
          cs = sm['carState']
          lead = sm['radarState'].leadOne
          if (cs.vEgo >= STOPPED or cs.brakePressed or not sm['selfdriveState'].enabled
              or (lead.present and lead.dRel <= LEAD_NEAR)):
            wait.c3_go_since = None
            continue
          wait.fired = round(t - wait.t0, 2)
          enabled = ENABLE.exists() and C3_GO.exists()
          if enabled:
            keypad.post('go')
          go_shown = (time.monotonic(), enabled, False)
          print(f'tlwatchd: {tag} green (C3), {"GO sent" if enabled else "would GO (shadow)"} after {wait.fired} s stopped',
                flush=True)
        time.sleep(0.5)
        continue
      if wait.src == 'c3':           # C3 gone mid-wait: start over with C4's own models
        wait.src, wait.lamps, wait.run, wait.det_runs, wait.reds = 'c4', None, 0, 0, []

      if not a.replay:
        band = grab()
        if band is None:
          continue
        busy0 = time.monotonic()

      if wait.lamps is None:
        wait.find(det, band, t)
        if wait.lamps:
          snap(out, band, f'{tag}_found')
      elif wait.lamps == [] and wait.watch_cls(cls, band, t):
        # classifier alone: shadow only, never pressed (1 early release in 35 offline, see the top)
        wait.fired = round(t - wait.t0, 2)
        go_shown = (time.monotonic(), False, True)
        snap(out, band, f'{tag}_go_cls')
        print(f'tlwatchd: {tag} green (classifier only), would GO (shadow, never pressed) after {wait.fired} s stopped', flush=True)
      elif wait.lamps and wait.watch(cls, band, t):
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
        go_shown = (time.monotonic(), enabled and not a.replay, False)
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
