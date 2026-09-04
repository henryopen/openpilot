#!/usr/bin/env python3
"""yolod: run YOLO11n on the road camera and serve the result to the PiBar HUD.

Deliberately a bystander. It reads the camera through VisionIPC and publishes nothing back
into openpilot, so nothing it does can reach the controls. It runs at the lowest priority
and paces itself off its own measured cost, so the driving stack always gets the CPU first.

  GET /frame.jpg -> latest overlay, JPEG
  GET /det.json  -> latest detections + timing
  GET /health    -> {"ok": true, ...}

Every detection is also appended to /data/yolo/runs/<start>/det.jsonl, and an overlay is
saved periodically, so a drive can be reviewed afterwards.
"""
import json
import os
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, '/data/yolo/pylibs')
sys.path.append('/data/openpilot')  # the openpilot package resolves off the repo root, not an install

import numpy as np

import lane_type
import yolo_core as yc
from openpilot.cereal import log, messaging
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient

PORT = 8903
# Two models rather than one: a light is 20 px tall and cannot be resampled, a car is 78 px
# and can. One model covering both would have to take the whole frame at native resolution,
# measured at 2027 ms against 1436 ms for these two together. They take turns, one frame each.
MODEL_ROAD = '/data/yolo/c4light13_672x384.onnx'
MODEL_BAND = '/data/yolo/c4band1_1344x352.onnx'
RUNS = Path('/data/yolo/runs')
DISABLE = Path('/data/yolo/DISABLE')

DUTY = 0.45         # fraction of wall time this process may spend inferring
DUTY_HOT = 0.12     # back well off once the device says it is overheating: thermal throttling
                    # would slow the driving stack down too, and this is the only part of it
                    # we are actually coupled to
MIN_PERIOD = 0.4    # never faster than 2.5 Hz; the HUD does not need more
SAVE_EVERY = 8.0    # seconds between overlays kept on disk
# motorcycles, people and traffic lights are the cases the driving model is weakest on, so
# keep more pictures of those to judge afterwards
SAVE_EVERY_INTERESTING = 2.0
INTERESTING = {0, 1, 3, 9}  # person, bicycle, motorcycle, traffic light
LANE_PROB_MIN = 0.3   # below this modelV2 is not really claiming a line is there
LANE_AT_M = 10.0      # where along the road to take the line's lateral position
# Road frames between markings passes. Measured at 104 ms on this CPU against the road
# model's 570, which is too much to pay every frame for something that does not change:
# the paint either side of a lane is the same paint for as long as the lane lasts.
LANE_EVERY = 3

_lock = threading.Lock()
_jpeg = b''
_state = {'ok': False, 'reason': 'starting'}
_frame_wanted = -1e9  # last time anyone asked for the picture; see the render decision below


class Handler(BaseHTTPRequestHandler):
  protocol_version = 'HTTP/1.1'

  def _send(self, code, ctype, body):
    self.send_response(code)
    self.send_header('Content-Type', ctype)
    self.send_header('Content-Length', str(len(body)))
    self.send_header('Access-Control-Allow-Origin', '*')
    self.send_header('Cache-Control', 'no-store')
    self.end_headers()
    self.wfile.write(body)

  def do_GET(self):
    path = self.path.split('?')[0]
    with _lock:
      jpeg, state = _jpeg, dict(_state)
    if path == '/frame.jpg':
      global _frame_wanted
      _frame_wanted = time.monotonic()
      if not jpeg:
        return self._send(503, 'text/plain', b'rendering, retry')
      return self._send(200, 'image/jpeg', jpeg)
    if path == '/det.json':
      # a frozen frame looks identical to a live one on screen, so age it explicitly
      if state.get('t_frame'):
        state['age'] = round(time.monotonic() - state.pop('t_frame'), 1)
        if state['age'] > 6:
          state.update(ok=False, reason='stale')
      return self._send(200, 'application/json', json.dumps(state).encode())
    if path == '/health':
      return self._send(200, 'application/json', json.dumps({'ok': state.get('ok', False),
                                                             'reason': state.get('reason')}).encode())
    self._send(404, 'text/plain', b'not found')

  def log_message(self, fmt, *args):
    pass


def serve():
  ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()


def model_lane_x(md):
  """Lateral metres of modelV2's lane lines, at the distance the paint gets measured.

  Both this and the flattened view take right as positive, so the number passes straight
  through. Handing these over is what keeps the markings search on lines that are actually
  there, instead of on whatever else in the frame happens to be brighter than asphalt.
  """
  out = []
  for line, prob in zip(md.laneLines, md.laneLineProbs, strict=False):
    ys = list(line.y)
    if prob < LANE_PROB_MIN or not ys:
      continue
    xs = list(line.x)
    i = min(range(len(xs)), key=lambda k: abs(xs[k] - LANE_AT_M))
    out.append(round(float(ys[i]), 2))
  return out


def camera_frames():
  """Frames from camerad, reconnecting across ignition cycles."""
  vipc = None
  misses = 0
  while True:
    if vipc is None or not vipc.is_connected():
      vipc = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_NARROW_ROAD, conflate=True)
      if not vipc.connect(False):
        with _lock:
          _state.update(ok=False, reason='waiting for camerad')
        vipc = None
        time.sleep(2.0)
        continue
      print(f'yolod: connected {vipc.width}x{vipc.height}', flush=True)

    buf = vipc.recv()
    if buf is None:
      # camerad going away (going offroad) leaves recv() returning immediately, which would
      # otherwise spin a core for nothing
      misses += 1
      if misses > 20 or not vipc.is_connected():
        vipc = None
        with _lock:
          _state.update(ok=False, reason='waiting for camerad')
        time.sleep(1.0)
      else:
        time.sleep(0.05)
      continue
    misses = 0
    yield buf, None


def replay_frames(segment):
  """Frames from a recorded drive, looping. Lets the HUD overlay be checked while parked."""
  import subprocess
  w, h = 672, 380
  size = w * h * 3
  hevc = Path(segment)
  hevc = hevc / 'fcamera.hevc' if hevc.is_dir() else hevc
  while True:
    cmd = ['ffmpeg', '-v', 'error', '-i', str(hevc), '-vf', f'scale={w}:{h}',
           '-sws_flags', 'neighbor', '-pix_fmt', 'rgb24', '-f', 'rawvideo', '-']
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=size * 2)
    try:
      n = 0
      while True:
        raw = p.stdout.read(size)
        if len(raw) < size:
          break
        # the clip is 20 fps and we detect at under 1 Hz, so skip ahead to keep replay
        # roughly in step with wall time instead of crawling
        if n % 25 == 0:
          yield None, np.frombuffer(raw, np.uint8).reshape(h, w, 3)
        n += 1
    finally:
      p.stdout.close()
      p.terminate()


def main(replay=None):
  global _jpeg
  try:
    os.nice(15)  # the driving stack outranks us on every core
  except OSError:
    pass

  threading.Thread(target=serve, daemon=True).start()
  road_sess = yc.make_session(MODEL_ROAD, threads=2)
  band_sess = yc.make_session(MODEL_BAND, threads=2)
  sm = messaging.SubMaster(['carState', 'deviceState', 'modelV2'])
  ThermalStatus = log.DeviceState.ThermalStatus
  pitch, yaw, height = yc.read_calibration()
  calib = (pitch, yaw, height)
  print(f'yolod: calibration pitch {pitch:.5f} yaw {yaw:.5f} height {height:.3f} m', flush=True)

  run = RUNS / datetime.now().strftime('%Y%m%d_%H%M%S')
  run.mkdir(parents=True, exist_ok=True)
  jl = (run / 'det.jsonl').open('a', buffering=1)
  # rows carry seconds since start; the wall clock is written once, here
  (run / 'meta.json').write_text(json.dumps({'start': datetime.now().isoformat(),
                                             'model': MODEL_ROAD, 'model_band': MODEL_BAND}))
  print(f'yolod: models {MODEL_ROAD} + {MODEL_BAND}, logging to {run}, port {PORT}', flush=True)

  frames = 0
  last_save = 0.0
  t_boot = time.monotonic()
  # each model keeps its last answer while the other one has the CPU
  dets_road, dets_band, rgb, lanes = [], [], None, []
  lane_turn = 0
  # Two band frames for every road frame. The cars already have openpilot's own lead and
  # this file's own tracker filling in between passes; a light has neither, and is the only
  # thing here the driver cannot get from anywhere else.
  turn = -1

  for buf, replay_rgb in (replay_frames(replay) if replay else camera_frames()):
    if DISABLE.exists():
      with _lock:
        _state.update(ok=False, reason='disabled by /data/yolo/DISABLE')
      time.sleep(5)
      continue

    sm.update(0)
    turn = (turn + 1) % 3
    band_turn = turn != 0 and buf is not None
    t0 = time.monotonic()
    if band_turn:
      # Shown as soon as it is seen. Asking for the colour twice was sized against a 1.4 s
      # pass; on the road a pass takes 5.4 s, so it meant 11 s, and only a car already
      # stopped at a red waits that long - a green never lasts long enough to qualify.
      # Nothing brakes on this, so a light that is late is worse than one that is wrong.
      dets_band = yc.detect_band(band_sess, yc.nv12_to_band(buf))
    else:
      rgb = replay_rgb if replay_rgb is not None else yc.nv12_to_rgb(buf)
      # the road model knows the light classes too, but sees a third of the reds the band
      # model does, so its lights are dropped rather than argued with
      dets_road = [d for d in yc.detect(road_sess, rgb, c4=True) if d['cls'] != yc.TRAFFIC_LIGHT]
      # Solid or dashed, single or double, yellow or white - whether the line may be crossed
      # is absent from modelV2 entirely. Kept on its own count so a pass the driver cannot
      # see the result of does not slow down the one they can.
      lane_turn = (lane_turn + 1) % LANE_EVERY
      if lane_turn == 0:
        lanes = lane_type.read_markings(rgb, model_lane_x(sm['modelV2']), calib)[1]
    ms = (time.monotonic() - t0) * 1000
    dets = dets_road + dets_band

    v_ego = float(sm['carState'].vEgo)
    counts = {}
    for d in dets:
      counts[d['name']] = counts.get(d['name'], 0) + 1
      if d['cls'] != yc.TRAFFIC_LIGHT:  # lights hang above the road, so the ground plane says nothing
        g = yc.ground_xy(d['box'], pitch, yaw, height, rgb.shape[0])
        if g:
          d['d'], d['lat'] = g
    lights = [d['light'] for d in dets if d.get('light')]

    frames += 1
    now = time.monotonic()

    rec = {'t': round(now - t_boot, 2), 'ms': round(ms, 1), 'v': round(v_ego * 3.6, 1),
           'counts': counts, 'lights': lights, 'dets': dets, 'lanes': lanes}
    jl.write(json.dumps(rec) + '\n')

    # The HUD draws icons from metres, so it never needs the picture. Rendering one costs
    # real time on this CPU, so only do it when something is actually going to look at it.
    gap = SAVE_EVERY_INTERESTING if any(d['cls'] in INTERESTING for d in dets) else SAVE_EVERY
    saving = now - last_save > gap
    if (saving or now - _frame_wanted < 5) and rgb is not None:
      hdr = f"{v_ego * 3.6:.0f}km/h {ms:.0f}ms " + ' '.join(f'{k}:{v}' for k, v in counts.items())
      jpeg = yc.draw(rgb, dets, hdr)
      if saving:
        (run / f'{frames:06d}.jpg').write_bytes(jpeg)
        last_save = now
      with _lock:
        _jpeg = jpeg

    # only what the HUD plots: a few hundred bytes instead of a 40 kB frame
    brief = [{k: v for k, v in d.items() if k in ('cls', 'conf', 'd', 'lat', 'light')}
             for d in dets if 'd' in d or d.get('light')]
    with _lock:
      _state.update(ok=True, reason='running', ms=round(ms, 1), v=round(v_ego * 3.6, 1),
                    counts=counts, lights=lights, dets=brief, lanes=lanes, frames=frames,
                    uptime=round(now - t_boot, 1), run=run.name, t_frame=now,
                    thermal=str(sm['deviceState'].thermalStatus))

    # pace off measured cost rather than a fixed rate, so a slow frame backs us off, and
    # stand down entirely if the device is already in trouble
    thermal = sm['deviceState'].thermalStatus
    duty = DUTY_HOT if thermal == ThermalStatus.overheated else DUTY
    if thermal == ThermalStatus.critical:
      with _lock:
        _state.update(ok=False, reason='device critical, standing down')
      time.sleep(10)
      continue
    idle = max(MIN_PERIOD, (ms / 1000) * (1 / duty - 1)) - (time.monotonic() - now)
    if idle > 0:
      time.sleep(idle)


if __name__ == '__main__':
  # --replay <segment> feeds recorded frames instead of the camera, so the HUD overlay can be
  # checked while the car is parked
  main(sys.argv[2] if len(sys.argv) > 2 and sys.argv[1] == '--replay' else None)
