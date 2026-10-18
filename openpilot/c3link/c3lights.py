#!/usr/bin/env python3
"""On C3: read the traffic lights the way C4's tlwatchd does - same models, same code (yolo/yolo_core.py) -
from C3's own road camera. Replaces the old c3detect (c4light13, a whole frame squeezed into 960x544) the
driver said C3 could drop (10/18: "C3 原本的都可以不要"); display only.

C4 reads a band of rows (1344x352, unscaled) because its lights are ~20 px tall and squeezing the frame is what
cost them (9/4: red recall 0.376 whole-frame vs 0.555 band). C3's narrow camera has 2.32x the focal length, so at
HALF resolution (964x604, f=1324) its lights are ~23 px - about what the band models were trained on. So: half
resolution, the rows from BAND_ABOVE above the horizon (from c3calib, else the image centre) down to BAND_BELOW
below it, centred in a 1344x352 canvas (C3 is 964 wide; the sides are black), through c4band1, the band detector.

Checked 10/18 on 355 frames from C3's own Taiwan drives (F:/realdata), every box looked at by eye: 48 real lights
and 34 not (tail lights, lanterns, LED signs, lane-control signals) at conf 0.25, against the old c4light13's 33
and 9 - half as many lights again, yellow ones included (0 before). c3calib drops boxes under 3 deg above the
horizon (keeps 46 of the 48, half the false ones go) and wants a colour twice in a row. tlwatchd's band classifier
(tlcls_v2) is NOT run: on C3's frames it called "red" 0.86-0.97 on queues of brake lights with no signal in sight
and "go" 0.91-0.98 on empty roads - it reads the traffic, which tlwatchd gets away with only because it asks near
an OSM signal.

Only while C4 is sending carState over the USB link (c3calib's "driving"); parked it idles (it ran at ~80 degC
before). Output /dev/shm/c3det/det.json, boxes in half-resolution coordinates, for c3calib.
"""
import json
import os
import sys
import time

import numpy as np
from msgq.visionipc import VisionIpcClient, VisionStreamType

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yolo_core as yc  # copied next to this file from openpilot/yolo/ on deploy

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DET = os.environ.get('C3_DET', os.path.join(HERE, 'models/c4band1_1344x352.int8v2.onnx'))
THREADS = int(os.environ.get('C3_THREADS', '4'))
ALWAYS = os.environ.get('C3_ALWAYS', '0') == '1'      # testing: run whether or not the car is driving
OUT_DIR = '/dev/shm/c3det'
CALIB = '/dev/shm/c3calib.json'
DISP_W, DISP_H = 964, 604          # half resolution of the 1928x1208 ar0231
DET_CONF = 0.25                     # detect_band's own default; the 10/18 check was done at this
BAND_ABOVE = 322                    # rows above the horizon (~13.7 deg at f=1324)
BAND_BELOW = yc.BAND_H - BAND_ABOVE
PAD_X = (yc.BAND_W - DISP_W) // 2
ZH = {'light_red': '紅燈', 'light_green': '綠燈', 'light_yellow': '黃燈', 'light_off': '熄燈',
      'light_pedestrian': '行人號誌', 'light_green_arrow': '綠箭頭'}


def band_half(buf, y0, y1):
  """Half-resolution RGB of half-res rows y0..y1 straight off the NV12 (full-res rows 2*y0..2*y1)."""
  data = np.frombuffer(buf.data, dtype=np.uint8)
  w, h, stride, uv_off = buf.width, buf.height, buf.stride, buf.uv_offset
  r0, r1 = 2 * y0, min(h, 2 * y1)
  y = data[r0 * stride:r1 * stride].reshape(r1 - r0, stride)[::2, :w:2].astype(np.int32)
  uv = data[uv_off + (r0 // 2) * stride:uv_off + (r1 // 2) * stride].reshape(-1, stride)[:, :w]
  u = uv[:, 0::2].astype(np.int32)[:y.shape[0], :y.shape[1]] - 128
  v = uv[:, 1::2].astype(np.int32)[:y.shape[0], :y.shape[1]] - 128
  rgb = np.empty(y.shape + (3,), dtype=np.int32)
  rgb[..., 0] = y + ((91881 * v) >> 16)
  rgb[..., 1] = y - ((22554 * u + 46802 * v) >> 16)
  rgb[..., 2] = y + ((116130 * u) >> 16)
  return np.clip(rgb, 0, 255).astype(np.uint8)


def calib_state():
  """(driving, horizon row at half resolution or None) from c3calib."""
  try:
    with open(CALIB) as f:
      c = json.load(f)
  except (OSError, ValueError):
    return False, None
  if time.monotonic() - float(c.get('t', 0)) > 15:
    return False, None
  return bool(c.get('driving')), c.get('horizon_row')


def cpu_temp():
  try:
    with open('/sys/class/thermal/thermal_zone0/temp') as f:
      return int(f.read()) / 1000.
  except (OSError, ValueError):
    return 0.


def write(state):
  tmp = os.path.join(OUT_DIR, 'det.json.tmp')
  with open(tmp, 'w') as f:
    json.dump(state, f, ensure_ascii=False)
  os.replace(tmp, os.path.join(OUT_DIR, 'det.json'))


def main():
  os.makedirs(OUT_DIR, exist_ok=True)
  det = yc.make_session(MODEL_DET, threads=THREADS)
  label = os.path.basename(MODEL_DET)
  client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, False)
  state = {'model': label, 'dets': [], 'n': 0, 'err': 'starting', 'idle': True, 'w': DISP_W, 'h': DISP_H}
  write(state)
  last_check, driving, horizon = 0., False, None
  fps, t_last = 0., time.monotonic()
  while True:
    now = time.monotonic()
    if now - last_check > 1.:
      last_check = now
      driving, horizon = calib_state()
    if not (driving or ALWAYS):
      state.update({'dets': [], 'n': 0, 'err': '', 'idle': True, 'temp': round(cpu_temp(), 1),
                    'ts': round(now, 2)})
      write(state)
      time.sleep(1.)
      continue
    if not client.is_connected() and not client.connect(False):
      state.update({'err': 'camerad 未連線', 'dets': [], 'idle': False})
      write(state)
      time.sleep(1.)
      continue
    buf = client.recv()
    if buf is None:
      time.sleep(0.02)
      continue

    hz = int(horizon) if horizon is not None else DISP_H // 2
    y0 = max(0, min(DISP_H - yc.BAND_H, hz - BAND_ABOVE))
    y1 = y0 + yc.BAND_H
    band = band_half(buf, y0, y1)
    canvas = np.zeros((yc.BAND_H, yc.BAND_W, 3), dtype=np.uint8)
    canvas[:band.shape[0], PAD_X:PAD_X + band.shape[1]] = band

    t0 = time.perf_counter()
    lamps = yc.detect_band(det, canvas, conf_thres=DET_CONF)
    t1 = time.perf_counter()

    dets = []
    for d in lamps:
      bx1, by1, bx2, by2 = d['box']
      if bx2 < PAD_X or bx1 > PAD_X + DISP_W:
        continue
      dets.append({'x': round(bx1 - PAD_X, 1), 'y': round(by1 + y0, 1), 'w': round(bx2 - bx1, 1), 'h': round(by2 - by1, 1),
                   'cls': d['name'], 'zh': ZH.get(d['name'], d['name']), 'conf': d['conf']})
    t_end = time.monotonic()
    dt = t_end - t_last
    t_last = t_end
    if dt > 0:
      fps = 1. / dt if fps == 0 else fps * 0.8 + 0.2 / dt
    state.update({'dets': dets, 'n': len(dets), 'det_ms': round((t1 - t0) * 1000), 'fps': round(fps, 1),
                  'band_y0': y0, 'temp': round(cpu_temp(), 1), 'err': '', 'idle': False, 'ts': round(now, 2)})
    write(state)


if __name__ == '__main__':
  main()
