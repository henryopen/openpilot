#!/usr/bin/env python3
"""On C3: people, bicycles, motorcycles, cars, trucks and buses in C3's road camera, with a distance - display only.

10/26, the driver: show what the radar does not (the Custin's radar gives one target): pedestrians, scooters, bikes,
and trucks and buses too. COCO yolo26n (no fine-tuning) on the whole frame at a third of the resolution (640x416,
onnxruntime on the CPU - the GPU is c3lights'), cut before its TopK tail like c3lights' finder (NMS-free head).

Distance from the object's real size, not from where its box meets the road: on the 10/25 drives (52 radar leads at
7-78 m, the lead box picked by eye without the radar distance in view) a car's width (1.8 m) gave the radar distance
to a median 7% (2% / 8% / 7% at 0-30 / 30-50 / 50-90 m, 2 of 52 off by > 25%); the ground plane through C3's
calibration put the camera 0.36-0.6 m above the road - a pixel or two of pitch or road grade at 50 m is the whole
answer there. People / riders by height. Boxes cut by the frame edge get no distance (their size is not theirs).

Output /dev/shm/c3obj/objs.json: boxes at half resolution (as c3det), cls, conf, d (m) - c3calib turns them into
car-frame bearings and sends them to C4 with the lights. Runs only while C4 is driving (c3calib's "driving").
  c3objects.py                       live (VisionIPC road camera)
  c3objects.py --frames DIR OUT.json offline on JPEG frames (full 1928x1208)
"""
import json
import os
import sys
import time

import numpy as np
import onnxruntime as ort

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.environ.get('C3_OBJ', os.path.join(HERE, 'models/c3obj_yolo26n_416x640_raw.onnx'))
THREADS = int(os.environ.get('C3_OBJ_THREADS', '3'))
ALWAYS = os.environ.get('C3_ALWAYS', '0') == '1'
OUT_DIR = '/dev/shm/c3obj'
CALIB = '/dev/shm/c3calib.json'
FULL_W, FULL_H = 1928, 1208
FOCAL = 2648.                      # full resolution
IN_W, IN_H = 640, 416              # model input: rows 1..1203, cols 4..1923 averaged 3x3 -> 401x640, padded to 416
ROW0, COL0, K = 1, 4, 3
NEW_H = (FULL_H - 2 * ROW0) // K   # 401
PAD_T = (IN_H - NEW_H) // 2
MIN_PERIOD = 0.4                   # s: at most 2.5 a second. 10/26 at 4/s with c3lights on the GPU: 2.4 cores, C3 at 72 C
# COCO class -> (name, size used for the distance, 'w' = box width / 'h' = box height, metres, min confidence)
CLASSES = {0: ('person', 'h', 1.65, 0.40), 1: ('bicycle', 'h', 1.60, 0.40), 3: ('motorcycle', 'h', 1.60, 0.40),
           2: ('car', 'w', 1.80, 0.35), 5: ('bus', 'w', 2.55, 0.35), 7: ('truck', 'w', 2.00, 0.35)}
RIDERS = (1, 3)                    # bicycle / motorcycle: measured with their rider (the union of the two boxes)
EDGE = 4                           # px (full res): a box this close to the frame edge is cut, no distance


def session():
  o = ort.SessionOptions()
  o.intra_op_num_threads = THREADS
  o.inter_op_num_threads = 1
  o.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
  return ort.InferenceSession(MODEL, o, providers=['CPUExecutionProvider'])


def model_input(rgb_small):
  """(401, 640, 3) uint8 -> (1, 3, 416, 640) float32, padded like ultralytics' letterbox (114)."""
  x = np.full((IN_H, IN_W, 3), 114, dtype=np.uint8)
  x[PAD_T:PAD_T + NEW_H] = rgb_small
  return x.transpose(2, 0, 1)[None].astype(np.float32) / 255.


def small_from_rgb(frame):
  """Full-res RGB -> (401, 640, 3) by 3x3 averaging (the same as small_from_nv12 does live)."""
  f = frame[ROW0:ROW0 + NEW_H * K, COL0:COL0 + IN_W * K].astype(np.float32)
  return f.reshape(NEW_H, K, IN_W, K, 3).mean(axis=(1, 3)).astype(np.uint8)


class Nv12Small:
  """NV12 camera buffer -> (401, 640, 3) RGB: luma averaged 3x3, chroma sampled at each block's centre."""
  def __init__(self):
    self.uv_r = (ROW0 + K * np.arange(NEW_H) + 1) // 2
    self.uv_c = ((COL0 + K * np.arange(IN_W) + 1) // 2) * 2

  def __call__(self, buf):
    data = np.frombuffer(buf.data, dtype=np.uint8)
    w, stride = buf.width, buf.stride
    y = data[:buf.height * stride].reshape(buf.height, stride)[ROW0:ROW0 + NEW_H * K, COL0:COL0 + IN_W * K]
    y = y.reshape(NEW_H, K, IN_W, K).mean(axis=(1, 3))
    uv = data[buf.uv_offset:buf.uv_offset + (buf.height // 2) * stride].reshape(buf.height // 2, stride)[:, :w]
    uvs = uv[self.uv_r[:, None], self.uv_c[None, :] + np.array([0, 1])[:, None, None]].astype(np.float32) - 128.
    u, v = uvs[0], uvs[1]
    rgb = np.stack([y + 1.402 * v, y - 0.344136 * u - 0.714136 * v, y + 1.772 * u], axis=-1)
    return np.clip(rgb, 0, 255).astype(np.uint8)


def detect(sess, small):
  """-> [(cls, conf, x1, y1, x2, y2)] full-res pixels, our six classes only."""
  out = sess.run(None, {sess.get_inputs()[0].name: model_input(small)})[0][0]    # (anchors, 84)
  ks = np.array(sorted(CLASSES))
  sc = out[:, 4 + ks]
  best, which = sc.max(1), ks[sc.argmax(1)]
  thr = np.array([CLASSES[int(k)][3] for k in which])
  keep = best >= thr
  res = []
  for r, s, c in zip(out[keep], best[keep], which[keep], strict=True):
    x1, y1 = COL0 + K * r[0], ROW0 + K * (r[1] - PAD_T)
    x2, y2 = COL0 + K * r[2], ROW0 + K * (r[3] - PAD_T)
    res.append((int(c), float(s), float(x1), float(y1), float(x2), float(y2)))
  return res


def with_distance(dets):
  """Each detection with its distance from its real size; riders measured together with their bicycle / scooter."""
  people = [d for d in dets if d[0] == 0]
  used, objs = set(), []
  for c, s, x1, y1, x2, y2 in dets:
    if c == 0:
      continue
    if c in RIDERS:              # the person on it: overlapping and above it
      for i, p in enumerate(people):
        if i not in used and p[2] < x2 and p[4] > x1 and p[3] < y1 and p[5] > y1:
          used.add(i)
          x1, y1, x2, y2 = min(x1, p[2]), min(y1, p[3]), max(x2, p[4]), max(y2, p[5])
          break
    objs.append((c, s, x1, y1, x2, y2))
  objs += [p for i, p in enumerate(people) if i not in used]
  out = []
  for c, s, x1, y1, x2, y2 in objs:
    name, dim, size, _ = CLASSES[c]
    cut = x1 < EDGE or x2 > FULL_W - EDGE or y2 > FULL_H - EDGE or (dim == 'h' and y1 < EDGE)
    px = (x2 - x1) if dim == 'w' else (y2 - y1)
    d = None if cut or px < 4 else round(size * FOCAL / px, 1)
    out.append({'cls': name, 'conf': round(s, 2), 'd': d, 'x': round(x1 / 2, 1), 'y': round(y1 / 2, 1),
                'w': round((x2 - x1) / 2, 1), 'h': round((y2 - y1) / 2, 1)})
  return out


def driving():
  try:
    with open(CALIB) as f:
      c = json.load(f)
    return time.monotonic() - float(c.get('t', 0)) < 15 and bool(c.get('driving'))
  except (OSError, ValueError):
    return False


def write(state):
  tmp = os.path.join(OUT_DIR, 'objs.json.tmp')
  with open(tmp, 'w') as f:
    json.dump(state, f, ensure_ascii=False)
  os.replace(tmp, os.path.join(OUT_DIR, 'objs.json'))


def offline(fdir, out_path):
  import glob
  from PIL import Image
  sess, res, ms = session(), {}, []
  for f in sorted(glob.glob(os.path.join(fdir, '*.jpg'))):
    small = small_from_rgb(np.asarray(Image.open(f).convert('RGB')))
    t0 = time.perf_counter()
    objs = with_distance(detect(sess, small))
    ms.append((time.perf_counter() - t0) * 1000)
    res[os.path.basename(f)] = objs
  json.dump(res, open(out_path, 'w'), ensure_ascii=False)
  print('frames', len(res), 'objects', sum(len(v) for v in res.values()), 'median ms', int(np.median(ms)))


def main():
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  os.makedirs(OUT_DIR, exist_ok=True)
  sess, conv = session(), Nv12Small()
  client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, False)
  state = {'model': os.path.basename(MODEL), 'objs': [], 'idle': True, 'err': 'starting'}
  write(state)
  last_check, drive, t_prev, fps = 0., False, 0., 0.
  while True:
    now = time.monotonic()
    if now - last_check > 1.:
      last_check, drive = now, driving()
    if not (drive or ALWAYS):
      state.update({'objs': [], 'idle': True, 'err': '', 'ts': round(now, 2)})
      write(state)
      time.sleep(1.)
      continue
    if not client.is_connected() and not client.connect(False):
      state.update({'objs': [], 'idle': False, 'err': 'camerad 未連線'})
      write(state)
      time.sleep(1.)
      continue
    buf = client.recv()
    if buf is None:
      time.sleep(0.02)
      continue
    if now - t_prev < MIN_PERIOD:
      continue
    t0 = time.perf_counter()
    small = conv(buf)
    t1 = time.perf_counter()
    objs = with_distance(detect(sess, small))
    t2 = time.perf_counter()
    if t_prev:
      fps = 1. / (now - t_prev) if fps == 0 else fps * 0.8 + 0.2 / (now - t_prev)
    t_prev = now
    state.update({'objs': objs, 'n': len(objs), 'conv_ms': round((t1 - t0) * 1000), 'det_ms': round((t2 - t1) * 1000),
                  'fps': round(fps, 2), 'idle': False, 'err': '', 'ts': round(now, 2)})
    write(state)


if __name__ == '__main__':
  if len(sys.argv) > 2 and sys.argv[1] == '--frames':
    offline(sys.argv[2], sys.argv[3])
  else:
    main()
