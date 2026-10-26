#!/usr/bin/env python3
"""On C3: find the traffic lights in C3's own road camera and tell their colour - display only.

10/25 rewrite (the 10/18 version ran C4's band detector c4band1 at half resolution: on 1180 judged C3 frames it found
a light in 16% of the frames that had one). Now two stages, both trained / checked on C3's own frames:

  1. finder  ft1n - COCO yolo26n fine-tuned on 4258 C3 frames whose light candidates haiku judged (10/24-25,
             /opt/yolo-train/c3train on .70). Full resolution, a 704x1920 band: on the Custin's dashboard the lights
             sit around rows 92-924 (median 746) with the horizon near row 912, so the band runs from BAND_ABOVE
             rows above the horizon (c3calib's, full-res) to BAND_BELOW below. Custin night/rain truth set
             (539 frames): rows 224-928 hold 97% of the judged lights; the finder found a light in 88% of the frames
             that had one. C3's GPU would do it in 0.66 s but tinygrad recompiles for ~8 min on every start, so
             onnxruntime on the CPU (~0.8 s).
  2. colour  lightcls - the 64x64 crop classifier of 9/18 (C4 crops), on each found box padded 25%. Against haiku on
             the Custin set: red 94%, green 82% (+11% as green arrow); looked at by eye most disagreements were
             haiku's (over-exposed red called yellow, countdown digits over a green called yellow).
             off / pedestrian are not passed on (the HUD shows one light: it should be ours).

10/26 (the driver: "C3 cannot even read the lights fast enough to do distance - is the method too dumb?" - it was):
  - the band is 1248 wide, centred on straight ahead (c3calib's ahead_col): only lights within 12 deg are used (C4's
    tlwatchd / HUD), the outer 35% of the 1920 band was scanned for nothing. Custin truth set, onnxruntime: finder
    855 -> 539 ms, front light shown 218 / 218 frames, colour right 187 -> 186, red read as go 4 -> 3, false 43 -> 40.
  - the finder runs on the GPU from a program compiled once (compile_lights_gpu.py, ~6 min, the way openpilot's
    compile3.py ships the driving model) instead of recompiling on every start: loads in ~4 s, 461 ms a frame, ~0.1
    CPU cores instead of 4, identical results. The model is cut before its TopK tail (the class index there is int64,
    which tinygrad on the Adreno got wrong - every class but the first row came back as 2.8e14 - and a sort is slow on
    the GPU): it returns boxes + 80 class scores per anchor and the light boxes are those with class-9 score >= DET_CONF
    (this head is NMS-free; same 46 boxes on 30 frames as the end2end export). No compiled program -> onnxruntime.

10/26 later: stopped with lights in view, their colour is read 10 times a second off just their rows (TRACK_*), the
full finder only once a second - the car does not move, so neither do the lights, and a green is known within ~0.1 s
instead of one finder cycle. Moving, every cycle is the full finder as before.

Output /dev/shm/c3det/det.json as before - boxes in half-resolution coordinates, cls light_red / light_green /
light_green_arrow (no yellow, see PASS_ON), zh, conf (the colour's probability), det (the finder's score) - for c3calib, which
turns them into car-frame directions for C4's HUD. Only while C4 is driving (c3calib's "driving"), else idle.

  c3lights.py                       live (VisionIPC road camera)
  c3lights.py --frames DIR OUT.json offline on JPEG frames (full 1928x1208), same code: ROW0 env or the default band
"""
import json
import os
import sys
import time

import numpy as np
import onnxruntime as ort
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DET = os.environ.get('C3_DET', os.path.join(HERE, 'models/c3light_ft1n_704x1248_raw.onnx'))
MODEL_GPU = os.environ.get('C3_GPU', os.path.join(HERE, 'models/c3light_ft1n_704x1248_raw_qcom.pkl'))
MODEL_CLS = os.environ.get('C3_CLS', os.path.join(HERE, 'models/lightcls_64.onnx'))
THREADS = int(os.environ.get('C3_THREADS', '4'))
ALWAYS = os.environ.get('C3_ALWAYS', '0') == '1'      # testing: run whether or not the car is driving
OUT_DIR = '/dev/shm/c3det'
CALIB = '/dev/shm/c3calib.json'
FULL_W, FULL_H = 1928, 1208        # ar0231
DISP_W, DISP_H = 964, 604          # half resolution: det.json's coordinates (c3calib's FOCAL / CX / CY)
BAND_H, BAND_W = 704, 1248         # the finder's input, no resize; columns centred on straight ahead (see the top)
LIGHT_CLASS = 9                    # COCO 'traffic light'
BAND_ABOVE = 690                   # rows above the horizon (full res; Custin: horizon ~912 -> rows 222-926)
ROW0_DEFAULT = 224                 # no calibration yet
DET_CONF = 0.15                    # Custin set, rows 224-928: 0.13 -> 84% of light frames found / 80 frames with only
                                   # false boxes, 0.22 -> 74% / 59; c3calib also wants a light twice in a row
COLOR_MIN = 0.5
PAD = 0.25                         # lightcls was trained on boxes padded like this (build_cls_dataset.py)
CLS_NAMES = ['green', 'green_arrow', 'off', 'pedestrian', 'red', 'yellow']   # lightcls_64.onnx's output order
# No yellow (10/25): of the 135 boxes passed on as yellow on the Custin set nearly all were yellow-black striped posts,
# chevron curve signs, yellow signs and countdown digits (by eye), and the 10/25 drive's three false lights included two
# roadside yellow markers. A yellow lasts 3 s, C3 reads ~1/s and wants a light twice - it would barely show anyway.
# Without it: frames with only false lights 51 -> 43 of 269, colour right 85 -> 86%, a light shown 85 -> 80%.
PASS_ON = {'red': 'light_red', 'green': 'light_green', 'green_arrow': 'light_green_arrow'}
TRACK_V = 0.3                      # m/s (C4's vEgo through c3calib): stopped
TRACK_PERIOD = 0.1                 # s: stopped with lights found, their colour this often ...
FIND_EVERY_S = 1.0                 # ... and the full finder this often (a light that appears / goes / was missed)
TRACK_PAD = 0.6                    # rows above / below a box read for its colour, in box heights (lightcls pads 25%)
ZH = {'light_red': '紅燈', 'light_green': '綠燈', 'light_yellow': '黃燈', 'light_green_arrow': '綠箭頭'}


def session(path):
  o = ort.SessionOptions()
  o.intra_op_num_threads = THREADS
  o.inter_op_num_threads = 1
  o.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
  return ort.InferenceSession(path, o, providers=['CPUExecutionProvider'])


def band_row0(horizon_half):
  """First full-res row of the band from c3calib's horizon (half-res row), else the Custin default."""
  if horizon_half is None:
    return ROW0_DEFAULT
  return int(max(0, min(FULL_H - BAND_H, 2 * float(horizon_half) - BAND_ABOVE)))


def band_col0(ahead_half):
  """First full-res column of the band, centred on straight ahead (c3calib's ahead_col, half res), else the middle."""
  centre = FULL_W / 2 if ahead_half is None else 2 * float(ahead_half)
  return int(max(0, min(FULL_W - BAND_W, round(centre - BAND_W / 2))))


class GpuFinder:
  """The finder compiled for the Adreno once (compile_lights_gpu.py) and loaded here."""
  def __init__(self, pkl):
    import pickle
    os.environ.setdefault('QCOM', '1')
    os.environ.setdefault('IMAGE', '2')
    os.environ.setdefault('NOLOCALS', '1')
    os.environ.setdefault('JIT_BATCH_SIZE', '0')
    sys.path.insert(0, '/data/openpilot/tinygrad_repo')
    from tinygrad import Tensor
    with open(pkl, 'rb') as f:
      self.jit = pickle.load(f)
    self.name = self.jit.captured.expected_names[0]
    self.buf = np.zeros((1, 3, BAND_H, BAND_W), dtype=np.float32)
    self.x = Tensor(self.buf, device='NPY').realize()        # wraps buf: writing into buf is the next run's input
    self.jit(**{self.name: self.x}).numpy()

  def __call__(self, x):
    self.buf[:] = x
    return self.jit(**{self.name: self.x}).numpy()


def finder():
  """(run(x) -> raw output, label): the GPU program if there is one, else onnxruntime on the CPU."""
  if os.path.exists(MODEL_GPU):
    try:
      return GpuFinder(MODEL_GPU), 'GPU ' + os.path.basename(MODEL_GPU)
    except Exception as e:  # a broken compile must not stop the lights
      print('c3lights: GPU finder failed, using the CPU:', repr(e)[:200], flush=True)
  sess = session(MODEL_DET)
  name = sess.get_inputs()[0].name
  return (lambda x: sess.run(None, {name: x})[0]), 'CPU ' + os.path.basename(MODEL_DET)


def find(det, band):
  """Finder on a (704, 1248, 3) uint8 RGB band -> [(x1, y1, x2, y2, score)] in band pixels, traffic lights only."""
  out = det(band.transpose(2, 0, 1)[None].astype(np.float32) / 255.)[0]   # (anchors, 84): x1 y1 x2 y2 + 80 scores
  out = out[out[:, 4 + LIGHT_CLASS] >= DET_CONF][:, [0, 1, 2, 3, 4 + LIGHT_CLASS]]
  return [tuple(float(v) for v in r) for r in out[np.argsort(-out[:, 4])]]


def cls_input(crop):
  """lightcls preprocessing as ultralytics' classify_transforms(64): shorter side to 64 (PIL bilinear), centre crop."""
  w, h = crop.size
  if w <= h:
    nw, nh = 64, int(64 * h / w)
  else:
    nw, nh = int(64 * w / h), 64
  im = crop.resize((nw, nh), Image.BILINEAR)
  left, top = int(round((nw - 64) / 2.)), int(round((nh - 64) / 2.))
  im = im.crop((left, top, left + 64, top + 64))
  return np.asarray(im, dtype=np.float32).transpose(2, 0, 1) / 255.


def colours(cls, frame, boxes):
  """lightcls on each box (full-frame coordinates) padded 25% -> list of probability vectors."""
  if not boxes:
    return []
  H, W = frame.shape[:2]
  img = Image.fromarray(frame)
  xs = []
  for x1, y1, x2, y2 in boxes:
    bw, bh = x2 - x1, y2 - y1
    xs.append(cls_input(img.crop((max(0., x1 - bw * PAD), max(0., y1 - bh * PAD), min(W, x2 + bw * PAD), min(H, y2 + bh * PAD)))))
  name = cls.get_inputs()[0].name
  return [cls.run(None, {name: x[None]})[0][0] for x in xs]   # the export has a fixed batch of 1


def dets_from(boxes, scores, probs):
  """Full-res boxes, finder scores and colour probabilities -> det.json entries (half-res), dropped count."""
  dets, dropped = [], 0
  for (x1, y1, x2, y2), score, p in zip(boxes, scores, probs, strict=True):
    k = int(np.argmax(p))
    name, pc = CLS_NAMES[k], float(p[k])
    if name not in PASS_ON or pc < COLOR_MIN:
      dropped += 1
      continue
    c = PASS_ON[name]
    dets.append({'x': round(x1 / 2, 1), 'y': round(y1 / 2, 1), 'w': round((x2 - x1) / 2, 1), 'h': round((y2 - y1) / 2, 1),
                 'cls': c, 'zh': ZH[c], 'conf': round(pc, 3), 'det': round(score, 3)})
  return dets, dropped


def read(det, cls, frame, row0, col0):
  """Full-res RGB frame -> det.json boxes (half-res) and counts."""
  band = np.ascontiguousarray(frame[row0:row0 + BAND_H, col0:col0 + BAND_W])
  t0 = time.perf_counter()
  found = find(det, band)
  t1 = time.perf_counter()
  boxes = [(x1 + col0, y1 + row0, x2 + col0, y2 + row0) for x1, y1, x2, y2, _ in found]
  probs = colours(cls, frame, boxes)
  t2 = time.perf_counter()
  dets, dropped = dets_from(boxes, [f[4] for f in found], probs)
  return dets, {'found': len(found), 'dropped': dropped, 'det_ms': round((t1 - t0) * 1000), 'cls_ms': round((t2 - t1) * 1000)}


class Reader:
  """One reading a call: the full finder + colour, or - stopped, lights found within FIND_EVERY_S - the colour of
  those lights only, off their own rows. rows(r0, r1) -> (full-width RGB of rows r0.., first row actually given)."""
  def __init__(self, det, cls):
    self.det, self.cls = det, cls
    self.boxes, self.scores, self.t_find = [], [], -1e9
    self.n_shown = 0
    self.went_dark = False     # all tracked lights stopped showing a colour (the lit lamp moved, e.g. to a lamp of the
                               # head the finder did not box): find again at once instead of after FIND_EVERY_S

  def tracking(self, now, stopped):
    return stopped and bool(self.boxes) and now - self.t_find < FIND_EVERY_S and not self.went_dark

  def step(self, rows, row0, col0, now, stopped):
    t0 = time.perf_counter()
    if self.tracking(now, stopped):
      top = int(max(0, min(y1 - (y2 - y1) * TRACK_PAD for _, y1, _, y2 in self.boxes)))
      bot = int(min(FULL_H, max(y2 + (y2 - y1) * TRACK_PAD for _, y1, _, y2 in self.boxes) + 2))
      rgb, r0 = rows(top, bot)
      probs = colours(self.cls, rgb, [(x1, y1 - r0, x2, y2 - r0) for x1, y1, x2, y2 in self.boxes])
      dets, dropped = dets_from(self.boxes, self.scores, probs)
      self.went_dark = not dets and self.n_shown > 0     # every tracked light dark: whatever is lit now is elsewhere
      return dets, {'mode': 'track', 'found': len(self.boxes), 'dropped': dropped, 'det_ms': 0,
                    'cls_ms': round((time.perf_counter() - t0) * 1000)}
    rgb, r0 = rows(row0, row0 + BAND_H)
    band = np.ascontiguousarray(rgb[row0 - r0:row0 - r0 + BAND_H, col0:col0 + BAND_W])
    found = find(self.det, band)
    t1 = time.perf_counter()
    self.boxes = [(x1 + col0, y1 + row0, x2 + col0, y2 + row0) for x1, y1, x2, y2, _ in found]
    self.scores, self.t_find = [f[4] for f in found], now
    probs = colours(self.cls, rgb, [(x1, y1 - r0, x2, y2 - r0) for x1, y1, x2, y2 in self.boxes])
    dets, dropped = dets_from(self.boxes, self.scores, probs)
    self.n_shown, self.went_dark = len(dets), False
    return dets, {'mode': 'find', 'found': len(found), 'dropped': dropped, 'det_ms': round((t1 - t0) * 1000),
                  'cls_ms': round((time.perf_counter() - t1) * 1000)}


def nv12_rgb(buf, r0, r1):
  """Full-res RGB of rows r0..r1 straight off the NV12 buffer."""
  data = np.frombuffer(buf.data, dtype=np.uint8)
  w, stride, uv_off = buf.width, buf.stride, buf.uv_offset
  r0 -= r0 % 2
  y = data[r0 * stride:r1 * stride].reshape(r1 - r0, stride)[:, :w].astype(np.int32)
  uv = data[uv_off + (r0 // 2) * stride:uv_off + ((r1 + 1) // 2) * stride].reshape(-1, stride)[:, :w]
  u = np.repeat(np.repeat(uv[:, 0::2].astype(np.int32) - 128, 2, axis=0), 2, axis=1)[:y.shape[0], :w]
  v = np.repeat(np.repeat(uv[:, 1::2].astype(np.int32) - 128, 2, axis=0), 2, axis=1)[:y.shape[0], :w]
  rgb = np.empty(y.shape + (3,), dtype=np.int32)
  rgb[..., 0] = y + ((91881 * v) >> 16)
  rgb[..., 1] = y - ((22554 * u + 46802 * v) >> 16)
  rgb[..., 2] = y + ((116130 * u) >> 16)
  return np.clip(rgb, 0, 255).astype(np.uint8), r0


def calib_state():
  """(driving, horizon row, straight-ahead column - both half resolution or None, C4's vEgo or None) from c3calib."""
  try:
    with open(CALIB) as f:
      c = json.load(f)
  except (OSError, ValueError):
    return False, None, None, None
  if time.monotonic() - float(c.get('t', 0)) > 15:
    return False, None, None, None
  return bool(c.get('driving')), c.get('horizon_row'), c.get('ahead_col'), c.get('v')


def cpu_temp():
  """Hottest real sensor (the lmh-dcvs zones read a fixed 75000: a limit, not a temperature - 10/25)."""
  best = 0.
  for z in range(40):
    try:
      with open(f'/sys/class/thermal/thermal_zone{z}/type') as f:
        if f.read().startswith('lmh'):
          continue
      with open(f'/sys/class/thermal/thermal_zone{z}/temp') as f:
        best = max(best, int(f.read()) / 1000.)
    except (OSError, ValueError):
      continue
  return best


def write(state):
  tmp = os.path.join(OUT_DIR, 'det.json.tmp')
  with open(tmp, 'w') as f:
    json.dump(state, f, ensure_ascii=False)
  os.replace(tmp, os.path.join(OUT_DIR, 'det.json'))


def offline(fdir, out_path):
  import glob
  (det, label), cls = finder(), session(MODEL_CLS)
  print('finder', label, flush=True)
  row0 = int(os.environ.get('ROW0', ROW0_DEFAULT))
  col0 = int(os.environ.get('COL0', band_col0(None)))
  res = {}
  for f in sorted(glob.glob(os.path.join(fdir, '*.jpg'))):
    frame = np.asarray(Image.open(f).convert('RGB'))
    dets, st = read(det, cls, frame, row0, col0)
    res[os.path.basename(f)] = {'dets': dets, **st}
  json.dump(res, open(out_path, 'w'), ensure_ascii=False)
  ms = [v['det_ms'] + v['cls_ms'] for v in res.values()]
  print('frames', len(res), 'lights passed on', sum(len(v['dets']) for v in res.values()), 'median ms', int(np.median(ms)))


def main():
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  os.makedirs(OUT_DIR, exist_ok=True)
  (det, label), cls = finder(), session(MODEL_CLS)
  label += ' + ' + os.path.basename(MODEL_CLS)
  client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, False)
  state = {'model': label, 'dets': [], 'n': 0, 'err': 'starting', 'idle': True, 'w': DISP_W, 'h': DISP_H}
  write(state)
  last_check, driving, horizon, ahead, v = 0., False, None, None, None
  fps, t_last = 0., time.monotonic()
  reader, last_out = Reader(det, cls), 0.
  while True:
    now = time.monotonic()
    if now - last_check > 1.:
      last_check = now
      driving, horizon, ahead, v = calib_state()
    if not (driving or ALWAYS):
      state.update({'dets': [], 'n': 0, 'err': '', 'idle': True, 'temp': round(cpu_temp(), 1), 'ts': round(now, 2)})
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
    stopped = v is not None and v < TRACK_V
    if reader.tracking(now, stopped) and now - last_out < TRACK_PERIOD:
      continue
    last_out = now
    row0, col0 = band_row0(horizon), band_col0(ahead)
    dets, st = reader.step(lambda a, b, buf=buf: nv12_rgb(buf, a, b), row0, col0, now, stopped)
    t_end = time.monotonic()
    dt = t_end - t_last
    t_last = t_end
    if dt > 0:
      fps = 1. / dt if fps == 0 else fps * 0.8 + 0.2 / dt
    state.update({'dets': dets, 'n': len(dets), **st, 'fps': round(fps, 2), 'band_y0': row0 // 2, 'band_x0': col0 // 2,
                  'temp': round(cpu_temp(), 1), 'err': '', 'idle': False, 'ts': round(now, 2)})
    write(state)


if __name__ == '__main__':
  if len(sys.argv) > 2 and sys.argv[1] == '--frames':
    offline(sys.argv[2], sys.argv[3])
  else:
    main()
