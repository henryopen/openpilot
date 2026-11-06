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

11/04 (TASK3, the driver 11/03: "沒法用地圖輔助就好提高辨燈的準確度嗎" -> "直接做第一項"): moving, the GPU finder is
started on the cycle's frame and, while it runs (~0.45 s, the CPU idle meanwhile - its jit call returns at once and
only .numpy() waits), the heads the follower is following are looked for again on NEWER frames: the same finder over a
small crop (LOCAL_S) around where each should be now, on the CPU (models/c3light_ft1n_192x192_raw.onnx, the deployed
weights exported at 192x192 and cut before TopK like the band's) - local passes, fed to the follower as extra
observations at the next cycle (c3track.Follower.step_local; their poses are in by then). And the camera is read
conflated: the client used to queue every frame, so a ~0.6 s moving cycle fell further behind each time - the frame's
timestamp seconds old while its buffer (20 of them) already held a frame up to 1 s newer (C3 test 11/03: after 10 s
the packet was 8.5 s behind); the follower's pose time and its pixels disagreed.

Output /dev/shm/c3det/det.json as before - boxes in half-resolution coordinates, cls light_red / light_green /
light_green_arrow / light_yellow (a yellow only where a light turned yellow, see YELLOW_*), zh, conf (the colour's
probability), det (the finder's score) - for c3calib, which turns them into car-frame directions for C4's HUD.
Only while C4 is driving (c3calib's "driving"), else idle.

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

import c3track
from c3pin import StopPin

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DET = os.environ.get('C3_DET', os.path.join(HERE, 'models/c3light_ft1n_704x1248_raw.onnx'))
MODEL_GPU = os.environ.get('C3_GPU', os.path.join(HERE, 'models/c3light_ft1n_704x1248_raw_qcom.pkl'))
MODEL_CLS = os.environ.get('C3_CLS', os.path.join(HERE, 'models/lightcls_64.onnx'))
# Per head (2026-11-01): which lamps of the head are lit - and whether it is a vehicle signal at all - from the arrow
# model trained on .70 (c3arrow: C4 and C3 heads labelled by two models). tlwatchd takes the red from the biggest
# vehicle head(s) in front; against the picture's truth on 10/29's 104 judged moments that left reds on green lights
# 39 -> 6 with no more greens read on a red (2 -> 1). Without the file nothing changes.
MODEL_LAMP = os.environ.get('C3_LAMP', os.path.join(HERE, 'models/lampcls_96.onnx'))
LAMP_OUTS = ['vehicle', 'pedestrian', 'red', 'yellow', 'green', 'left', 'straight', 'right', 'countdown']
LAMP_MEAN, LAMP_STD = np.array([0.485, 0.456, 0.406], np.float32), np.array([0.229, 0.224, 0.225], np.float32)
# ~33 ms a head on C3's CPU: on every box it added 40-255 ms to a ~480 ms cycle. tlwatchd takes the red from the biggest
# vehicle head, so only boxes at least this share of the biggest box's height are read (the same 104 moments: one more
# red on a green, 6 -> 7, nothing else changed).
LAMP_MIN_FRAC = 0.5
LAMP_MAX_N = 2      # ...and at most this many of them (on 10/29 frames 2-5 boxes passed the share: 37-167 ms)
THREADS = int(os.environ.get('C3_THREADS', '4'))
ALWAYS = os.environ.get('C3_ALWAYS', '0') == '1'      # testing: run whether or not the car is driving
OUT_DIR = '/dev/shm/c3det'
CALIB = '/dev/shm/c3calib.json'
EGO = '/dev/shm/c3ego.json'        # c3calib: pose (C4 speed + C3 gyro), camera basis, C4's extras - the light follower's
EGO_FRESH_S = 1.0
TRK = os.environ.get('C3_TRK', '1') == '1'   # the light follower (c3track, 11/03); C3_TRK=0 leaves it out
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
# Yellow back (10/28, the driver: "黃燈你認為可以就做吧") - only where a light turned yellow: a box read yellow counts if
# a green (or the yellow it already became) was read at about the same place in the image within YELLOW_AFTER_S. That
# leaves out what the 10/25 cut was about - striped posts, curve chevrons, yellow signs, bus lamps never were green -
# and the flashing yellows, which never turn from green. 10/28 drive (C3 route 000002d0, keyframes every 1.5 s over
# every approach, 1781 frames, judged by eye): any yellow 200 calls, mostly not lights or flashing yellows; this rule
# 22 events: 11 a light turning yellow (14:36:57 among them, where the car had sped up through the yellow), 4 wrong
# (3 of them with C4 already calling red), 7 too blurred to tell.
YELLOW_AFTER_S = 3.1
YELLOW_HEAD_PX = 40.               # full res; or 2.5 box widths, whichever is more: the same head a cycle later
TRACK_V = 0.3                      # m/s (C4's vEgo through c3calib): stopped
TRACK_PERIOD = 0.1                 # s: stopped with lights found, their colour this often ...
FIND_EVERY_S = 1.0                 # ... and the full finder this often (a light that appears / goes / was missed)
TRACK_PAD = 0.6                    # rows above / below a box read for its colour, in box heights (lightcls pads 25%)
# local passes (TASK3, see the top); without the model file, the CPU finder or C3_LOCAL=0: none, as before
MODEL_LOCAL = os.environ.get('C3_LOCAL_MODEL', os.path.join(HERE, f'models/c3light_ft1n_{c3track.LOCAL_S}x{c3track.LOCAL_S}_raw.onnx'))
LOCAL = os.environ.get('C3_LOCAL', '0') == '1'
LOCAL_THREADS = int(os.environ.get('C3_LOCAL_THREADS', '2'))   # C3 11/03: 192 px 70 ms on 1 thread, 42 on 2; the GPU
                                   # finder kept its 450 ms with 1-2 (4: +3-7%)
LOCAL_GAP_S = 0.15                 # s: a pass's frame at least this after the previous observation (cycle frame / pass)
LOCAL_SPARE_MS = 25.               # a pass is started only if it should end this long before the GPU's expected end
PIPE = os.environ.get('C3_PIPE', '0') == '1'   # moving: the finder pipelined with the CPU work (Reader.step_pipe) -
                                   # off: the GPU then never rests, C3 ran 9-10 C hotter (TASK3 leg 1-2, 30 min on C3)
BAND_MARGIN = 96                   # px each side of the band converted in step_pipe (colour / lamp crops pad the box)
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

  def submit(self, x):
    """Start the finder on x and return at once (the jit queues its kernels; result() waits for them)."""
    self.buf[:] = x
    self.out = self.jit(**{self.name: self.x})

  def result(self):
    out, self.out = self.out, None
    return out.numpy()


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
  return found_boxes(det(band.transpose(2, 0, 1)[None].astype(np.float32) / 255.)[0])


def found_boxes(out, conf=DET_CONF):
  """The finder's raw output (anchors, 84): x1 y1 x2 y2 + 80 scores -> traffic lights >= conf, best first."""
  out = out[out[:, 4 + LIGHT_CLASS] >= conf][:, [0, 1, 2, 3, 4 + LIGHT_CLASS]]
  return [tuple(float(v) for v in r) for r in out[np.argsort(-out[:, 4])]]


def local_session():
  """The small finder for local passes (onnxruntime, LOCAL_THREADS) or None."""
  if not LOCAL or not os.path.exists(MODEL_LOCAL):
    return None
  try:
    o = ort.SessionOptions()
    o.intra_op_num_threads = LOCAL_THREADS
    o.inter_op_num_threads = 1
    o.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    return ort.InferenceSession(MODEL_LOCAL, o, providers=['CPUExecutionProvider'])
  except Exception:
    return None


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


def lamp_session():
  try:
    return session(MODEL_LAMP) if os.path.exists(MODEL_LAMP) else None
  except Exception:
    return None


def lamps(sess, frame, boxes):
  """The arrow model on each box: the c3arrow training crop (box padded 25% of its longer side + 3 px, longer side to
  144, LANCZOS), letterboxed to 96 on black, ImageNet normalised -> [{lamp: p}] or None."""
  if sess is None or not boxes:
    return None
  img = Image.fromarray(frame)
  hmax = max(y2 - y1 for _, y1, _, y2 in boxes)
  # tlwatchd only looks at the biggest heads - see LAMP_MIN_FRAC
  big = sorted(range(len(boxes)), key=lambda i: boxes[i][1] - boxes[i][3])[:LAMP_MAX_N]
  out = []
  for i, b in enumerate(boxes):
    if i not in big or b[3] - b[1] < LAMP_MIN_FRAC * hmax:
      out.append(None)
      continue
    out.append(lamp_one(sess, img, b))
  return out


def lamp_one(sess, img, box):
  """The arrow model on one box of a PIL image (lamps())."""
  x1, y1, x2, y2 = box
  pad = 0.25 * max(x2 - x1, y2 - y1) + 3
  c = img.crop((int(x1 - pad), int(y1 - pad), int(x2 + pad), int(y2 + pad)))   # outside the frame comes in black, as trained
  s = 144 / max(c.size)
  c = c.resize((max(1, round(c.size[0] * s)), max(1, round(c.size[1] * s))), Image.LANCZOS)
  s = 96 / max(c.size)
  c = c.resize((max(1, round(c.size[0] * s)), max(1, round(c.size[1] * s))), Image.BILINEAR)
  lb = Image.new('RGB', (96, 96), (0, 0, 0))
  lb.paste(c, ((96 - c.size[0]) // 2, (96 - c.size[1]) // 2))
  x = ((np.asarray(lb, np.float32) / 255. - LAMP_MEAN) / LAMP_STD).transpose(2, 0, 1)[None].astype(np.float32)
  y = sess.run(None, {sess.get_inputs()[0].name: x})[0][0]
  if y.min() < 0 or y.max() > 1:
    y = 1. / (1. + np.exp(-y))
  return {k: round(float(v), 3) for k, v in zip(LAMP_OUTS, y, strict=True)}


class YellowGate:
  """Where a green (or a yellow let through) was read lately, to let a yellow through at the same head (YELLOW_*)."""
  def __init__(self):
    self.seen = []      # (t, xc, yc, w) full res

  def turned(self, box, now):
    x1, y1, x2, y2 = box
    xc, yc, w = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1
    for t, x, y, wp in self.seen:
      tol = max(YELLOW_HEAD_PX, 2.5 * max(w, wp))
      if now - t <= YELLOW_AFTER_S and abs(xc - x) < tol and abs(yc - y) < tol:
        return True
    return False

  def note(self, boxes, now):
    self.seen = [s for s in self.seen if now - s[0] <= YELLOW_AFTER_S] + \
                [(now, (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1) for x1, y1, x2, y2 in boxes]


def dets_from(boxes, scores, probs, gate=None, now=0., lamp=None):
  """Full-res boxes, finder scores and colour probabilities -> det.json entries (half-res), dropped count.
  gate (YellowGate, live only): a yellow passes where a light turned yellow; without it no yellow, as before.
  lamp (lamps()): the arrow model's read of each box, passed on as 'lamp'."""
  dets, dropped, keep = [], 0, []
  for i, ((x1, y1, x2, y2), score, p) in enumerate(zip(boxes, scores, probs, strict=True)):
    k = int(np.argmax(p))
    name, pc = CLS_NAMES[k], float(p[k])
    if name == 'yellow' and pc >= COLOR_MIN and gate is not None and gate.turned((x1, y1, x2, y2), now):
      c = 'light_yellow'
    elif name not in PASS_ON or pc < COLOR_MIN:
      dropped += 1
      continue
    else:
      c = PASS_ON[name]
    if c in ('light_green', 'light_green_arrow', 'light_yellow'):
      keep.append((x1, y1, x2, y2))
    dets.append({'x': round(x1 / 2, 1), 'y': round(y1 / 2, 1), 'w': round((x2 - x1) / 2, 1), 'h': round((y2 - y1) / 2, 1),
                 'cls': c, 'zh': ZH[c], 'conf': round(pc, 3), 'det': round(score, 3)})
    if lamp is not None and lamp[i] is not None:
      dets[-1]['lamp'] = lamp[i]
  if gate is not None:
    gate.note(keep, now)
  return dets, dropped


def read(det, cls, frame, row0, col0, lamp_sess=None):
  """Full-res RGB frame -> det.json boxes (half-res) and counts."""
  band = np.ascontiguousarray(frame[row0:row0 + BAND_H, col0:col0 + BAND_W])
  t0 = time.perf_counter()
  found = find(det, band)
  t1 = time.perf_counter()
  boxes = [(x1 + col0, y1 + row0, x2 + col0, y2 + row0) for x1, y1, x2, y2, _ in found]
  probs = colours(cls, frame, boxes)
  t2 = time.perf_counter()
  lp = lamps(lamp_sess, frame, boxes)
  t3 = time.perf_counter()
  dets, dropped = dets_from(boxes, [f[4] for f in found], probs, lamp=lp)
  return dets, {'found': len(found), 'dropped': dropped, 'det_ms': round((t1 - t0) * 1000), 'cls_ms': round((t2 - t1) * 1000),
                'lamp_ms': round((t3 - t2) * 1000)}


class Reader:
  """One reading a call: the full finder + colour, or - stopped, lights found within FIND_EVERY_S - the colour of
  those lights only, off their own rows. rows(r0, r1) -> (full-width RGB of rows r0.., first row actually given)."""
  def __init__(self, det, cls):
    self.det, self.cls = det, cls
    self.lamp = lamp_session()
    self.boxes, self.scores, self.t_find = [], [], -1e9
    self.n_shown = 0
    self.gate = YellowGate()   # a yellow only where a light turned yellow (YELLOW_*)
    self.went_dark = False     # all tracked lights stopped showing a colour (the lit lamp moved, e.g. to a lamp of the
                               # head the finder did not box): find again at once instead of after FIND_EVERY_S
    # stopped: the head that stopped us, pinned and read on its own (c3pin, 11/03) -> det.json 'pin' for tlwatchd
    self.pin = StopPin((lambda rgb, boxes: lamps(self.lamp, rgb, boxes)) if self.lamp is not None else None)
    # moving: follow the heads, lock ours, read it in order (c3track, 11/03) -> det.json 'trk' for tlwatchd
    self.fol = c3track.Follower() if TRK else None
    self.trk = None
    # moving: the followed heads looked for again on newer frames while the GPU finds (TASK3) -> the follower next cycle
    self.local = local_session() if self.fol is not None and isinstance(det, GpuFinder) else None
    self.pending = []          # this cycle's local passes, for the follower after this cycle's own frame
    self.gpu_ms, self.crop_ms, self.conv_ms = 450., 75., 35.   # running estimates (ms): the GPU finder, one local crop
                                                             # (model + colour), a band's NV12 conversion
    self.pipe = PIPE and isinstance(det, GpuFinder) and self.fol is not None
    self.inflight = None       # the frame the GPU is on (step_pipe)
    self.loc = None            # the last cycle's local-pass counts for det.json

  def follow(self, t_frame, rgb, r0, local, probs, lp):
    """The light follower on this finder cycle (c3track): its own lamp reads on the boxes it may track (lamp_idx; the
    ones lamps() read for det.json are reused), the pose / camera / C4 extras from c3calib (EGO). -> det.json 'trk'."""
    if self.fol is None:
      return None
    t0 = time.perf_counter()
    boot = time.clock_gettime(time.CLOCK_BOOTTIME)
    ego = load_ego(boot)
    if ego is None:
      self.fol.reset()
      return None
    if t_frame is None or abs(t_frame - boot) > 2.:
      t_frame = boot
    boxes, scores = self.boxes, self.scores
    lam = list(lp) if lp else [None] * len(boxes)
    if self.lamp is not None:
      img = None
      for i in c3track.lamp_idx(boxes, scores, ego.cam, must=self.fol.ours_near(t_frame, ego, boxes)):
        if lam[i] is None:
          img = Image.fromarray(rgb) if img is None else img
          lam[i] = lamp_one(self.lamp, img, local[i])
    dets = [c3track.Det(boxes[i], scores[i], {k: float(p) for k, p in zip(CLS_NAMES, probs[i], strict=True)}, lam[i], ego.cam)
            for i in c3track.dedupe(boxes, scores)]
    tl = ego.tl

    def lamp_fn(b):
      return lamp_one(self.lamp, Image.fromarray(rgb), (b[0], b[1] - r0, b[2], b[3] - r0)) if self.lamp is not None else None
    r = self.fol.step(t_frame, dets, ego, osm=tl.get('osm'), blink=(bool(tl.get('bl')), bool(tl.get('br'))),
                      radar=tl.get('rad'), crop=lambda b: c3track.crop_box(rgb, r0, b), lamp_fn=lamp_fn)
    r.update({'t': round(t_frame, 3), 'ms': round((time.perf_counter() - t0) * 1000),
              'box': None if r['box'] is None else [round(v / 2, 1) for v in r['box']]})   # det.json's half res
    self.trk = r
    return r

  def local_pass(self, crop_fn, t, ego, n_crops=None):
    """One local pass at frame time t: crop_fn(r0, r1, c0, c1) -> full-res RGB of that window (zero outside the
    frame); the follower's targets (c3track.local_targets / local_crops), the small finder on each crop, matched to the
    targets (local_match), lightcls on each match -> the pass for Follower.step_local, or None (nothing to look for)."""
    fol = self.fol
    crops = c3track.local_crops(fol.local_targets(t, ego))[:n_crops]
    if not crops:
      return None
    S = c3track.LOCAL_S
    name = self.local.get_inputs()[0].name
    found, dets, wins = {}, [], []
    for x0, y0, k, tgs in crops:
      t0 = time.perf_counter()
      R = S * k
      win = crop_fn(y0, y0 + R, x0, x0 + R)
      img = win if k == 1 else win.reshape(S, k, S, k, 3).mean(axis=(1, 3))
      out = self.local.run(None, {name: np.ascontiguousarray(img.transpose(2, 0, 1)[None], dtype=np.float32) / 255.})[0][0]
      fb = found_boxes(out, c3track.LOCAL_CONF)
      boxes = [(x1 * k + x0, y1 * k + y0, x2 * k + x0, y2 * k + y0) for x1, y1, x2, y2, _ in fb]
      keep = c3track.dedupe(boxes, [f[4] for f in fb])
      boxes, scores = [boxes[i] for i in keep], [fb[i][4] for i in keep]
      m = c3track.local_match(tgs, boxes, t, ego)
      if m:
        local = [(b[0] - x0, b[1] - y0, b[2] - x0, b[3] - y0) for b in boxes]
        probs = colours(self.cls, win, local)
        ids = {i: tid for tid, i in m.items()}
        lam = [None] * len(boxes)
        if c3track.LOCAL_READ and self.lamp is not None and fol.ours is not None and fol.ours.id in m:
          lam[m[fol.ours.id]] = lamp_one(self.lamp, Image.fromarray(win), local[m[fol.ours.id]])
        for i, b in enumerate(boxes):
          d = c3track.Det(b, scores[i], {c: float(q) for c, q in zip(CLS_NAMES, probs[i], strict=True)}, lam[i], ego.cam, local=True)
          dets.append(d)
          if i in ids:
            found[ids[i]] = d
      wins.append((x0, y0, win))
      self.crop_ms = 0.8 * self.crop_ms + 0.2 * (time.perf_counter() - t0) * 1000
    return {'t': t, 'found': found, 'dets': dets, 'wins': wins, 'crops': len(crops), 'tl': ego.tl}

  def apply_pending(self, ego):
    """The last cycle's local passes into the follower, in time order (their poses are in c3calib's EGO by now)."""
    pend, self.pending = self.pending, []
    if self.fol is None or ego is None:
      return
    for p in pend:
      tl = p['tl']                               # C4's OSM / blinkers / radar as they were at the pass
      def crop(b, wins=p['wins']):
        x1, y1, x2, y2 = b
        for x0, y0, w in wins:
          if x0 <= x1 and y0 <= y1 and x2 <= x0 + w.shape[1] and y2 <= y0 + w.shape[0]:
            return c3track.crop_box(w, y0, (x1 - x0, y1, x2 - x0, y2))
        return None
      t0 = time.perf_counter()
      r = self.fol.step_local(p['t'], p['found'], p['dets'], ego, osm=tl.get('osm'), blink=(bool(tl.get('bl')), bool(tl.get('br'))),
                              radar=tl.get('rad'), crop=crop)
      if r is not None:
        r.update({'t': round(p['t'], 3), 'ms': round((time.perf_counter() - t0) * 1000),
                  'box': None if r['box'] is None else [round(v / 2, 1) for v in r['box']]})
        self.trk = r

  def local_window(self, t_frame, more, t_sub):
    """While the GPU finds on this cycle's frame (submitted at t_sub, perf_counter): the last cycle's passes into the
    follower, then passes on newer frames as long as one should end LOCAL_SPARE_MS before the GPU's expected end.
    more(t_min, deadline) -> (crop_fn, frame time) of the newest frame at or after t_min, or None by deadline."""
    st = {'n': 0, 'crops': 0, 'found': 0}
    boot = time.clock_gettime(time.CLOCK_BOOTTIME)
    ego = load_ego(boot)
    self.apply_pending(ego)
    gpu_end = t_sub + (self.gpu_ms - self.conv_ms) / 1000.   # ...and the next frame's conversion fits in the GPU's tail
    t_last = t_frame if t_frame is not None else boot
    while ego is not None:
      end = gpu_end - 0.001 * LOCAL_SPARE_MS
      fr = more(t_last + LOCAL_GAP_S, end - 0.001 * self.crop_ms)       # time left for one crop at least
      if fr is None:
        break
      crop_fn, t = fr
      n = int((end - time.perf_counter()) * 1000 / max(self.crop_ms, 1.))  # as many crops as there is time for
      if n < 1:
        break
      ego = load_ego(time.clock_gettime(time.CLOCK_BOOTTIME)) or ego
      p = self.local_pass(crop_fn, t, ego, min(n, c3track.LOCAL_MAX))
      if p is None:
        break
      self.pending.append(p)
      st['n'] += 1
      st['crops'] += p['crops']
      st['found'] += len(p['found'])
      t_last = t
    return st

  def tracking(self, now, stopped):
    return stopped and bool(self.boxes) and now - self.t_find < FIND_EVERY_S and not self.went_dark

  def step(self, rows, row0, col0, now, stopped, horizon=None, ahead=None, t_frame=None, more=None):
    """more (live, moving): the camera's newer frames for local passes (local_window); None - no local passes."""
    t0 = time.perf_counter()
    # c3pin's straight ahead / horizon, full res (c3calib's are half res; none yet -> the band's defaults)
    hv = 2 * float(horizon) if horizon is not None else ROW0_DEFAULT + BAND_ABOVE
    au = 2 * float(ahead) if ahead is not None else FULL_W / 2
    if self.tracking(now, stopped):
      self.flush()
      top = int(max(0, min(y1 - (y2 - y1) * TRACK_PAD for _, y1, _, y2 in self.boxes)))
      bot = int(min(FULL_H, max(y2 + (y2 - y1) * TRACK_PAD for _, y1, _, y2 in self.boxes) + 2))
      rgb, r0 = rows(top, bot)
      local = [(x1, y1 - r0, x2, y2 - r0) for x1, y1, x2, y2 in self.boxes]
      probs = colours(self.cls, rgb, local)
      lp = lamps(self.lamp, rgb, local)
      dets, dropped = dets_from(self.boxes, self.scores, probs, self.gate, now, lp)
      self.went_dark = not dets and self.n_shown > 0     # every tracked light dark: whatever is lit now is elsewhere
      pin = self.pin.update(now, stopped, self.boxes, probs, lp, rgb, r0, hv, au)
      return dets, {'mode': 'track', 'found': len(self.boxes), 'dropped': dropped, 'det_ms': 0,
                    'cls_ms': round((time.perf_counter() - t0) * 1000), 'pin': pin, 'trk': trk_aged(self.trk)}
    if self.pipe and more is not None and not stopped:
      return self.step_pipe(rows, row0, col0, now, stopped, hv, au, t_frame, more)
    self.flush()
    rgb, r0 = rows(row0, row0 + BAND_H)
    band = np.ascontiguousarray(rgb[row0 - r0:row0 - r0 + BAND_H, col0:col0 + BAND_W])
    found = find(self.det, band)
    t1 = time.perf_counter()
    fr = {'rgb': rgb, 'r0': r0, 'row0': row0, 'col0': col0, 'now': now, 'stopped': stopped, 'hv': hv, 'au': au,
          't_frame': t_frame}
    dets, st = self.process(fr, found)
    st.update({'det_ms': round((t1 - t0) * 1000)})
    return dets, st

  def process(self, fr, found):
    """Everything after the finder for one cycle's frame fr: colour, lamp model, c3pin, the follower -> dets, st."""
    t1 = time.perf_counter()
    rgb, r0, row0, col0, now, stopped = fr['rgb'], fr['r0'], fr['row0'], fr['col0'], fr['now'], fr['stopped']
    self.boxes = [(x1 + col0, y1 + row0, x2 + col0, y2 + row0) for x1, y1, x2, y2, _ in found]
    self.scores, self.t_find = [f[4] for f in found], now
    local = [(x1, y1 - r0, x2, y2 - r0) for x1, y1, x2, y2 in self.boxes]
    probs = colours(self.cls, rgb, local)
    lp = lamps(self.lamp, rgb, local)
    dets, dropped = dets_from(self.boxes, self.scores, probs, self.gate, now, lp)
    self.n_shown, self.went_dark = len(dets), False
    pin = self.pin.update(now, stopped, self.boxes, probs, lp, rgb, r0, fr['hv'], fr['au'])
    t2 = time.perf_counter()
    try:
      trk = self.follow(fr['t_frame'], rgb, r0, local, probs, lp)
    except Exception as e:  # the follower must never stop the lights
      print('c3lights: follower error', repr(e)[:200], flush=True)
      self.fol, self.trk, trk = (c3track.Follower() if TRK else None), None, None
      self.pending = []
    return dets, {'mode': 'find', 'found': len(found), 'dropped': dropped, 'cls_ms': round((t2 - t1) * 1000), 'pin': pin,
                  'trk': trk_aged(trk)}

  def flush(self):
    """Leave the pipeline (stopped / no newer frames): the frame in flight is dropped, the local passes too."""
    if self.inflight is not None:
      self.det.result()
      self.inflight = None
    self.pending = []

  def step_pipe(self, rows, row0, col0, now, stopped, hv, au, t_frame, more):
    """Moving (TASK3): the GPU finds on this frame while the CPU finishes the LAST frame (colour, lamp model, the
    follower), then the last cycle's local passes go in and new ones are made on frames newer than this one - a finder
    cycle takes about the GPU's time instead of conversion + GPU + all the CPU work after it. Returns the last frame's
    dets / st (None, None on the first frame: nothing finished yet)."""
    t0 = time.perf_counter()
    # this frame's band (+ a margin for the colour / lamp crops of boxes at its edge), converted while the GPU may
    # still be on the last one
    rgb, r0 = rows(row0, row0 + BAND_H, max(0, col0 - BAND_MARGIN), min(FULL_W, col0 + BAND_W + BAND_MARGIN))
    x = np.ascontiguousarray(rgb[row0 - r0:row0 - r0 + BAND_H, col0:col0 + BAND_W]).transpose(2, 0, 1)[None].astype(np.float32) / 255.
    t1 = time.perf_counter()
    self.conv_ms = 0.8 * self.conv_ms + 0.2 * (t1 - t0) * 1000
    prev, found = self.inflight, None
    if prev is not None:
      found = found_boxes(self.det.result()[0])
      t_res = time.perf_counter()
      if t_res - t1 > 0.005:                     # waited for it: the GPU's own time
        self.gpu_ms = 0.8 * self.gpu_ms + 0.2 * (t_res - prev['t_sub']) * 1000
      prev['wait_ms'] = round((t_res - t1) * 1000)
    self.det.submit(x)
    t_sub = time.perf_counter()
    self.inflight = {'rgb': rgb, 'r0': r0, 'row0': row0, 'col0': col0, 'now': now, 'stopped': stopped, 'hv': hv,
                     'au': au, 't_frame': t_frame, 't_sub': t_sub}
    if prev is None:
      self.pending = []
      return None, None
    dets, st = self.process(prev, found)
    t2 = time.perf_counter()
    loc = None
    if self.local is not None and self.fol is not None:
      try:
        loc = self.local_window(t_frame, more, t_sub)
      except Exception as e:  # local passes must never stop the lights
        print('c3lights: local pass error', repr(e)[:200], flush=True)
        self.pending = []
    st.update({'det_ms': round(self.gpu_ms), 'loc': loc,
               'pipe': {'conv_ms': round((t1 - t0) * 1000), 'wait_ms': prev['wait_ms'], 'proc_ms': round((t2 - t_sub) * 1000),
                        'loc_ms': round((time.perf_counter() - t2) * 1000), 'gpu_ms': round(self.gpu_ms)}})
    return dets, st


def trk_aged(trk):
  """det.json 'trk' with its age (s since its frame, C3's boot clock) - tlwatchd leaves an old one alone."""
  if trk is None:
    return None
  return dict(trk, age=round(time.clock_gettime(time.CLOCK_BOOTTIME) - trk['t'], 2))


def load_ego(boot_now):
  """c3calib's EGO -> c3track.Ego (cam and C4's extras attached), or None while it is stale / has no camera basis or
  C4 sent no extras (an older C4: no OSM, no blinkers - the follower would only guess)."""
  try:
    with open(EGO) as f:
      j = json.load(f)
  except (OSError, ValueError):
    return None
  if boot_now - float(j.get('tb', 0)) > EGO_FRESH_S or not j.get('basis') or len(j.get('t') or []) < 2 or not j.get('tl'):
    return None
  ego = c3track.Ego(j['t'], j['v'], j['n'], j['e'], j['hd'])
  ego.cam, ego.tl = c3track.Cam(j['basis']), j['tl']
  return ego


def nv12_rgb(buf, r0, r1, c0=None, c1=None):
  """Full-res RGB of rows r0..r1 straight off the NV12 buffer; with c0 / c1 only those columns are converted (the rest
  zero) - 11/04: the chroma terms at half resolution, then repeated (the same integers as before, ~3x faster: the
  moving cycle's band took 96 ms on C3)."""
  c0 = 0 if c0 is None else c0 - c0 % 2
  c1 = buf.width if c1 is None else min(buf.width, c1 + c1 % 2)
  data = np.frombuffer(buf.data, dtype=np.uint8)
  w, stride, uv_off = buf.width, buf.stride, buf.uv_offset
  r0 -= r0 % 2
  y = data[r0 * stride:r1 * stride].reshape(r1 - r0, stride)[:, c0:c1].astype(np.int32)
  uv = data[uv_off + (r0 // 2) * stride:uv_off + ((r1 + 1) // 2) * stride].reshape(-1, stride)[:, c0:c1]
  u = uv[:, 0::2].astype(np.int32) - 128
  v = uv[:, 1::2].astype(np.int32) - 128
  out = np.zeros((r1 - r0, w, 3), np.uint8)
  n = y.shape[0]
  for ch, term in ((0, (91881 * v) >> 16), (1, -((22554 * u + 46802 * v) >> 16)), (2, (116130 * u) >> 16)):
    out[:, c0:c1, ch] = np.clip(y + np.repeat(np.repeat(term, 2, axis=0), 2, axis=1)[:n], 0, 255)
  return out, r0


def nv12_crop(buf, r0, r1, c0, c1):
  """Full-res RGB of rows r0..r1, columns c0..c1 off the NV12 buffer (zeros outside the frame) - a local pass's window."""
  out = np.zeros((r1 - r0, c1 - c0, 3), np.uint8)
  a0, a1 = max(r0, 0), min(r1, buf.height)
  b0, b1 = max(c0, 0), min(c1, buf.width)
  if a1 <= a0 or b1 <= b0:
    return out
  e0, f0 = a0 - a0 % 2, b0 - b0 % 2                 # NV12: chroma per 2x2
  e1, f1 = a1 + a1 % 2, min(b1 + b1 % 2, buf.width)
  data = np.frombuffer(buf.data, dtype=np.uint8)
  stride, uv_off = buf.stride, buf.uv_offset
  y = data[e0 * stride:e1 * stride].reshape(e1 - e0, stride)[:, f0:f1].astype(np.int32)
  uv = data[uv_off + (e0 // 2) * stride:uv_off + (e1 // 2) * stride].reshape(-1, stride)[:, f0:f1]
  u = np.repeat(np.repeat(uv[:, 0::2].astype(np.int32) - 128, 2, axis=0), 2, axis=1)[:y.shape[0], :y.shape[1]]
  v = np.repeat(np.repeat(uv[:, 1::2].astype(np.int32) - 128, 2, axis=0), 2, axis=1)[:y.shape[0], :y.shape[1]]
  rgb = np.empty(y.shape + (3,), dtype=np.int32)
  rgb[..., 0] = y + ((91881 * v) >> 16)
  rgb[..., 1] = y - ((22554 * u + 46802 * v) >> 16)
  rgb[..., 2] = y + ((116130 * u) >> 16)
  out[a0 - r0:a1 - r0, b0 - c0:b1 - c0] = np.clip(rgb, 0, 255).astype(np.uint8)[a0 - e0:a1 - e0, b0 - f0:b1 - f0]
  return out


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
  lamp_sess = lamp_session()
  print('lamp', MODEL_LAMP if lamp_sess is not None else 'none', flush=True)
  for f in sorted(glob.glob(os.path.join(fdir, '*.jpg'))):
    frame = np.asarray(Image.open(f).convert('RGB'))
    dets, st = read(det, cls, frame, row0, col0, lamp_sess)
    res[os.path.basename(f)] = {'dets': dets, **st}
  json.dump(res, open(out_path, 'w'), ensure_ascii=False)
  ms = [v['det_ms'] + v['cls_ms'] for v in res.values()]
  print('frames', len(res), 'lights passed on', sum(len(v['dets']) for v in res.values()), 'median ms', int(np.median(ms)))


def main():
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  os.makedirs(OUT_DIR, exist_ok=True)
  (det, label), cls = finder(), session(MODEL_CLS)
  label += ' + ' + os.path.basename(MODEL_CLS) + (' + ' + os.path.basename(MODEL_LAMP) if os.path.exists(MODEL_LAMP) else '')
  # conflated: the newest frame - not the oldest one queued (see the top: queued, a moving cycle read ever older packets
  # whose buffers already held newer frames)
  client = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
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
    t_frame = client.timestamp_sof / 1e9 if client.timestamp_sof else None    # boot clock, as c3calib's pose
    lag_ms = round((time.clock_gettime(time.CLOCK_BOOTTIME) - t_frame) * 1000) if t_frame else None   # the frame's age

    def more(t_min, deadline):
      """The newest frame from t_min on (a local pass), waiting until deadline (perf_counter) -> (crop_fn, t) or None."""
      while True:
        rem = deadline - time.perf_counter()
        if rem <= 0.002:
          return None
        b = client.recv(max(1, int(min(rem, 0.1) * 1000)))
        if b is None or not client.timestamp_sof:
          continue
        t = client.timestamp_sof / 1e9
        if t >= t_min:
          return (lambda r0, r1, c0, c1, b=b: nv12_crop(b, r0, r1, c0, c1)), t
    dets, st = reader.step(lambda a, b, c=None, d=None, buf=buf: nv12_rgb(buf, a, b, c, d), row0, col0, now, stopped, horizon,
                           ahead, t_frame, more)
    if dets is None:                             # the pipeline's first frame: nothing finished yet
      continue
    t_end = time.monotonic()
    dt = t_end - t_last
    t_last = t_end
    if dt > 0:
      fps = 1. / dt if fps == 0 else fps * 0.8 + 0.2 / dt
    state.update({'dets': dets, 'n': len(dets), **st, 'fps': round(fps, 2), 'band_y0': row0 // 2, 'band_x0': col0 // 2,
                  'temp': round(cpu_temp(), 1), 'err': '', 'idle': False, 'ts': round(now, 2), 'lag_ms': lag_ms})
    write(state)


if __name__ == '__main__':
  if len(sys.argv) > 2 and sys.argv[1] == '--frames':
    offline(sys.argv[2], sys.argv[3])
  else:
    main()
