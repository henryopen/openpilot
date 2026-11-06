#!/usr/bin/env python3
"""yolo_core: YOLO11n object detection sized for the comma device's road camera.

The road camera is 1344x760; dropping every other pixel gives 672x380, which pads to the
model's 672x384 input with four rows and no scaling, so nothing is cropped or resampled.

Runs on onnxruntime's CPU provider on purpose: the GPU belongs to modeld, and the CPU cost
of one frame is measurable and bounded in a way a second tinygrad graph on the same GPU
would not be.
"""
import io
import math

import numpy as np

MODEL_W, MODEL_H = 672, 384

# os04c10 narrow road camera (1344x760, f=1141.5), halved to match the frames we run on
FOCAL = 1141.5 / 2
CX, CY = 1344 / 4, 760 / 4
CAM_HEIGHT = 1.22          # calibrationd's HEIGHT_INIT; the cached calibration refines it
MAX_RANGE = 150.0          # beyond this the box bottom is within a pixel of the horizon
# Checked against radar-matched leads over three recorded segments: the estimate runs about
# 10% long, because YOLO's box bottom sits a little above the real contact patch. Segment
# medians were +3%, +11% and +36%, so treat anything past ~40 m as a rough placement.
RANGE_SCALE = 0.9

COCO = [
  'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
  'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
  'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
  'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
  'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
  'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
  'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
  'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
  'scissors', 'teddy bear', 'hair drier', 'toothbrush',
]

# what matters on a road; everything else is dropped before it reaches the HUD
ROAD_CLASSES = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck',
                9: 'traffic light', 11: 'stop sign'}
TRAFFIC_LIGHT = 9

# The model trained on this car's own footage. Same seven road classes, but its own order,
# and the light split into the six states COCO cannot tell apart. Mapped back onto COCO's
# indices on the way out so nothing downstream has to know which model produced a box.
C4 = ['person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck', 'stop_sign',
      'light_red', 'light_green', 'light_yellow', 'light_off', 'light_pedestrian',
      'light_green_arrow']
C4_TO_COCO = {0: 0, 1: 1, 2: 2, 3: 3, 4: 5, 5: 7, 6: 11,
              7: 9, 8: 9, 9: 9, 10: 9, 11: 9, 12: 9}
C4_ROAD = set(C4_TO_COCO)

# The band: lights are 20 px tall in the native frame and 10 px after the halving that
# nv12_to_rgb does, under YOLO's finest stride of 8. These rows are read at full width and
# full resolution instead, which is the whole reason the band model sees three times the
# red lights the road model does. Lights sit in these rows and nothing else does: 99% of
# labelled lights fall inside, while 80% of cars have their bottom edge below them.
BAND_W, BAND_H, BAND_Y0 = 1344, 352, 32
BAND_LIGHT = ['red', 'green', 'amber', 'off', 'ped', 'arrow']
BAND_NAMES = ['light_red', 'light_green', 'light_yellow', 'light_off',
              'light_pedestrian', 'light_green_arrow']

COLORS = {0: (255, 96, 96), 1: (255, 200, 60), 2: (80, 200, 255), 3: (255, 140, 40),
          5: (160, 130, 255), 7: (120, 255, 160), 9: (255, 255, 255), 11: (255, 60, 60)}


def make_session(onnx_path, threads=2):
  import onnxruntime as ort
  opts = ort.SessionOptions()
  # openpilot pins its processes to cores, which makes onnxruntime's own affinity calls fail;
  # setting the thread counts explicitly is what its error message asks for
  opts.intra_op_num_threads = threads
  opts.inter_op_num_threads = 1
  opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
  opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
  return ort.InferenceSession(onnx_path, opts, providers=['CPUExecutionProvider'])


def nv12_to_rgb(buf):
  """Half-resolution RGB from an openpilot NV12 VisionBuf, without a full-size conversion.

  Both planes are padded past the visible frame (the device reports 1344x760 inside a
  1408x768 luma allocation), so each is sliced to the rows actually asked for rather than
  reshaped over the whole buffer.
  """
  data = np.frombuffer(buf.data, dtype=np.uint8)
  w, h, stride, uv_off = buf.width, buf.height, buf.stride, buf.uv_offset
  # int32 throughout: the fixed-point coefficients below overflow int16, which numpy 2 raises on
  y = data[:h * stride].reshape(h, stride)[::2, :w:2].astype(np.int32)
  uv = data[uv_off:uv_off + (h // 2) * stride].reshape(h // 2, stride)[:, :w]
  # chroma is already half resolution, so take every other pair to match the luma we kept
  u = uv[::2, 0::2][:, ::2].astype(np.int32) - 128
  v = uv[::2, 1::2][:, ::2].astype(np.int32) - 128
  u = np.repeat(np.repeat(u, 2, 0), 2, 1)[:y.shape[0], :y.shape[1]]
  v = np.repeat(np.repeat(v, 2, 0), 2, 1)[:y.shape[0], :y.shape[1]]
  rgb = np.empty(y.shape + (3,), dtype=np.int32)
  rgb[..., 0] = y + ((91881 * v) >> 16)
  rgb[..., 1] = y - ((22554 * u + 46802 * v) >> 16)
  rgb[..., 2] = y + ((116130 * u) >> 16)
  return np.clip(rgb, 0, 255).astype(np.uint8)


def nv12_to_band(buf):
  """Native-resolution RGB of the band rows, full width, straight off the NV12.

  nv12_to_rgb halves the frame, which is what costs the lights their pixels. This keeps
  every column and every row it reads, and reads only 352 of the 760 rows, so it is the
  cheaper of the two conversions despite being at full resolution.
  """
  data = np.frombuffer(buf.data, dtype=np.uint8)
  w, h, stride, uv_off = buf.width, buf.height, buf.stride, buf.uv_offset
  y0 = BAND_Y0 - BAND_Y0 % 2          # chroma is shared between row pairs; start on an even row
  y1 = min(h, y0 + BAND_H)
  y = data[y0 * stride:y1 * stride].reshape(y1 - y0, stride)[:, :w].astype(np.int32)
  # one chroma row per two luma rows, U and V interleaved along it
  uv = data[uv_off + (y0 // 2) * stride:uv_off + ((y1 + 1) // 2) * stride].reshape(-1, stride)[:, :w]
  u = np.repeat(np.repeat(uv[:, 0::2].astype(np.int32) - 128, 2, 0), 2, 1)[:y.shape[0], :y.shape[1]]
  v = np.repeat(np.repeat(uv[:, 1::2].astype(np.int32) - 128, 2, 0), 2, 1)[:y.shape[0], :y.shape[1]]
  rgb = np.empty(y.shape + (3,), dtype=np.int32)
  rgb[..., 0] = y + ((91881 * v) >> 16)
  rgb[..., 1] = y - ((22554 * u + 46802 * v) >> 16)
  rgb[..., 2] = y + ((116130 * u) >> 16)
  return np.clip(rgb, 0, 255).astype(np.uint8)


def detect_band(sess, rgb, conf_thres=0.25, iou_thres=0.45):
  """-> [{'cls','name','conf','box','light'}] for the six light states.

  0.25 rather than the road model's 0.35: measured on 245 held-out red lights, 0.25 finds
  60% of them where 0.35 finds 28%. It also leaves a false red in one frame in eight, which
  is why yolod asks to see a colour twice before passing it on rather than raising this.
  The colour comes from the class here, not from reading the pixels inside the box.
  """
  x = np.zeros((BAND_H, BAND_W, 3), dtype=np.uint8)
  x[:rgb.shape[0], :rgb.shape[1]] = rgb[:BAND_H, :BAND_W]
  x = np.ascontiguousarray(x.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)
  out = sess.run(None, {sess.get_inputs()[0].name: x})[0]
  p = out[0].T
  scores = p[:, 4:]
  cls = scores.argmax(1)
  conf = scores[np.arange(len(cls)), cls]
  m = conf > conf_thres
  if not m.any():
    return []
  p, cls, conf = p[m], cls[m], conf[m]
  cx, cy, w, h = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
  boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], 1)
  dets = []
  for c in np.unique(cls):
    idx = np.where(cls == c)[0]
    for k in _nms(boxes[idx], conf[idx], iou_thres):
      i = idx[k]
      dets.append({'cls': TRAFFIC_LIGHT, 'name': BAND_NAMES[c], 'conf': round(float(conf[i]), 3),
                   'box': [round(float(v), 1) for v in boxes[i]], 'light': BAND_LIGHT[c]})
  dets.sort(key=lambda d: -d['conf'])
  return dets


def pad_top(rgb):
  """Rows of letterbox above the frame; half-res 1344x760 leaves two, and nothing is scaled."""
  return max(0, (MODEL_H - rgb.shape[0]) // 2)


def to_input(rgb):
  """(h,w,3) uint8 -> (1,3,384,672) float32, padding rather than scaling."""
  padded = np.zeros((MODEL_H, MODEL_W, 3), dtype=np.uint8)
  top = pad_top(rgb)
  h = min(rgb.shape[0], MODEL_H - top)
  w = min(rgb.shape[1], MODEL_W)
  padded[top:top + h, :w] = rgb[:h, :w]
  return np.ascontiguousarray(padded.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)


def _nms(boxes, scores, iou_thres):
  order = scores.argsort()[::-1]
  areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
  keep = []
  while order.size:
    i = order[0]
    keep.append(i)
    if order.size == 1:
      break
    rest = order[1:]
    xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
    yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
    xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
    yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
    inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
    iou = inter / (areas[i] + areas[rest] - inter + 1e-9)
    order = rest[iou <= iou_thres]
  return keep


def light_colour(rgb, box):
  """Which lamp is lit inside a traffic light box. YOLO gives the box but not the state."""
  x1, y1, x2, y2 = (int(max(0, v)) for v in box)
  crop = rgb[y1:y2, x1:x2].astype(np.int16)
  if crop.size < 27:
    return None
  r, g, b = crop[..., 0], crop[..., 1], crop[..., 2]
  bright = (r + g + b) > 240  # only lit lamps, not the housing
  if bright.sum() < 3:
    return None
  r, g, b = r[bright], g[bright], b[bright]
  red = int(((r > g + 40) & (r > b + 40)).sum())
  green = int(((g > r + 25) & (g > b - 10)).sum())
  amber = int(((r > b + 60) & (g > b + 40) & (abs(r - g) < 70)).sum())
  best = max((red, 'red'), (amber, 'amber'), (green, 'green'))
  return best[1] if best[0] >= max(3, 0.15 * bright.sum()) else None


def read_calibration():
  """-> (pitch, yaw, height) from calibrationd's cached result, or the defaults.

  Worth reading rather than assuming: this car calibrates to 1.339 m, not the 1.22 m
  default, and distance scales directly with it.
  """
  try:
    from openpilot.cereal import log
    from openpilot.common.params import Params
    raw = Params().get('CalibrationParams')
    if not raw:
      return 0.0, 0.0, CAM_HEIGHT
    with log.Event.from_bytes(raw) as msg:
      e = msg.extrinsicsCalibration
      rpy = list(e.rpyCalib)
      h = list(e.height)
      return (float(rpy[1]) if len(rpy) == 3 else 0.0,
              float(rpy[2]) if len(rpy) == 3 else 0.0,
              float(h[0]) if h else CAM_HEIGHT)
  except Exception:
    return 0.0, 0.0, CAM_HEIGHT


def ground_xy(box, pitch=0.0, yaw=0.0, height=CAM_HEIGHT, img_h=380):
  """Where a detection meets the road, in metres: (distance ahead, lateral, + is right).

  Assumes the bottom of the box is the object's contact patch on a flat road, which is the
  standard monocular trick. It degrades with distance - near the horizon one pixel is many
  metres - and it is wrong on slopes and under braking, so far numbers are placements
  rather than measurements. Returns None when the estimate cannot be trusted at all.
  """
  v = box[3]
  if v >= img_h - 6:
    return None  # bottom runs off the frame (or into the bonnet), so it is not the contact patch
  v_horizon = CY - FOCAL * math.tan(pitch)
  if v - v_horizon < 1.0:
    return None  # at or above the horizon
  d = FOCAL * height / (v - v_horizon) * RANGE_SCALE
  if d > MAX_RANGE:
    return None
  lat = ((box[0] + box[2]) / 2 - (CX + FOCAL * math.tan(yaw))) * d / FOCAL
  return round(d, 1), round(lat, 2)


def _is_rider(person, twowheeler):
  """Is this person the one riding that motorcycle or bicycle?

  YOLO boxes the rider and the machine separately, and a rider is not a second thing on the
  road. The rider's box sits above the machine and shares its centreline, so it is matched
  on horizontal alignment plus feet landing within the machine's vertical span - not IoU,
  which stays low because the two boxes are stacked rather than overlapping.
  """
  px1, _, px2, py2 = person
  mx1, my1, mx2, my2 = twowheeler
  mw, mh = mx2 - mx1, my2 - my1
  if mw <= 0 or mh <= 0:
    return False
  if abs((px1 + px2) / 2 - (mx1 + mx2) / 2) > mw * 0.7:
    return False
  return my1 - mh * 0.6 < py2 < my2 + mh * 0.3


def detect(sess, rgb, conf_thres=0.35, iou_thres=0.45, road_only=True, drop_riders=True, c4=False):
  """-> list of dicts in the coordinate frame of `rgb` (i.e. half-res camera pixels).

  `c4` switches to the model trained on this car, whose classes are its own; its indices
  are mapped back onto COCO's so a caller cannot tell which model it got.
  """
  names, road = (C4, C4_ROAD) if c4 else (COCO, ROAD_CLASSES)
  out = sess.run(None, {sess.get_inputs()[0].name: to_input(rgb)})[0]
  p = out[0].T                                   # (n, 4 + 80)
  scores = p[:, 4:]
  cls = scores.argmax(1)
  conf = scores[np.arange(len(cls)), cls]
  m = conf > conf_thres
  if road_only:
    m &= np.isin(cls, list(road))
  if not m.any():
    return []
  p, cls, conf = p[m], cls[m], conf[m]
  cx, cy, w, h = p[:, 0], p[:, 1] - pad_top(rgb), p[:, 2], p[:, 3]
  boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], 1)

  dets = []
  for c in np.unique(cls):
    idx = np.where(cls == c)[0]
    for k in _nms(boxes[idx], conf[idx], iou_thres):
      i = idx[k]
      box = [round(float(v), 1) for v in boxes[i]]
      out_cls = C4_TO_COCO[int(c)] if c4 else int(c)
      d = {'cls': out_cls, 'name': names[c], 'conf': round(float(conf[i]), 3), 'box': box}
      if out_cls == TRAFFIC_LIGHT:
        # the trained model says which lamp is lit; COCO only says there is a light there
        d['light'] = BAND_LIGHT[int(c) - 7] if c4 else light_colour(rgb, box)
      dets.append(d)
  if drop_riders:
    twos = [d['box'] for d in dets if d['cls'] in (1, 3)]
    if twos:
      dets = [d for d in dets if d['cls'] != 0 or not any(_is_rider(d['box'], m) for m in twos)]
  dets.sort(key=lambda d: -d['conf'])
  return dets


def draw(rgb, dets, header=None):
  """Overlay boxes and return JPEG bytes for the HUD."""
  from PIL import Image, ImageDraw
  img = Image.fromarray(rgb)
  d = ImageDraw.Draw(img)
  for det in dets:
    x1, y1, x2, y2 = det['box']
    colour = COLORS.get(det['cls'], (200, 200, 200))
    if det.get('light'):
      colour = {'red': (255, 40, 40), 'amber': (255, 190, 0), 'green': (0, 230, 120)}[det['light']]
    d.rectangle([x1, y1, x2, y2], outline=colour, width=2)
    label = det.get('light') or det['name']
    d.text((x1 + 2, max(0, y1 - 11)), f"{label} {det['conf']:.2f}", fill=colour)
  if header:
    d.rectangle([0, 0, img.width, 14], fill=(0, 0, 0))
    d.text((3, 3), header, fill=(255, 255, 255))
  out = io.BytesIO()
  img.save(out, 'JPEG', quality=60)
  return out.getvalue()
