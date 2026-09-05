"""Is a traffic light further away on a comma three only because the sensor is wider?

Runs on Windows against F:\\realdata. The comma three road camera and the comma four road
camera do not differ in resolution the way they differ in reach: 1928x1208 at f=2648 against
1344x760 at f=1141.5, which is 40 degrees against 61. Scaling the pixel count gives 1.6x and
is wrong; the focal lengths give 2.32x and are what an object subtends.

What that is worth to detection cannot be read off the two numbers, so it is measured. The
same recording is shown to the same model at the same input size twice, and the only thing
that changes between them is angular scale:

  native    a 1344x768 window cut from the comma three frame, unscaled
  c4        the whole frame scaled by 1141.5/2648 and padded back out to 1344x768

Weather, dirt on the glass, the sensor's own colour rendering and the scene itself are
identical across the pair, so the difference in what comes back is angular resolution and
nothing else. The padding replicates the edge rather than filling grey, because a hard
border of its own is a feature the model will happily find.

Feeding the comma four model comma three frames is not a fair test of that model and is not
meant as one - the point is the ratio between the two columns, not either column's absolute
recall. Both bottom out at the same box height, which is what makes the ratio mean something.

    python openpilot/yolo/analysis/light_cam_ab_20260905.py \\
        00000048--2dd4029593--3 0000004e--97e876cb81--51 00000052--f3e9f612f3--30
"""

import os
import subprocess
import sys

import cv2
import numpy as np
import onnxruntime as ort

W, H = 1928, 1208
IN_W, IN_H = 1344, 768
F_C3, F_C4 = 2648.0, 1141.5
SCALE = F_C4 / F_C3
HOUSING = 0.35                      # m, a three aspect signal head
DATA = os.environ.get("REALDATA", "F:/realdata")
MODEL = os.environ.get("YOLO_MODEL", "c4light13_1344x760.onnx")
CONF, IOU = 0.20, 0.45
NAMES = ["person", "bicycle", "car", "motorcycle", "bus", "truck", "stop_sign",
         "light_red", "light_green", "light_yellow", "light_off", "light_pedestrian",
         "light_green_arrow"]


def detect(sess, name, frame):
  x = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None].astype(np.float32) / 255.0
  y = sess.run(None, {name: x})[0][0]
  cls = y[4:].T
  score, which = cls.max(1), cls.argmax(1)
  m = score > CONF
  if not m.sum():
    return []
  box, score, which = y[:4].T[m], score[m], which[m]
  xywh = np.stack([box[:, 0] - box[:, 2] / 2, box[:, 1] - box[:, 3] / 2, box[:, 2], box[:, 3]], 1)
  keep = cv2.dnn.NMSBoxes(xywh.tolist(), score.tolist(), CONF, IOU)
  return [(NAMES[int(which[i])], float(score[i]), xywh[i]) for i in np.array(keep).flatten()]


def native(img):
  return img[180:180 + IN_H, 292:292 + IN_W]


def c4_equivalent(img):
  small = cv2.resize(img, (int(W * SCALE), int(H * SCALE)), interpolation=cv2.INTER_AREA)
  h, w = small.shape[:2]
  top, left = (IN_H - h) // 2, (IN_W - w) // 2
  return cv2.copyMakeBorder(small, top, IN_H - h - top, left, IN_W - w - left, cv2.BORDER_REPLICATE)


def main():
  sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
  name = sess.get_inputs()[0].name
  views = (("native", native, F_C3), ("c4", c4_equivalent, F_C4))
  heights = {k: [] for k, _, _ in views}
  seen = {k: 0 for k, _, _ in views}
  sampled = 0

  for seg in sys.argv[1:]:
    proc = subprocess.Popen(
      ["ffmpeg", "-v", "error", "-i", f"{DATA}/{seg}/fcamera.hevc",
       "-f", "rawvideo", "-pix_fmt", "bgr24", "-"], stdout=subprocess.PIPE)
    size = W * H * 3
    i = 0
    while True:
      buf = proc.stdout.read(size)
      if len(buf) < size:
        break
      if i % 20 == 0:
        img = np.frombuffer(buf, np.uint8).reshape(H, W, 3)
        sampled += 1
        for key, view, _ in views:
          lights = [d for d in detect(sess, name, view(img)) if d[0].startswith("light")]
          seen[key] += bool(lights)
          heights[key] += [float(b[3]) for _, _, b in lights]
      i += 1
    proc.stdout.close()
    proc.kill()

  print(f"{sampled} frames sampled over {len(sys.argv) - 1} segments")
  for key, _, focal in views:
    h = np.array(heights[key])
    if not len(h):
      print(f"  {key:<8} nothing found")
      continue
    rng = focal * HOUSING / h
    box = f"box height min {h.min():.0f} median {np.median(h):.0f} px"
    reach = f"range median {np.median(rng):.0f} m furthest {rng.max():.0f} m"
    print(f"  {key:<8} {len(h):3d} lights over {seen[key]:3d} frames | {box} | {reach}")


if __name__ == "__main__":
  main()
