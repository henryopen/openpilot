"""How far away is that traffic light, when nothing in the log says where it is?

Runs on Windows against F:\\realdata, which is comma three footage recorded in Taiwan. The
question behind it is how many pixels a traffic light gets on the comma three road camera,
and the honest way to answer that is to measure a light whose distance is known. Nothing in
the log knows where a light is, so the distance has to come out of the measurement itself.

A lamp of fixed size subtends pixels in inverse proportion to its range, so

    1 / d_px  =  (Z_stop + s) / (f * D)

where s is the distance still to be driven before the car comes to rest and Z_stop is
whatever gap is left between the resting car and the light. Regressing 1/d_px on s gives
f*D from the slope and Z_stop from the intercept, with no prior on either. The lamp diameter
that falls out is the check: if it does not land near the 0.30 m a Taiwanese lamp actually
is, the focal length or the tracking is wrong and the numbers built on top of it are worth
nothing.

Distance still to drive is the speed integral from each frame to the first frame under
0.3 m/s, taken over carState. Frames are tied to that clock through narrowRoadCameraState -
the message is not called roadCameraState in these logs, and there are 1200 of them to the
segment.

The lamp is tracked backwards from the frame where it is largest, because a saturated core
close up is easy to lock onto and the window can then follow it out to where it is four
pixels wide. Tracking forwards from the far end finds tail lights instead: they are the same
colour, they are brighter, and there are more of them. Restricting the search by height in
the frame does not separate them either, which was tried first - a light on a mast arm and
the brake lights of a lorry sit at the same height.

    python openpilot/yolo/analysis/light_selfcal_20260905.py 0000000b--37d20200db--35
"""

import os
import subprocess
import sys

import capnp
import cv2
import numpy as np

W, H = 1928, 1208
FOCAL = 2648.0                      # ar0231 road camera, common/transformations/camera.py
DATA = os.environ.get("REALDATA", "F:/realdata")
SCHEMA = os.environ.get("OP_SCHEMA", "F:/c4sunny/schema_hcop")
STOPPED = 0.3                       # m/s


def load_schema():
  capnp.remove_import_hook()
  os.chdir(SCHEMA)
  return capnp.load("log.capnp", imports=[SCHEMA])


def read_log(log, seg):
  """carState speeds and the camera clock, from an uncompressed rlog."""
  data = open(f"{DATA}/{seg}/rlog", "rb").read()
  pump = iter(log.Event.read_multiple_bytes(data))
  speed, cam = [], []
  while True:
    try:
      e = next(pump)
    except StopIteration:
      break
    except Exception:
      break                         # every route truncates its last segment
    try:
      w = e.which()
    except Exception:
      continue                      # message types this schema predates
    if w == "carState":
      speed.append((e.logMonoTime, e.carState.vEgo))
    elif w == "narrowRoadCameraState":
      cam.append((e.narrowRoadCameraState.frameId, e.narrowRoadCameraState.timestampSof))
  return np.array(speed, dtype=float), np.array(cam, dtype=float)


def distance_to_stop(speed, cam):
  """Metres still to drive, per video frame index, up to the moment of rest."""
  resting = np.where(speed[:, 1] < STOPPED)[0]
  if not len(resting):
    raise SystemExit("no stop in this segment")
  mono_stop = speed[resting[0], 0]
  first_frame = cam[0, 0]
  out = {}
  for frame_id, sof in cam:
    if sof > mono_stop:
      continue
    m = (speed[:, 0] >= sof) & (speed[:, 0] <= mono_stop)
    if m.sum() < 2:
      out[int(frame_id - first_frame)] = 0.0
    else:
      out[int(frame_id - first_frame)] = float(np.trapezoid(speed[m, 1], (speed[m, 0] - sof) / 1e9))
  return out


def frames(seg, upto):
  proc = subprocess.Popen(
    ["ffmpeg", "-v", "error", "-i", f"{DATA}/{seg}/fcamera.hevc",
     "-f", "rawvideo", "-pix_fmt", "bgr24", "-"], stdout=subprocess.PIPE)
  size = W * H * 3
  held = {}
  for i in range(upto + 1):
    buf = proc.stdout.read(size)
    if len(buf) < size:
      break
    held[i] = np.frombuffer(buf, np.uint8).reshape(H, W, 3).copy()
  proc.stdout.close()
  proc.kill()
  return held


def core(win):
  """The saturated part of the lamp, which is what stays measurable at range."""
  b, _, r = win[..., 0].astype(int), win[..., 1], win[..., 2].astype(int)
  mask = ((r > 215) & (r - b > 40)).astype(np.uint8)
  n, _, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
  best = None
  for k in range(1, n):
    if stats[k][4] >= 2 and (best is None or stats[k][4] > best[0][4]):
      best = (stats[k], cent[k])
  return best


def track(held, seed, first, last):
  """Follow the lamp from the near end outwards; returns {frame: equivalent diameter}."""
  cx, cy = seed
  out = {}
  for n in range(last, first - 1, -1):
    if n not in held:
      continue
    x0, x1 = max(0, cx - 60), min(W, cx + 60)
    y0, y1 = max(0, cy - 45), min(H, cy + 45)
    hit = core(held[n][y0:y1, x0:x1])
    if hit is None:
      continue
    stats, cent = hit
    nx, ny = int(x0 + cent[0]), int(y0 + cent[1])
    if abs(nx - cx) < 25 and abs(ny - cy) < 20:
      cx, cy = nx, ny
    out[n] = 2 * np.sqrt(stats[4] / np.pi)
  return out


def main():
  seg = sys.argv[1]
  seed = (int(sys.argv[2]), int(sys.argv[3])) if len(sys.argv) > 3 else (1012, 338)
  first, last = 560, 790            # the approach, short of the frames where the lamp blooms

  log = load_schema()
  speed, cam = read_log(log, seg)
  to_stop = distance_to_stop(speed, cam)
  sizes = track(frames(seg, last), seed, first, last)

  s = np.array([to_stop[n] for n in sorted(sizes) if n in to_stop])
  d = np.array([sizes[n] for n in sorted(sizes) if n in to_stop])
  keep = d > 2.7                    # below this the core is a couple of pixels of noise
  s, d = s[keep], d[keep]

  A = np.vstack([s, np.ones_like(s)]).T
  (slope, intercept), *_ = np.linalg.lstsq(A, 1.0 / d, rcond=None)
  k = 1.0 / slope
  fitted = A @ [slope, intercept]
  r2 = 1 - ((1.0 / d - fitted) ** 2).sum() / ((1.0 / d - (1.0 / d).mean()) ** 2).sum()

  print(f"{seg}: {len(s)} frames")
  print(f"  lamp diameter      {k / FOCAL:.3f} m      (a Taiwanese lamp is 0.30 m)")
  print(f"  light beyond stop  {intercept * k:.1f} m")
  print(f"  R2                 {r2:.3f}   residual {np.sqrt(((1 / fitted - d) ** 2).mean()):.2f} px")
  print()
  print(f"  {'range m':>9} {'measured px':>12} {'fitted px':>10}")
  for i in range(0, len(s), max(1, len(s) // 12)):
    print(f"  {s[i] + intercept * k:9.1f} {d[i]:12.1f} {1 / fitted[i]:10.1f}")


if __name__ == "__main__":
  main()
