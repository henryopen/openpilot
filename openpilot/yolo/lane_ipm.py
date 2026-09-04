#!/usr/bin/env python3
"""lane_ipm: flatten the road out of a camera frame, so lane markings can be measured.

Line type is the question - solid, dashed, double, and yellow versus white - and none of it
is answerable in the camera frame directly: perspective squeezes a dash from tens of pixels
to one over the same stretch of road, so "how long is the gap" has no fixed answer there.
On a flattened view a dash is the same length wherever it is, and a double line is two
bands a fixed distance apart.

Uses the same ground-plane projection as the detector's distance estimate, with this car's
calibration: f=1141.5 at 1344x760, pitch and height read from CalibrationParams.

  ./lane_ipm.py <image.jpg> [--out ipm.png]
"""
import argparse
import math

import numpy as np

# cv2 is not on the device - yolo_core does everything with numpy, and adding a dependency
# the driving stack would have to carry for a millisecond of work is the wrong trade. The
# projection and the resampling below are numpy; cv2 is imported inside main() for reading
# and writing files, which only happens off the car.

# os04c10 narrow road camera, full resolution
FOCAL = 1141.5
CX, CY = 1344 / 2, 760 / 2
# measured on this car (CalibrationParams -> extrinsicsCalibration)
PITCH, YAW, HEIGHT = 0.00482, -0.01655, 1.339

# the patch of road we flatten, in metres
NEAR, FAR = 4.0, 34.0
HALF_WIDTH = 5.5
PX_PER_M = 22          # output resolution


def ground_to_image(d, lat, pitch=PITCH, yaw=YAW, height=HEIGHT, scale=1.0):
  """A point on the road (d ahead, lat right) -> pixel in the camera frame.

  scale is the frame's width over the 1344 these intrinsics were measured at. The detector
  reads the camera at half resolution, and the same road point has to land on the pixel that
  actually holds it, whichever frame it is being asked about.
  """
  f, cx, cy = FOCAL * scale, CX * scale, CY * scale
  v = cy - f * math.tan(pitch) + f * height / d
  u = cx + f * math.tan(yaw) + f * lat / d
  return u, v


def perspective_transform(src, dst):
  """The 3x3 taking the four src corners onto the four dst ones.

  Eight unknowns - the ninth entry is fixed at 1, since the matrix only matters up to scale -
  and each corner gives two equations.
  """
  rows, rhs = [], []
  for (x, y), (u, v) in zip(src, dst, strict=True):
    rows.append([x, y, 1, 0, 0, 0, -u * x, -u * y])
    rhs.append(u)
    rows.append([0, 0, 0, x, y, 1, -v * x, -v * y])
    rhs.append(v)
  h = np.linalg.solve(np.array(rows, np.float64), np.array(rhs, np.float64))
  return np.append(h, 1.0).reshape(3, 3)


def warp(img, M, w, h, rows=None):
  """Resample img onto a w x h grid through M, bilinearly.

  Runs backwards - each output pixel is asked where it came from - so the output has no gaps,
  which is what forward mapping would leave wherever the road stretches.

  rows limits the work to the output lines a caller is going to read; the rest comes back
  black. Without cv2's SIMD behind it this is the difference between resampling the whole
  4 to 34 m patch and only the few metres the markings are measured over.
  """
  r0, r1 = rows if rows else (0, h)
  r0, r1 = max(0, int(r0)), min(h, int(r1))
  ys, xs = np.mgrid[r0:r1, 0:w].astype(np.float64)
  Mi = np.linalg.inv(M)
  den = Mi[2, 0] * xs + Mi[2, 1] * ys + Mi[2, 2]
  den[den == 0] = 1e-9
  sx = (Mi[0, 0] * xs + Mi[0, 1] * ys + Mi[0, 2]) / den
  sy = (Mi[1, 0] * xs + Mi[1, 1] * ys + Mi[1, 2]) / den

  H, W = img.shape[:2]
  x0 = np.floor(sx).astype(np.int32)
  y0 = np.floor(sy).astype(np.int32)
  inside = (x0 >= 0) & (x0 < W - 1) & (y0 >= 0) & (y0 < H - 1)
  xc = np.clip(x0, 0, W - 2)
  yc = np.clip(y0, 0, H - 2)
  fx = (sx - x0)[..., None]
  fy = (sy - y0)[..., None]
  im = img.astype(np.float32)
  top = im[yc, xc] * (1 - fx) + im[yc, xc + 1] * fx
  bot = im[yc + 1, xc] * (1 - fx) + im[yc + 1, xc + 1] * fx
  band = top * (1 - fy) + bot * fy
  band[~inside] = 0
  if rows is None:
    return band.astype(np.uint8)
  out = np.zeros((h, w, img.shape[2]), np.uint8)
  out[r0:r1] = band.astype(np.uint8)
  return out


def homography(scale=1.0, calib=None):
  """Map the road patch to a top-down image."""
  pitch, yaw, height = calib if calib else (PITCH, YAW, HEIGHT)
  src, dst = [], []
  w = int(2 * HALF_WIDTH * PX_PER_M)
  h = int((FAR - NEAR) * PX_PER_M)
  for d, lat in ((NEAR, -HALF_WIDTH), (NEAR, HALF_WIDTH), (FAR, HALF_WIDTH), (FAR, -HALF_WIDTH)):
    src.append(ground_to_image(d, lat, pitch, yaw, height, scale))
    # top of the output is far away, left of it is negative lat
    dst.append(((lat + HALF_WIDTH) * PX_PER_M, (FAR - d) * PX_PER_M))
  return perspective_transform(src, dst), (w, h)


def flatten(img, calib=None, rows=None):
  """calib is this car's live (pitch, yaw, height); without it the values measured here."""
  M, (w, h) = homography(img.shape[1] / 1344, calib)
  return warp(img, M, w, h, rows)


def main():
  import cv2      # off-car only: reading and writing image files
  ap = argparse.ArgumentParser()
  ap.add_argument('image')
  ap.add_argument('--out', default=None)
  args = ap.parse_args()

  img = cv2.imread(args.image)
  if img is None:
    raise SystemExit(f'cannot read {args.image}')
  top = flatten(img)

  # grid every metre laterally and every 5 m ahead, to check the geometry by eye
  marked = top.copy()
  for lat in range(-int(HALF_WIDTH), int(HALF_WIDTH) + 1):
    x = int((lat + HALF_WIDTH) * PX_PER_M)
    cv2.line(marked, (x, 0), (x, marked.shape[0]), (0, 90, 0), 1)
  for d in range(int(NEAR), int(FAR) + 1, 5):
    y = int((FAR - d) * PX_PER_M)
    cv2.line(marked, (0, y), (marked.shape[1], y), (0, 90, 0), 1)
    cv2.putText(marked, f'{d}m', (3, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 0), 1)

  out = args.out or args.image.rsplit('.', 1)[0] + '_ipm.png'
  cv2.imwrite(out, marked)
  print(f'{img.shape[1]}x{img.shape[0]} -> {top.shape[1]}x{top.shape[0]}  ({PX_PER_M} px/m)')
  print(f'covers {NEAR}-{FAR} m ahead, +/-{HALF_WIDTH} m across')
  print(f'wrote {out}')


if __name__ == '__main__':
  main()
