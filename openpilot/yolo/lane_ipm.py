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

import cv2
import numpy as np

# os04c10 narrow road camera, full resolution
FOCAL = 1141.5
CX, CY = 1344 / 2, 760 / 2
# measured on this car (CalibrationParams -> extrinsicsCalibration)
PITCH, YAW, HEIGHT = 0.00482, -0.01655, 1.339

# the patch of road we flatten, in metres
NEAR, FAR = 4.0, 34.0
HALF_WIDTH = 5.5
PX_PER_M = 22          # output resolution


def ground_to_image(d, lat, pitch=PITCH, yaw=YAW, height=HEIGHT):
  """A point on the road (d ahead, lat right) -> pixel in the camera frame."""
  v = CY - FOCAL * math.tan(pitch) + FOCAL * height / d
  u = CX + FOCAL * math.tan(yaw) + FOCAL * lat / d
  return u, v


def homography():
  """Map the road patch to a top-down image."""
  src, dst = [], []
  w = int(2 * HALF_WIDTH * PX_PER_M)
  h = int((FAR - NEAR) * PX_PER_M)
  for d, lat in ((NEAR, -HALF_WIDTH), (NEAR, HALF_WIDTH), (FAR, HALF_WIDTH), (FAR, -HALF_WIDTH)):
    src.append(ground_to_image(d, lat))
    # top of the output is far away, left of it is negative lat
    dst.append(((lat + HALF_WIDTH) * PX_PER_M, (FAR - d) * PX_PER_M))
  return cv2.getPerspectiveTransform(np.float32(src), np.float32(dst)), (w, h)


def flatten(img):
  M, (w, h) = homography()
  return cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('image')
  ap.add_argument('--out', default=None)
  args = ap.parse_args()

  img = cv2.imread(args.image)
  if img is None:
    raise SystemExit(f'cannot read {args.image}')
  if img.shape[1] != 1344:
    img = cv2.resize(img, (1344, 760))
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
