#!/usr/bin/env python3
"""verify_range: check the monocular distance estimate against the radar's own numbers.

The HUD is about to place icons by distance, so the distance has to be worth believing.
This replays a recorded segment, finds the frame each radarState lead belongs to via
roadCameraState's frameId, runs the detector on that frame, and compares the box-bottom
estimate with dRel for the car nearest the radar's lateral position.

  ./verify_range.py <segment-dir> [--samples 40]
"""
import argparse
import subprocess
from collections import defaultdict
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, '/data/yolo/pylibs')
sys.path.append('/data/openpilot')

import yolo_core as yc
from openpilot.tools.lib.logreader import LogReader

W, H = 672, 380
FRAME = W * H * 3
VEHICLES = {2, 3, 5, 7}  # car, motorcycle, bus, truck


def read_log(seg):
  """-> (frame index -> dRel/yRel).

  rlog on this device carries neither roadCameraState nor extrinsicsCalibration, so frames
  are matched by position: radarState runs at the camera's 20 Hz, one message per frame.
  """
  out = {}
  n = 0
  for msg in LogReader(str(seg / 'rlog.zst')):
    if msg.which() != 'radarState':
      continue
    r = msg.radarState.leadOne
    if r.present and r.radar:  # radar-matched leads only; vision-only ones prove nothing here
      out[n] = (float(r.dRel), -float(r.yRel))  # yRel is left-positive, ours is right-positive
    n += 1
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('segment')
  ap.add_argument('--samples', type=int, default=40)
  ap.add_argument('--model', default='/data/yolo/yolo11n.onnx')
  ap.add_argument('--pitch', type=float, help='override calibration pitch, to check its sign')
  args = ap.parse_args()

  seg = Path(args.segment)
  want = read_log(seg)
  pitch, yaw, height = yc.read_calibration()
  if args.pitch is not None:
    pitch = args.pitch
  print(f'radar leads on {len(want)} frames; pitch {pitch:.5f} rad, yaw {yaw:.5f}, height {height:.3f} m')
  if not want:
    sys.exit('no radar leads in this segment')

  # stratify by distance: sampling evenly over time lands entirely in whatever the car spent
  # its time doing, which for a commute is sitting in traffic at 4 m
  buckets = defaultdict(list)
  for f, (d, _) in want.items():
    buckets[int(d // 10)].append(f)
  wanted = []
  per = max(1, args.samples // max(1, len(buckets)))
  for b in sorted(buckets):
    fs = sorted(buckets[b])
    wanted += fs[::max(1, len(fs) // per)][:per]
  wanted = sorted(set(wanted))
  sess = yc.make_session(args.model, 2)

  cmd = ['ffmpeg', '-v', 'error', '-i', str(seg / 'fcamera.hevc'), '-vf', f'scale={W}:{H}',
         '-sws_flags', 'neighbor', '-pix_fmt', 'rgb24', '-f', 'rawvideo', '-']
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=FRAME * 2)

  rows = []
  todo = set(wanted)
  i = 0
  try:
    while todo:
      raw = p.stdout.read(FRAME)
      if len(raw) < FRAME:
        break
      if i in todo:
        todo.discard(i)
        rgb = np.frombuffer(raw, np.uint8).reshape(H, W, 3)
        d_radar, y_radar = want[i]
        best = None
        for det in yc.detect(sess, rgb, conf_thres=0.3):
          if det['cls'] not in VEHICLES:
            continue
          g = yc.ground_xy(det['box'], pitch, yaw, height)
          if g is None:
            continue
          # the radar's lead is a specific object; compare against the box nearest it laterally
          err = abs(g[1] - y_radar)
          if best is None or err < best[0]:
            best = (err, g, det)
        if best and best[0] < 2.5:
          _, (d, lat), det = best
          rows.append((d_radar, d, y_radar, lat, det['name']))
      i += 1
  finally:
    p.stdout.close()
    p.terminate()

  if not rows:
    sys.exit('no frames where a detection matched the radar lead')

  dr = np.array([r[0] for r in rows])
  dy = np.array([r[1] for r in rows])
  rel = (dy - dr) / dr * 100
  print(f'\n{len(rows)} matched frames')
  print(f'{"radar":>7} {"yolo":>7} {"err":>7} {"err%":>7}  lat radar/yolo   class')
  for r in sorted(rows):
    err = (r[1] - r[0]) / r[0] * 100
    print(f'{r[0]:7.1f} {r[1]:7.1f} {r[1] - r[0]:7.1f} {err:6.0f}%  {r[2]:5.1f}/{r[3]:5.1f}   {r[4]}')

  print(f'\n偏差中位 {np.median(rel):+.0f}%   平均絕對 {np.mean(np.abs(rel)):.0f}%')
  for lo, hi in [(0, 20), (20, 40), (40, 70), (70, 200)]:
    m = (dr >= lo) & (dr < hi)
    if m.sum():
      print(f'  {lo:>3}-{hi:<3} m: n={m.sum():<3} 中位偏差 {np.median(rel[m]):+.0f}%')
  # a single scale factor is what a wrong effective height would look like
  print(f'\n最佳縮放係數 {np.median(dr / dy):.3f}  (等效相機高度 {height * np.median(dr / dy):.2f} m)')


if __name__ == '__main__':
  main()
