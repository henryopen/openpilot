#!/usr/bin/env python3
"""make_calib: sample frames from recorded drives to calibrate INT8 quantisation.

Static quantisation needs representative inputs, and the most representative ones available
are this car's own recordings on these roads, so frames are spread across many segments
rather than taken from one stretch.

  ./make_calib.py [--segments 12] [--per-segment 6] [--out /data/yolo/calib.npz]
"""
import argparse
import random
import subprocess
import sys
from pathlib import Path

import numpy as np

W, H = 672, 380
FRAME = W * H * 3


def sample(hevc, count, skip=40):
  """Take `count` frames spaced through the clip, decoded the same way yolod sees them."""
  cmd = ['ffmpeg', '-v', 'error', '-i', str(hevc), '-vf', f'scale={W}:{H}', '-sws_flags', 'neighbor',
         '-pix_fmt', 'rgb24', '-f', 'rawvideo', '-']
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=FRAME * 2)
  out, i = [], 0
  try:
    while len(out) < count:
      raw = p.stdout.read(FRAME)
      if len(raw) < FRAME:
        break
      if i % skip == 0:
        out.append(np.frombuffer(raw, np.uint8).reshape(H, W, 3))
      i += 1
  finally:
    p.stdout.close()
    p.terminate()
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--root', default='/data/media/0/realdata')
  ap.add_argument('--segments', type=int, default=12)
  ap.add_argument('--per-segment', type=int, default=6)
  ap.add_argument('--out', default='/data/yolo/calib.npz')
  args = ap.parse_args()

  segs = sorted(p for p in Path(args.root).iterdir() if (p / 'fcamera.hevc').exists())
  if not segs:
    sys.exit(f'no segments under {args.root}')
  random.seed(0)
  picked = random.sample(segs, min(args.segments, len(segs)))

  frames = []
  for s in picked:
    got = sample(s / 'fcamera.hevc', args.per_segment)
    frames += got
    print(f'{s.name}: {len(got)}', flush=True)

  arr = np.stack(frames)
  np.savez_compressed(args.out, frames=arr)
  print(f'\n{arr.shape} -> {args.out} ({Path(args.out).stat().st_size / 1e6:.1f} MB)')


if __name__ == '__main__':
  main()
