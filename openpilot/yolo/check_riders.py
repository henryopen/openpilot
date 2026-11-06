#!/usr/bin/env python3
"""check_riders: how many 'person' boxes are riders, and does dropping them lose real people.

Run against a recorded segment. Saves overlays of the frames where something was dropped,
with the dropped rider outlined so the call can be judged by eye rather than by count.

  ./check_riders.py <segment-dir> [--every 20] [--limit 40]
"""
import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, '/data/yolo/pylibs')

import yolo_core as yc

W, H = 672, 380
FRAME = W * H * 3


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('segment')
  ap.add_argument('--every', type=int, default=20)
  ap.add_argument('--limit', type=int, default=40)
  ap.add_argument('--out', default='/data/yolo/riders')
  ap.add_argument('--model', default='/data/yolo/yolo11n.onnx')
  args = ap.parse_args()

  seg = Path(args.segment)
  hevc = seg / 'fcamera.hevc' if seg.is_dir() else seg
  out = Path(args.out)
  out.mkdir(parents=True, exist_ok=True)
  sess = yc.make_session(args.model, 2)

  cmd = ['ffmpeg', '-v', 'error', '-i', str(hevc), '-vf', f'scale={W}:{H}',
         '-sws_flags', 'neighbor', '-pix_fmt', 'rgb24', '-f', 'rawvideo', '-']
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=FRAME * 2)

  n = people = riders = twos = saved = 0
  i = 0
  try:
    while n < args.limit:
      raw = p.stdout.read(FRAME)
      if len(raw) < FRAME:
        break
      if i % args.every == 0:
        rgb = np.frombuffer(raw, np.uint8).reshape(H, W, 3)
        alld = yc.detect(sess, rgb, drop_riders=False)
        kept = yc.detect(sess, rgb, drop_riders=True)
        pa = [d for d in alld if d['cls'] == 0]
        pk = [d for d in kept if d['cls'] == 0]
        people += len(pa)
        riders += len(pa) - len(pk)
        twos += len([d for d in alld if d['cls'] in (1, 3)])
        if len(pa) > len(pk) and saved < 6:
          # mark what was dropped so it can be checked by eye
          dropped = [d for d in pa if d not in pk]
          for d in dropped:
            d = dict(d, name='RIDER', cls=11)
            kept = kept + [d]
          hdr = f'f{i}  kept {len(pk)} people, dropped {len(pa) - len(pk)} riders'
          (out / f'r{i:05d}.jpg').write_bytes(yc.draw(rgb, kept, hdr))
          saved += 1
        n += 1
      i += 1
  finally:
    p.stdout.close()
    p.terminate()

  print(f'{n} frames: {people} person boxes, {riders} of them riders ({twos} two-wheelers)')
  print(f'{people - riders} people kept; {saved} overlays in {out}')


if __name__ == '__main__':
  main()
