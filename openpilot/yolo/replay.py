#!/usr/bin/env python3
"""replay: run the detector over a recorded fcamera.hevc, to check accuracy and cost offroad.

Decodes with nearest-neighbour scaling to 672x380 so the frames match what yolod gets from
the half-resolution VisionIPC path, rather than a nicer-looking resize that would flatter
the results.

  ./replay.py <segment-dir-or-hevc> [--every N] [--out DIR] [--limit N]
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import yolo_core as yc

W, H = 672, 380
FRAME_BYTES = W * H * 3


def frames(hevc, every):
  cmd = ['ffmpeg', '-v', 'error', '-i', str(hevc), '-vf', f'scale={W}:{H}',
         '-sws_flags', 'neighbor', '-pix_fmt', 'rgb24', '-f', 'rawvideo', '-']
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=FRAME_BYTES * 2)
  i = 0
  try:
    while True:
      raw = p.stdout.read(FRAME_BYTES)
      if len(raw) < FRAME_BYTES:
        return
      if i % every == 0:
        yield i, np.frombuffer(raw, np.uint8).reshape(H, W, 3)
      i += 1
  finally:
    p.stdout.close()
    p.terminate()


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('source')
  ap.add_argument('--every', type=int, default=10, help='detect every Nth frame (camera is 20 fps)')
  ap.add_argument('--out', default='/data/yolo/replay_out')
  ap.add_argument('--limit', type=int, default=0)
  ap.add_argument('--conf', type=float, default=0.35)
  ap.add_argument('--model', default='/data/yolo/yolo11n.onnx')
  ap.add_argument('--threads', type=int, default=2)
  ap.add_argument('--save-every', type=int, default=5, help='write an overlay jpeg every Nth detection')
  args = ap.parse_args()

  src = Path(args.source)
  hevc = src / 'fcamera.hevc' if src.is_dir() else src
  out = Path(args.out)
  out.mkdir(parents=True, exist_ok=True)

  sess = yc.make_session(args.model, args.threads)
  print(f'model {args.model} threads={args.threads}  source {hevc}', flush=True)

  times, counts, n = [], {}, 0
  lights = {}
  jl = (out / 'det.jsonl').open('w')
  t_start = time.monotonic()
  for idx, rgb in frames(hevc, args.every):
    t = time.monotonic()
    dets = yc.detect(sess, rgb, conf_thres=args.conf)
    ms = (time.monotonic() - t) * 1000
    times.append(ms)
    for d in dets:
      counts[d['name']] = counts.get(d['name'], 0) + 1
      if d.get('light'):
        lights[d['light']] = lights.get(d['light'], 0) + 1
    jl.write(json.dumps({'frame': idx, 'ms': round(ms, 1), 'dets': dets}) + '\n')
    if n % args.save_every == 0:
      hdr = f'f{idx} {ms:.0f}ms ' + ' '.join(f"{d.get('light') or d['name']}:{d['conf']:.2f}" for d in dets[:5])
      (out / f'f{idx:05d}.jpg').write_bytes(yc.draw(rgb, dets, hdr))
    n += 1
    if args.limit and n >= args.limit:
      break
  jl.close()

  t = np.array(times)
  print(f'\n{n} detections over {time.monotonic() - t_start:.1f}s wall')
  if n:
    print(f'inference ms: median {np.median(t):.0f}  p90 {np.percentile(t, 90):.0f}  max {t.max():.0f}')
  print('objects:', dict(sorted(counts.items(), key=lambda kv: -kv[1])))
  print('lights :', lights)
  print('overlays in', out)


if __name__ == '__main__':
  main()
