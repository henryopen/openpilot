#!/usr/bin/env python3
"""Run the detector over one recorded segment, writing what yolod's /det.json would have said.

Lets a drive's HUD be rebuilt offline: mk_playback.py supplies the road and the leads,
this supplies the detections that go on top of them.

  ./yolo_seg.py /data/media/0/realdata/<route>--<seg> 25 /tmp/yolo.json
"""
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, '/data/yolo')
sys.path.insert(0, '/data/openpilot')
import yolo_core as yc
from openpilot.tools.lib.logreader import LogReader

seg_dir = Path(sys.argv[1])
every = int(sys.argv[2])
out = Path(sys.argv[3])
W, H = 672, 380
FRAME_BYTES = W * H * 3

# the calibration this drive actually ran with, not today's: distance scales directly with it
pitch = yaw = 0.0
height = yc.CAM_HEIGHT
for m in LogReader(str(seg_dir / 'rlog.zst')):
  if m.which() == 'extrinsicsCalibration':
    e = m.extrinsicsCalibration
    rpy = list(e.rpyCalib)
    h = list(e.height)
    if len(rpy) == 3:
      pitch, yaw = float(rpy[1]), float(rpy[2])
    if h:
      height = float(h[0])
    break
print(f'calib pitch={pitch:.5f} yaw={yaw:.5f} height={height:.3f}', flush=True)

sess = yc.make_session('/data/yolo/yolo11n.onnx', 2)
# nearest-neighbour to match what yolod gets off the half-resolution VisionIPC path
cmd = ['ffmpeg', '-v', 'error', '-i', str(seg_dir / 'fcamera.hevc'), '-vf', f'scale={W}:{H}',
       '-sws_flags', 'neighbor', '-pix_fmt', 'rgb24', '-f', 'rawvideo', '-']
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=FRAME_BYTES * 2)
recs = []
i = 0
t0 = time.monotonic()
while True:
  raw = proc.stdout.read(FRAME_BYTES)
  if len(raw) < FRAME_BYTES:
    break
  if i % every == 0:
    rgb = np.frombuffer(raw, np.uint8).reshape(H, W, 3)
    dets = yc.detect(sess, rgb)
    counts = {}
    for d in dets:
      counts[d['name']] = counts.get(d['name'], 0) + 1
      if d['cls'] != yc.TRAFFIC_LIGHT:      # lights hang above the road, so the ground plane says nothing
        g = yc.ground_xy(d['box'], pitch, yaw, height, H)
        if g:
          d['d'], d['lat'] = g
    brief = [{k: v for k, v in d.items() if k in ('cls', 'conf', 'd', 'lat', 'light')}
             for d in dets if 'd' in d or d.get('light')]
    recs.append({'frame': i, 'ok': True, 'counts': counts,
                 'lights': [d['light'] for d in dets if d.get('light')], 'dets': brief})
  i += 1
proc.stdout.close()
proc.terminate()
json.dump(recs, open(out, 'w'))
print(f'{len(recs)} detections over {i} frames in {time.monotonic() - t0:.0f}s -> {out}')
