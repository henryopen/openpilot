#!/usr/bin/env python3
"""verify_nv12: check nv12_to_rgb against ffmpeg on the same frame.

The live camera is dark in a garage, so a wrong red/blue swap would not show up on screen
but would break traffic-light colour. This decodes one recorded frame two ways - through
ffmpeg to rgb24, and through our NV12 path with the device's real stride/padding - and
reports the per-channel error.
"""
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import yolo_core as yc

W, H = 1344, 760
STRIDE, Y_ROWS = 1408, 768  # what camerad reports for this sensor


def ffmpeg_frame(hevc, fmt, frame=60):
  """The device's ffmpeg build rejects the select filter's escaping, so skip frames by reading."""
  size = W * H * 3 if fmt == 'rgb24' else W * H * 3 // 2
  cmd = ['ffmpeg', '-v', 'error', '-i', str(hevc), '-pix_fmt', fmt, '-f', 'rawvideo',
         '-frames:v', str(frame + 1), '-']
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=size)
  try:
    for _ in range(frame + 1):
      chunks, got = [], 0
      while got < size:
        b = p.stdout.read(size - got)
        if not b:
          raise RuntimeError(f'short read from ffmpeg ({fmt})')
        chunks.append(b)
        got += len(b)
    return b''.join(chunks)
  finally:
    p.stdout.close()
    p.terminate()


def main():
  seg = Path(sys.argv[1] if len(sys.argv) > 1 else '/data/media/0/realdata')
  hevc = seg / 'fcamera.hevc' if seg.is_dir() else seg
  ref = np.frombuffer(ffmpeg_frame(hevc, 'rgb24'), np.uint8).reshape(H, W, 3)[::2, ::2].astype(int)

  nv12 = np.frombuffer(ffmpeg_frame(hevc, 'nv12'), np.uint8)
  y, uv = nv12[:W * H].reshape(H, W), nv12[W * H:].reshape(H // 2, W)
  # rebuild the padded layout the device actually hands us
  buf = np.zeros(STRIDE * Y_ROWS + STRIDE * (H // 2), np.uint8)
  buf[:H * STRIDE].reshape(H, STRIDE)[:, :W] = y
  buf[STRIDE * Y_ROWS:].reshape(H // 2, STRIDE)[:, :W] = uv
  got = yc.nv12_to_rgb(SimpleNamespace(data=buf.tobytes(), width=W, height=H,
                                       stride=STRIDE, uv_offset=STRIDE * Y_ROWS)).astype(int)

  print(f'ffmpeg {ref.shape}  ours {got.shape}')
  if ref.shape != got.shape:
    sys.exit('shape mismatch')
  for i, ch in enumerate('RGB'):
    d = np.abs(ref[..., i] - got[..., i])
    print(f'  {ch}: mean abs err {d.mean():5.2f}   p99 {np.percentile(d, 99):5.1f}   max {d.max():3d}')
  swapped = np.abs(ref[..., 0] - got[..., 2]).mean()
  print(f'\nR-vs-our-B mean err {swapped:.2f} (should be much worse than R above if channels are right)')
  worst = max(np.abs(ref[..., i] - got[..., i]).mean() for i in range(3))
  print('VERDICT:', 'channels correct' if worst < swapped / 2 else 'CHANNELS LOOK WRONG')


if __name__ == '__main__':
  main()
