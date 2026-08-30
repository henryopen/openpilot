#!/usr/bin/env python3
"""summarize: read back a drive's detections.

Splits by speed band, because the open question is how the detector behaves at highway
speeds where the lead is far away and small - the earlier radar work only ever covered
city driving, so that band has no data yet.

  ./summarize.py [run-dir]        # defaults to the newest run
  ./summarize.py --list
"""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

RUNS = Path('/data/yolo/runs')
BANDS = [(0, 5, '靜止'), (5, 40, '市區慢速'), (40, 70, '市區/郊區'), (70, 95, '快速道路'), (95, 999, '高速')]


def band_of(v):
  for lo, hi, name in BANDS:
    if lo <= v < hi:
      return name
  return '其他'


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('run', nargs='?')
  ap.add_argument('--list', action='store_true')
  ap.add_argument('--root', default=str(RUNS))
  args = ap.parse_args()

  root = Path(args.root)
  runs = sorted(p for p in root.iterdir() if (p / 'det.jsonl').exists()) if root.exists() else []
  if args.list or not runs:
    for p in runs:
      n = sum(1 for _ in (p / 'det.jsonl').open())
      print(f'{p.name}  {n} 幀  {len(list(p.glob("*.jpg")))} 圖  {sum(f.stat().st_size for f in p.iterdir()) / 1e6:.1f} MB')
    if not runs:
      print(f'no runs under {root}')
    return

  run = Path(args.run) if args.run else runs[-1]
  rows = [json.loads(ln) for ln in (run / 'det.jsonl').open() if ln.strip()]
  if not rows:
    print(f'{run.name}: empty')
    return

  span = rows[-1]['t'] - rows[0]['t']
  ms = sorted(r['ms'] for r in rows)
  print(f'== {run.name} ==')
  print(f'{len(rows)} 幀 / {span / 60:.1f} 分鐘  ({len(rows) / max(span, 1):.2f} Hz)')
  print(f'推論耗時 中位 {ms[len(ms) // 2]:.0f} ms  p90 {ms[int(len(ms) * .9)]:.0f} ms  最大 {ms[-1]:.0f} ms')
  print(f'車速 最高 {max(r["v"] for r in rows):.0f} km/h')

  by_band = defaultdict(list)
  for r in rows:
    by_band[band_of(r['v'])].append(r)

  print('\n速度分段：')
  for _, _, name in BANDS:
    rs = by_band.get(name)
    if not rs:
      continue
    c = Counter()
    for r in rs:
      c.update(r['counts'])
    frames_with = Counter()
    for r in rs:
      frames_with.update(set(r['counts']))
    pct = {k: f'{100 * v / len(rs):.0f}%' for k, v in frames_with.most_common(5)}
    share = 100 * len(rs) / len(rows)
    print(f'  {name:<10} {len(rs):>5} 幀 ({share:>4.1f}%)  每幀物件 {sum(c.values()) / len(rs):.1f}  出現率 {pct}')

  lights = Counter()
  for r in rows:
    lights.update(r.get('lights') or [])
  print(f'\n紅綠燈判讀：{dict(lights) or "無"}')

  total = Counter()
  for r in rows:
    total.update(r['counts'])
  print(f'物件總計：{dict(total.most_common())}')

  # the frames most worth looking at by eye
  def score(r):
    return sum(3 if d['cls'] in (0, 1, 3, 9) else 1 for d in r['dets'])

  print('\n最值得看的幀（有存圖的話在同目錄）：')
  for r in sorted(rows, key=score, reverse=True)[:8]:
    names = ' '.join(f"{d.get('light') or d['name']}:{d['conf']:.2f}" for d in r['dets'][:6])
    print(f'  t+{r["t"] - rows[0]["t"]:>6.0f}s  {r["v"]:>5.1f} km/h  {names}')

  jpgs = sorted(run.glob('*.jpg'))
  print(f'\n存圖 {len(jpgs)} 張 in {run}')


if __name__ == '__main__':
  main()
