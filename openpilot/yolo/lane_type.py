#!/usr/bin/env python3
"""lane_type: read the markings off a flattened road view - solid or dashed, single or
double, yellow or white.

None of this exists anywhere in openpilot: modelV2 carries lane line geometry and its
confidence, and nothing about what the line means. Whether you may cross it is not in the
data, so it has to come from the picture.

Works on the flattened view rather than the camera frame, because there a dash is the same
length wherever it is and two lines of a double are a fixed distance apart. Deliberately
not a neural network: the lines are high contrast paint on grey asphalt at a known place on
the ground, which is the case classical vision is good at, and it costs a few milliseconds
instead of competing with modeld for the CPU.

  ./lane_type.py <image.jpg> [--debug out.png]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lane_ipm import FAR, HALF_WIDTH, NEAR, PX_PER_M, flatten

MIN_LINE_W_M, MAX_LINE_W_M = 0.06, 0.35
DOUBLE_GAP_MAX_M = 0.55      # two bands closer than this are one double line
# Past about 16 m the source pixels run out and the flattened view smears, which reads as
# false gaps in a solid line. Measure where the picture actually is.
NEAR_BAND_M = (NEAR + 1, 16)
# Urban dashes here are roughly 4 m of paint to 6 m of gap. Anything under this is smear,
# a join, or a patch in the asphalt - not a dash.
DASH_GAP_MIN_M, DASH_GAP_MAX_M = 2.5, 12.0
# How far either side of a line modelV2 reports to look for the paint. Wide enough for the
# far band of a double and for the model being a little off, narrow enough to leave out the
# next line over.
LANE_SEARCH_M = 0.7


def _columns(top):
  """Brightness of each lateral position, over the near part of the view."""
  y0 = int((FAR - NEAR_BAND_M[1]) * PX_PER_M)
  y1 = int((FAR - NEAR_BAND_M[0]) * PX_PER_M)
  band = top[y0:y1]
  grey = cv2.cvtColor(band, cv2.COLOR_BGR2GRAY).astype(np.float32)
  # paint is brighter than the asphalt around it, but the asphalt itself shades unevenly,
  # so measure each column against a local average a lane-marking-width wider than itself
  prof = grey.mean(axis=0)
  k = int(1.2 * PX_PER_M) | 1
  base = np.convolve(np.pad(prof, k // 2, mode='edge'), np.ones(k) / k, mode='valid')
  return band, prof - base[:len(prof)]


def find_lines(top, lane_x_m=None):
  """-> list of (x_px, width_px) for each painted band across the road.

  lane_x_m confines the search to the lines modelV2 has already found. Across the full
  width anything brighter than the asphalt qualifies - a shadow edge, a tar seam, paint
  in the oncoming lane - and those are exactly the bands that get read wrong, because
  nothing is holding them to a place a lane line can be.
  """
  _, contrast = _columns(top)
  thr = max(6.0, float(np.percentile(contrast, 97)) * 0.45)
  above = contrast > thr
  if lane_x_m:
    near_a_line = np.zeros_like(above)
    for x_m in lane_x_m:
      a = int((x_m - LANE_SEARCH_M + HALF_WIDTH) * PX_PER_M)
      b = int((x_m + LANE_SEARCH_M + HALF_WIDTH) * PX_PER_M)
      near_a_line[max(0, a):max(0, b)] = True
    above = above & near_a_line
  runs, start = [], None
  for i, a in enumerate(above):
    if a and start is None:
      start = i
    elif not a and start is not None:
      runs.append((start, i))
      start = None
  if start is not None:
    runs.append((start, len(above)))
  out = []
  for a, b in runs:
    w_m = (b - a) / PX_PER_M
    if MIN_LINE_W_M <= w_m <= MAX_LINE_W_M:
      out.append(((a + b) / 2, b - a))
  return out


def classify(top, x_px, w_px):
  """Colour and continuity of the band at this lateral position."""
  y0 = int((FAR - NEAR_BAND_M[1]) * PX_PER_M)
  y1 = int((FAR - NEAR_BAND_M[0]) * PX_PER_M)
  half = max(1, int(w_px / 2))
  x0, x1 = int(max(0, x_px - half)), int(min(top.shape[1], x_px + half + 1))
  strip = top[y0:y1, x0:x1]
  if strip.size == 0:
    return None

  b, g, r = (strip[..., i].astype(np.float32) for i in range(3))
  # yellow paint: red and green high, blue clearly lower. White: all three together.
  yellowness = float(np.mean((r + g) / 2 - b))
  bright = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY).astype(np.float32).mean(axis=1)

  # a dash gives a run of dark between runs of paint; a solid line does not
  lit = bright > (bright.min() + bright.max()) / 2
  gaps, run = [], 0
  for v in lit:
    if not v:
      run += 1
    elif run:
      gaps.append(run / PX_PER_M)
      run = 0
  if run:
    gaps.append(run / PX_PER_M)
  duty = float(lit.mean())
  dash_gaps = [g for g in gaps if DASH_GAP_MIN_M <= g <= DASH_GAP_MAX_M]
  return {
    'lat_m': round(x_px / PX_PER_M - HALF_WIDTH, 2),
    'width_m': round(w_px / PX_PER_M, 2),
    'yellowness': round(yellowness, 1),
    'colour': 'yellow' if yellowness > 18 else 'white',
    'duty': round(duty, 2),
    'dashed': bool(dash_gaps),
    'gaps_m': [round(g, 1) for g in dash_gaps],
    'contrast': round(float(bright.max() - bright.min()), 1),
  }


def group_doubles(lines):
  """Bands closer together than a lane's worth are one marking."""
  groups, cur = [], []
  for x, w in sorted(lines):
    if cur and (x - cur[-1][0]) / PX_PER_M <= DOUBLE_GAP_MAX_M:
      cur.append((x, w))
    else:
      if cur:
        groups.append(cur)
      cur = [(x, w)]
  if cur:
    groups.append(cur)
  return groups


def read_markings(img, lane_x_m=None, calib=None):
  """-> (flattened view, markings). lane_x_m: lateral metres of modelV2's laneLines."""
  top = flatten(img, calib)
  out = []
  for grp in group_doubles(find_lines(top, lane_x_m)):
    parts = [classify(top, x, w) for x, w in grp]
    parts = [p for p in parts if p]
    if not parts:
      continue
    # The two bands of a double measure differently - 19.0 and 13.7 of yellowness across the
    # same line - so the marking is yellow if either band is, not if the first one happens
    # to be the one that reads high.
    colour = 'yellow' if any(p['colour'] == 'yellow' for p in parts) else 'white'
    kind = 'double' if len(parts) > 1 else 'single'
    dashed = [p for p in parts if p['dashed']]
    if not dashed:
      style = 'solid'
    elif len(dashed) == len(parts):
      style = 'dashed'
    else:
      style = 'mixed'      # a solid beside a dashed: crossable from the dashed side only
    out.append({
      'lat_m': parts[0]['lat_m'],
      'kind': kind,
      'style': style,
      'colour': colour,
      'crossable': not (kind == 'double' or colour == 'yellow') and style == 'dashed',
      'parts': parts,
    })
  return top, out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('image')
  ap.add_argument('--debug')
  ap.add_argument('--lanes', help='lateral metres of known lane lines, e.g. "-1.9,1.9"')
  args = ap.parse_args()
  img = cv2.imread(args.image)
  if img is None:
    raise SystemExit(f'cannot read {args.image}')

  lane_x_m = [float(v) for v in args.lanes.split(',')] if args.lanes else None
  top, marks = read_markings(img, lane_x_m)
  lo, hi = NEAR_BAND_M
  where = f'，只看 {args.lanes} m 附近' if lane_x_m else ''
  print(f'{Path(args.image).name} {img.shape[1]}x{img.shape[0]}：找到 {len(marks)} 條標線（量測範圍 {lo}-{hi} m{where}）')
  print()
  for m in marks:
    tag = '可跨越' if m['crossable'] else '不可跨越'
    print(f"  橫向 {m['lat_m']:+5.2f} m  {m['colour']:6} {m['kind']:6} {m['style']:6}  {tag}")
    for p in m['parts']:
      gaps = p['gaps_m'] or '無'
      print(f"      寬 {p['width_m']:.2f}m 黃度 {p['yellowness']:+5.1f} 實線比例 {p['duty']:.2f} 空隙 {gaps}")

  if args.debug:
    dbg = top.copy()
    for m in marks:
      x = int((m['lat_m'] + HALF_WIDTH) * PX_PER_M)
      col = (0, 255, 255) if m['colour'] == 'yellow' else (255, 255, 255)
      cv2.line(dbg, (x, 0), (x, dbg.shape[0]), col, 1)
      cv2.putText(dbg, f"{m['kind'][:3]}/{m['style'][:3]}", (max(0, x - 20), 14),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.32, col, 1)
    cv2.imwrite(args.debug, dbg)
    print(f'\n{args.debug}')


if __name__ == '__main__':
  main()
