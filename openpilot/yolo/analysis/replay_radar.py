#!/usr/bin/env python3
"""Feed a recorded segment's CAN through the real RadarInterface and print what radard would get.

Checks the shipped decode rather than a copy of it, so a change can be seen on a drive
that already happened before it is driven again.

  ./replay_radar.py <route> <seg> 14:06:34 14:06:40
"""
import datetime
import sys

from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.pandad import can_capnp_to_list
from opendbc.car.hyundai.radar_interface import RadarInterface

route, seg, h0, h1 = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]


def sec(s):
  a, b, c = s.split(':')
  return int(a) * 3600 + int(b) * 60 + float(c)


w0, w1 = sec(h0), sec(h1)
path = f"/data/media/0/realdata/{route}--{seg}/rlog.zst"

CP = None
for m in LogReader(path):
  if m.which() == 'carParams':
    CP = m.carParams.as_builder()
    break
RI = RadarInterface(CP)

wall0 = wallmono = None      # the clock syncs about 49 s after boot; anchor on the first sane one
vego = 0.
last = 0.
for m in LogReader(path):
  w = m.which()
  if w == 'clocks':
    wt = m.clocks.wallTimeNanos / 1e9
    if wt > 1.7e9 and wall0 is None:
      wall0, wallmono = wt, m.logMonoTime / 1e9
    continue
  if w == 'carState':
    vego = m.carState.vEgo
    continue
  if w != 'can':
    continue
  rr = RI.update(can_capnp_to_list([m.as_builder().to_bytes()]))
  if rr is None or wall0 is None:
    continue
  t = m.logMonoTime / 1e9
  tw = wall0 + (t - wallmono) + 8 * 3600      # the device keeps UTC
  if not (w0 <= tw % 86400 <= w1):
    continue
  if t - last < 0.25:
    continue
  last = t
  ts = datetime.datetime.fromtimestamp(tw, datetime.UTC).strftime("%H:%M:%S.%f")[:-5]
  pts = ' | '.join(f"d={p.dRel:5.1f} y={p.yRel:+5.2f} vRel={p.vRel:+6.2f} vLead={vego + p.vRel:+6.2f}"
                   for p in rr.points)
  print(f"{ts} vEgo={vego * 3.6:5.1f} | {pts or '(no point)'}")
