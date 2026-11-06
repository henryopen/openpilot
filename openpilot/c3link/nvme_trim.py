#!/usr/bin/env python3
"""On C3: keep at least KEEP_FREE_GB free on the NVMe (/data/media), oldest drive segments first (2026-11-02).

Until 11/02 only c4_archive.sh trimmed, and only C4's copies, at 90 %: C3's own recordings (loggerd ->
/data/media/0/realdata, there is no openpilot manager / deleter on C3) grew to 869 GB, the NVMe filled on 11/01 12:40,
C3 recorded nothing after that, and the trim had removed every C4 copy. Now both trees are trimmed together.

Order: a segment's time is its route's time - the newest mtime among that route's segments that is not from before
the clock was set (C3 boots at 2023-11-22 until it syncs; 102 of 6132 segment folders carry that date); a route with
none takes the previous route's (by counter). Segments touched in the last BUSY_S (the one being recorded) are left
alone."""
import os
import re
import shutil
import sys
import syslog
import time

KEEP_FREE_GB = 100
TREES = ["/data/media/0/realdata", "/data/media/0/c4_archive/realdata"]
MOUNT = "/data/media"
BUSY_S = 120
CLOCK_SET = time.mktime((2025, 1, 1, 0, 0, 0, 0, 0, -1))   # mtimes before this are the unset boot clock
SEG = re.compile(r"^(?P<route>[0-9a-f]{8}--[0-9a-f]{10})--(?P<seg>\d+)$")


def free_bytes() -> int:
  return shutil.disk_usage(MOUNT).free


def segments():
  """-> [(key, path)] oldest first."""
  now = time.time()   # noqa: TID251 - compared with file mtimes, which are wall-clock
  segs, route_t = [], {}
  for tree in TREES:
    try:
      names = os.listdir(tree)
    except OSError:
      continue
    for name in names:
      m = SEG.match(name)
      if not m:
        continue                       # 'boot' and anything else that is not a drive segment
      path = os.path.join(tree, name)
      try:
        mt = os.stat(path).st_mtime
      except OSError:
        continue
      if now - mt < BUSY_S:
        continue
      route = (tree, m["route"])
      segs.append((route, int(m["seg"]), mt, path))
      if mt >= CLOCK_SET:
        route_t[route] = max(route_t.get(route, 0.0), mt)
  # a route with no good mtime at all (the clock never got set that drive) takes the time of the route before it
  # by counter (the first 8 hex digits count up drive by drive), so it is not taken for the oldest
  last: dict[str, float] = {}
  for route in sorted({r for r, _, _, _ in segs}, key=lambda r: (r[0], int(r[1][:8], 16))):
    if route in route_t:
      last[route[0]] = route_t[route]
    else:
      route_t[route] = last.get(route[0], 0.0)
  out = []
  for route, seg, _, path in segs:
    out.append(((route_t[route], int(route[1][:8], 16), seg), path))
  out.sort()
  return out


def main():
  want = KEEP_FREE_GB * 1024 ** 3
  if free_bytes() >= want:
    return 0
  removed = []
  for _, path in segments():
    if free_bytes() >= want:
      break
    shutil.rmtree(path, ignore_errors=True)
    removed.append(path)
  span = f" ({removed[0]} .. {removed[-1]})" if removed else ""
  free_gb = free_bytes() / 1024 ** 3
  syslog.syslog(f"free under {KEEP_FREE_GB} GB: removed {len(removed)} segments{span}, free now {free_gb:.0f} GB")
  return 0


if __name__ == "__main__":
  syslog.openlog("nvme_trim")
  sys.exit(main())
