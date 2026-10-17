"""The posted limit, with the offline map corrected against the official one where they disagree.

The driver (2026-10-17): many OSM speed limits are wrong - on the Keelung Road viaduct and the Shuiyuan
Expressway, both 60, the car set itself to 90 and 80 and he kept ending up over the limit. Against the
National Police Agency's list of fixed speed cameras (data.gov.tw 7320, each with its posted limit), at
the 1894 cameras in Taiwan OSM agreed at 38%, had no limit at 53% and a different one at 7% - about one in
seven of the limits it does have. On that drive OSM had the viaduct southbound at 80 (northbound 70) and
the expressway in pieces of 60 and 70; the cameras there say 60 for both.

speed_limit_fixes.json holds the corrections: an OSM road name and where it applies (the whole road's box,
or a radius round a camera that disagreed), with the official limit. mapd_bridge, which alone has the GPS
position, writes MapSpeedLimitFixed twice a second - the official limit when the road the map has us on is
in the table, the map's own otherwise - and everything that acts on or shows the limit reads that.
/data/speed_limit_fix_off goes back to the map alone.
"""
import json
import math
import os
import time

FIX_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "speed_limit_fixes.json")
OFF_FLAG = "/data/speed_limit_fix_off"
FRESH_S = 3.0          # an older MapSpeedLimitFixed means mapd_bridge is not running: use the map's
KPH_TO_MS = 1 / 3.6


def load_fixes(path=FIX_FILE):
  try:
    with open(path, encoding="utf-8") as f:
      return json.load(f).get("fixes", [])
  except (OSError, ValueError):
    return []


def _inside(fix, lat, lon):
  if "box" in fix:
    la1, lo1, la2, lo2 = fix["box"]
    return la1 <= lat <= la2 and lo1 <= lon <= lo2
  if "circle" in fix:
    la, lo, r = fix["circle"]
    dy = (lat - la) * 110540.
    dx = (lon - lo) * 111320. * math.cos(math.radians(la))
    return dx * dx + dy * dy <= r * r
  return False


def lookup(fixes, road, lat, lon):
  """(official limit in km/h, index) for this road here, or None."""
  if not road:
    return None
  for i, fix in enumerate(fixes):
    if fix.get("name") == road and _inside(fix, lat, lon):
      return fix["limit"], i
  return None


def effective_limit(mem_params):
  """The limit to act on, in m/s: mapd_bridge's corrected one while it is fresh, else the map's."""
  osm = float(mem_params.get("MapSpeedLimit") or 0.)
  j = mem_params.get("MapSpeedLimitFixed")
  if isinstance(j, dict) and time.monotonic() - float(j.get("t", 0.)) < FRESH_S:
    return float(j.get("limit", osm))
  return osm
