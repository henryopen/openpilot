#!/usr/bin/env python3
"""Bridge between the car's GPS and pfeiferj's mapd.

mapd is a separate binary that reads our position out of the memory params and
writes back the speed limit and road name for wherever we are, from offline OSM
tiles. It only needs to be told where the car is.

sunnypilot feeds it liveLocationKalman, but master no longer publishes that
service, so the position comes straight off the GPS instead. The receiver
reports better than a metre on this car, which is well inside what matching a
road needs.
"""
import json
import os
import time

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.mapd.speed_limit import KPH_TO_MS, OFF_FLAG, load_fixes, lookup

MAPD_ROOT = "/data/media/0/osm"
MAPD_PATH = os.path.join(MAPD_ROOT, "mapd")

MIN_ACCURACY = 25.0     # metres; past this the fix is too loose to place us on a road
MIN_HEADING_SPEED = 3.0  # m/s


def main():
  config_realtime_process([0, 1, 2, 3], 5)

  mem_params = Params("/dev/shm/params")
  sm = messaging.SubMaster(['gpsLocationExternal'])
  rk = Ratekeeper(2, print_delay_threshold=None)

  # mapd will not start looking for tiles until these exist
  if mem_params.get("OSMDownloadBounds") is None:
    mem_params.put("OSMDownloadBounds", "")
  if mem_params.get("LastGPSPosition") is None:
    mem_params.put("LastGPSPosition", "{}")

  fixes = load_fixes()
  cloudlog.info("mapd: %d speed limit corrections", len(fixes))
  pos = None              # last good (lat, lon)
  bearing = None          # last heading while moving (deg), for the direction-dependent sign stretches
  fix_off = os.path.exists(OFF_FLAG)
  logged_fix = False
  frame = 0
  while True:
    sm.update(0)
    frame += 1
    if frame % 20 == 0:
      fix_off = os.path.exists(OFF_FLAG)

    gps = sm['gpsLocationExternal']
    if sm.updated['gpsLocationExternal'] and gps.horizontalAccuracy < MIN_ACCURACY and \
       not (gps.latitude == 0 and gps.longitude == 0):
      mem_params.put("LastGPSPosition", json.dumps({
        "latitude": float(gps.latitude),
        "longitude": float(gps.longitude),
        "bearing": float(gps.bearingDeg),
      }))
      pos = (float(gps.latitude), float(gps.longitude))
      if gps.speed > MIN_HEADING_SPEED:   # the GPS heading means nothing standing still: keep the last moving one
        bearing = float(gps.bearingDeg)
      if not logged_fix:
        cloudlog.info("mapd: first fix at %.5f,%.5f (%.1f m)",
                      gps.latitude, gps.longitude, gps.horizontalAccuracy)
        logged_fix = True

    # the limit to act on: the official one where the map is known to be wrong - see speed_limit
    osm = float(mem_params.get("MapSpeedLimit") or 0.)
    road = mem_params.get("RoadName") or ""
    hit = None if (fix_off or pos is None) else lookup(fixes, road, *pos, bearing)
    mem_params.put("MapSpeedLimitFixed", {"limit": hit[0] * KPH_TO_MS if hit else osm, "osm": osm, "road": road,
                                          "fix": hit[1] if hit else -1, "t": time.monotonic()})

    rk.keep_time()


if __name__ == "__main__":
  main()
