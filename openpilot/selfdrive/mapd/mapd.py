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

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, config_realtime_process
from openpilot.common.swaglog import cloudlog

MAPD_ROOT = "/data/media/0/osm"
MAPD_PATH = os.path.join(MAPD_ROOT, "mapd")

MIN_ACCURACY = 25.0     # metres; past this the fix is too loose to place us on a road


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

  logged_fix = False
  while True:
    sm.update(0)

    gps = sm['gpsLocationExternal']
    if sm.updated['gpsLocationExternal'] and gps.horizontalAccuracy < MIN_ACCURACY and \
       not (gps.latitude == 0 and gps.longitude == 0):
      mem_params.put("LastGPSPosition", json.dumps({
        "latitude": float(gps.latitude),
        "longitude": float(gps.longitude),
        "bearing": float(gps.bearingDeg),
      }))
      if not logged_fix:
        cloudlog.info("mapd: first fix at %.5f,%.5f (%.1f m)",
                      gps.latitude, gps.longitude, gps.horizontalAccuracy)
        logged_fix = True

    rk.keep_time()


if __name__ == "__main__":
  main()
