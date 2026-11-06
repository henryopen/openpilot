#!/usr/bin/env python3
import datetime
import os
import subprocess
import time
from typing import NoReturn

import openpilot.cereal.messaging as messaging
from openpilot.common.time_helpers import min_date, MAX_DATE, system_time_valid
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.common.gps import get_gps_location_service


def set_time(new_time):
  diff = datetime.datetime.now(datetime.UTC).replace(tzinfo=None) - new_time
  if abs(diff) < datetime.timedelta(seconds=10):
    cloudlog.debug(f"Time diff too small: {diff}")
    return

  cloudlog.debug(f"Setting time to {new_time}")
  try:
    subprocess.run(f"TZ=UTC date -s '{new_time}'", shell=True, check=True)
  except subprocess.CalledProcessError:
    cloudlog.exception("timed.failed_setting_time")


LAST_TIME_PARAM = "LastKnownGoodTime"
NTP_SYNCED_FLAG = "/run/systemd/timesync/synchronized"   # systemd-timesyncd writes this once it agrees
SAVE_INTERVAL = 60.


def time_is_trusted(gps_set: bool) -> bool:
  return gps_set or os.path.exists(NTP_SYNCED_FLAG)


def restore_time(params: Params) -> None:
  """Carry the clock forward to the last reading we knew was right.

  The RTC has no backup power: it reads 1970 at boot and systemd falls back to its own
  build date, which is months in the past. Neither source of truth is available then -
  GPS only runs onroad and needs a cold start, NTP needs a network the car does not have -
  so everything recorded until one arrives is stamped with that stale date, which quietly
  voids any analysis keyed on wall time.

  Only ever move the clock forward, and only from a reading that was itself trustworthy,
  so this can never drag a correct clock backwards.
  """
  try:
    raw = params.get(LAST_TIME_PARAM)
    if raw is None:
      return
    saved = datetime.datetime.fromtimestamp(float(raw), datetime.UTC).replace(tzinfo=None)
    if saved > datetime.datetime.now(datetime.UTC).replace(tzinfo=None):
      cloudlog.info(f"Restoring time to last known good {saved}")
      set_time(saved)
  except Exception:
    # nothing in here may take timed down with it: it is always_run, and a missing process
    # blocks longitudinal control entirely
    cloudlog.exception("timed.failed_restoring_time")


def save_time(params: Params) -> None:
  try:
    params.put(LAST_TIME_PARAM, str(time.time_ns() / 1e9))
  except Exception:
    cloudlog.exception("timed.failed_saving_time")


def main() -> NoReturn:
  """
    timed has two responsibilities:
    - getting the current time from GPS
    - publishing the time in the logs

    AGNOS will also use NTP to update the time.
  """

  params = Params()
  restore_time(params)
  gps_location_service = get_gps_location_service(params)

  pm = messaging.PubMaster(['clocks'])
  sm = messaging.SubMaster([gps_location_service])
  gps_set = False
  last_save = 0.
  while True:
    sm.update(1000)

    if time_is_trusted(gps_set) and time.monotonic() - last_save > SAVE_INTERVAL:
      last_save = time.monotonic()
      save_time(params)

    msg = messaging.new_message('clocks')
    msg.valid = system_time_valid()
    msg.clocks.wallTimeNanos = time.time_ns()
    pm.send('clocks', msg)

    gps = sm[gps_location_service]
    gps_time = datetime.datetime.fromtimestamp(gps.unixTimestampMillis / 1000., datetime.UTC).replace(tzinfo=None)
    if not sm.updated[gps_location_service] or (time.monotonic() - sm.logMonoTime[gps_location_service] / 1e9) > 2.0:
      continue
    if not gps.hasFix:
      continue
    if gps_time < min_date() or gps_time > MAX_DATE:
      continue

    set_time(gps_time)
    gps_set = True
    save_time(params)
    time.sleep(10)

if __name__ == "__main__":
  main()
