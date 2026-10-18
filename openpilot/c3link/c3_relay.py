#!/usr/bin/env python3
"""C4 side of C3's self-calibration and light reading, over the USB link (c3link/usb_link.sh) - DISPLAY ONLY.

The driver (2026-10-18): C3 will sit wherever it fits that day - on the dashboard, lower than C4, moved every
time he gets in - and it should calibrate itself, the way openpilot does. C3 finds "up" from its own
accelerometer (gravity) and "forward" from how its accelerometer follows the car speeding up and slowing
down - which needs the car's own longitudinal acceleration, from the wheels. That is all C4 sends: carState
vEgo / aEgo as they come (100 Hz), with C4's CLOCK_BOOTTIME at the moment of sending so C3 can put the sample
times on its own clock. (C4's IMU is not used: on mici its published axes are not openpilot's device frame -
at rest gravity reads on +x and forward is -z, 10/18 - and the car's acceleration is the better reference
anyway.)

It also takes back what C3 reads off the lights, already turned into car-frame directions, and leaves it in
/dev/shm/c3lights.json for the HUD. Nothing here reaches the planner or anything that controls.

UDP both ways: to C3 10.44.0.1:4410, from C3 on 4411. With the cable out the sends just fail and are dropped.
"""
import json
import os
import socket
import time

from openpilot.cereal import messaging
from openpilot.common.swaglog import cloudlog

C3 = ("10.44.0.1", 4410)
LISTEN = ("0.0.0.0", 4411)
OUT = "/dev/shm/c3lights.json"
BATCH_S = 0.1


def open_rx():
  rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  rx.bind(LISTEN)
  rx.setblocking(False)
  return rx


def main():
  # This is a managed process: if it ever exits, selfdrived raises processNotRunning (soft disable) and manager does
  # not restart it. Being display only, it must never take the drive down - so nothing in here may end it: a
  # failed bind is retried, and anything a loop round throws is logged (rate-limited) and the loop goes on.
  sm = messaging.SubMaster(["carState"])
  tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  rx = None
  cs = []
  last_send = last_bind = last_err = 0.
  while True:
    try:
      sm.update(50)
      now = time.monotonic()
      if rx is None and now - last_bind >= 5.:
        last_bind = now
        try:
          rx = open_rx()
        except OSError as e:
          cloudlog.warning(f"c3_relay: cannot listen on {LISTEN}: {e}")
      if sm.updated["carState"]:
        c = sm["carState"]
        cs.append([sm.logMonoTime["carState"], round(float(c.vEgo), 3), round(float(c.aEgo), 4)])
      if now - last_send >= BATCH_S:
        if cs:
          pkt = {"s": time.clock_gettime_ns(time.CLOCK_BOOTTIME), "cs": cs}
          try:
            tx.sendto(json.dumps(pkt, separators=(",", ":")).encode(), C3)
          except OSError:
            pass      # link down
        cs = []
        last_send = now
      # C3's light reading, already in car-frame directions
      while rx is not None:
        try:
          data, _ = rx.recvfrom(65536)
        except BlockingIOError:
          break
        try:
          j = json.loads(data)
        except ValueError:
          continue
        if not isinstance(j, dict):
          continue
        j["t"] = time.monotonic()
        tmp = OUT + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
          json.dump(j, f, ensure_ascii=False)
        os.replace(tmp, OUT)
    except Exception:
      cs = []
      if time.monotonic() - last_err >= 10.:
        last_err = time.monotonic()
        cloudlog.exception("c3_relay: error, carrying on")
      time.sleep(0.05)


if __name__ == "__main__":
  main()
