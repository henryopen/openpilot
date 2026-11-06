#!/usr/bin/env python3
"""C4 side of C3's self-calibration and light reading, over the USB link (c3link/usb_link.sh) - DISPLAY ONLY.

The driver (2026-10-18): C3 will sit wherever it fits that day - on the dashboard, lower than C4, moved every
time he gets in - and it should calibrate itself, the way openpilot does. C3 finds "up" from its own
accelerometer (gravity) and "forward" from how its accelerometer follows the car speeding up and slowing
down - which needs the car's own longitudinal acceleration, from the wheels: carState vEgo / aEgo, with C4's
CLOCK_BOOTTIME at the moment of sending so C3 can put the sample times on its own clock. (C4's IMU is not used:
on mici its published axes are not openpilot's device frame - at rest gravity reads on +x and forward is -z,
10/18 - and the car's acceleration is the better reference anyway.)

Those samples are sent by the HUD server (hud/server.py), not from here (2026-10-22): msgq allows a topic 15
readers, carState already had exactly 15, and this process reading it as a 16th made every reader drop messages -
the 10/21 drive's "communication issue" on every engage. So this process reads no openpilot topic at all.

It takes back what C3 reads off the lights, already turned into car-frame directions, and leaves it in
/dev/shm/c3lights.json for the HUD. Nothing here reaches the planner or anything that controls.

UDP from C3 on 4411.
"""
import json
import os
import socket
import time

from openpilot.common.swaglog import cloudlog

LISTEN = ("0.0.0.0", 4411)
OUT = "/dev/shm/c3lights.json"


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
  rx = None
  last_bind = last_err = 0.
  while True:
    try:
      time.sleep(0.05)
      now = time.monotonic()
      if rx is None and now - last_bind >= 5.:
        last_bind = now
        try:
          rx = open_rx()
        except OSError as e:
          cloudlog.warning(f"c3_relay: cannot listen on {LISTEN}: {e}")
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
      if time.monotonic() - last_err >= 10.:
        last_err = time.monotonic()
        cloudlog.exception("c3_relay: error, carrying on")
      time.sleep(0.05)


if __name__ == "__main__":
  main()
