#!/usr/bin/env python3
"""Bring the car's hotspot up by itself, so the HUD Pi has something to join.

The Pi reaches the car over wifi, and in the car the only network is the car's own hotspot.
NetworkManager keeps that as a profile called Hotspot (mode ap, ssid weedle-94cc, ipv4
shared) with connection.autoconnect set to no, and openpilot's settings screen only ever
calls activate_connection on it - so it has to be switched on by hand after every boot, and
until it is, the Pi sits waiting.

The obvious fix - setting connection.autoconnect on the profile - is the wrong one. An AP
profile is always "available" to NetworkManager, so at boot it can win before the home
network has even been scanned, and NetworkManager does not leave a connection it has already
brought up for a higher-priority one that turns up later. The car would sit on its own
hotspot in the driveway with nothing able to reach it.

So this waits instead. If nothing has connected by the time the grace period is up, there is
no known network here and the hotspot goes on. If the car is parked and on the hotspot, it
tries the known networks again every few minutes, which is how it finds its way back onto
home wifi after a drive without needing a reboot - and it only does that while parked, so a
drive is never interrupted by the Pi losing its link for a few seconds.

  sudo systemctl enable --now car-hotspot
"""
import os
import subprocess
import time

HOTSPOT = "Hotspot"
GRACE = 75.0          # seconds from boot before giving up on a known network
POLL = 20.0
RETRY_HOME = 300.0    # while parked on the hotspot, how often to look for home again
OFFROAD = "/data/params/d/IsOffroad"


def nmcli(*args, timeout=45):
  try:
    r = subprocess.run(["nmcli", *args], capture_output=True, text=True, timeout=timeout)
    return r.returncode == 0, r.stdout.strip()
  except (subprocess.TimeoutExpired, OSError):
    return False, ""


def active_wifi():
  """-> (station connection name or None, hotspot up?)"""
  ok, out = nmcli("-t", "-f", "NAME,TYPE", "con", "show", "--active")
  if not ok:
    return None, False
  station, hotspot = None, False
  for line in out.splitlines():
    name, _, kind = line.rpartition(":")
    if kind != "802-11-wireless":
      continue
    if name == HOTSPOT:
      hotspot = True
    else:
      station = name
  return station, hotspot


def known_stations():
  ok, out = nmcli("-t", "-f", "NAME,TYPE", "con", "show")
  if not ok:
    return []
  names = []
  for line in out.splitlines():
    name, _, kind = line.rpartition(":")
    if kind == "802-11-wireless" and name != HOTSPOT:
      names.append(name)
  return names


def offroad():
  try:
    with open(OFFROAD, "rb") as f:
      return f.read().strip() == b"1"
  except OSError:
    return True          # if openpilot has not said otherwise, assume parked


def uptime():
  with open("/proc/uptime") as f:
    return float(f.read().split()[0])


def main():
  last_home_try = 0.0
  said = None
  while True:
    station, hotspot = active_wifi()

    if station is not None:
      if hotspot:                                   # both up: the hotspot is the odd one out
        nmcli("con", "down", HOTSPOT)
      state = f"on {station}"
    elif hotspot:
      state = "on the hotspot"
      if offroad() and time.monotonic() - last_home_try > RETRY_HOME:
        last_home_try = time.monotonic()
        for name in known_stations():
          if nmcli("con", "up", name, timeout=60)[0]:
            break
        else:
          nmcli("con", "up", HOTSPOT, timeout=60)   # none of them are here, carry on hosting
    elif uptime() < GRACE:
      state = "waiting for a known network"
    else:
      state = "no known network, starting the hotspot"
      nmcli("con", "up", HOTSPOT, timeout=60)

    if state != said:
      print(state, flush=True)
      said = state
    time.sleep(POLL)


if __name__ == "__main__":
  if os.geteuid() != 0:
    print("needs root for nmcli con up", flush=True)
  main()
