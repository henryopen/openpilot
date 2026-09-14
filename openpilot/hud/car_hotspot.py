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
import datetime
import os
import subprocess
import time

HOTSPOT = "Hotspot"
HOTSPOT_IP = "192.168.43.1"   # pinned in the profile (ipv4.addresses, method shared)
GRACE = 75.0          # seconds from boot before giving up on a known network
POLL = 20.0
RETRY_HOME = 300.0    # while parked on the hotspot, how often to look for home again
OFFROAD = "/data/params/d/IsOffroad"
# Where the handover actually went. print() alone goes to the journal, and this device has no
# /var/log/journal - it is all in memory, so a reboot takes it with it. On 2026-09-14 the Pi
# spent a drive unable to find the car and by the time there was anything to look at, the
# reboot that got the screen back had already erased the only record of when the hotspot came
# up. Both halves of that question live on two different boxes; at least keep this half.
LOG = "/data/hotspot.log"
LOG_MAX = 200_000     # bytes; a few lines a drive, so this is years. Truncated, not rotated.


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
  # "in --active" is not "up". NetworkManager lists the profile from the moment it starts
  # bringing it up, and it stays listed while a failed activation rolls back - on 2026-09-14
  # a con up that never took still read as the hotspot being on, with wlan0 sitting on the
  # home lease the whole time. Believe the address instead: the profile pins HOTSPOT_IP, so
  # wlan0 being somewhere else means the AP is not up, whatever the connection list says.
  #
  # Getting this wrong is expensive. station is None at the same moment, so the loop takes
  # the "elif hotspot" branch, which only retries while parked and only every RETRY_HOME -
  # driving, it does nothing at all, and the Pi has no way in until the car is parked again.
  if hotspot and wlan_ip() != HOTSPOT_IP:
    hotspot = False
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


def wlan_ip() -> str:
  """Which address wlan0 is on, which is what says whether the hotspot is actually up: the
  Hotspot profile pins 192.168.43.1, anything else is a station lease."""
  try:
    out = subprocess.check_output(["ip", "-4", "-o", "addr", "show", "wlan0"],
                                  text=True, timeout=5, stderr=subprocess.DEVNULL)
    for part in out.split():
      if "." in part and "/" in part:
        return part.split("/")[0]
  except Exception:
    pass
  return "?"


def say(state: str) -> None:
  line = f"{datetime.datetime.now().isoformat(timespec='seconds')} {state} [wlan0 {wlan_ip()}]"
  print(line, flush=True)
  try:
    if os.path.exists(LOG) and os.path.getsize(LOG) > LOG_MAX:
      os.remove(LOG)
    with open(LOG, "a") as f:
      f.write(line + "\n")
  except OSError:
    pass                # a log that cannot be written is not a reason to stop hosting


def main():
  last_home_try = 0.0
  said = None
  while True:
    station, hotspot = active_wifi()

    if station is not None:
      if not offroad():
        # Driving: the known network is only going to get further away, and the Pi has no
        # way to reach the car except the hotspot. Waiting for home wifi to drop first cost
        # the Pi its whole first scan - the car was still on home wifi when the Pi went
        # looking, so the Pi found nothing, and NetworkManager's retry backoff then held it
        # off the hotspot for minutes. Switch as soon as openpilot says we are onroad.
        # Nothing is lost by leaving home wifi here: the car is moving away from it anyway,
        # and RETRY_HOME below brings it back once parked.
        state = "driving, moving to the hotspot"
        nmcli("con", "up", HOTSPOT, timeout=60)
      else:
        if hotspot:                                 # both up: the hotspot is the odd one out
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
      say(state)
      said = state
    time.sleep(POLL)


if __name__ == "__main__":
  if os.geteuid() != 0:
    print("needs root for nmcli con up", flush=True)
  main()
