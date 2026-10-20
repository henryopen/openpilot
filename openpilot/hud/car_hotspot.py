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
# The phone's own hotspot, which the car's head unit (D531) is on for internet. 2026-10-20 the user:
# in the car everything goes on the phone's hotspot - C4, Pi and D531 on one network is what lets
# the Pi read the D531's navigation - and "if that cannot connect, it MUST go back to the C4
# hotspot, or the Pi and the car cannot reach each other". So driving, the phone comes first and
# this car's own AP is the fallback, never the other way round.
PHONE_SSID = "MotowayapC"
PHONE_SCAN_EVERY = 30.0   # s: while hosting and driving, how often to look for the phone again
PHONE_RETRY_AFTER = 300.0  # s: after a failed move to the phone, how long before trying again
# Tested 2026-10-20 in the office: with the Hotspot up, `nmcli dev wifi rescan` keeps the AP
# serving (still 192.168.43.1, state connected) and does see a network that appeared after the AP
# came up (a test AP raised on the Pi 12 s later showed at signal 80). So the car can watch for
# the phone by itself while it hosts.
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
  """-> (station connection name or None, hotspot up?, wlan0's address)

  The address comes back with the answer rather than being read again by whoever logs it:
  sampled a second time it is a different moment, and a hotspot that came up and rolled back
  in between reads as a contradiction that was never there."""
  ok, out = nmcli("-t", "-f", "NAME,TYPE", "con", "show", "--active")
  if not ok:
    return None, False, "?"
  station, hotspot = None, False
  for line in out.splitlines():
    name, _, kind = line.rpartition(":")
    if kind != "802-11-wireless":
      continue
    if name == HOTSPOT:
      hotspot = True
    else:
      station = name
  # "In --active" is not "up": NetworkManager lists the profile from the moment it starts
  # bringing it up and keeps it listed while a failed activation rolls back. The profile pins
  # HOTSPOT_IP, so wlan0 being somewhere else means the AP is not serving, whatever the
  # connection list says - and what the Pi needs is an AP that serves, not one that is listed.
  #
  # This is a guard, not a diagnosis. It was added on 2026-09-14 chasing a drive where the Pi
  # never found the car, on the strength of a log line that read "on the hotspot" with wlan0
  # on the home lease - which turned out to be two samples a moment apart rather than a
  # contradiction, and is why the address now travels with the answer. The failure it would
  # cause is real either way: station reads None at the same moment, so the loop takes the
  # "elif hotspot" branch, which retries only while parked and only every RETRY_HOME. Driving,
  # it would do nothing at all. Whether that is what happened that morning is still unknown.
  ip = wlan_ip()
  if hotspot and ip != HOTSPOT_IP:
    hotspot = False
  return station, hotspot, ip


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


def phone_profile() -> str | None:
  """The saved profile for PHONE_SSID (openpilot names them "openpilot connection <ssid>")."""
  for name in known_stations():
    ok, ssid = nmcli("-g", "802-11-wireless.ssid", "con", "show", name, timeout=10)
    if ok and ssid == PHONE_SSID:
      return name
  return None


def phone_visible() -> bool:
  """Fresh scan, then is the phone's SSID in it. Safe while hosting (see PHONE_SSID)."""
  nmcli("dev", "wifi", "rescan", "ifname", "wlan0", timeout=15)
  time.sleep(5)
  ok, out = nmcli("-t", "-f", "SSID", "dev", "wifi", "list", "ifname", "wlan0", timeout=15)
  return ok and PHONE_SSID in [s.strip() for s in out.splitlines()]


def to_phone() -> bool:
  name = phone_profile()
  return name is not None and nmcli("con", "up", name, timeout=45)[0]


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


def say(state: str, ip: str) -> None:
  line = f"{datetime.datetime.now().isoformat(timespec='seconds')} {state} [wlan0 {ip}]"
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
  last_phone_scan = 0.0
  phone_fail_at = -PHONE_RETRY_AFTER
  phone_seen = 0          # consecutive scans, while hosting and driving, that saw the phone
  said = None
  while True:
    station, hotspot, ip = active_wifi()
    phone = phone_profile()

    if station is not None and station == phone:
      # On the phone's hotspot: stay, driving or not. Everything (C4, Pi, D531) is on this one.
      if hotspot:
        nmcli("con", "down", HOTSPOT)
      state = f"on the phone hotspot {station}"
    elif station is not None:
      if not offroad():
        # Driving on home/office wifi: it is only going to get further away. Phone first; if
        # it is not here or will not connect, the car's own AP as before. Waiting for home
        # wifi to drop first cost the Pi its whole first scan - the car was still on home wifi
        # when the Pi went looking, so the Pi found nothing, and NetworkManager's retry backoff
        # then held it off the hotspot for minutes. So switch as soon as openpilot says onroad.
        # RETRY_HOME below brings it back to home wifi once parked.
        if phone_visible() and to_phone():
          state = "driving, moved to the phone hotspot"
        else:
          state = "driving, moving to the hotspot"
          nmcli("con", "up", HOTSPOT, timeout=60)
      else:
        if hotspot:                                 # both up: the hotspot is the odd one out
          nmcli("con", "down", HOTSPOT)
        state = f"on {station}"
    elif hotspot:
      state = "on the hotspot"
      if offroad():
        phone_seen = 0
        if time.monotonic() - last_home_try > RETRY_HOME:
          last_home_try = time.monotonic()
          for name in known_stations():
            if nmcli("con", "up", name, timeout=60)[0]:
              break
          else:
            nmcli("con", "up", HOTSPOT, timeout=60)   # none of them are here, carry on hosting
      elif time.monotonic() - last_phone_scan > PHONE_SCAN_EVERY and \
           time.monotonic() - phone_fail_at > PHONE_RETRY_AFTER:
        # Driving on our own AP: watch for the phone coming back. Two scans in a row before
        # moving, because every move drops the Pi off this AP for a while; and a failed move
        # is not tried again for PHONE_RETRY_AFTER, for the same reason.
        last_phone_scan = time.monotonic()
        phone_seen = phone_seen + 1 if phone_visible() else 0
        if phone_seen >= 2:
          phone_seen = 0
          if to_phone():
            state = "driving, the phone is back, moved to it"
          else:
            phone_fail_at = time.monotonic()
            state = "driving, the phone would not connect, back to the hotspot"
            nmcli("con", "up", HOTSPOT, timeout=60)
    elif uptime() < GRACE:
      state = "waiting for a known network"
    elif not offroad() and phone_visible() and to_phone():
      # Nothing up while driving (the phone's hotspot just dropped, say): the phone if it is
      # there, otherwise - always - our own AP, so the Pi has something to reach the car on.
      state = "no network, moved to the phone hotspot"
    else:
      state = "no known network, starting the hotspot"
      nmcli("con", "up", HOTSPOT, timeout=60)

    if state != said:
      say(state, ip)
      said = state
    time.sleep(POLL)


if __name__ == "__main__":
  if os.geteuid() != 0:
    print("needs root for nmcli con up", flush=True)
  main()
