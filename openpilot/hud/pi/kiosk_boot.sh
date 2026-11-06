#!/bin/bash
# Started by ~/.config/labwc/autostart at login (2026-10-31). Chromium used to be launched straight from there, and on
# every boot that day it came up 3.5-15 s before hud-agent was serving 127.0.0.1:8080 (hud-agent waited for
# network-online, which in the car can take a while): the kiosk opened on the connection-refused page and only
# sometimes reloaded by itself. So wait for the page (60 s at most), then start chromium with the usual flags.
for _ in $(seq 1 120); do
  curl -s -o /dev/null --max-time 1 http://127.0.0.1:8080/dash.html && break
  sleep 0.5
done
exec chromium --ozone-platform=wayland --kiosk --password-store=basic --noerrdialogs --disable-infobars \
  --disable-session-crashed-bubble --app=http://127.0.0.1:8080/dash.html
