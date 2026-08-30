#!/bin/bash
# Point the PiBar kiosk at a page. Chromium caches aggressively and will not reload a
# replaced file on its own, so this always restarts it with the disk cache disabled.
#   ./kiosk.sh            -> yolo.html
#   ./kiosk.sh dash.html  -> back to the HUD
page="${1:-yolo.html}"
pkill -x chromium
sleep 2
export WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000
# same flags as ~/.config/labwc/autostart, or chromium stops for a keyring prompt
setsid nohup chromium --ozone-platform=wayland --kiosk --password-store=basic --noerrdialogs \
  --disable-infobars --disable-session-crashed-bubble --disk-cache-dir=/dev/null \
  --app="http://127.0.0.1:8080/$page" </dev/null >/dev/null 2>&1 &
disown
sleep 3  # outlive the ssh session that started us
echo "kiosk -> $page"
