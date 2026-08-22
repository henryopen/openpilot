#!/bin/bash
# Install the sunnypilot local settings web UI as a systemd service on a comma device.
# Run once on-device after a fresh install:  bash openpilot/sunnypilot/webui/install.sh
# It survives openpilot branch switches (lives in /data/sp_webui + /etc), only a full
# AGNOS reflash requires re-running it.
set -e

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST=/data/sp_webui

mkdir -p "$DEST"
cp "$SRC_DIR/server.py" "$DEST/server.py"

sudo mount -o remount,rw /
sudo cp "$SRC_DIR/sp-webui.service" /etc/systemd/system/sp-webui.service
sudo systemctl daemon-reload
sudo systemctl enable --now sp-webui
sudo mount -o remount,ro /

sleep 2
systemctl status sp-webui --no-pager -l | head -6
echo
echo "sp-webui installed. Open http://<device-ip>:8899 from a device on the same LAN."
