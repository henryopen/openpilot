#!/usr/bin/env bash
# Install the HUD stream on the comma device. Run as comma on the device:
#   bash /data/openpilot/openpilot/hud/install.sh
set -e
HUD=/data/openpilot/openpilot/hud
sudo mount -o remount,rw /
sudo cp "$HUD/sp-hud.service" /etc/systemd/system/sp-hud.service
sudo systemctl daemon-reload
sudo systemctl enable sp-hud.service
sudo mount -o remount,ro / || true
sudo systemctl restart sp-hud.service
sleep 2
systemctl is-active sp-hud.service
curl -s http://127.0.0.1:8902/health && echo " <- health OK"
