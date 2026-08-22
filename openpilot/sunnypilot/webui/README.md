# sunnypilot local settings web UI

A standalone, stdlib-only web page for changing sunnypilot settings over the LAN.
Built for **comma four (mici)**, whose on-device screen has no Cruise/OSM settings
pages. Renders the device's own `settings_ui.json`, so it shows the same options
(ICBM, Custom ACC increments, Speed Limit, SCC, OSM map download, etc.) with the
same offroad-only / capability gating as the full tici UI.

## Design

- **No cloud.** Reads and writes `/data/params/d` directly on the device; sunnylink
  can stay disabled.
- **LAN-only by trust.** Binds `0.0.0.0:8899`, no auth (owner's decision for a home/
  office LAN). Do not expose to the internet.
- **Lives outside `/data/openpilot`.** Installed to `/data/sp_webui` + a systemd unit,
  so an openpilot branch switch does not disturb it. Only a full AGNOS reflash needs
  a re-install.
- Writes are whitelisted to keys present in `settings_ui.json` (plus OSM keys and a
  couple of extras); offroad-only keys are refused while the car is on.

## Install (on device)

```bash
bash openpilot/sunnypilot/webui/install.sh
```

Then open `http://<device-ip>:8899` from any device on the same network.

## Notes

- OSM map download for Speed Limit Assist / SCC-Map is driven from the Cruise tab
  (writes `OsmLocationName` + `OsmDbUpdatesCheck`, same as the tici OSM page).
- To change the port: edit `SPWEB_PORT` in the systemd unit / environment.
- Capability context (brand, has_icbm, stock-long) is hard-coded near the top of
  `server.py` for this device; adjust `CAPABILITIES` if reused on another car.
