# hud

Two halves that run on different machines.

| Runs on | Files | Started by |
|---|---|---|
| comma device | `server.py`, `radar_tracker.py`, `sp-hud.service` | `install.sh` |
| PiBar | `dash.html`, `index.html`, `pi/hud_agent.py`, `pi/kiosk.sh`, `pi/brightness.sh` | `pi/hud-agent.service` |

`dash.html` is served by the Pi's own agent out of `/home/pi/hud`, never by the device, so
the device's copy of it is not what anyone is looking at. Deploy it to the Pi:

```bash
scp openpilot/hud/dash.html openpilot/hud/index.html pi@<pibar>:/home/pi/hud/
ssh pi@<pibar> '/home/pi/hud/kiosk.sh dash.html'   # chromium caches; it needs the restart
```

This tree is the source for both halves. On 2026-08-29 the Pi's copy ran a day ahead of
it, so deploying `dash.html` from here would have silently reverted the display to one
without the detection overlay.
