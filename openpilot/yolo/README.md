# yolod

Object detection on the device, drawn on the PiBar HUD. Nothing here reaches the car's
controls: yolod serves its results over HTTP and the display is the only reader.

## What runs where

| Runs on | Path | Started by |
|---|---|---|
| comma device | `/data/yolo/` | `yolod.service`, `boot-probe.service` |

This directory is the source and `/data/yolo` is a deploy target, not a checkout - it is
not a git repository. There is no build step:

```bash
scp openpilot/yolo/*.py openpilot/yolo/*.service comma@<device>:/data/yolo/
ssh comma@<device> 'sudo systemctl restart yolod'
```

## Deliberately not tracked

- `yolo11n.onnx` (10.7 MB) - the model itself, kept on the device
- `pylibs/` - onnxruntime and its dependencies, installed on the device
- `runs/`, `bootlog/` - recorded output

## analysis/

Offline scripts, run on the device against recorded routes under
`/data/media/0/realdata`. They are kept because the evidence behind a change is worth as
much as the change: `vrel_ab3.py` is the A/B that decided the Custin radar should take
its relative speed from the track's V_ABS rather than from differentiating range, and
`replay_radar.py` feeds a recorded segment back through the real `RadarInterface` so the
shipped code can be checked before it is driven.
