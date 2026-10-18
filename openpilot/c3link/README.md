# C3 <-> C4 USB link

A wired network between the two devices over their second USB-C ports (2026-10-18), with one USB-C cable.
Same script on both, `usb_link.sh <address>`: C3 is 10.44.0.1, C4 10.44.0.2.

- The second port is on the dual-role controller `a600000`; its Type-C side is the PMI8998 charger (checked
  with a phone on each port: CC, orientation and PD seen). Both ends are dual power role, so Type-C makes
  exactly one of them the 5 V source - never both.
- Which one becomes the source changes between plug-ins and cannot be pinned on this kernel, so the data role
  follows it: the source is the USB host (sees a `cdc_ncm` card), the sink presents the card (configfs NCM
  gadget `c3link`). Read the state: `/sys/class/power_supply/usb/typec_mode`, `journalctl -t usb_link`.
- Tested 10/18 (C3 source): SuperSpeed, ping ~2.2 ms, C4 -> C3 TCP 246 Mbit/s.

| | unit | script runs from |
|---|---|---|
| C3 (port 8022) | `c3link.service` | `/data/c3link/usb_link.sh` (copied there) |
| C4 | `c4link.service` | the repo, `/data/openpilot/openpilot/c3link/usb_link.sh` |

Install (rootfs read-only on both): `sudo mount -o remount,rw /`, copy the unit to `/etc/systemd/system/` and
`usb_link-nm.conf` to `/etc/NetworkManager/conf.d/`, `sudo systemctl enable <unit>`, `sudo mount -o remount,ro /`,
`sudo systemctl restart NetworkManager`, `sudo systemctl start <unit>`.

Undo: `sudo systemctl disable --now <unit>`, remove the unit and the NM conf.

## C4 drives archived on C3 (`c4_archive.sh`, `c4archive.service`, C3 only)

C3 pulls C4's `/data/media/0/realdata` into `/data/media/0/c4_archive/realdata` on its NVMe every minute while the
link is up (rsync over ssh with `/data/c3link/c4key`; C4 runs nothing new). C4's side is nice 19 / idle IO; onroad the
pull is capped at 3 MB/s (recording ~1.4 MB/s). The segment being recorded is skipped. Nothing is deleted on C4; on C3
the oldest drives go above 90%. Log: `journalctl -t c4_archive`. First full copy 10/18: ~18 MB/s parked.

## C3 calibrates itself every drive (`c3calib.py` on C3, `c3_relay.py` on C4) - display only

C3 sits wherever it fits that day, so it works out where its camera points by itself: "up" from its accelerometer
(gravity), "forward" from how its accelerometer follows C4's carState.aEgo (sent over the link by `c3_relay`, a
managed process on C4, onroad), leaving out turns (C3's gyro). Offline check on C4's own drives: 1-1.5 deg after
10-15 minutes of driving; until then the previous drive's result is used, marked "標定中". If C3 gets knocked
(gravity moves > 4 deg for 20 s) it starts over. The light boxes from `c3detect` (C3's own service) become car-frame
azimuth / elevation; below the horizon is dropped (dashboard). C4's HUD shows the strongest light within 12 deg of
straight ahead (at least 3 deg above the horizon, seen twice in a row).

## C3's light reader (`c3lights.py`, `c3-lights.service`) - replaces the old c3detect / c3-screen

C4's tlwatchd detector and code (`yolo/yolo_core.py`, copied next to it): `c4band1_1344x352.int8v2`, in
`/data/c3link/models/`. Not tlwatchd's classifier: on C3's frames it reads brake-light queues as red, empty roads as go. C3's narrow camera has 2.32x C4's focal length, so at half
resolution its lights are ~23 px - about what the band models learnt on (~20 px): the half-resolution rows from 322
above the horizon (c3calib) are centred in the 1344x352 band. Only while driving. ~240 ms a frame on C3.
10/18 check on 355 frames of C3's own drives: 48 real lights / 34 false at conf 0.25 (old c4light13: 33 / 9). State: C3 `/dev/shm/c3calib.json`, C4 `/dev/shm/c3lights.json`.

C3 services: `c3-sensord.service` (sensord standalone, like `c3-camerad`), `c3calib.service`.

## C3's screen (`c3_screen.cc`, `c3-screen.service`)

A status page, no camera picture: temperature (hottest cpu/gpu/kryo zone - the lmh-dcvs zones are limits, 75000
always), WiFi IP and name, the USB link (usb0 carrier) and whether C4 is sending carState, calibration
(`/dev/shm/c3calib.json`), lights (`/dev/shm/c3det/det.json` + the lights c3calib passes to C4), the archive (drive
count, NVMe use, last copy from `journalctl -t c4_archive`). Built on C3 (`g++` line at the top of the source) into
`/data/c3link/c3_screen`; `QT_QPA_PLATFORM=offscreen ./c3_screen --shot x.png` renders it without the panel.
