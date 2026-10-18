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
