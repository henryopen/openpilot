# C3 <-> C4 USB link

A wired network between the two devices over their second USB-C ports (2026-10-18). Both have the
dual-role controller `a600000` there (the main port, with panda and the modem, is the host-only
`a800000`). C3 is the host, C4 presents a USB NCM network card:

| | role | address | files |
|---|---|---|---|
| C3 | host (`a600000.ssusb/mode` = host), sees `cdc_ncm` | 10.44.0.1/24 | `c3_host.sh`, `c3link.service`, `c3link-nm.conf` |
| C4 | peripheral, configfs NCM gadget `usb0` | 10.44.0.2/24 | `c4_gadget.sh`, `c4link.service`, `c4link-nm.conf` |

Install (rootfs is read-only on both):

C3 (port 8022): copy the three C3 files to `/data/c3link/`, then
`sudo mount -o remount,rw / && sudo cp /data/c3link/c3link.service /etc/systemd/system/ &&
sudo cp /data/c3link/c3link-nm.conf /etc/NetworkManager/conf.d/ && sudo systemctl enable c3link &&
sudo mount -o remount,ro / && sudo systemctl restart NetworkManager && sudo systemctl start c3link`

C4: the script runs from the repo; install the unit and NM config the same way from
`/data/openpilot/openpilot/c3link/`, unit `c4link`.

Undo: `systemctl disable --now c3link` / `c4link`, remove the unit and the NM conf.
