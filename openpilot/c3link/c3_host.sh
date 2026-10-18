#!/bin/bash
# C3 side of the C3 <-> C4 USB link (2026-10-18).
#
# The second USB-C port of both devices is on the dual-role controller a600000 (the main port, with
# panda and the modem, is the host-only a800000). Here that port is forced to host; C4 forces its own to
# peripheral and presents a USB NCM network card (c4_gadget.sh). Whatever cdc_ncm interface appears gets
# 10.44.0.1/24; C4 is 10.44.0.2. NetworkManager is told to leave cdc_ncm alone (c3link-nm.conf), else it
# would try DHCP on it and flush the address.
MODE=/sys/devices/platform/soc/a600000.ssusb/mode
ADDR=10.44.0.1/24

while true; do
  [ "$(cat $MODE)" = host ] || { echo host > $MODE; logger -t c3link "a600000 -> host"; }
  for d in /sys/class/net/*; do
    n=${d##*/}
    drv=$(readlink "$d/device/driver" 2>/dev/null)
    [ "${drv##*/}" = cdc_ncm ] || continue
    if ! ip -4 addr show dev "$n" | grep -q "${ADDR%/*}/"; then
      ip addr add $ADDR dev "$n" && ip link set "$n" up && logger -t c3link "$n up $ADDR"
    fi
  done
  sleep 5
done
