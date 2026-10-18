#!/bin/bash
# C3 side of the C3 <-> C4 USB link (2026-10-18).
#
# The second USB-C port of both devices is on the dual-role controller a600000 (the main port, with
# panda and the modem, is the host-only a800000). Here that port is forced to host; C4 forces its own to
# peripheral and presents a USB NCM network card (c4_gadget.sh). Whatever cdc_ncm interface appears gets
# 10.44.0.1/24; C4 is 10.44.0.2. NetworkManager is told to leave cdc_ncm alone (c3link-nm.conf), else it
# would try DHCP on it and flush the address.
#
# Plugging the cable in while the port already sits in host mode does not enumerate C4 (tested 10/18: the
# Type-C side negotiates - C3 became the 5 V source, C4 the sink - but nothing showed up until the port
# was cycled none -> host). So when Type-C says something is attached and no cdc_ncm card has appeared
# for KICK_AFTER seconds, cycle it, at most every KICK_EVERY seconds.
MODE=/sys/devices/platform/soc/a600000.ssusb/mode
TYPEC=/sys/class/power_supply/usb/typec_mode
ADDR=10.44.0.1/24
KICK_AFTER=10
KICK_EVERY=30
missing_since=0
last_kick=0

while true; do
  [ "$(cat $MODE)" = host ] || { echo host > $MODE; logger -t c3link "a600000 -> host"; }
  now=$(cut -d. -f1 /proc/uptime)
  have=0
  for d in /sys/class/net/*; do
    drv=$(readlink "$d/device/driver" 2>/dev/null)
    [ "${drv##*/}" = cdc_ncm ] && have=1
  done
  if [ $have = 0 ] && ! grep -q "Nothing attached" $TYPEC; then
    [ $missing_since = 0 ] && missing_since=$now
    if [ $((now - missing_since)) -ge $KICK_AFTER ] && [ $((now - last_kick)) -ge $KICK_EVERY ]; then
      echo none > $MODE; sleep 2; echo host > $MODE
      last_kick=$now
      logger -t c3link "attached ($(cat $TYPEC)) but no C4 card: cycled host mode"
    fi
  else
    missing_since=0
  fi
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
