#!/bin/bash
# C4 side of the C3 <-> C4 USB link (2026-10-18).
#
# C4's second USB-C port (dual-role controller a600000) is forced to peripheral and presents a USB NCM
# network card through configfs; C3 (c3_host.sh) is the host and sees it as cdc_ncm. C4 is 10.44.0.2/24,
# C3 10.44.0.1. NetworkManager leaves usb0 alone (c4link-nm.conf).
#
# Like on C3, plugging the cable in did not bring the link up by itself (10/18): the gadget had to be
# unbound and bound again once the cable was in. When Type-C says something is attached and usb0 has had no
# carrier for KICK_AFTER seconds, rebind, at most every KICK_EVERY seconds.
MODE=/sys/devices/platform/soc/a600000.ssusb/mode
TYPEC=/sys/class/power_supply/usb/typec_mode
UDC=a600000.dwc3
KICK_AFTER=10
KICK_EVERY=30
down_since=0
last_kick=0
G=/sys/kernel/config/usb_gadget/c3link
ADDR=10.44.0.2/24

setup() {
  mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config
  [ -d $G/functions/ncm.usb0 ] && return
  mkdir -p $G && cd $G || return
  echo 0x1d6b > idVendor          # Linux Foundation
  echo 0x0104 > idProduct         # multifunction composite gadget
  mkdir -p strings/0x409 configs/c.1/strings/0x409 functions/ncm.usb0
  echo c4link > strings/0x409/serialnumber
  echo comma > strings/0x409/manufacturer
  echo "C4 link" > strings/0x409/product
  echo ncm > configs/c.1/strings/0x409/configuration
  # (fixed MACs via dev_addr/host_addr: this kernel answers ENODEV - C3 goes by the cdc_ncm driver, not the MAC)
  ln -s functions/ncm.usb0 configs/c.1/
  logger -t c4link "gadget created"
}

while true; do
  [ "$(cat $MODE)" = peripheral ] || { echo peripheral > $MODE; logger -t c4link "a600000 -> peripheral"; }
  setup
  [ "$(cat $G/UDC 2>/dev/null)" = $UDC ] || { echo $UDC > $G/UDC && logger -t c4link "bound to $UDC"; }
  n=$(cat $G/functions/ncm.usb0/ifname 2>/dev/null)
  if [ -n "$n" ] && ! ip -4 addr show dev "$n" | grep -q "${ADDR%/*}/"; then
    ip addr add $ADDR dev "$n" && ip link set "$n" up && logger -t c4link "$n up $ADDR"
  fi
  now=$(cut -d. -f1 /proc/uptime)
  if [ -n "$n" ] && [ "$(cat /sys/class/net/$n/carrier 2>/dev/null)" != 1 ] && ! grep -q "Nothing attached" $TYPEC; then
    [ $down_since = 0 ] && down_since=$now
    if [ $((now - down_since)) -ge $KICK_AFTER ] && [ $((now - last_kick)) -ge $KICK_EVERY ]; then
      echo "" > $G/UDC; sleep 1; echo $UDC > $G/UDC
      last_kick=$now
      logger -t c4link "attached ($(cat $TYPEC)) but no carrier: rebound the gadget"
    fi
  else
    down_since=0
  fi
  sleep 5
done
