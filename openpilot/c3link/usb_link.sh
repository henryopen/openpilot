#!/bin/bash
# C3 <-> C4 wired network over the second USB-C ports - the same script on both, with this device's address:
#   C3: usb_link.sh 10.44.0.1      C4: usb_link.sh 10.44.0.2
#
# The second USB-C port is on the dual-role controller a600000; its Type-C side is the PMI8998 charger
# (checked 10/18 with a phone on each: CC, orientation and PD all seen there). Both ends are dual power
# role, so Type-C makes exactly one of them the 5 V source - which one changes from one plug-in to the next
# (10/18: C3 the first time, C4 after a replug), and the power role cannot be pinned on this kernel
# (typec_power_role is read-only). Forcing C3 to always be the USB host fought the charger whenever C3
# ended up the sink, and the link never came up. So the data role follows the power role, as Type-C
# intends: the source is the host and sees the other's NCM card (cdc_ncm); the sink presents that card
# (configfs NCM gadget). Each device keeps its own address on whichever interface it has.
#
# Right after a plug-in the link did not come up by itself (10/18, both times): the host's port had to be
# cycled and the gadget rebound. So when attached but not up after a while, do that again - the sink at
# 10 s and every 40 s, the source at 20 s and every 40 s, staggered so the two do not keep resetting each
# other.
ADDR=${1:?usage: usb_link.sh <this device address, e.g. 10.44.0.1>}/24
MODE=/sys/devices/platform/soc/a600000.ssusb/mode
TYPEC=/sys/class/power_supply/usb/typec_mode
UDC=a600000.dwc3
G=/sys/kernel/config/usb_gadget/c3link
KICK_EVERY=40
role=""
next_kick=0

log() { logger -t usb_link "$*"; }

gadget_setup() {
  mountpoint -q /sys/kernel/config || mount -t configfs none /sys/kernel/config
  [ -d $G/functions/ncm.usb0 ] && return
  mkdir -p $G && cd $G || return
  echo 0x1d6b > idVendor          # Linux Foundation
  echo 0x0104 > idProduct         # multifunction composite gadget
  mkdir -p strings/0x409 configs/c.1/strings/0x409 functions/ncm.usb0
  echo "c3link-$(hostname)" > strings/0x409/serialnumber
  echo comma > strings/0x409/manufacturer
  echo "C3/C4 link" > strings/0x409/product
  echo ncm > configs/c.1/strings/0x409/configuration
  ln -s functions/ncm.usb0 configs/c.1/
  cd / || return
  log "gadget created"
}

gadget_bind()   { [ "$(cat $G/UDC 2>/dev/null)" = $UDC ] || echo $UDC > $G/UDC; }
gadget_unbind() { [ -n "$(cat $G/UDC 2>/dev/null)" ] && echo "" > $G/UDC; }

# the interface the link runs on in the current role, if it exists
link_if() {
  if [ "$role" = host ]; then
    for d in /sys/class/net/*; do
      drv=$(readlink "$d/device/driver" 2>/dev/null)
      [ "${drv##*/}" = cdc_ncm ] && { echo "${d##*/}"; return; }
    done
  elif [ "$role" = peripheral ]; then
    cat $G/functions/ncm.usb0/ifname 2>/dev/null
  fi
}

gadget_setup
while true; do
  now=$(cut -d. -f1 /proc/uptime)
  case "$(cat $TYPEC)" in
    "Sink attached"*)   want=host ;;         # the other end is the sink: we are the source
    "Source attached"*) want=peripheral ;;   # we are the sink
    *)                  want=idle ;;
  esac

  if [ "$want" != "$role" ]; then
    case $want in
      host)       gadget_unbind; echo host > $MODE ;;
      peripheral) echo peripheral > $MODE; gadget_bind ;;
      idle)       gadget_unbind; echo none > $MODE ;;
    esac
    log "$(cat $TYPEC) -> $want"
    role=$want
    next_kick=$((now + 10)); [ "$role" = host ] && next_kick=$((now + 20))
  fi
  # the charger can switch the controller behind our back: keep it where the role says
  case $role in
    host)       [ "$(cat $MODE)" = host ] || echo host > $MODE ;;
    peripheral) [ "$(cat $MODE)" = peripheral ] || echo peripheral > $MODE; gadget_bind ;;
  esac

  n=$(link_if)
  if [ -n "$n" ]; then
    ip -4 addr show dev "$n" | grep -q "${ADDR%/*}/" || { ip addr add $ADDR dev "$n" && log "$n $ADDR"; }
    ip link set "$n" up 2>/dev/null
  fi
  up=0
  [ -n "$n" ] && [ "$(cat /sys/class/net/$n/carrier 2>/dev/null)" = 1 ] && up=1

  if [ "$role" != idle ] && [ $up = 0 ] && [ $now -ge $next_kick ]; then
    if [ "$role" = host ]; then
      echo none > $MODE; sleep 2; echo host > $MODE
    else
      echo "" > $G/UDC; sleep 1; echo $UDC > $G/UDC
    fi
    next_kick=$((now + KICK_EVERY))
    log "$role, link not up: kicked"
  fi
  sleep 2
done
