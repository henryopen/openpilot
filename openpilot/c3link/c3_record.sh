#!/bin/bash
# C3 drive recorder: openpilot's encoderd + loggerd (no manager), only while C4 is driving (2026-11-02).
#
# Until 11/02 it recorded whenever C3 had power - parked in the office too - two cameras at ~150 MB a minute, and the
# 916 GB NVMe held ~4 days (it filled on 11/01). The user: record only while C4 drives. "Driving" is c3calib's flag in
# /dev/shm/c3calib.json (C4 sent carState over the USB link in the last 5 s - the same flag c3lights works by); a
# stale file (c3calib not running) counts as not driving. Recording stops after STOP_AFTER s without it, so a link
# hiccup or a long red light does not cut the drive into pieces; the service then exits 0, systemd starts it again
# and it waits for the next drive.
# If EITHER encoderd or loggerd exits, both go and the service exits 1 and is restarted: 10/23 encoderd died at boot
# (the hardware encoder would not open yet) while loggerd ran on, and a whole boot had logs and no video.
STATE=/dev/shm/c3calib.json
STOP_AFTER=60
LOGGERD=/data/openpilot/system/loggerd

driving() {
  [ -f "$STATE" ] || return 1
  [ $(( $(date +%s) - $(stat -c %Y "$STATE") )) -le 15 ] || return 1
  grep -q '"driving": true' "$STATE"
}

while ! driving; do sleep 2; done
echo "$(date '+%F %T') c3-record: C4 is driving, recording"
$LOGGERD/encoderd & E=$!
sleep 4
$LOGGERD/loggerd & L=$!
idle=0
while kill -0 $E 2>/dev/null && kill -0 $L 2>/dev/null; do
  sleep 2
  if driving; then idle=0; else idle=$((idle + 2)); fi
  if [ $idle -ge $STOP_AFTER ]; then
    echo "$(date '+%F %T') c3-record: C4 not driving for ${STOP_AFTER} s, stopping"
    kill -TERM $L; wait $L
    kill -TERM $E; wait $E
    exit 0
  fi
done
echo "$(date '+%F %T') c3-record: encoderd or loggerd exited, restarting both"
kill $E $L 2>/dev/null
exit 1
