#!/bin/bash
# On C3: keep a copy of C4's drives (/data/media/0/realdata) on C3's 916 GB NVMe, over the USB link (2026-10-18).
#
# C4's /data is 89 GB and 86% full, and its deleter removes the oldest drives; C3's NVMe was empty. C3 pulls,
# so C4 runs nothing new: rsync over ssh to 10.44.0.2 with the same key the PC uses (/data/c3link/c4key).
# Driving comes first on C4: its side of rsync runs at nice 19 / idle IO, and while C4 is onroad the pull is
# capped at ONROAD_KBPS (recording makes ~1.4 MB/s; ssh on C4 took 70% of a core at the full ~18 MB/s). Segments touched in the last 2 minutes - the one being
# recorded - are left for the next round. Nothing is ever deleted on C4; on C3 nvme_trim.py keeps 100 GB free,
# oldest segments first, of C3's own recordings and these copies together (11/02: the old trim here only removed
# C4's copies at 90 %, C3's own recordings filled the NVMe on 11/01 and C3 stopped recording).
C4=comma@10.44.0.2
KEY=/data/c3link/c4key
SRC=/data/media/0/realdata
DST=/data/media/0/c4_archive/realdata
ONROAD_KBPS=3000
EVERY=60
SSH="ssh -i $KEY -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o Ciphers=aes128-gcm@openssh.com -o Compression=no"

log() { logger -t c4_archive "$*"; }
mkdir -p "$DST"

while true; do
  if ping -c 1 -W 2 10.44.0.2 >/dev/null 2>&1; then
    onroad=$($SSH $C4 'cat /data/params/d/IsOnroad 2>/dev/null' 2>/dev/null)
    busy=$($SSH $C4 "find $SRC -mindepth 1 -maxdepth 1 -type d -mmin -2 -printf '%f\n'" 2>/dev/null)
    args=(-a --partial --exclude=boot --exclude='*.lock')
    for b in $busy; do args+=(--exclude="$b"); done
    [ "$onroad" = 1 ] && args+=(--bwlimit=$ONROAD_KBPS)
    before=$(du -sm "$DST" | cut -f1)
    t0=$(date +%s)
    nice -n 10 rsync "${args[@]}" -e "$SSH" --rsync-path="nice -n 19 ionice -c3 rsync" "$C4:$SRC/" "$DST/" \
      > /tmp/c4_archive.out 2>&1
    rc=$?
    [ $rc != 0 ] && log "rsync exit $rc: $(grep -v 'Warning: Permanently added' /tmp/c4_archive.out | head -3 | tr '\n' ' ')"
    after=$(du -sm "$DST" | cut -f1)
    [ "$after" != "$before" ] && log "copied $((after - before)) MB in $(( $(date +%s) - t0 )) s (onroad=${onroad:-0}, archive ${after} MB)"
  fi
  # keep 100 GB free on the NVMe: oldest segments first, C3's own recordings and C4's copies together
  python3 /data/c3link/nvme_trim.py || log "nvme_trim exit $?"
  sleep $EVERY
done
