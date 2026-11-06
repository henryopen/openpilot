#!/usr/bin/env python3
"""boot_probe: time the boot, to find where the minutes before onroad actually go.

Reads /proc only - it starts no work of its own and touches nothing openpilot owns. Every
process start time comes from /proc/<pid>/stat field 22, which is measured in clock ticks
since boot, so the numbers are immune to the clock jumping when time finally syncs (which
is what made the swaglog timestamps read as Jul 28).

Writes one small JSON per boot and then exits, so it cannot fill the disk.
"""
import json
import os
import time
from datetime import datetime
from pathlib import Path

OUT = Path('/data/yolo/bootlog')
HZ = os.sysconf('SC_CLK_TCK')
MAX_MINUTES = 20          # stop watching regardless, so a parked car does not log forever
# loggerd is not the onroad marker it looks like - it runs offroad too, which cut the first
# recording short at 71s. IsOnroad is what manager itself sets.
ONROAD_PARAM = Path('/data/params/d/IsOnroad')
QUIET_AFTER_ONROAD = 90   # modeld's first model load is the thing we are trying to catch

WATCH = ('launch_chffrplus', 'agnos', 'scons', 'build.py', 'manager.py', 'camerad',
         'modeld', 'dmonitoringmodeld', 'controlsd', 'card', 'plannerd', 'radard',
         'selfdrived', 'loggerd', 'encoderd', 'ui.py', 'sensord', 'pandad', 'locationd',
         'calibrationd', 'paramsd', 'torqued', 'updated', 'athenad', 'mapd',
         'yolod', 'boot_probe')


def uptime():
  with open('/proc/uptime') as f:
    return float(f.read().split()[0])


def started_at(pid):
  """When this process started, in seconds since boot."""
  try:
    with open(f'/proc/{pid}/stat') as f:
      after_comm = f.read().rsplit(') ', 1)[1].split()
    return int(after_comm[19]) / HZ
  except (OSError, IndexError, ValueError):
    return None


def cmdline(pid):
  try:
    with open(f'/proc/{pid}/cmdline', 'rb') as f:
      return f.read().replace(b'\0', b' ').decode(errors='replace').strip()
  except OSError:
    return ''


def scan(seen):
  """Record the first sighting of anything we care about."""
  for pid in os.listdir('/proc'):
    if not pid.isdigit() or pid in seen:
      continue
    cmd = cmdline(pid)
    if not cmd:
      continue
    hit = next((w for w in WATCH if w in cmd), None)
    if hit is None:
      continue
    t = started_at(pid)
    if t is None:
      continue
    seen[pid] = {'name': hit, 'start_s': round(t, 2), 'cmd': cmd[:120]}


def dump(path, boot_wall, onroad_at, seen, samples):
  first = {}
  for v in seen.values():
    if v['name'] not in first or v['start_s'] < first[v['name']]['start_s']:
      first[v['name']] = v
  path.write_text(json.dumps({
    'boot_wall_time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(boot_wall)),
    'onroad_at_s': round(onroad_at, 1) if onroad_at else None,
    'watched_first_seen': dict(sorted(first.items(), key=lambda kv: kv[1]['start_s'])),
    'trace': samples,
  }, indent=1))


def main():
  OUT.mkdir(parents=True, exist_ok=True)
  boot_wall = datetime.now().timestamp() - uptime()
  # The clock is still ~31 days out this early, so naming the file now would date it wrong
  # (the first recording landed as boot_20260728). Write to a fixed name and rename at the
  # end, by which point the clock has synced.
  path = OUT / 'boot_current.json'

  seen: dict[str, dict] = {}
  samples = []
  onroad_at = None
  t_start = uptime()

  while True:
    up = uptime()
    scan(seen)

    if onroad_at is None:
      onroad = any(v['name'] == 'camerad' for v in seen.values())  # onroad-only, unlike loggerd
      try:
        onroad = onroad or ONROAD_PARAM.read_bytes().strip() == b'1'
      except OSError:
        pass
      if onroad:
        onroad_at = up

    # a coarse load trace, enough to tell "compiling" from "waiting for something"
    if len(samples) < 400:
      with open('/proc/loadavg') as f:
        load = f.read().split()[0]
      samples.append({'up': round(up, 1), 'load': float(load),
                      'procs': len(seen), 'clock_offset': round(datetime.now().timestamp() - up - boot_wall, 1)})

    done = (up - t_start > MAX_MINUTES * 60) or \
           (onroad_at is not None and up - onroad_at > QUIET_AFTER_ONROAD)
    # write as we go: a boot that ends with the ignition going off is exactly the one worth
    # having, and a report that only lands at the end would be lost precisely then
    if done or len(samples) % 5 == 0:
      dump(path, boot_wall, onroad_at, seen, samples)
    if done:
      break
    time.sleep(2)

  synced_boot = datetime.now().timestamp() - uptime()
  final = OUT / f'boot_{time.strftime("%Y%m%d_%H%M%S", time.localtime(synced_boot))}.json'
  dump(final, synced_boot, onroad_at, seen, samples)
  path.unlink(missing_ok=True)
  print(f'boot_probe: wrote {final}')


if __name__ == '__main__':
  main()
