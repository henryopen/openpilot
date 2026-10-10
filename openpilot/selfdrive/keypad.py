"""Commands from the Bluetooth keypad on the PiBar, for the processes that act on them.

The Pi reads the keypad and posts each key as a command to the HUD server on the car
(hud/server.py, POST /keypad). The server appends it here, and card, selfdrived and plannerd
each pick up what is new for them. A file in /dev/shm rather than a param: two commands can land
between two reads (+10 twice in a row), and a single value would lose one of them.

The keypad stands in for the wheel buttons, but the panda cannot see it. So nothing here can
authorise the panda: engage only brings openpilot back while the panda still allows control,
which the wheel's RES/SET (or the accelerator at a standstill) is still needed for.

Every command is stamped with the car's monotonic clock when it arrives and is dropped by its
reader once older than MAX_AGE: a press that sat in a stalled hotspot must not start the car
seconds later. The Pi likewise gives up on a press it could not deliver within half a second.
"""
import json
import os
import threading
import time

PATH = "/dev/shm/keypad.json"
MAX_AGE = 1.0          # s
KEEP = 16              # commands kept in the file; far more than can arrive between two reads
COMMANDS = frozenset({"main_on", "main_off", "engage", "disengage", "speed_up", "speed_down", "go"})

_lock = threading.Lock()


def _read() -> list:
  try:
    with open(PATH) as f:
      entries = json.load(f)
  except (OSError, ValueError):
    return []
  if not isinstance(entries, list):
    return []
  return [e for e in entries if isinstance(e, dict) and isinstance(e.get("seq"), int) and isinstance(e.get("cmd"), str)]


def post(cmd: str) -> bool:
  """Append one command (hud/server.py). False if it is not a command."""
  if cmd not in COMMANDS:
    return False
  with _lock:
    entries = _read()[-(KEEP - 1):]
    seq = time.monotonic_ns()
    if entries:
      seq = max(seq, entries[-1]["seq"] + 1)
    entries.append({"seq": seq, "cmd": cmd})
    tmp = PATH + ".tmp"
    with open(tmp, "w") as f:
      json.dump(entries, f)
    os.replace(tmp, PATH)
  return True


class KeypadReader:
  """What has arrived for one process since it last asked. Cheap enough to call every frame:
  the file is only read when it has changed."""

  def __init__(self, wanted):
    self.wanted = frozenset(wanted)
    self.mtime: int | None = None
    self.last_seq = time.monotonic_ns()   # nothing from before this process started

  def poll(self) -> list[str]:
    try:
      mtime = os.stat(PATH).st_mtime_ns
    except OSError:
      return []
    if mtime == self.mtime:
      return []
    self.mtime = mtime

    oldest = time.monotonic_ns() - int(MAX_AGE * 1e9)
    out = []
    for e in _read():
      if e["seq"] <= self.last_seq:
        continue
      self.last_seq = e["seq"]
      if e["seq"] >= oldest and e["cmd"] in self.wanted:
        out.append(e["cmd"])
    return out
