#!/usr/bin/env python3
"""藍牙小鍵盤（COIDEA，2 旋鈕＋4×4）→ 車機指令。

跑在 PiBar 上，跟 hud_agent.py 放同一個目錄。鍵盤配對在 Pi 上，每按一個鍵就 POST 一個指令到
車機 hud/server.py 的 /keypad，車機那邊再交給 card／selfdrived／plannerd（selfdrive/keypad.py）。

  第 1 排：1 MAIN ON   2 MAIN OFF   3 ENGAGE    4 DISENGAGE
  第 2 排：5 速度 -10  6 速度 +10   7 起步      8 （空）
  第 3、4 排（9 0 A B / C D E F）和兩個旋鈕：先不用（旋鈕轉快會掉格，2026-10-11 實測）

- 直接讀 /dev/input/eventN，不靠 python-evdev（pi 在 input 群組，讀得到）。
- 一定要 grab 獨佔：Pi 上有 HUD 瀏覽器，不獨佔的話按鍵會打進瀏覽器。
- 裝置用名稱找（eventN 每次連線可能不同），鍵盤睡眠斷線後會自己重找。
- 只看按下（value 1）；放開（0）和按住不放的自動重複（2）都不理。
- 按下超過 MAX_AGE 還沒送出去就丟掉，送不到也不重試：網路卡住後才送到的「起步」比沒送到危險。
- token 在 ~/.keypad_token，要跟車機 /data/keypad_token 一樣；車機沒有那個檔就整個不收。
"""
import fcntl
import glob
import json
import os
import struct
import time
import urllib.error
import urllib.request

DEVICE_NAME = "COIDEA KM Keyboard"
HOST_JSON = "http://127.0.0.1:8080/host.json"
CAR_PORT = 8902
TOKEN_FILE = os.path.expanduser("~/.keypad_token")
MAX_AGE = 0.5          # s
SEND_TIMEOUT = 0.5     # s

EV_KEY = 1
EVENT = struct.Struct("llHHi")   # struct input_event on aarch64: timeval, type, code, value
EVIOCGRAB = 0x40044590
EVIOCSCLOCKID = 0x400445a0
CLOCK_MONOTONIC = 1

KEYS = {
  2: "main_on",      # KEY_1
  3: "main_off",     # KEY_2
  4: "engage",       # KEY_3
  5: "disengage",    # KEY_4
  6: "speed_down",   # KEY_5
  7: "speed_up",     # KEY_6
  8: "go",           # KEY_7
}


def say(msg):
  print(time.strftime("%H:%M:%S"), msg, flush=True)


def find_device():
  for path in sorted(glob.glob("/sys/class/input/event*/device/name")):
    try:
      with open(path) as f:
        if f.read().strip() == DEVICE_NAME:
          return "/dev/input/" + path.split("/")[4]
    except OSError:
      continue
  return None


def car_host():
  try:
    with urllib.request.urlopen(HOST_JSON, timeout=1) as r:
      return json.loads(r.read().decode()).get("c4") or None
  except Exception:
    return None


def send(host, token, cmd):
  body = json.dumps({"cmd": cmd, "token": token}).encode()
  req = urllib.request.Request(f"http://{host}:{CAR_PORT}/keypad", data=body, method="POST",
                               headers={"Content-Type": "application/json"})
  try:
    with urllib.request.urlopen(req, timeout=SEND_TIMEOUT) as r:
      return r.status
  except urllib.error.HTTPError as e:
    return e.code
  except Exception as e:
    return type(e).__name__


def run(dev, token):
  host = car_host()
  with open(dev, "rb", buffering=0) as f:
    fcntl.ioctl(f, EVIOCGRAB, 1)
    # stamp the events on the monotonic clock: the Pi has no RTC and its wall clock jumps
    fcntl.ioctl(f, EVIOCSCLOCKID, struct.pack("i", CLOCK_MONOTONIC))
    say(f"keypad on {dev}, car {host or '-'}")
    while True:
      data = f.read(EVENT.size)
      if len(data) < EVENT.size:
        raise OSError("short read")
      sec, usec, etype, code, value = EVENT.unpack(data)
      if etype != EV_KEY or value != 1:
        continue
      cmd = KEYS.get(code)
      if cmd is None:
        continue
      age = time.monotonic() - (sec + usec / 1e6)
      if age > MAX_AGE:
        say(f"{cmd}: dropped, {age:.2f} s old")
        continue
      if host is None:
        host = car_host()
      result = send(host, token, cmd) if host else "no car"
      if result != 200:
        host = car_host()    # it may have moved (DHCP); look again for the next press
      say(f"{cmd}: {result}")


def main():
  try:
    with open(TOKEN_FILE) as f:
      token = f.read().strip()
  except OSError:
    token = ""
  if not token:
    raise SystemExit(f"no token in {TOKEN_FILE}")
  said_missing = False
  while True:
    dev = find_device()
    if dev is None:
      if not said_missing:
        say("keypad not connected")
        said_missing = True
      time.sleep(1)
      continue
    said_missing = False
    try:
      run(dev, token)
    except OSError as e:
      say(f"keypad gone ({e})")
      time.sleep(1)


if __name__ == "__main__":
  main()
