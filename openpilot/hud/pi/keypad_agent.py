#!/usr/bin/env python3
"""藍牙小鍵盤（COIDEA，2 旋鈕＋4×4）→ 車機指令。

跑在 PiBar 上，跟 hud_agent.py 放同一個目錄。鍵盤配對在 Pi 上，每按一個鍵就 POST 一個指令到
車機 hud/server.py 的 /keypad，車機那邊再交給 card／selfdrived／plannerd（selfdrive/keypad.py）。

  第 1 排：1 MAIN ON   2 MAIN OFF   3 ENGAGE    4 DISENGAGE   （3／4＝方向盤中間鍵拆成兩顆）
  第 2 排：5 SET-／-10 6 RES+／+10  7 起步      8 （空）     （5／6 跟方向盤一樣：沒接管時接回）
  第 3 排：9 禮讓行人  0 禮讓救護車  A 前方事故  B 準備靠邊      ┐ 後窗 LED 手動字幕：顯示 10 秒、
  第 4 排：C 路況不熟  D 謝謝禮讓    E 不好意思  F 臨時停車      ┘ 再按同一顆取消（led_director.MANUAL）
  兩個旋鈕：不用（轉快會掉格，2026-10-11 實測）

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
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from led_director import MANUAL_FILE

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

# 第 3、4 排：後窗 LED 手動字幕，在 Pi 本機交給 led_agent（不經車機，車機斷線也能用）
LED_KEYS = {
  10: "m_ped",         # KEY_9  禮讓行人
  11: "m_ambulance",   # KEY_0  禮讓救護車
  30: "m_accident",    # KEY_A  前方事故
  48: "m_pull_over",   # KEY_B  準備靠邊
  46: "m_unfamiliar",  # KEY_C  路況不熟
  32: "m_thanks",      # KEY_D  謝謝禮讓
  18: "m_sorry",       # KEY_E  不好意思
  33: "m_temp_stop",   # KEY_F  臨時停車
}

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
  except urllib.error.URLError as e:
    return f"URLError({e.reason})"
  except Exception as e:
    return f"{type(e).__name__}({e})"


_led_seq = 0


def led_manual(key):
  """寫一筆按鍵給 led_agent（原子寫入）；顯示多久、再按取消都由 led_director 決定。"""
  global _led_seq
  _led_seq += 1
  tmp = MANUAL_FILE + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump({"seq": f"{os.getpid()}-{_led_seq}", "key": key, "t": time.monotonic()}, f)
  os.replace(tmp, MANUAL_FILE)


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
      if code in LED_KEYS:
        led_manual(LED_KEYS[code])
        say(f"led {LED_KEYS[code]}")
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
      t0 = time.monotonic()
      result = send(host, token, cmd) if host else "no car"
      took = time.monotonic() - t0
      if result != 200:
        host = car_host()    # it may have moved (DHCP); look again for the next press
      say(f"{cmd}: {result} ({age * 1000:.0f} ms after the press, {took * 1000:.0f} ms to send)")


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
