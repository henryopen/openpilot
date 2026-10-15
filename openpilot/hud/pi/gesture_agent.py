#!/usr/bin/env python3
"""手勢模組（emakefun GS20，YS4009，I2C 0x39）→ HUD。

跑在 PiBar 上，跟 hud_agent.py 放同一個目錄。現在只把讀到的手勢給 HUD 顯示（校正方向、看感應距離），
還不接任何控制。

接線（2026-10-15）：V→Pin 1（3.3V，不要接 5V：板上電平轉換讓 SDA/SCL 跟著 V 腳的電壓）、G→Pin 6、
SDA→Pin 3、SCL→Pin 5；/boot/firmware/config.txt 要 dtparam=i2c_arm=on。

協定（廠商 Arduino 函式庫）：讀暫存器 0x01 一個 byte＝最後一個手勢，讀了就清成 0。
  1 右移 2 左移 3 後移 4 前移 5 上拉 6 下壓 7 離開 8 懸停（方向是模組自己的，跟裝的角度有關）
其他暫存器的意思文件沒寫，原樣放進 raw 給 HUD 顯示，看有沒有跟手的距離一起變的。
"""
import json
import os
import time

from smbus2 import SMBus

ADDR = 0x39
REG_GESTURE = 0x01
RAW_REGS = range(0x08)
PERIOD = 0.05          # s
STATE_FILE = "/dev/shm/gesture_state.json"
STATE_LINK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gesture_state.json")
LOG = os.path.expanduser("~/hud/gesture.log")
LOG_MAX = 1_000_000
NAMES = {1: "右移", 2: "左移", 3: "後移", 4: "前移", 5: "上拉", 6: "下壓", 7: "離開", 8: "懸停"}


def say(msg):
  print(time.strftime("%H:%M:%S"), msg, flush=True)
  try:
    if os.path.exists(LOG) and os.path.getsize(LOG) > LOG_MAX:
      os.replace(LOG, LOG + ".1")
    with open(LOG, "a", encoding="utf-8") as f:
      f.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')}(pi) up={time.monotonic():.0f}s {msg}\n")
  except OSError:
    pass


def write_state(d):
  try:
    if not (os.path.islink(STATE_LINK) and os.readlink(STATE_LINK) == STATE_FILE):
      if os.path.lexists(STATE_LINK):
        os.remove(STATE_LINK)
      os.symlink(STATE_FILE, STATE_LINK)
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump(d, f)
    os.replace(tmp, STATE_FILE)
  except OSError as e:
    say(f"state: {e}")


def main():
  seq, last = 0, 0
  ok_before = None
  bus = None
  while True:
    raw, ok = None, False
    try:
      if bus is None:
        bus = SMBus(1)
      g = bus.read_byte_data(ADDR, REG_GESTURE)
      raw = [bus.read_byte_data(ADDR, r) for r in RAW_REGS if r != REG_GESTURE]
      ok = True
      if g:
        seq, last = seq + 1, g
        say(f"{g} {NAMES.get(g, '?')}")
    except OSError:
      if bus is not None:
        try:
          bus.close()
        except OSError:
          pass
      bus = None
    if ok != ok_before:
      say("module ok" if ok else "module not answering at 0x39")
      ok_before = ok
    write_state({"ok": ok, "seq": seq, "code": last, "name": NAMES.get(last, ""), "raw": raw})
    time.sleep(PERIOD if ok else 1.0)


if __name__ == "__main__":
  main()
