#!/usr/bin/env python3
"""語音辨識模組（emakefun 語音識別 2.1，LD3320，I2C 0x30）→ 車機指令／後窗字幕，結果也給 HUD 顯示。

跑在 PiBar 上，跟 keypad_agent.py 放同一個目錄，送指令／字幕的方式跟藍牙鍵盤完全一樣（共用它的函式）。

接線（2026-10-15）：Pi 的 I2C（Pin 1 3.3V／3 SDA／5 SCL／6 GND）接一分四，手勢模組和這片都插在上面。
不要讓一分四吃 5V：模組的 SDA/SCL 跟著 V 腳的電壓。

模組只認拼音（內建普通話發音模型），英文要用拼音近似。關鍵詞喚醒模式：第 0 條是喚醒詞，喊了才聽後面的，
TIMEOUT_MS 沒聽到就回待命 —— 車上有人聊天、廣播，自動模式太容易誤觸。

協定（廠商 Arduino 函式庫 emakefun_speech_recognizer）：
  0x01 忙碌（0＝閒）  0x02 寫 1＝重置  0x03 模式（0 自動 1 按鍵 2 關鍵詞 3 按鍵或關鍵詞）
  0x04 結果 int16 LE（-1＝沒有）  0x06 事件  0x08 逾時 uint32 LE ms
  加詞：0x0C 序號 → 0x0D 拼音 → 0x3F 長度 → 0x40 寫 1；辨識：0x41 寫 1 後讀 0x04
"""
import json
import os
import struct
import sys
import time

from smbus2 import SMBus, i2c_msg

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from keypad_agent import TOKEN_FILE, car_host, led_manual, send

ADDR = 0x30
MODE_AUTO, MODE_KEYWORD = 0, 2
# 2026-10-15 使用者：開車時沒辦法先喊喚醒詞 → 先用自動模式（一直在聽）試辨識率。誤觸風險（聊天、收音機
# 說到「前進」就送 GO）待實測後再決定 GO 留不留在語音、詞要不要拉長。
MODE = MODE_AUTO
TIMEOUT_MS = 10000
PERIOD = 0.1           # s

# 序號 → (拼音, 顯示文字, 動作)。動作：("car", 指令) 送車機（同鍵盤）、("led", 字幕) 後窗字幕（同鍵盤 9-F）
WAKE = ("xiao chen xiao chen", "小陳小陳", None)
WORDS = [
  ("jia su",   "加速",  ("car", "max_up")),        # MAX +10（同左旋鈕：不會接回）
  ("jian su",  "減速",  ("car", "max_down")),      # MAX -10
  ("lv deng", "綠燈", ("car", "go")),            # 起步（同鍵盤 7）。10/15 「前進」常把加速、減速、謝謝吸走、「走走」被判成停車；「綠燈出發」→「綠燈」（使用者）
  ("xie xie",  "謝謝",  ("led", "m_thanks")),
  ("bao qian", "抱歉",  ("led", "m_sorry")),       # 字幕「不好意思」
  ("ting che", "停車",  ("led", "m_temp_stop")),   # 字幕「臨時停車」
]
# 垃圾詞：這顆晶片聽到任何聲音都會硬配到清單裡最像的一條，沒有「都不像」。10/15 實測講清單外的「走走」
# 被判成停車、講話開頭含糊時常被判成前進。清單多放日常用語和語助詞把那些聲音吸走（LD3320 的標準做法），
# 配到它們什麼都不做。
# 只用兩個音節以上的：10/15 15:21-15:27 單音節的 en（嗯）6 分鐘吸走 22 次、a 3 次，把每句開頭都吃掉了，
# 指令只剩 3 次 —— 看起來像「每句要說兩次」。
FILLERS = ["ni hao", "shen me", "zhe ge", "na ge", "hao de", "bu shi",
           "ran hou", "zen me", "mei you", "zhi dao", "ke yi", "xian zai", "wo men", "ta men"]
KEYWORDS = [WAKE] + WORDS + [(f, f, None) for f in FILLERS]

EVENTS = {0: "", 1: "待命", 2: "按鍵喚醒", 3: "喚醒", 4: "聆聽中", 5: "辨識成功", 6: "逾時"}
STATE_FILE = "/dev/shm/speech_state.json"
STATE_LINK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "speech_state.json")
LOG = os.path.expanduser("~/hud/speech.log")
LOG_MAX = 1_000_000


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
      json.dump(d, f, ensure_ascii=False)
    os.replace(tmp, STATE_FILE)
  except OSError as e:
    say(f"state: {e}")


class Module:
  def __init__(self, bus):
    self.bus = bus

  def write(self, data):
    self.bus.i2c_rdwr(i2c_msg.write(ADDR, list(data)))

  def read(self, reg, n):
    # the library writes the register and reads in two separate transactions, not a repeated start
    self.write([reg])
    m = i2c_msg.read(ADDR, n)
    self.bus.i2c_rdwr(m)
    return bytes(m)

  def wait_idle(self, limit=5.0):
    t0 = time.monotonic()
    while self.read(0x01, 1)[0] != 0:
      if time.monotonic() - t0 > limit:
        raise OSError("module stays busy")
      time.sleep(0.01)

  def setup(self):
    self.write([0x02, 1])
    time.sleep(0.05)
    self.wait_idle()
    self.write([0x03, MODE])
    self.wait_idle()
    self.write([0x08] + list(struct.pack("<I", TIMEOUT_MS)))
    for i, (pinyin, _, _) in enumerate(KEYWORDS):
      data = pinyin.encode("ascii")[:50]
      self.wait_idle()
      self.write([0x0C, i])
      self.write([0x0D] + list(data))
      self.write([0x3F, len(data)])
      self.write([0x40, 1])
    self.wait_idle()

  def recognize(self):
    self.wait_idle()
    self.write([0x41, 1])
    return struct.unpack("<h", self.read(0x04, 2))[0]

  def event(self):
    return self.read(0x06, 1)[0]


def act(action, token, host):
  kind, what = action
  if kind == "led":
    led_manual(what)
    return host, "led ok"
  if host is None:
    host = car_host()
  result = send(host, token, what) if host else "no car"
  if result != 200:
    host = car_host()
  return host, result


def main():
  try:
    with open(TOKEN_FILE) as f:
      token = f.read().strip()
  except OSError:
    token = ""
  host = None
  seq, last, awake, event_name = 0, None, False, ""
  ok_before = None
  while True:
    mod = None
    try:
      mod = Module(SMBus(1))
      mod.setup()
      say(f"module ready, {len(KEYWORDS)} keywords, " + ("always listening" if MODE == MODE_AUTO else f"wake word '{WAKE[1]}'"))
      ok_before = True
      while True:
        r = mod.recognize()
        ev = mod.event()
        if ev:
          event_name = EVENTS.get(ev, str(ev))
          awake = MODE != MODE_AUTO and ev in (2, 3, 4)   # auto mode is always 'recognizing'; nothing to show
          if ev in (3, 6):
            say(f"event {event_name}")
        if 1 <= r < len(KEYWORDS) and KEYWORDS[r][2] is None:
          say(f"heard {r} (filler '{KEYWORDS[r][0]}')")
        elif 1 <= r < len(KEYWORDS):
          _, text, action = KEYWORDS[r]
          seq, last = seq + 1, text
          host, result = act(action, token, host) if token or action[0] == "led" else (host, "no token")
          say(f"heard {r} {text} -> {action[1]}: {result}")
        elif r == 0 and MODE != MODE_AUTO:
          awake = True
        write_state({"ok": True, "seq": seq, "text": last, "awake": awake, "event": event_name})
        time.sleep(PERIOD)
    except OSError as e:
      if ok_before is not False:
        say(f"module not answering at 0x30 ({e})")
      ok_before = False
      write_state({"ok": False, "seq": seq, "text": last, "awake": False, "event": ""})
      try:
        if mod is not None:
          mod.bus.close()
      except OSError:
        pass
      time.sleep(2)


if __name__ == "__main__":
  main()
