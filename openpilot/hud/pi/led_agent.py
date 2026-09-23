#!/usr/bin/env python3
"""後窗 LED：收車機 HUD 串流 → 決定畫面（led_director）→ 畫圖（led_render）→ 推給控制卡（led_card）。

跑在 PiBar 上，跟 hud_agent.py 放在同一個目錄。車機位址不自己找，讀 hud_agent 的
http://127.0.0.1:8080/host.json 的 c4 欄 —— 它已經處理好家用 WiFi／車機熱點的切換。

控制卡連不到時照樣跑：畫面照算、狀態照記（~/hud/led.log），只是不推。所以網卡還沒到、
Pi 還連不到卡的時候，開車也能先把「屏會顯示什麼」記下來。

  python3 led_agent.py                 # 正常跑
  python3 led_agent.py --card none     # 只記錄、完全不碰控制卡
  python3 led_agent.py --host 192.168.2.149 --card 192.168.22.1
"""
import argparse
import datetime
import json
import os
import socket
import sys
import threading
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import led_render as R
from led_card import Led
from led_director import Director

HOST_JSON = "http://127.0.0.1:8080/host.json"
STREAM_PORT = 8902
FPS = 6.0                 # 控制卡整張重建約 155 ms，6 格／秒剛好
STREAM_STALE = 3.0        # 秒沒收到串流就當作斷線，畫面回日期時間
CARD_RETRY = 5.0          # 控制卡推失敗後多久再試
CARD_TIMEOUT = 1.5
LOG = os.path.expanduser("~/hud/led.log")
LOG_MAX = 2_000_000


def say(msg):
  line = "{} {}".format(datetime.datetime.now().isoformat(timespec="seconds"), msg)
  print(line, flush=True)
  try:
    if os.path.exists(LOG) and os.path.getsize(LOG) > LOG_MAX:
      os.replace(LOG, LOG + ".1")
    with open(LOG, "a", encoding="utf-8") as f:
      f.write(line + "\n")
  except OSError:
    pass


class Stream(threading.Thread):
  """一直連著車機的 /stream，保留最新一格。斷了就重讀 host.json 再連。"""

  def __init__(self, fixed_host=None):
    super().__init__(daemon=True)
    self.fixed_host = fixed_host
    self.latest, self.latest_t, self.host = {}, 0.0, None

  def _find_host(self):
    if self.fixed_host:
      return self.fixed_host
    try:
      with urllib.request.urlopen(HOST_JSON, timeout=2) as r:
        return json.loads(r.read().decode()).get("c4") or None
    except Exception:
      return None

  def run(self):
    said = None
    while True:
      host = self._find_host()
      if host != said:
        say("car host %s" % (host or "-"))
        said = host
      if not host:
        time.sleep(2)
        continue
      self.host = host
      try:
        with urllib.request.urlopen(f"http://{host}:{STREAM_PORT}/stream", timeout=STREAM_STALE) as r:
          for raw in r:
            if raw.startswith(b"data: "):
              self.latest = json.loads(raw[6:])
              self.latest_t = time.monotonic()
      except (TimeoutError, OSError, ValueError):
        time.sleep(1)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--host", help="車機位址（預設讀 hud_agent 的 host.json）")
  ap.add_argument("--card", default="192.168.22.1", help="控制卡位址；none = 不推，只記錄")
  args = ap.parse_args()

  stream = Stream(args.host)
  stream.start()
  director = Director()
  led = None if args.card == "none" else Led(args.card, timeout=CARD_TIMEOUT)
  card_ok, card_next = None, 0.0
  last_key = None
  say(f"led_agent start card={args.card}")

  while True:
    tick = time.monotonic()
    fresh = tick - stream.latest_t < STREAM_STALE
    d: dict = stream.latest if fresh else {"standby": True}
    screen, shown = director.update(d, tick)

    if screen.key != last_key:
      cs, ctl = d.get("carState", {}), d.get("control", {})
      params = json.dumps(screen.params, ensure_ascii=False, default=str)
      v, a = cs.get("vEgo", 0.0), ctl.get("aTarget", 0.0)
      say(f"screen {screen.key:<11} {params} v={v:.1f} a={a:.2f} reason={ctl.get('reason', '')}")
      last_key = screen.key

    if led is not None and tick >= card_next:
      try:
        if card_ok is not True:                    # 斷線中：先快速探一下，別讓主迴圈卡 1.5 秒逾時
          socket.create_connection((args.card, 80), timeout=0.3).close()
        led.show(R.draw(screen, shown))
        if card_ok is not True:
          say(f"card up {args.card}")
        card_ok = True
      except Exception as e:
        if card_ok is not False:
          say(f"card down {args.card}: {e}")
        card_ok, card_next = False, tick + CARD_RETRY
        led.P = None                               # 下次重新登入

    time.sleep(max(0.0, 1 / FPS - (time.monotonic() - tick)))


if __name__ == "__main__":
  main()
