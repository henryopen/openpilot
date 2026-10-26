#!/usr/bin/env python3
"""後窗 LED：收車機 HUD 串流 → 決定畫面（led_director）→ 畫圖（led_render）→ 推給控制卡（led_card）。

跑在 PiBar 上，跟 hud_agent.py 放在同一個目錄。車機位址不自己找，讀 hud_agent 的
http://127.0.0.1:8080/host.json 的 c4 欄 —— 它已經處理好家用 WiFi／車機熱點的切換。

控制卡連不到時照樣跑：畫面照算、狀態照記（~/hud/led.log），只是不推。所以網卡還沒到、
Pi 還連不到卡的時候，開車也能先把「屏會顯示什麼」記下來。

  python3 led_agent.py                 # 正常跑（經 ESP32-C3 中繼推給控制卡）
  python3 led_agent.py --card none     # 只記錄、完全不碰控制卡
  python3 led_agent.py --card 192.168.2.148   # 直接走網路（例如辦公室拉網路線到卡）
  python3 led_agent.py --host 192.168.2.149 --card 192.168.22.1
"""
import argparse
import datetime
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import led_render as R
from led_card import Led
from led_director import MANUAL_FILE, NAV_FILE, Director

HOST_JSON = "http://127.0.0.1:8080/host.json"
STREAM_PORT = 8902
FPS = 6.0                 # 控制卡整張重建約 155 ms，6 格／秒剛好
STREAM_STALE = 3.0        # 秒沒收到串流就當作斷線，畫面回日期時間
CARD_RETRY = 5.0          # 控制卡推失敗後多久再試
SLOW_LIMIT = 3            # 連續這麼多張逾時才算斷線（偶爾一張慢只跳過）
CARD_TIMEOUT = 1.5
LOG = os.path.expanduser("~/hud/led.log")
# HUD 上看得到後窗 LED 正在顯示什麼（駕駛在車裡看不到後窗）：每幾格把同一張圖寫到 RAM，
# hud_agent 的 :8080 從 ~/hud 供應靜態檔，那裡放兩個 symlink 指過來，dash.html 同源就讀得到。
# 寫 /dev/shm 不寫 SD 卡：一秒三次，寫卡會磨損。
MIRROR_DIR = "/dev/shm"
MIRROR_FPS = 3.0
HUD_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_MAX = 2_000_000


TW = datetime.timezone(datetime.timedelta(hours=8))
# PI 在車上連車機熱點、沒有網路，RTC 也沒電池，自己的時鐘停在上次關機的時間（09-25 那趟慢了 15 小時）。
# 所以 PI 上顯示和記錄的時間一律從車機來：收到車機的可信時間（clock.ok）就記下「車機 epoch ↔ 本機
# monotonic」對照點，之後用 monotonic 往下推 —— 串流斷一下也照樣準，PI 自己的時鐘跳動也不影響。
# 對照點同時寫到 CAR_CLOCK_FILE 給 hud_agent 的 net.log 用（monotonic 在同一台機器各程序間共通）。
CAR_CLOCK_FILE = os.path.join(MIRROR_DIR, "car_clock.json")
_car_anchor = None   # (車機 epoch, 本機 monotonic)
_anchor_written = 0.0


def note_car_clock(d, recv_mono):
  global _car_anchor, _anchor_written
  ck = d.get("clock") or {}
  ep = ck.get("epoch")
  if not ck.get("ok") or not isinstance(ep, (int, float)):
    return
  _car_anchor = (float(ep), recv_mono)
  if recv_mono - _anchor_written > 10.0:
    _anchor_written = recv_mono
    try:
      tmp = CAR_CLOCK_FILE + ".tmp"
      with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"epoch": ep, "mono": recv_mono}, f)
      os.replace(tmp, CAR_CLOCK_FILE)
    except OSError:
      pass


def car_now():
  """車機的時間（台灣時區）；開機後還沒拿到過車機的可信時間就回 None。"""
  a = _car_anchor
  if a is None:
    return None
  return datetime.datetime.fromtimestamp(a[0] + (time.monotonic() - a[1]), TW)


def stamp():
  """log 時間戳：車機時間；還沒拿到就用 PI 自己的時間並標 (pi)，看 log 的人知道那段不可信。"""
  now = car_now()
  if now is not None:
    return now.strftime("%Y-%m-%dT%H:%M:%S")
  return datetime.datetime.now().isoformat(timespec="seconds") + "(pi)"


def say(msg):
  line = f"{stamp()} {msg}"
  print(line, flush=True)
  try:
    if os.path.exists(LOG) and os.path.getsize(LOG) > LOG_MAX:
      os.replace(LOG, LOG + ".1")
    with open(LOG, "a", encoding="utf-8") as f:
      f.write(line + "\n")
  except OSError:
    pass


def mirror_setup():
  for name in ("led_now.png", "led_now.json"):
    link, target = os.path.join(HUD_DIR, name), os.path.join(MIRROR_DIR, name)
    try:
      if os.path.islink(link) and os.readlink(link) == target:
        continue
      if os.path.lexists(link):
        os.remove(link)
      os.symlink(target, link)
    except OSError as e:
      say(f"mirror link {name}: {e}")


class ManualKeys:
  """鍵盤的字幕鍵：keypad_agent 每按一下就寫一次 MANUAL_FILE（seq 遞增），這裡把新的交給 Director。"""
  MAX_AGE = 2.0          # 秒：比這舊的按鍵不算（例如 led_agent 重啟時檔案裡留著的那一下）

  def __init__(self):
    self.seq = None

  def poll(self, director, now):
    try:
      with open(MANUAL_FILE, encoding="utf-8") as f:
        m = json.load(f)
    except (OSError, ValueError):
      return
    if not isinstance(m, dict) or m.get("seq") == self.seq:
      return
    self.seq = m.get("seq")
    # 舊的（led_agent 重啟前留下的）靠 MAX_AGE 擋；別用「啟動後第一筆不算」——檔案還沒出現過時，
    # 那會把啟動後的第一次按鍵吃掉（10/11 端到端測試抓到）
    if now - float(m.get("t", 0.0)) <= self.MAX_AGE:
      director.manual_press(m.get("key"), now)
      say(f"manual {m.get('key')} -> {director.manual or 'off'}")


class C3VoiceCaptions:
  """C3 語音的字幕（c3link/c3voice.py，10/23）：小白謝謝／抱歉／臨時；10/26 起鍵盤第 3、4 排的八個字幕全部都有，
  另加「小白取消」關掉正在顯示的字幕（常駐的那幾個原本要再按同一顆才關）。C3 聽到的經 C4 的 HUD 資料流過來
  （c3.voice：n＝C3 累計聽到幾次、last＝最後一句、ago＝幾秒前），n 變了就是新的一次。交給跟鍵盤字幕同一個入口，
  所以再說一次同一句一樣是取消。"""
  KEYS = {"小白謝謝": "m_thanks", "小白抱歉": "m_sorry", "小白臨時": "m_temp_stop",
          "小白禮讓行人": "m_ped", "小白救護車": "m_ambulance", "小白事故": "m_accident",
          "小白靠邊": "m_pull_over", "小白不熟": "m_unfamiliar", "小白取消": "off"}
  MAX_AGO = 3.0          # 秒：比這舊的不算（例如 led_agent 重啟、C3 斷線又接回時留著的那一句）

  def __init__(self):
    self.n = None

  def poll(self, director, d, now):
    v = (d.get("c3") or {}).get("voice") if isinstance(d, dict) else None
    if not isinstance(v, dict) or "n" not in v:
      return
    n, self.n = self.n, v.get("n")
    if n is None or n == self.n:
      return
    key, ago = self.KEYS.get(v.get("last")), v.get("ago")
    if key and isinstance(ago, (int, float)) and ago <= self.MAX_AGO:
      director.manual_press(key, now)
      say(f"c3 voice {v.get('last')} -> {director.manual or 'off'}")


def nav_poll(director, now):
  """導航：hud_agent 每秒把 D531 NavDash 的 /nav 寫到 NAV_FILE（mono = 讀到的那一刻，同一台機器的
  monotonic）。讀不到檔（hud_agent 沒在跑）就當作沒在導航。"""
  try:
    with open(NAV_FILE, encoding="utf-8") as f:
      n = json.load(f)
    age = now - float(n["mono"])
    nav = n.get("nav")
  except (OSError, ValueError, KeyError, TypeError):
    nav, age = None, None
  director.nav_update(nav, age, now)


def relay_diag(led):
  """Right after the card drops: is the relay itself still up, how long since it booted, and how
  strong the card's AP is from where it sits - and whether the Pi has seen under-voltage. 10/11
  the link dropped every few seconds to minutes all drive and the log could not say whether that
  was signal, power or the board resetting. up small = the board rebooted on its own (power or a
  crash); rssi low = signal; throttled bit 0x1/0x10000 = the Pi's supply sagged."""
  parts = []
  relay = getattr(led, "relay", None)
  if relay is not None and relay.s is not None:
    # The card's late answer to the frame that timed out can arrive first and be read as this reply
    # (10/12: every diag read wifi=None - it was the card's JSON). Ask again until the relay's own
    # answer, the one with "wifi", comes back.
    i, err = None, None
    for _ in range(3):
      try:
        r = relay.info(timeout=0.8)
        if isinstance(r, dict) and "wifi" in r:
          i = r
          break
      except Exception as e:
        err = e
    if i is not None:
      parts.append(f"relay wifi={i.get('wifi')} rssi={i.get('rssi')} up={i.get('up_ms', 0) / 1000:.0f}s heap={i.get('heap')}")
    else:
      parts.append(f"relay silent ({err})" if err else "relay answered with the card's late reply only")
  parts.append(f"pi {throttled()}")
  return " | " + " | ".join(parts)


def throttled():
  try:
    out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout.strip()
    return out or "throttled=?"
  except (OSError, subprocess.SubprocessError):
    return "throttled=?"


CARD_CON = "led-card"       # NM connection of the Pi's USB wifi (wlan1) to the card's own AP, since 10/19
CARD_MAC = "98:48:27:e0:4b:7f"   # the USB wifi (rtl8821au). 10/25: found by MAC, not by name - on a cold boot it can
                                 # come up as wlan0 (the led-card NM profile is bound to this MAC too)


def card_iface() -> str:
  try:
    for name in os.listdir("/sys/class/net"):
      try:
        with open(f"/sys/class/net/{name}/address") as f:
          if f.read().strip().lower() == CARD_MAC:
            return name
      except OSError:
        continue
  except OSError:
    pass
  return "wlan1"
WIFI_CHECK_EVERY = 15.0
WIFI_LOG_EVERY = 120.0


def card_wifi_watch(card_down):
  """While the card is down and the card's adapter is sitting disconnected: ask that adapter for a scan, and log
  whether the card's AP is on the air. 10/23 the adapter sat disconnected for 290 s after boot and then joined in
  2 s, with nothing in the logs to say whether the AP was not up yet or NetworkManager had not looked. A scan on a
  disconnected adapter costs nothing (with the card joined it would cost the card's replies - see hud_agent net_list)."""
  try:
    ssid = subprocess.run(["nmcli", "-g", "802-11-wireless.ssid", "con", "show", CARD_CON],
                          capture_output=True, text=True, timeout=5).stdout.strip()
  except (OSError, subprocess.SubprocessError):
    ssid = ""
  said, said_t = None, 0.
  while True:
    time.sleep(WIFI_CHECK_EVERY)
    if not ssid or not card_down():
      said = None
      continue
    try:
      state = subprocess.run(["nmcli", "-g", "GENERAL.STATE", "dev", "show", card_iface()],
                             capture_output=True, text=True, timeout=5).stdout.strip()
      if not state.startswith("30"):        # 30 = disconnected; connecting or connected is left alone
        continue
      subprocess.run(["nmcli", "dev", "wifi", "rescan", "ifname", card_iface()], capture_output=True, timeout=10)
      time.sleep(5)
      out = subprocess.run(["nmcli", "-t", "-f", "SSID,SIGNAL,CHAN", "dev", "wifi", "list", "ifname", card_iface(),
                            "--rescan", "no"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError) as e:
      say(f"card wifi check failed: {e}")
      continue
    seen = [ln.rsplit(":", 2) for ln in out.splitlines() if ln.rsplit(":", 2)[0] == ssid]
    now = time.monotonic()
    if bool(seen) != said or now - said_t >= WIFI_LOG_EVERY:
      said, said_t = bool(seen), now
      say(f"card wifi {card_iface()} disconnected: AP " +
          (f"on air, signal {seen[0][1]} ch {seen[0][2]}" if seen else "not seen in a scan"))


def mirror(img, key, text, card_ok, manual_left=None):
  """原子寫入（先寫暫存檔再 rename），HUD 不會讀到寫一半的圖。"""
  try:
    tmp = os.path.join(MIRROR_DIR, ".led_now.png")
    img.save(tmp, "PNG")
    os.replace(tmp, os.path.join(MIRROR_DIR, "led_now.png"))
    tmp = os.path.join(MIRROR_DIR, ".led_now.json")
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump({"key": key, "text": text, "card": card_ok,
                 "manual": manual_left if manual_left is None or isinstance(manual_left, str)
                 else round(manual_left, 1)}, f, ensure_ascii=False)   # 秒數，或常駐字幕的 "sticky"
    os.replace(tmp, os.path.join(MIRROR_DIR, "led_now.json"))
  except OSError:
    pass


def _words(text):
  """去掉數字，只比文字：距離、秒數一直在變，不要每格都記一行。"""
  return "".join(c for c in text if not c.isdigit())


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
              note_car_clock(self.latest, self.latest_t)
      except (TimeoutError, OSError, ValueError):
        time.sleep(1)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--host", help="車機位址（預設讀 hud_agent 的 host.json）")
  ap.add_argument("--card", default="serial:auto", help="serial:auto = 經 USB 上的 ESP32-C3 中繼（預設）；IP = 直接走網路；none = 不推，只記錄")
  args = ap.parse_args()

  stream = Stream(args.host)
  stream.start()
  director = Director()
  manual = ManualKeys()
  c3voice = C3VoiceCaptions()
  led = None if args.card == "none" else Led(args.card, timeout=CARD_TIMEOUT)
  card_ok, card_next = None, 0.0
  card_state = {"ok": None}                        # for card_wifi_watch, which runs in its own thread
  if led is not None and not args.card.startswith("serial"):
    threading.Thread(target=card_wifi_watch, args=(lambda: card_state["ok"] is not True,), daemon=True).start()
  slow = 0                                         # 連續逾時的張數（見 SLOW_LIMIT）
  last_key, last_text = None, ""
  mirror_next = 0.0
  power_next, power_last = 0.0, None   # Pi supply: logged whenever vcgencmd's flags change
  mirror_setup()
  say(f"led_agent start card={args.card}")

  while True:
    tick = time.monotonic()
    fresh = tick - stream.latest_t < STREAM_STALE
    d: dict = stream.latest if fresh else {"standby": True}
    manual.poll(director, tick)
    c3voice.poll(director, d, tick)
    nav_poll(director, tick)
    if tick >= power_next:
      power_next = tick + 10.0
      p = throttled()
      if p != power_last:
        say(f"pi power {p}")
        power_last = p
    screen, shown = director.update(d, tick)

    page = screen.page(shown)
    text = f"{page.big}／{page.small}" if page.small else page.big
    if screen.key != last_key:
      cs, ctl = d.get("carState", {}), d.get("control", {})
      params = json.dumps(screen.params, ensure_ascii=False, default=str)
      v, a = cs.get("vEgo", 0.0), ctl.get("aTarget", 0.0)
      say(f"screen {screen.key:<14} {params} v={v:.1f} a={a:.2f} reason={ctl.get('reason', '')}")
    if page.icon != "clock" and (screen.key != last_key or _words(text) != _words(last_text)):
      say(f"  text {text}")                       # 屏上實際的字（換頁才記，數字變動不記）
    last_key, last_text = screen.key, text

    img = R.draw(screen, shown, car_now() or R.NO_TIME)
    if tick >= mirror_next:                        # HUD 鏡像：跟推給控制卡的是同一張圖
      mirror_next = tick + 1 / MIRROR_FPS
      mirror(img, screen.key, text, card_ok, director.manual_left(tick))

    if led is not None and tick >= card_next:
      try:
        if card_ok is not True:                    # 斷線中：先快速探一下，別讓主迴圈卡 1.5 秒逾時
          led.probe()
        led.show(img)
        if card_ok is not True:
          say(f"card up {args.card}")
        card_ok, slow = True, 0
      except Exception as e:
        # 一張圖回得慢不算斷線：10/12 斷線後問中繼，讀到的是控制卡遲到的回覆 —— 卡和中繼都活著，
        # 只是那張超過 CARD_TIMEOUT。以前一慢就判斷線、等 CARD_RETRY 重登，畫面停 3-4 秒；
        # 現在連續 SLOW_LIMIT 張都逾時才算斷線，中間只跳過那幾張，同一個登入繼續用。
        # "no reply" is the ESP32 relay's word for it; since 10/19 the card is reached over the Pi's own USB wifi,
        # where the same late reply reads "timed out" - and was taken as the card gone every time (10/23).
        late = "no reply" in str(e) or "timed out" in str(e)
        if card_ok is True and late and slow + 1 < SLOW_LIMIT:
          slow += 1
          say(f"card slow ({slow}/{SLOW_LIMIT}), frame skipped")
        else:
          if card_ok is not False:
            say(f"card down {args.card}: {e}{relay_diag(led)}")
          card_ok, card_next, slow = False, tick + CARD_RETRY, 0
          led.P = None                               # 下次重新登入

    card_state["ok"] = card_ok
    time.sleep(max(0.0, 1 / FPS - (time.monotonic() - tick)))


if __name__ == "__main__":
  main()
