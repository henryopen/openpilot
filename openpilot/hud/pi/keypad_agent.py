#!/usr/bin/env python3
"""藍牙小鍵盤（COIDEA，2 旋鈕＋4×4）→ 車機指令。

跑在 PiBar 上，跟 hud_agent.py 放同一個目錄。鍵盤配對在 Pi 上，每按一個鍵就 POST 一個指令到
車機 hud/server.py 的 /keypad，車機那邊再交給 card／selfdrived／plannerd（selfdrive/keypad.py）。

  第 1 排：1 MAIN ON   2 MAIN OFF   3 ENGAGE    4 DISENGAGE   （3／4＝方向盤中間鍵拆成兩顆）
  第 2 排：5 SET-／-10 6 RES+／+10  7 起步      8 （空）     （5／6 跟方向盤一樣：沒接管時接回）
  第 3 排：9 禮讓行人  0 禮讓救護車  A 前方事故  B 準備靠邊      ┐ 後窗 LED 手動字幕：顯示 10 秒、
  第 4 排：C 路況不熟  D 謝謝禮讓    E 不好意思  F 臨時停車      ┘ 再按同一顆取消（led_director.MANUAL）
  左旋鈕：調 MAX —— 順時針轉 3 格 +10、逆時針 3 格 -10（沒接管時也能調＝接回時要恢復的速度，但不會接回；
          轉快會掉格（10/11 實測），所以算收到的格數；兩格隔超過 1.5 s 或轉向反過來就重算）
  右旋鈕：網路設定頁（10/26：觸控接頭在車上不穩，原本只能用手點的那頁改用旋鈕）—— 按下＝開頁／執行框到的
          那一項，轉＝在「返回 HUD／重新連線／各個網路」之間移動。只在 Pi 本機寫 KNOB_MENU_FILE 給 HUD，
          不經車機；HUD 主畫面時轉動沒有作用。

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
import select
import subprocess
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from led_director import MANUAL_FILE

DEVICE_NAME = "COIDEA KM Keyboard"
KEYPAD_MAC = "d0:54:7b:63:c7:7d"   # paired 10/11; find_device goes by this
HOST_JSON = "http://127.0.0.1:8080/host.json"
CAR_PORT = 8902
TOKEN_FILE = os.path.expanduser("~/.keypad_token")
MAX_AGE = 0.5          # s
SEND_TIMEOUT = 0.5     # s
# 鍵盤狀態給 HUD：閒置約 20 分鐘鍵盤會睡（藍牙斷線、裝置消失，10/11 實測 01:42→02:01），
# 駕駛要知道下一下按鍵只會叫醒它。寫在 RAM，~/hud 放一個 symlink 讓 HUD 同源讀得到。
STATE_FILE = "/dev/shm/keypad_state.json"
STATE_LINK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "keypad_state.json")
# 右旋鈕給 HUD 的網路設定頁：最近幾筆動作，HUD 每 150 ms 讀一次、只做 id 比上次大的（同樣 symlink 到 ~/hud）
KNOB_MENU_FILE = "/dev/shm/knob_menu.json"
KNOB_MENU_LINK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "knob_menu.json")
KNOB_MENU_KEEP = 8
LOG = os.path.expanduser("~/hud/keypad.log")
LOG_MAX = 1_000_000    # bytes, then one rotation to keypad.log.1
BT_EVERY = 30.0        # s: how often to look at BlueZ while the keyboard node is missing
BATTERY_EVERY = 60.0   # s
BATTERY_RETRY = 5.0    # s

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

# 左旋鈕：每一格送一次按鍵（10/11 evtest：左轉 G、按下 H、右轉 I）
KNOB = {
  34: -1,            # KEY_G  逆時針
  23: +1,            # KEY_I  順時針
}
KNOB_STEPS = 3         # detents in one direction for one +10 / -10: one nudged by accident does nothing
KNOB_GAP = 1.5         # s between detents before the count starts over

# 右旋鈕：網路設定頁（左轉 L、按下 M、右轉 N —— 照左旋鈕 G-H-I 的排法推定，10/26 還沒逐格確認過）
MENU_KNOB = {
  38: "prev",        # KEY_L  逆時針
  50: "press",       # KEY_M  按下
  49: "next",        # KEY_N  順時針
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
  """journal (gone when the car powers the Pi off) and ~/hud/keypad.log on the SD card (kept).
  10/11 the drive home had the keypad never connect and nothing left to say why. Pi time is not
  trustworthy in the car (no RTC), so each line also carries seconds since boot."""
  print(time.strftime("%H:%M:%S"), msg, flush=True)
  try:
    if os.path.exists(LOG) and os.path.getsize(LOG) > LOG_MAX:
      os.replace(LOG, LOG + ".1")
    with open(LOG, "a", encoding="utf-8") as f:
      f.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')}(pi) up={time.monotonic():.0f}s {msg}\n")
  except OSError:
    pass


def bt_connected():
  """BlueZ's view, independent of whether an input node exists: tells 'keypad asleep / out of
  range' (not connected) apart from 'connected but no keyboard node' (the 10/11 rename)."""
  try:
    out = subprocess.run(["bluetoothctl", "info", KEYPAD_MAC], capture_output=True, text=True, timeout=5).stdout
  except (OSError, subprocess.SubprocessError):
    return None
  return "Connected: yes" in out


def find_device():
  """The keypad's keyboard node. By its Bluetooth address, not its name: the name it reports
  changes - 'COIDEA KM' when first paired, 'REMOTER' once BlueZ has its real name (10/11: after
  the Pi rebooted in the car the node was 'REMOTER Keyboard' and the exact-name match never found
  it, so no key was read at all). Its Consumer Control and Mouse nodes share the address."""
  for path in sorted(glob.glob("/sys/class/input/event*/device/name")):
    base = os.path.dirname(path)
    try:
      with open(path) as f:
        name = f.read().strip()
      with open(os.path.join(base, "uniq")) as f:
        uniq = f.read().strip().lower()
    except OSError:
      continue
    if (uniq == KEYPAD_MAC and name.endswith(" Keyboard")) or name == DEVICE_NAME:
      return "/dev/input/" + path.split("/")[4]
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


_menu_events = []


def knob_menu(action):
  """右旋鈕的一下寫給 HUD（原子寫入）。pid 換了（程式重啟）HUD 就從頭數 id。"""
  _menu_events.append([(_menu_events[-1][0] + 1) if _menu_events else 1, action])
  del _menu_events[:-KNOB_MENU_KEEP]
  try:
    if not (os.path.islink(KNOB_MENU_LINK) and os.readlink(KNOB_MENU_LINK) == KNOB_MENU_FILE):
      if os.path.lexists(KNOB_MENU_LINK):
        os.remove(KNOB_MENU_LINK)
      os.symlink(KNOB_MENU_FILE, KNOB_MENU_LINK)
    tmp = KNOB_MENU_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump({"pid": os.getpid(), "ev": _menu_events}, f)
    os.replace(tmp, KNOB_MENU_FILE)
  except OSError as e:
    say(f"knob menu: {e}")


def write_state(connected, battery=None):
  try:
    if not (os.path.islink(STATE_LINK) and os.readlink(STATE_LINK) == STATE_FILE):
      if os.path.lexists(STATE_LINK):
        os.remove(STATE_LINK)
      os.symlink(STATE_FILE, STATE_LINK)
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump({"connected": connected, "battery": battery}, f)
    os.replace(tmp, STATE_FILE)
  except OSError as e:
    say(f"state: {e}")


def battery(dev):
  """藍牙回報的電量（%），讀不到就 None。MAC 從輸入裝置的 uniq 拿（藍牙 HID 填的就是 MAC）。"""
  try:
    with open(f"/sys/class/input/{os.path.basename(dev)}/device/uniq") as f:
      mac = f.read().strip()
    out = subprocess.run(["bluetoothctl", "info", mac], capture_output=True, text=True, timeout=5).stdout
  except (OSError, subprocess.SubprocessError):
    return None
  for line in out.splitlines():
    if "Battery Percentage" in line and "(" in line:
      try:
        return int(line.rsplit("(", 1)[1].rstrip(")").strip())
      except ValueError:
        return None
  return None


def run(dev, token):
  host = car_host()
  with open(dev, "rb", buffering=0) as f:
    fcntl.ioctl(f, EVIOCGRAB, 1)
    # stamp the events on the monotonic clock: the Pi has no RTC and its wall clock jumps
    fcntl.ioctl(f, EVIOCSCLOCKID, struct.pack("i", CLOCK_MONOTONIC))
    bat = battery(dev)
    write_state(True, bat)
    # 剛連上時藍牙還沒讀到電量（10/11 實測連上那一刻 None、一分鐘後 100），讀不到就 5 秒後再試
    bat_next = time.monotonic() + (BATTERY_EVERY if bat is not None else BATTERY_RETRY)
    say(f"keypad on {dev}, car {host or '-'}, battery {bat}")
    knob_dir, knob_n, knob_t = 0, 0, 0.0
    while True:
      if not select.select([f], [], [], max(0.0, bat_next - time.monotonic()))[0]:
        bat = battery(dev)
        write_state(True, bat)
        bat_next = time.monotonic() + (BATTERY_EVERY if bat is not None else BATTERY_RETRY)
        continue
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
      if code in MENU_KNOB:
        knob_menu(MENU_KNOB[code])
        say(f"menu {MENU_KNOB[code]}")
        continue
      if code in KNOB:
        d, now = KNOB[code], time.monotonic()
        if d != knob_dir or now - knob_t > KNOB_GAP:
          knob_dir, knob_n = d, 0
        knob_n, knob_t = knob_n + 1, now
        if knob_n < KNOB_STEPS:
          continue
        knob_n = 0
        cmd = "max_up" if d > 0 else "max_down"
      else:
        cmd = KEYS.get(code)
      if cmd is None:
        say(f"key code {code} (no function)")   # 8 = the wake key; the log shows it was read
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
  bt_last, bt_next = "unknown", 0.0
  while True:
    dev = find_device()
    if dev is None:
      if not said_missing:
        say("keypad not connected")
        write_state(False)
        said_missing = True
      if time.monotonic() >= bt_next:
        bt = bt_connected()
        if bt != bt_last:
          say(f"  bluetooth says connected={bt} while no keyboard node")
          bt_last = bt
        bt_next = time.monotonic() + BT_EVERY
      time.sleep(1)
      continue
    said_missing = False
    bt_last, bt_next = "unknown", 0.0
    try:
      run(dev, token)
    except OSError as e:
      say(f"keypad gone ({e})")
      time.sleep(1)


if __name__ == "__main__":
  main()
