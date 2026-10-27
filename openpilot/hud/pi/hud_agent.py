#!/usr/bin/env python3
"""hud_agent: runs on the HUD Pi. Serves the HUD page and discovers the comma device.

The C4's IP is dynamic (phone hotspot DHCP), so hardcoding hosts doesn't work.
A background thread scans the Pi's local subnets for an open sp_hud port (8902)
and exposes the result at /host.json for the HUD page (same origin, no CORS).

  GET /host.json  -> {"hosts": ["<found-ip>", ...fallbacks], "self": "<this pi's ip>"}
  GET /<file>     -> static files from this script's directory
"""
import ipaddress
import json
import socket
import subprocess
import threading
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

PORT = 8080
SP_HUD_PORT = 8902
# The car's Hotspot profile pins ipv4.addresses to 192.168.43.1/24 (method shared), so
# on its AP the car is always exactly here - no scan, no gateway lookup, one connect.
CAR_HOTSPOT_IP = "192.168.43.1"
FALLBACK_HOSTS = [CAR_HOTSPOT_IP, "192.168.2.122"]  # AP 上的固定位址；LAN 上的 C4（.143 於 2026-09-02 失效）
HUD_DIR = Path(__file__).resolve().parent
CACHE_FILE = HUD_DIR / "last_host.txt"
# Which network this was on and whether the car answered, kept across reboots. On 2026-09-14
# the screen sat empty for a whole drive and the reboot that fixed it erased the journal, so
# there was nothing to say whether the Pi had joined the car's AP or something else on the
# same subnet - the car's Hotspot is 192.168.43.1/24 and an Android tether is 192.168.43.1/24
# too, so the address alone cannot tell them apart. The SSID can, so write that down.
NET_LOG = HUD_DIR / "net.log"
NET_LOG_MAX = 200_000

_found: str | None = None
_ssid: str = ""
_lock = threading.Lock()
_reconnect_lock = threading.Lock()

# The Pi does not choose a network, it follows the C4 (2026-10-22). The C4 decides (hud/car_hotspot.py): plugged
# into the car the phone's hotspot, else its own AP; on the desk home / office wifi, else its own AP. The Pi stays
# wherever the car answers; only after the car has not answered for LOST_AFTER does it look for it on the other
# known networks, in this order - the phone's hotspot (the car's usual network), the car's own AP, home / office -
# and stops on the first one the car is on. Until 10/22 the two made their own choices by different rules (the C4
# on onroad, the Pi on a 0.4 s probe) and chased each other; on the office wifi the C4 answers in 0.3-1.3 s, so the
# Pi called it gone every one to three minutes.
KNOWN_WIFI = ["MotowayapC", "weedle-94cc", "Motowayap"]
CAR_WIFI = "weedle-94cc"      # the car's own AP
PROBE_TIMEOUT = 2.0           # s: a connect to the car we already know
LOST_AFTER = 20.0             # s of the car not answering before looking on another network
FOLLOW_EVERY = 45.0           # s: at most this often go round the other networks
# 10/25: on the 2nd-4th drives wlan0 sat on MotowayapC with no address for the whole drive (the phone's DHCP never
# answered after the 1st drive's boot - the lease file shows no MotowayapC lease after it), and nothing ever asked
# again: follow_car skipped the network it was on. Associated but no address for this long -> bring the same
# connection up again (a fresh DHCP), at most every NO_IP_RETRY_EVERY.
NO_IP_RETRY_S = 20.0
NO_IP_RETRY_EVERY = 30.0


def own_ip() -> str:
  """This Pi's own address, so the screen can say where to reach it."""
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      s.connect(("8.8.8.8", 1))   # nothing is sent; this just picks the outbound interface
      return s.getsockname()[0]
  except OSError:
    return ""


# The Pi's built-in wifi does the car's networks; the USB one (rtl8821au) only the rear LED card. 10/25: the kernel names
# are not fixed - on a cold boot the USB driver can come up first and take wlan0 - and on the 2nd-4th drives net.log
# showed the card's AP on "wlan0" and MotowayapC without an address, which a swap explains. So the card is found by its
# MAC (the NM profiles are bound by MAC too) and every start writes which name is which into net.log.
CAR_MAC = "88:a2:9e:87:6d:de"


def iface_by_mac(mac: str, fallback: str) -> str:
  try:
    for p in Path("/sys/class/net").iterdir():
      a = p / "address"
      if a.exists() and a.read_text().strip().lower() == mac:
        return p.name
  except OSError:
    pass
  return fallback


def car_iface() -> str:
  return iface_by_mac(CAR_MAC, "wlan0")


def iface_map() -> str:
  try:
    return " ".join(f"{p.name}={(p / 'address').read_text().strip()}" for p in sorted(Path("/sys/class/net").iterdir())
                    if p.name.startswith("wl"))
  except OSError:
    return "?"


def read_ssid() -> str:
  """內建網卡（車上網路那張，照 MAC 找）現在掛的連線名稱（＝KNOWN_WIFI 的名字），不觸發掃描。
  10/25：原本先試 iwgetid，但 Pi 上沒裝；而且它不分網卡，wlan1（字幕卡專用）的 SSID 也會被當成目前網路。"""
  try:
    out = subprocess.check_output(["nmcli", "-t", "-f", "GENERAL.CONNECTION", "dev", "show", car_iface()],
                                  text=True, timeout=3, stderr=subprocess.DEVNULL).strip()
    out = out.split(":", 1)[-1].strip()
    return out if out and out != "--" else ""
  except Exception:
    return ""


def wlan0_ip() -> str:
  """內建網卡（照 MAC 找，平常叫 wlan0）自己的 IPv4（own_ip 走預設路由，看不出這張卡有沒有拿到位址）。"""
  try:
    out = subprocess.check_output(["ip", "-4", "-o", "addr", "show", "dev", car_iface()],
                                  text=True, timeout=3, stderr=subprocess.DEVNULL)
    for line in out.splitlines():
      parts = line.split()
      if "inet" in parts:
        return parts[parts.index("inet") + 1].split("/")[0]
  except Exception:
    pass
  return ""


def port_open(ip: str, timeout: float = 0.4) -> bool:
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.settimeout(timeout)
  try:
    return s.connect_ex((ip, SP_HUD_PORT)) == 0
  except OSError:
    return False
  finally:
    s.close()


def local_networks() -> list[ipaddress.IPv4Network]:
  nets = []
  try:
    out = subprocess.check_output(["ip", "-4", "-o", "addr", "show", "scope", "global"], text=True)
    for line in out.splitlines():
      parts = line.split()
      cidr = parts[3]  # e.g. 192.168.2.54/24
      net = ipaddress.ip_network(cidr, strict=False)
      if net.num_addresses <= 1024:  # don't scan huge subnets
        nets.append(net)
  except Exception:
    pass
  return nets


def gateway_ip() -> str:
  """Default gateway, which on the car's own hotspot is the car itself.

  The hotspot is NetworkManager ipv4 shared, so the car is the AP, the DHCP server and
  the gateway all at once. Asking the routing table costs one syscall and is exact,
  where the subnet scan below is 254 connects and only gets started after the cached
  address and the fallbacks have each timed out.
  """
  try:
    out = subprocess.check_output(["ip", "-o", "route", "show", "default"],
                                  text=True, timeout=3, stderr=subprocess.DEVNULL)
  except Exception:
    return ""
  for line in out.splitlines():
    parts = line.split()
    if "via" in parts:
      i = parts.index("via") + 1
      if i < len(parts):
        return parts[i]
  return ""


def scan_once() -> str | None:
  candidates = []
  # On the car's AP the car is at CAR_HOTSPOT_IP, pinned in its Hotspot profile. Try that
  # before anything that can time out: the cached address is from the home/office LAN and
  # cannot answer from here, so without this every hop onto the hotspot pays for its
  # timeout and then a full subnet scan before finding the car. The gateway lookup is kept
  # behind it as a backstop in case that profile is ever re-addressed.
  if read_ssid() == CAR_WIFI:
    candidates.append(CAR_HOTSPOT_IP)
    gw = gateway_ip()
    if gw and gw != CAR_HOTSPOT_IP:
      candidates.append(gw)          # only if the profile's address ever changes
  if CACHE_FILE.exists():
    candidates.append(CACHE_FILE.read_text().strip())
  candidates += FALLBACK_HOSTS
  for ip in candidates:
    if ip and port_open(ip, timeout=PROBE_TIMEOUT):
      return ip
  for net in local_networks():
    hosts = [str(h) for h in net.hosts()]
    with ThreadPoolExecutor(max_workers=64) as pool:
      for ip, ok in zip(hosts, pool.map(partial(port_open, timeout=1.0), hosts), strict=True):
        if ok:
          return ip
  return None


CAR_CLOCK_FILE = Path("/dev/shm/car_clock.json")   # written by led_agent: car epoch <-> this PI's monotonic


def log_stamp() -> str:
  """The car's time. The PI's own clock cannot be trusted in the car - no route out on the
  hotspot and no RTC battery, so it resumes from the last shutdown (15 h behind on 09-25).
  Until led_agent has seen a valid car time since boot, the PI's time marked (pi)."""
  try:
    a = json.loads(CAR_CLOCK_FILE.read_text())
    ep = float(a["epoch"]) + (time.monotonic() - float(a["mono"]))
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(ep + 8 * 3600))
  except (OSError, ValueError, KeyError, TypeError):
    return time.strftime('%Y-%m-%dT%H:%M:%S') + "(pi)"


def net_log(event: str) -> None:
  """One line per change of network or of whether the car is reachable. Only on change - the
  loop runs every 10-15 s and a drive would otherwise be thousands of identical lines."""
  ts = log_stamp()
  line = f"{ts} {event} ssid={read_ssid() or '-'} self={own_ip() or '-'} car={_found or '-'}"
  try:
    if NET_LOG.exists() and NET_LOG.stat().st_size > NET_LOG_MAX:
      NET_LOG.unlink()
    with NET_LOG.open("a") as f:
      f.write(line + "\n")
  except OSError:
    pass


def wait_for_change(seconds: int) -> None:
  """Sleep up to `seconds`, but return the moment the network or this Pi's address changes (10/20: sitting out a
  whole fixed sleep after a network change was most of the ~30 s of empty HUD)."""
  before = (read_ssid(), own_ip())
  for _ in range(seconds):
    time.sleep(1)
    if (read_ssid(), own_ip()) != before:
      return


def discover_loop():
  global _found, _ssid
  said = None
  lost_since = None
  no_ip_since, last_ip_retry = None, -NO_IP_RETRY_EVERY
  while True:
    # associated but no address (10/25): ask the same network again rather than sit there for the whole drive
    here = read_ssid()
    if here and not wlan0_ip():
      no_ip_since = no_ip_since or time.monotonic()
      now = time.monotonic()
      if now - no_ip_since >= NO_IP_RETRY_S and now - last_ip_retry >= NO_IP_RETRY_EVERY:
        last_ip_retry = now
        reconnect_same(here, now - no_ip_since)
        if wlan0_ip():
          no_ip_since = None
    else:
      no_ip_since = None
    with _lock:
      current = _found
    if current:
      if port_open(current, timeout=PROBE_TIMEOUT):
        lost_since = None
        s2 = read_ssid()
        with _lock:
          _ssid = s2
        key = ("up", s2, current)
        if key != said:
          net_log("car reachable")
          said = key
        wait_for_change(10)
        continue
      # One missed answer is not a lost car - on the office wifi the C4 takes up to 1.3 s. Only LOST_AFTER of
      # silence is, and nothing moves before that.
      if lost_since is None:
        lost_since = time.monotonic()
      if time.monotonic() - lost_since < LOST_AFTER:
        time.sleep(2)
        continue
    found = scan_once() if own_ip() else None
    if not found:
      found = follow_car()
    ssid = read_ssid()
    with _lock:
      _found = found
      _ssid = ssid
    key = ("scan", ssid, found)
    if key != said:
      net_log("car found" if found else "car NOT found")
      said = key
    if found:
      lost_since = None
      try:
        CACHE_FILE.write_text(found)
      except OSError:
        pass
      wait_for_change(10)
    else:
      wait_for_change(15)


_last_follow = -FOLLOW_EVERY


def reconnect_same(name: str, waited: float) -> None:
  """wlan0 is on `name` but has had no address for `waited` s: bring the same connection up again (new DHCP)."""
  try:
    r = subprocess.run(["nmcli", "-w", "20", "con", "up", name], capture_output=True, text=True, timeout=30)
    ok = r.returncode == 0
  except Exception:
    ok = False
  for _ in range(10):          # the address can take a few seconds after association
    if wlan0_ip():
      break
    time.sleep(1)
  ip = wlan0_ip()
  got = f"got {ip}" if ip else "still no address"
  net_log(f"no address on {name} for {waited:.0f} s: reconnect {'ok' if ok else 'failed'}, {got}")


def visible_ssids() -> set[str]:
  try:
    subprocess.run(["nmcli", "dev", "wifi", "rescan"], timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    out = subprocess.check_output(["nmcli", "-t", "-f", "SSID", "dev", "wifi", "list"],
                                  text=True, timeout=10, stderr=subprocess.DEVNULL)
    return {ln.strip() for ln in out.splitlines()}
  except Exception:
    return set()


def follow_car() -> str | None:
  """The car is not on this network: go round the other known networks in KNOWN_WIFI order and stay on the first
  one the car answers on. NetworkManager never leaves a connection it has for a better one, so this has to make
  the move itself. At most every FOLLOW_EVERY, since each hop drops the HUD for a few seconds."""
  global _last_follow
  if time.monotonic() - _last_follow < FOLLOW_EVERY:
    return None
  _last_follow = time.monotonic()
  seen = visible_ssids()
  here = read_ssid() if wlan0_ip() else ""     # on it but without an address counts as not on it (10/25)
  for name in KNOWN_WIFI:
    if name == here or name not in seen:
      continue
    try:
      r = subprocess.run(["nmcli", "-w", "20", "con", "up", name], capture_output=True, text=True, timeout=30)
    except Exception:
      continue
    if r.returncode != 0:
      continue
    for _ in range(10):          # the address can take a few seconds after association
      if own_ip():
        break
      time.sleep(1)
    found = scan_once()
    net_log(f"looked on {name}: {'car here' if found else 'not here'}")
    if found:
      return found
  return None


def net_reconnect() -> dict:
  """待機畫面的重連按鈕。Pi 在車上重開機後若沒自動接回熱點，螢幕上沒有任何辦法補救 —
  這裡給 dash.html 一個出口。只在「目前沒連線」時才會 con up，在家按不會踢掉現有連線。"""
  for cmd in (["nmcli", "radio", "wifi", "on"], ["nmcli", "dev", "wifi", "rescan"]):
    try:
      subprocess.run(cmd, timeout=10, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
      pass
  time.sleep(3)
  ssid = read_ssid()
  if ssid:
    return {"ok": True, "ssid": ssid, "action": "already-connected"}
  tried, last_err = [], ""
  for name in KNOWN_WIFI:
    tried.append(name)
    try:
      r = subprocess.run(["nmcli", "-w", "15", "con", "up", name],
                         capture_output=True, text=True, timeout=25)
      if r.returncode == 0:
        return {"ok": True, "ssid": read_ssid(), "action": "connected", "tried": tried}
      last_err = (r.stderr or r.stdout).strip().splitlines()[-1] if (r.stderr or r.stdout).strip() else ""
    except Exception as e:
      last_err = str(e)
  return {"ok": False, "ssid": "", "tried": tried, "error": last_err}


def net_list() -> dict:
  """已存網路清單＋現在連哪個＋掃描有沒有看到（用 NM 的快取掃描結果，不主動觸發掃描）。
  一定要 --rescan no：預設快取超過 30 s 就會讓每張網卡都重掃。dash.html 每 20 s 叫一次，10/23 實測 wlan1
  每次被拉去掃 4.9 s、收不到字幕卡的回覆，LED 每分鐘斷一次。"""
  visible: set[str] = set()
  try:
    out = subprocess.check_output(["nmcli", "-t", "-f", "SSID", "dev", "wifi", "list", "--rescan", "no"],
                                  text=True, timeout=8, stderr=subprocess.DEVNULL)
    visible = {line.strip() for line in out.splitlines() if line.strip()}
  except Exception:
    pass
  cur = read_ssid()
  return {"current": cur,
          "known": [{"name": n, "visible": n in visible, "current": n == cur} for n in KNOWN_WIFI]}


def net_connect(name: str) -> dict:
  """切到指定的已存網路。這是使用者在螢幕上明確按的，允許斷開現有連線。"""
  if name not in KNOWN_WIFI:
    return {"ok": False, "error": "unknown network"}
  try:
    r = subprocess.run(["nmcli", "-w", "20", "con", "up", name],
                       capture_output=True, text=True, timeout=30)
    if r.returncode == 0:
      return {"ok": True, "ssid": read_ssid()}
    err = (r.stderr or r.stdout).strip()
    return {"ok": False, "error": err.splitlines()[-1] if err else ""}
  except Exception as e:
    return {"ok": False, "error": str(e)}


class Handler(SimpleHTTPRequestHandler):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, directory=str(HUD_DIR), **kwargs)

  def log_message(self, format, *args):
    pass

  def _json(self, obj):
    body = json.dumps(obj).encode()
    self.send_response(200)
    self.send_header("Content-Type", "application/json")
    self.send_header("Cache-Control", "no-store")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def do_GET(self):
    if self.path.startswith("/net/list"):
      self._json(net_list())
      return
    if self.path.startswith("/net/connect"):
      name = parse_qs(urlparse(self.path).query).get("name", [""])[0]
      if not _reconnect_lock.acquire(blocking=False):
        self._json({"ok": False, "error": "busy"})
        return
      try:
        self._json(net_connect(name))
      finally:
        _reconnect_lock.release()
      return
    if self.path.startswith("/net/reconnect"):
      if not _reconnect_lock.acquire(blocking=False):
        self._json({"ok": False, "error": "busy"})
        return
      try:
        self._json(net_reconnect())
      finally:
        _reconnect_lock.release()
      return
    if self.path.startswith("/nav.json"):
      # 給 HUD 的導航卡：age = 距離上次讀到 D531 幾秒（HUD 用它判斷資料是不是還新）
      last = _nav_last
      age = round(time.monotonic() - last["mono"], 1) if last.get("mono") else None
      self._json({"found": bool(last.get("ip")), "age": age, "nav": last.get("nav")})
      return
    if self.path.startswith("/host.json"):
      with _lock:
        found = _found
        ssid = _ssid
      hosts = ([found] if found else []) + [h for h in FALLBACK_HOSTS if h != found]
      body = json.dumps({"hosts": hosts, "ssid": ssid, "c4": found or "",
                         "self": own_ip()}).encode()
      self.send_response(200)
      self.send_header("Content-Type", "application/json")
      self.send_header("Cache-Control", "no-store")
      self.send_header("Content-Length", str(len(body)))
      self.end_headers()
      self.wfile.write(body)
    else:
      self._static = True
      super().do_GET()

  def end_headers(self):
    # Static files (dash.html and friends) went out with Last-Modified only, so chromium kept serving
    # its cached copy by heuristic. kiosk.sh runs with the disk cache off, but the browser that the Pi
    # starts at boot (labwc autostart) does not - 10/11 in the car a boot showed a HUD without the
    # keypad chip that had been deployed hours before. no-cache = revalidate every load (a 304 when
    # nothing changed), never use a stale copy.
    if getattr(self, "_static", False):
      self.send_header("Cache-Control", "no-cache")
    super().end_headers()


# ---------------------------------------------------------------- 導航（車機 D531 上的 NavDash）
# 2026-10-20 使用者：用 NavDash 設定目的地、Google 導航；Pi 讀結果上 HUD 和後窗 LED，不控制車子。
# D531 和 Pi 在同一個網段（手機熱點）時才讀得到；D531 的位址是 DHCP 給的，所以跟找 C4 一樣掃網段，
# 認 /health 回 server=NavDash-D531 的那一台。沒開導航時 NavDash 只回 state=idle，這裡照寫，
# HUD/LED 看 state 決定顯不顯示 —— 沒導航時兩邊的行為跟以前完全一樣。
NAV_PORT = 5002
NAV_FILE = Path("/dev/shm/nav.json")          # {"mono", "ip", "nav"}；led_agent 讀
NAV_ROUTE_FILE = Path("/dev/shm/nav_route.json")
NAV_POLL = 1.0           # s：讀 /nav 的間隔（NavDash 每秒更新一次）
NAV_SCAN_EVERY = 20.0    # s：還沒找到 D531 時多久掃一次網段
NAV_LOST_AFTER = 5       # 連續讀不到幾次就當作 D531 不在了，回去掃
_nav_ip: str | None = None
_nav_last: dict = {}


def _http_json(url: str, timeout: float):
  with urllib.request.urlopen(url, timeout=timeout) as r:
    return json.loads(r.read().decode("utf-8"))


def _nav_port_open(ip: str) -> bool:
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.settimeout(0.3)
  try:
    return s.connect_ex((ip, NAV_PORT)) == 0
  except OSError:
    return False
  finally:
    s.close()


def _is_navdash(ip: str) -> bool:
  try:
    return _http_json(f"http://{ip}:{NAV_PORT}/health", 1.5).get("server") == "NavDash-D531"
  except Exception:
    return False


def nav_scan() -> str | None:
  for net in local_networks():
    hosts = [str(h) for h in net.hosts()]
    with ThreadPoolExecutor(max_workers=64) as pool:
      opened = [ip for ip, ok in zip(hosts, pool.map(_nav_port_open, hosts), strict=True) if ok]
    for ip in opened:
      if _is_navdash(ip):
        return ip
  return None


def _write_json(path: Path, obj) -> None:
  tmp = path.with_suffix(".tmp")
  try:
    tmp.write_text(json.dumps(obj, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)
  except OSError:
    pass


NAV_RETRY_LAST = 3.0     # s：掉線後先每這麼久敲一次上次的位址（WiFi 閃一下多半還是同一個位址）
# 10/27：每秒把導航的下一步和亮著的手動字幕轉給車機（hud/server.py POST /nav，跟鍵盤同一把 token），
# 讓 desire_helper 判斷打燈是轉彎還是換道（selfdrive/mapd/turn_ahead.py）；臨時停車／路況不熟的字幕亮著時
# 打燈不判轉彎也不換道。導航只送 state 和下一步，送不到就算了。
KEYPAD_TOKEN_FILE = Path.home() / ".keypad_token"
LED_NOW_FILE = Path("/dev/shm/led_now.json")     # led_agent 寫的，manual_key＝亮著的手動字幕
FORWARD_EVERY = 1.0


def forward_loop():
  while True:
    time.sleep(FORWARD_EVERY)
    host = _found
    if not host:
      continue
    try:
      token = KEYPAD_TOKEN_FILE.read_text().strip()
    except OSError:
      continue
    nav = {}
    last = _nav_last
    if last and time.monotonic() - float(last.get("mono", 0)) < 3:
      n = last.get("nav") or {}
      nav = {"state": n.get("state"), "step": n.get("step")}
    try:
      caption = json.loads(LED_NOW_FILE.read_text(encoding="utf-8")).get("manual_key")
    except (OSError, ValueError, AttributeError):
      caption = None
    body = json.dumps({"token": token, "nav": nav, "caption": caption}).encode("utf-8")
    req = urllib.request.Request(f"http://{host}:{SP_HUD_PORT}/nav", data=body, method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
      with urllib.request.urlopen(req, timeout=0.5):
        pass
    except Exception:
      pass


def nav_loop():
  global _nav_ip, _nav_last
  fails, route_ver, said, last_ip = 0, None, None, None
  while True:
    if _nav_ip is None:
      ip = nav_scan()
      if ip is None:
        if said != "none":
          net_log("navdash NOT found")
          said = "none"
        # 等下一次整段掃描之前，每 NAV_RETRY_LAST 秒敲一下上次的位址 —— 10/20 辦公室實測斷一下之後
        # 只靠 20 秒一次的整段掃描，過了一分鐘才接回。不更新 _nav_last：HUD 照舊顯示「連線中斷 N 秒」
        until = time.monotonic() + NAV_SCAN_EVERY
        while time.monotonic() < until and ip is None:
          time.sleep(NAV_RETRY_LAST)
          if last_ip and _is_navdash(last_ip):
            ip = last_ip
        if ip is None:
          continue
      _nav_ip, last_ip, fails, route_ver = ip, ip, 0, None
      net_log(f"navdash found {ip}")
      said = "found"
    t0 = time.monotonic()
    try:
      nav = _http_json(f"http://{_nav_ip}:{NAV_PORT}/nav", 1.5)
      fails = 0
      _nav_last = {"mono": time.monotonic(), "ip": _nav_ip, "nav": nav}
      _write_json(NAV_FILE, _nav_last)
      rv = nav.get("routeVer")
      if rv != route_ver:
        try:
          _write_json(NAV_ROUTE_FILE, _http_json(f"http://{_nav_ip}:{NAV_PORT}/route", 5.0))
          route_ver = rv
        except Exception:
          pass                          # 下一輪再拿
    except Exception:
      fails += 1
      if fails >= NAV_LOST_AFTER:
        net_log(f"navdash lost {_nav_ip}")
        _nav_ip = None
        continue
    time.sleep(max(0.0, NAV_POLL - (time.monotonic() - t0)))


def main():
  net_log(f"start: car network on {car_iface()} ({iface_map()})")   # which kernel name each card got this boot
  threading.Thread(target=discover_loop, daemon=True).start()
  threading.Thread(target=nav_loop, daemon=True).start()
  threading.Thread(target=forward_loop, daemon=True).start()
  server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
  server.daemon_threads = True
  server.serve_forever()


if __name__ == "__main__":
  main()
