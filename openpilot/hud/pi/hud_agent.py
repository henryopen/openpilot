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
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = 8080
SP_HUD_PORT = 8902
FALLBACK_HOSTS = ["192.168.2.143"]  # 辦公室 LAN 上的 C4
HUD_DIR = Path(__file__).resolve().parent
CACHE_FILE = HUD_DIR / "last_host.txt"

_found: str | None = None
_ssid: str = ""
_lock = threading.Lock()
_reconnect_lock = threading.Lock()

# 車上熱點優先，家裡 AP 其次；密碼都已存在 NM keyfile，這裡只負責叫起來
KNOWN_WIFI = ["MotowayapC", "Motowayap"]


def own_ip() -> str:
  """This Pi's own address, so the screen can say where to reach it."""
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      s.connect(("8.8.8.8", 1))   # nothing is sent; this just picks the outbound interface
      return s.getsockname()[0]
  except OSError:
    return ""


def read_ssid() -> str:
  """目前連上的 AP 名稱。iwgetid 最輕量，沒有再退回 nmcli（都不觸發掃描）。"""
  for cmd in (["iwgetid", "-r"],
              ["nmcli", "-t", "-f", "GENERAL.CONNECTION", "dev", "show", "wlan0"]):
    try:
      out = subprocess.check_output(cmd, text=True, timeout=3, stderr=subprocess.DEVNULL).strip()
      if cmd[0] == "nmcli":
        out = out.split(":", 1)[-1].strip()
      if out and out != "--":
        return out
    except Exception:
      continue
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


def scan_once() -> str | None:
  candidates = []
  if CACHE_FILE.exists():
    candidates.append(CACHE_FILE.read_text().strip())
  candidates += FALLBACK_HOSTS
  for ip in candidates:
    if ip and port_open(ip):
      return ip
  for net in local_networks():
    hosts = [str(h) for h in net.hosts()]
    with ThreadPoolExecutor(max_workers=64) as pool:
      for ip, ok in zip(hosts, pool.map(partial(port_open, timeout=0.4), hosts)):
        if ok:
          return ip
  return None


def discover_loop():
  global _found, _ssid
  while True:
    with _lock:
      current = _found
    if current and port_open(current):
      s2 = read_ssid()
      with _lock:
        _ssid = s2
      time.sleep(10)
      continue
    found = scan_once()
    ssid = read_ssid()
    with _lock:
      _found = found
      _ssid = ssid
    if found:
      try:
        CACHE_FILE.write_text(found)
      except OSError:
        pass
      time.sleep(10)
    else:
      time.sleep(15)


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
    if self.path.startswith("/net/reconnect"):
      if not _reconnect_lock.acquire(blocking=False):
        self._json({"ok": False, "error": "busy"})
        return
      try:
        self._json(net_reconnect())
      finally:
        _reconnect_lock.release()
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
      super().do_GET()


def main():
  threading.Thread(target=discover_loop, daemon=True).start()
  server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
  server.daemon_threads = True
  server.serve_forever()


if __name__ == "__main__":
  main()
