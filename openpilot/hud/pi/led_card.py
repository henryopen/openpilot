"""BX-Y04 控制卡的最小客戶端：登入、送整張圖到動態區、截屏回來看。

只碰動態區（存在記憶體、掉電就消失）和截屏，不碰屏參、節目、網路設定。
動態區參數沿用 2026-09-23 01:51 驗證過的 動態區_正確參數.md。
"""
import base64
import glob
import hashlib
import io
import json
import socket
import struct
import time
import urllib.request

W, H = 256, 64


class Relay:
  """ESP32-C3 on the PI's USB, joined to the card's own 2.4 GHz AP (esp32_led_relay/).

  In the car the PI's only WiFi holds the car's 5 GHz hotspot and the card speaks only 2.4 GHz;
  the USB WiFi sticks tried on 09-26 either had no 2.4 GHz under Linux or tripped the PI's
  600 mA USB limit. Frames: "LRQ1" method pathLen(2) bodyLen(4) path body -> "LRS1" status(2)
  bodyLen(4) body, big-endian. The chip prints its boot banner on the same USB link, so the
  reply is found by its magic, not by position."""

  def __init__(self, dev="auto", timeout=12):
    self.dev, self.timeout, self.s = dev, timeout, None

  def _path(self):
    if self.dev != "auto":
      return self.dev
    # native USB (Espressif USB-Serial/JTAG) shows as ttyACM, a CH340/CP2102 board as ttyUSB
    for pat in ("/dev/serial/by-id/*Espressif*", "/dev/ttyACM*", "/dev/ttyUSB*"):
      hit = sorted(glob.glob(pat))
      if hit:
        return hit[0]
    raise OSError("no ESP32 relay on USB")

  def close(self):
    if self.s is not None:
      try:
        self.s.close()
      except Exception:
        pass
    self.s = None

  def request(self, method, path, body=b"", timeout=None):
    import serial   # python3-serial; only needed when the relay is in use
    try:
      if self.s is None:
        # DTR/RTS drive the board's reset and boot pins (that is how esptool gets it into the
        # bootloader), so both are held released. On the PI that still does not stop a reset:
        # cdc_acm raises the lines itself as the port opens, and on 09-26 the board answered
        # with up_ms 67 right after an open, then took ~2 s to rejoin the card's AP. It comes back
        # running, not in the bootloader - so keep this port open for the life of the process
        # and let the caller's retry cover those two seconds.
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self._path(), 921600, 0.2
        s.dtr, s.rts = False, False
        s.open()
        s.reset_input_buffer()
        self.s = s
      p = path.encode()
      self.s.write(b"LRQ1" + method.encode() + struct.pack(">HI", len(p), len(body)) + p + body)
      deadline = time.monotonic() + (timeout or self.timeout)
      win = b""
      while win != b"LRS1":
        c = self.s.read(1)
        if c:
          win = (win + c)[-4:]
        elif time.monotonic() > deadline:
          raise TimeoutError("relay: no reply")
      hdr = self._read(6, deadline)
      code, n = struct.unpack(">hI", hdr)
      data = self._read(n, deadline) if n else b""
    except (OSError, serial.SerialException) as e:
      self.close()   # unplugged or reset: reopen next time
      raise OSError(f"relay: {e}") from e
    if code < 0:
      raise OSError(f"relay {code}: {data.decode(errors='replace')}")
    if code >= 400:
      raise OSError(f"card HTTP {code}")
    return data

  def _read(self, n, deadline):
    buf = b""
    while len(buf) < n:
      c = self.s.read(n - len(buf))
      if c:
        buf += c
      elif time.monotonic() > deadline:
        raise TimeoutError("relay: short reply")
    return buf

  def info(self):
    return json.loads(self.request("I", "/").decode("utf-8"))


class Led:
  def __init__(self, host="192.168.22.1", timeout=12):
    """host: the card's IP, or "serial:auto" / "serial:/dev/ttyACM0" for the ESP32 relay."""
    self.host, self.timeout = host, timeout
    self.relay = Relay(host.split(":", 1)[1] or "auto", timeout) if host.startswith("serial") else None
    self.P = None

  # ---- 傳輸 ----
  def _post(self, path, payload):
    d = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    if self.relay is not None:
      return json.loads(self.relay.request("P", path, d).decode("utf-8"))
    r = urllib.request.Request(f"http://{self.host}{path}", data=d, method="POST",
                               headers={"Content-Type": "application/json;charset=UTF-8", "Accept": "text/json"})
    with urllib.request.urlopen(r, timeout=self.timeout) as resp:
      return json.loads(resp.read().decode("utf-8"))

  def probe(self):
    """Cheap reachability check before a real push, so a missing card does not stall the caller
    for a full timeout. Raises if the card cannot be reached."""
    if self.relay is not None:
      if not self.relay.info().get("wifi"):
        raise OSError("relay: not on the card's AP")
      return
    socket.create_connection((self.host, 80), timeout=0.3).close()

  def _get(self, path, timeout):
    if self.relay is not None:
      return self.relay.request("G", path, timeout=timeout)
    with urllib.request.urlopen(f"http://{self.host}{path}", timeout=timeout) as resp:
      return resp.read()

  @staticmethod
  def _fn(name, inp=None):
    rf = {"name": name}
    if inp is not None:
      rf["input"] = inp
    return {"protocol": {"name": "YQ-COM2", "version": "1.0", "remotefunction": rf}}

  def call(self, name, inp=None):
    if self.P is None:
      self.login()
    return self._post(self.P, self._fn(name, inp))

  def login(self):
    v = self._post("/", self._fn("getVerificationCode", {"username": "guest"}))["remotefunction"]["output"]["verificationcode"]
    pw = hashlib.sha1((v + hashlib.sha1(b"guest").hexdigest()).encode()).hexdigest()
    out = self._post("/", self._fn("userLogin", {"username": "guest", "verificationcode": v, "password": pw}))
    self.P = "/;stok={}/".format(out["remotefunction"]["output"]["sessionID"])
    return self.P

  # ---- 顯示 ----
  def clear(self):
    return self.call("clearDynamic", {"id": [str(i) for i in range(32)]})

  @staticmethod
  def encode(img):
    """PNG（約 3 KB）。BMP 256x64 base64 後超過協定內嵌上限 30000 B，卡會回 500。"""
    buf = io.BytesIO()
    img.convert("RGB").save(buf, "PNG")
    return base64.b64encode(buf.getvalue()).decode()

  def _pic_area(self, content):
    return {"id": "0", "xCoord": "0", "yCoord": "0", "width": str(W), "height": str(H),
            "transparency": "100", "relativeProgram": "", "runMode": "2", "updateFrequency": "",
            "unit": [{"type": "Picture", "order": "0", "stuntType": "0", "stuntSpeed": "16",
                      "stayTime": "0", "content": content, "gifFlag": "0"}]}

  def show(self, img):
    """整張 256x64 圖送到動態區 0，每次都用 UpdateDynamic 整個重建。

    ⛔ 不要改用 UpdateDynamicUnits 只換素材：2026-09-23 實測 Picture 型別用它換圖，卡回成功
    但畫面完全不動（截屏停在第一格）；UpdateDynamic 重建才會真的更新，每格約 155 ms，
    逐格量亮度也沒有閃黑。"""
    r = self.call("UpdateDynamic", {"immediatelyPlay": "0", "cover": "1",
                                    "dynamics": [self._pic_area(self.encode(img))]})
    if "error" in r or "error" in r.get("remotefunction", {}):
      raise RuntimeError(f"UpdateDynamic: {json.dumps(r, ensure_ascii=False)[:200]}")
    return r

  # ---- 截屏（驗證用）----
  def capture(self, path_out, wait=6.0):
    r = self.call("ScreenCapture", {"suffix": "png", "framecount": "1", "picwidth": str(W), "picheight": str(H)})
    fp = r["remotefunction"]["output"]["filepath"]
    time.sleep(wait)
    self.call("enableUploadDownload", {"type": "download", "flag": "on"})
    try:
      data = self._get(f"/download/{fp}", 15)
    finally:
      self.call("enableUploadDownload", {"type": "download", "flag": "off"})
    with open(path_out, "wb") as f:
      f.write(data)
    return fp, len(data)
