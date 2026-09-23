"""BX-Y04 控制卡的最小客戶端：登入、送整張圖到動態區、截屏回來看。

只碰動態區（存在記憶體、掉電就消失）和截屏，不碰屏參、節目、網路設定。
動態區參數沿用 2026-09-23 01:51 驗證過的 動態區_正確參數.md。
"""
import base64
import hashlib
import io
import json
import time
import urllib.request

W, H = 256, 64


class Led:
  def __init__(self, host="192.168.22.1", timeout=12):
    self.host, self.timeout = host, timeout
    self.P = None

  # ---- 傳輸 ----
  def _post(self, path, payload):
    d = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    r = urllib.request.Request(f"http://{self.host}{path}", data=d, method="POST",
                               headers={"Content-Type": "application/json;charset=UTF-8", "Accept": "text/json"})
    with urllib.request.urlopen(r, timeout=self.timeout) as resp:
      return json.loads(resp.read().decode("utf-8"))

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
      with urllib.request.urlopen(f"http://{self.host}/download/{fp}", timeout=15) as resp:
        data = resp.read()
    finally:
      self.call("enableUploadDownload", {"type": "download", "flag": "off"})
    with open(path_out, "wb") as f:
      f.write(data)
    return fp, len(data)
