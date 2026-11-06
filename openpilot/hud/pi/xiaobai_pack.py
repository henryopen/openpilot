"""小白表情包 —— 後窗 LED（256x64 全彩、6 格／秒）用的像素角色動畫。

每個場景都是 scene(t, **參數) -> PIL.Image(256x64)，t 是秒數，同一個 t 永遠畫出同一張（不存狀態）。
小白用 2x2 個 LED 當一格像素畫（角色比較大、後車看得清楚），背景和特效用 1x1 畫得細一點。
不用大面積紅色（＝煞車）、不做紅藍閃（像警車）；背景壓暗，小白最亮。

2026-11-03 夥計做的（F:\\huoji\\tmp\\led_eyes\\xiaobai_pack），原樣搬進來只修了 lint。要 numpy，只在開發機跑：
xiaobai_build.py 用它把要用的場景先算成 xiaobai/*.png，PiBar 上的 led_render 只讀那些圖。
"""
import math
import datetime
import numpy as np
from PIL import Image

W, H = 256, 64
FPS = 6.0

# ---------------------------------------------------------------- 色票
WHITE = (250, 250, 255)
SHADE = (165, 185, 215)
EYE = (20, 24, 60)
BLUSH = (255, 120, 175)
MOUTH = (70, 40, 90)
PINK = (255, 105, 185)
MINT = (70, 255, 190)
CYAN = (0, 200, 255)
PURPLE = (165, 110, 255)
YELLOW = (255, 225, 70)
SKYBLUE = (90, 170, 255)
GREEN = (60, 230, 90)
BOBA_TEA = (215, 160, 105)
BOBA = (60, 35, 25)
GRAY = (120, 130, 150)
RAINBOW = [(255, 90, 140), (255, 160, 60), (255, 225, 70), (70, 230, 120), (0, 200, 255), (150, 110, 255)]


def q(t):
  return math.floor(t * FPS) / FPS


def fidx(t):
  return int(math.floor(t * FPS + 1e-6))


def rnd(i, salt=0):
  x = math.sin(i * 12.9898 + salt * 78.233) * 43758.5453
  return x - math.floor(x)


class Canvas:
  def __init__(self, bg=(0, 0, 0)):
    self.a = np.zeros((H, W, 3), np.float32)
    self.a[:] = bg

  def px(self, x, y, c, alpha=1.0):
    x, y = int(round(x)), int(round(y))
    if 0 <= x < W and 0 <= y < H:
      self.a[y, x] = self.a[y, x] * (1 - alpha) + np.array(c, np.float32) * alpha

  def rect(self, x0, y0, w, h, c, alpha=1.0):
    x0, y0 = int(round(x0)), int(round(y0))
    xa, xb, ya, yb = max(0, x0), min(W, x0 + w), max(0, y0), min(H, y0 + h)
    if xa < xb and ya < yb:
      self.a[ya:yb, xa:xb] = self.a[ya:yb, xa:xb] * (1 - alpha) + np.array(c, np.float32) * alpha

  def cell(self, ox, oy, cx, cy, c, s=2):
    """角色格（s x s 個 LED）。"""
    self.rect(ox + cx * s, oy + cy * s, s, s, c)

  def img(self):
    return Image.fromarray(np.clip(self.a, 0, 255).astype(np.uint8))


# ---------------------------------------------------------------- 小白本體（22x17 格，每格 2x2 LED＝44x34 點）
BW, BH = 22, 17
SHADE = (200, 212, 235)


def _body_cells():
  """雲朵：上面三團、下面一個寬身體，取聯集。"""
  blobs = [(6.0, 7.5, 5.2, 5.2), (11.0, 5.6, 6.0, 6.0), (16.0, 7.5, 5.2, 5.2), (11.0, 10.6, 10.6, 6.1)]
  cells = {}
  for cy in range(BH):
    for cx in range(BW):
      x, y = cx + 0.5, cy + 0.5
      if any(((x - bx) / rx) ** 2 + ((y - by) / ry) ** 2 < 1 for bx, by, rx, ry in blobs):
        cells[(cx, cy)] = SHADE if cy >= BH - 3 else WHITE
  return cells


BODY = _body_cells()
EYE_L, EYE_R = (5, 7), (14, 7)          # 3x4 的大眼睛（左上格）


def eyes_cells(kind, look=0):
  out = {}
  for (ex, ey), side in ((EYE_L, -1), (EYE_R, 1)):
    ex += look
    if kind == "open":
      for dx in range(3):
        for dy in range(4):
          if (dx, dy) not in ((0, 0), (2, 0), (0, 3), (2, 3)):
            out[(ex + dx, ey + dy)] = EYE
      out[(ex + 1, ey + 1)] = WHITE                         # 反光
      out[(ex, ey + 1)] = EYE
    elif kind == "blink":
      for dx in range(3):
        out[(ex + dx, ey + 3)] = EYE
    elif kind == "happy":                                   # ^ ^
      out.update({(ex, ey + 2): EYE, (ex + 1, ey + 1): EYE, (ex + 2, ey + 2): EYE, (ex - 1, ey + 3): EYE, (ex + 3, ey + 3): EYE})
    elif kind == "sleep":                                   # ︶ ︶
      out.update({(ex - 1, ey + 2): EYE, (ex, ey + 3): EYE, (ex + 1, ey + 3): EYE, (ex + 2, ey + 3): EYE, (ex + 3, ey + 2): EYE})
    elif kind == "surprise":
      for dx, dy in ((0, 0), (1, 0), (2, 0), (0, 1), (2, 1), (0, 2), (2, 2), (0, 3), (1, 3), (2, 3)):
        out[(ex + dx, ey + dy)] = EYE
    elif kind == "heart":
      for dx, dy in ((0, 0), (2, 0), (-1, 1), (0, 1), (1, 1), (2, 1), (3, 1), (0, 2), (1, 2), (2, 2), (1, 3)):
        out[(ex + dx, ey + dy)] = PINK
    elif kind == "wink":
      if side < 0:
        for dx in range(3):
          for dy in range(4):
            if (dx, dy) not in ((0, 0), (2, 0), (0, 3), (2, 3)):
              out[(ex + dx, ey + dy)] = EYE
        out[(ex + 1, ey + 1)] = WHITE
      else:
        out.update({(ex, ey + 2): EYE, (ex + 1, ey + 1): EYE, (ex + 2, ey + 2): EYE, (ex - 1, ey + 3): EYE, (ex + 3, ey + 3): EYE})
    elif kind == "tired":                                   # 眼皮垂一半
      for dx in range(3):
        out[(ex + dx, ey + 2)] = EYE
        out[(ex + dx, ey + 3)] = EYE
      out[(ex - 1, ey + 2)] = EYE
  return out


def mouth_cells(kind):
  m = {"smile": [(9, 12), (10, 13), (11, 13), (12, 12)],
       "flat": [(10, 13), (11, 13)],
       "o": [(10, 12), (11, 12), (10, 13), (11, 13)],
       "wavy": [(9, 13), (10, 12), (11, 13), (12, 12)]}
  if kind == "open":
    return {(9, 12): MOUTH, (10, 12): MOUTH, (11, 12): MOUTH, (12, 12): MOUTH, (10, 13): PINK, (11, 13): PINK}
  return dict.fromkeys(m.get(kind, []), MOUTH)


ARMS = {
  "down": [(-1, 11), (0, 12), (22, 11), (21, 12)],
  "up": [(-1, 7), (-2, 6), (-2, 5), (22, 7), (23, 6), (23, 5)],
  "wave1": [(-1, 11), (0, 12), (22, 8), (23, 7), (23, 6)],
  "wave2": [(-1, 11), (0, 12), (22, 9), (23, 9), (24, 8)],
  "pray": [(9, 15), (10, 15), (11, 15), (12, 15), (10, 14), (11, 14)],
  "run1": [(-1, 9), (-2, 10), (22, 12), (23, 13)],
  "run2": [(-1, 12), (-2, 13), (22, 9), (23, 10)],
  "hold": [(-1, 11), (0, 12), (21, 11), (22, 10)],
  "point_r": [(-1, 11), (0, 12), (22, 10), (23, 10), (24, 10), (25, 10)],
  "point_l": [(-1, 10), (-2, 10), (-3, 10), (-4, 10), (22, 11), (21, 12)],
  "brake": [(22, 9), (23, 9), (24, 9), (-1, 9), (-2, 9), (-3, 9)],
  "none": [],
}
FEET = {
  "stand": [(6, 17), (7, 17), (14, 17), (15, 17)],
  "step1": [(5, 17), (6, 17), (15, 16), (16, 16)],
  "step2": [(6, 16), (7, 16), (14, 17), (15, 17)],
  "none": [],
}

HALO_DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]


def hue(t, speed=0.25, off=0.0):
  """在彩虹色之間平滑轉換。"""
  k = (t * speed + off) % 1.0 * len(RAINBOW)
  i = int(k)
  f = k - i
  a, b = RAINBOW[i % 6], RAINBOW[(i + 1) % 6]
  return tuple(a[j] * (1 - f) + b[j] * f for j in range(3))


def draw_xb(cv, ox, oy, eyes="open", mouth="smile", arms="down", feet="stand", look=0, blush=True,
            squash=0, flip=False, clip_bottom=None, halo=None):
  """畫小白。ox, oy＝左上角（LED 座標）。squash：往下壓幾格（鞠躬、嘆氣）。halo：外圈光暈顏色。"""
  cells = dict(BODY)
  for c in FEET.get(feet, []):
    cells[c] = SHADE
  for c in ARMS.get(arms, []):
    cells[c] = WHITE
  if blush:
    for c in ((3, 11), (4, 11), (17, 11), (18, 11)):
      cells[c] = BLUSH
  cells.update(eyes_cells(eyes, look))
  cells.update(mouth_cells(mouth))
  placed = []
  for (cx, cy), col in cells.items():
    if flip:
      cx = BW - 1 - cx
    yy = cy + (squash if cy >= 4 else squash * 2)            # 壓扁：頭頂下沉較多
    if squash and cy < 4 and (cx, cy) in BODY:
      yy = cy + squash * 2
    x, y = ox + cx * 2, oy + yy * 2
    if clip_bottom is not None and y >= clip_bottom:
      continue
    placed.append((x, y, col))
  if halo is not None:
    for x, y, _col in placed:
      for dx, dy in HALO_DIRS:
        cv.rect(x + dx, y + dy, 2, 2, halo, 0.55)
  for x, y, col in placed:
    cv.rect(x, y, 2, 2, col)


def blink_eyes(t, period=4.5, salt=0, base="open"):
  i = math.floor(t / period)
  at = i * period + 0.6 + rnd(i, salt) * (period - 1.5)
  return "blink" if 0 <= t - at < 1 / FPS * 1.01 else base


# ---------------------------------------------------------------- 背景與特效
def sky(cv, hour):
  """依時段的天空漸層（壓暗，小白才跳得出來）。"""
  if 5 <= hour < 9:
    top, bot = (40, 30, 80), (150, 80, 70)        # 清晨：紫到粉橘
  elif 9 <= hour < 16:
    top, bot = (10, 50, 120), (60, 130, 170)      # 白天：藍
  elif 16 <= hour < 19:
    top, bot = (50, 20, 70), (170, 80, 50)        # 黃昏：紫到橘
  else:
    top, bot = (2, 4, 18), (10, 20, 55)           # 夜晚：深藍
  k = 0.55                                         # 整體再壓暗
  for y in range(H):
    f = y / (H - 1)
    cv.a[y, :] = [(top[i] * (1 - f) + bot[i] * f) * k for i in range(3)]
  return "night" if (hour >= 19 or hour < 5) else "day"


def stars(cv, t, n=26, salt=1):
  for i in range(n):
    x, y = int(rnd(i, salt) * W), int(rnd(i, salt + 9) * 40)
    ph = (t * 0.8 + rnd(i, salt + 3)) % 1.0
    b = 0.35 + 0.65 * (0.5 + 0.5 * math.sin(ph * 2 * math.pi))
    cv.px(x, y, (255, 255, 220), b)
    if b > 0.92:
      for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        cv.px(x + dx, y + dy, (255, 255, 200), 0.5)


def cloud_puff(cv, x, y, s=1.0, c=(200, 210, 230), alpha=0.5):
  for dx, dy, r in ((0, 0, 4), (5, -2, 5), (10, 0, 4), (5, 2, 4)):
    rr = r * s
    for yy in range(int(-rr), int(rr) + 1):
      for xx in range(int(-rr), int(rr) + 1):
        if xx * xx + yy * yy <= rr * rr:
          cv.px(x + dx * s + xx, y + dy * s + yy, c, alpha)


def road(cv, t, speed=40.0, y0=56):
  cv.rect(0, y0, W, H - y0, (28, 30, 40))
  off = (t * speed) % 24
  for x in range(-24, W + 24, 24):
    cv.rect(x - off, y0 + 3, 12, 2, (200, 200, 210), 0.8)


def sparkle(cv, x, y, size, c):
  for d in range(-size, size + 1):
    cv.px(x + d, y, c, 1 - abs(d) / (size + 1) * 0.6)
    cv.px(x, y + d, c, 1 - abs(d) / (size + 1) * 0.6)


def heart_px(cv, x, y, c, s=1):
  pat = [".##.##.", "#######", "#######", ".#####.", "..###..", "...#..."]
  for r, row in enumerate(pat):
    for k, ch in enumerate(row):
      if ch == "#":
        cv.rect(x + k * s, y + r * s, s, s, c)


def note_px(cv, x, y, c):
  pat = ["..##", "..#.", "..#.", "###.", "##.."]
  for r, row in enumerate(pat):
    for k, ch in enumerate(row):
      if ch == "#":
        cv.px(x + k, y + r, c)


DIGITS = {
  "0": ["###", "#.#", "#.#", "#.#", "###"], "1": [".#.", "##.", ".#.", ".#.", "###"],
  "2": ["###", "..#", "###", "#..", "###"], "3": ["###", "..#", "###", "..#", "###"],
  "4": ["#.#", "#.#", "###", "..#", "..#"], "5": ["###", "#..", "###", "..#", "###"],
  "6": ["###", "#..", "###", "#.#", "###"], "7": ["###", "..#", ".#.", ".#.", ".#."],
  "8": ["###", "#.#", "###", "#.#", "###"], "9": ["###", "#.#", "###", "..#", "###"],
  ":": [".", "#", ".", "#", "."],
}


def text_px(cv, x, y, s, c, scale=3):
  for ch in s:
    pat = DIGITS[ch]
    for r, row in enumerate(pat):
      for k, b in enumerate(row):
        if b == "#":
          cv.rect(x + k * scale, y + r * scale, scale, scale, c)
    x += (len(pat[0]) + 1) * scale


# ================================================================ 場景
GROUND = 27          # 小白站在地上時的 oy（身體 34 點高＋腳）


def idle_peek(t, **_):
  """探頭張望（OP 沒接手）：從下緣不同位置冒出頭、左看右看、縮回去，每次冒出來光暈換一個顏色、身邊閃星星。"""
  cv = Canvas()
  seg = 4.0
  i = math.floor(t / seg)
  c = t - i * seg
  x = [14, 106, 198][int(rnd(i, 4) * 3) % 3]
  col = RAINBOW[i % 6]
  up = c / 0.67 if c < 0.67 else (max(0.0, (seg - c) / 0.67) if c > 3.33 else 1.0)
  y = 64 - up * 34
  look = -1 if 1.0 <= c < 1.83 else (1 if 1.83 <= c < 2.67 else 0)
  eyes = blink_eyes(t, 3.0, 2) if look == 0 else "open"
  draw_xb(cv, x, int(y), eyes=eyes, mouth="smile" if c < 2.67 else "open", arms="none", look=look,
          clip_bottom=64, halo=col)
  if 0.67 <= c < 1.6:
    for k in range(3):
      sparkle(cv, x - 4 + k * 26, int(y) - 3 - (k % 2) * 4, 2 + (fidx(t) + k) % 2, col)
  return cv.img()


def idle_wander(t, **_):
  """散步（OP 沒接手）：從左走到右，中間停下來跟後車揮手，走出去再從左邊進來；地上一條彩虹線。"""
  cv = Canvas()
  L = 14.0
  c = t % L
  stop_at, stop_len = 6.0, 2.5
  speed = (W + 50) / (L - stop_len)
  walk_t = c if c < stop_at else (stop_at if c < stop_at + stop_len else c - stop_len)
  x = -48 + walk_t * speed
  f = fidx(t)
  walking = not (stop_at <= c < stop_at + stop_len)
  for xx in range(W):
    cv.px(xx, 62, hue(t, 0.3, xx / W), 0.6)
  if walking:
    draw_xb(cv, x, GROUND - 2 + [0, -2, -3, -2][f % 4], eyes=blink_eyes(t), mouth="smile",
            arms="run1" if f % 2 else "run2", feet="step1" if f % 2 else "step2", look=1, halo=hue(t, 0.3))
  else:
    draw_xb(cv, x, GROUND - 2, eyes="happy", mouth="open", arms="wave1" if f % 2 else "wave2", halo=hue(t, 0.3))
    sparkle(cv, x + 54, 12, 2, YELLOW)
    sparkle(cv, x + 60, 22, 1, PINK)
  return cv.img()


def idle_clock(t, now=None, **_):
  """看時間（取代原本的日期時間）：小白在左邊抱著時鐘牌，偶爾轉頭看時間；牌框顏色慢慢換。"""
  cv = Canvas()
  now = now or datetime.datetime.now()
  f = fidx(t)
  c = t % 8
  look = 1 if 5 <= c < 6.5 else 0
  draw_xb(cv, 4, GROUND - 1 + (1 if (f // 3) % 2 else 0), eyes=blink_eyes(t) if not look else "open",
          mouth="smile", arms="hold", look=look, halo=hue(t, 0.12))
  fc = hue(t, 0.12, 0.5)
  cv.rect(60, 8, 192, 48, (10, 16, 36))
  cv.rect(60, 8, 192, 2, fc)
  cv.rect(60, 54, 192, 2, fc)
  cv.rect(60, 8, 2, 48, fc)
  cv.rect(250, 8, 2, 48, fc)
  colon = (t % 1.0) < 0.5
  x = 78
  for ch in f"{now.hour:02d}:{now.minute:02d}":
    if ch == ":" and not colon:
      x += 2 * 6
      continue
    text_px(cv, x, 17, ch, WHITE, scale=6)
    x += (len(DIGITS[ch][0]) + 1) * 6
  for k in range(7):                                          # 週一到週日，今天那顆亮彩色
    cv.rect(212 + (k % 4) * 9, 20 + (k // 4) * 12, 6, 6, RAINBOW[k % 6] if k == now.weekday() else (35, 40, 58))
  return cv.img()


def idle_nap(t, **_):
  """打瞌睡（停很久）：小白窩在彎月上閉眼呼吸，彩色 Z 一顆顆飄上去，星星閃。"""
  cv = Canvas()
  sky(cv, 22)
  stars(cv, t, 22, 7)
  mx, my = 70, 30
  for yy in range(-30, 31):                                   # 彎月
    for xx in range(-30, 31):
      if xx * xx + yy * yy < 28 * 28 and (xx - 12) ** 2 + (yy - 8) ** 2 > 26 * 26:
        cv.px(mx + xx, my + yy + 6, (255, 220, 120), 0.75)
  breathe = 1 if (t % 2.0) < 1.0 else 0
  draw_xb(cv, 60, 22 + breathe, eyes="sleep", mouth="flat", arms="none", halo=(120, 110, 255))
  for k in range(3):
    ph = (t / 3.0 + k / 3.0) % 1.0
    zx, zy = 120 + ph * 40 + k * 8, 30 - ph * 26
    col = [PURPLE, CYAN, PINK][k]
    sz = 1 + int(ph * 2)
    for r, row in enumerate(["####", "..#.", ".#..", "####"]):
      for kk, ch in enumerate(row):
        if ch == "#":
          cv.rect(zx + kk * sz, zy + r * sz, sz, sz, col, 1 - ph * 0.5)
  return cv.img()


def cruise_road(t, hour=None, **_):
  """平穩行駛：小白在流動的路上輕快跳步，慢慢左右晃位置；天空依時段變色，雲或星星往後飄。"""
  cv = Canvas()
  hour = datetime.datetime.now().hour if hour is None else hour
  mode = sky(cv, hour)
  if mode == "night":
    stars(cv, t, 22, 3)
  else:
    for k in range(4):
      x = (W - (t * 10 + k * 70) % (W + 40)) + 20
      cloud_puff(cv, x, 8 + (k * 7) % 16, 0.8 + 0.2 * (k % 2), (235, 240, 255), 0.35)
  road(cv, t, 36, y0=58)
  f = fidx(t)
  x = 106 + 70 * math.sin(t * 0.35)
  eyes = "happy" if (t % 9) > 6.5 else blink_eyes(t)
  draw_xb(cv, x, 21 + [0, -2, -3, -2][f % 4], eyes=eyes, mouth="smile", arms="run1" if f % 2 else "run2",
          feet="step1" if f % 2 else "step2", look=1, halo=(255, 255, 255))
  return cv.img()


def cruise_music(t, **_):
  """開心聽歌：戴耳機跟著拍子左右跳著換位置，彩色音符一路飄。"""
  cv = Canvas()
  f = fidx(t)
  beat = math.floor(t / 1.5)
  x = [30, 106, 182, 106][beat % 4]
  hop = [0, -4, -6, -4, 0, 0, 0, 0, 0][f % 9]
  draw_xb(cv, x, GROUND + hop, eyes="happy", mouth="open" if f % 4 < 2 else "smile",
          arms="up" if (f // 3) % 2 else "down", halo=hue(t, 0.5))
  oy = GROUND + hop
  for cx in range(3, 19):                                     # 耳機
    cv.rect(x + cx * 2, oy - 4 + (0 if 6 <= cx <= 15 else 2), 2, 2, PURPLE)
  for cy in range(5, 11):
    cv.rect(x - 2, oy + cy * 2, 2, 2, PURPLE)
    cv.rect(x + 44, oy + cy * 2, 2, 2, PURPLE)
  for k in range(8):
    ph = (t / 2.4 + k / 8) % 1.0
    nx = (k * 37 + ph * 30) % W
    ny = 56 - ph * 52 + 3 * math.sin(ph * 9)
    note_px(cv, nx, ny, RAINBOW[k % 6])
  return cv.img()


def cruise_surf(t, **_):
  """順滑衝浪：小白踩在一道彩色漸層波浪上前後滑行（呼應開車要順滑），浪一直往後流。"""
  cv = Canvas()
  base = 54

  def wave(x):
    return base + 4 * math.sin(x / 18 + t * 2.2) + 2 * math.sin(x / 7 - t * 3)
  for x in range(W):
    yw = wave(x)
    col = hue(0, 0, (x + t * 40) / 240.0)
    for y in range(int(yw), H):
      cv.px(x, y, col, 0.85 if y < yw + 2 else 0.3)
  x = 106 + 60 * math.sin(t * 0.7)
  draw_xb(cv, x, int(wave(x + 22)) - 35, eyes="happy" if (t % 6) > 4 else blink_eyes(t), mouth="open", arms="up",
          look=1, halo=(180, 240, 255))
  for k in range(5):
    ph = (t * 1.5 + k / 5) % 1.0
    cv.rect(x - 2 - ph * 26, wave(x) - 3 - ph * 8, 2, 2, (220, 255, 255), 1 - ph)
  return cv.img()


def accel_run(t, **_):
  """省油加速：小白從左跑到右，綠色速度線往後飛。"""
  cv = Canvas()
  f = fidx(t)
  for k in range(10):
    y = 8 + k * 5
    ln = 14 + int(rnd(k, 2) * 30)
    x = W - ((t * 170 + rnd(k, 5) * W) % (W + 60))
    cv.rect(x, y, ln, 1, GREEN if k % 3 else MINT, 0.8)
  x = -40 + ((t * 70) % (W + 60))
  draw_xb(cv, x, GROUND + [0, -2, 0, -1][f % 4], eyes="open", mouth="open", arms="run1" if f % 2 else "run2",
          feet="step1" if f % 2 else "step2", look=1, halo=GREEN)
  return cv.img()


def follow_watch(t, **_):
  """跟車：小白拿望遠鏡盯著前車，兩車間的虛線流動，偶爾點點頭。"""
  cv = Canvas()
  f = fidx(t)
  nod = 1 if (t % 3.0) > 2.5 else 0
  x = 12
  draw_xb(cv, x, GROUND + nod, eyes="open", mouth="flat", arms="hold", look=1, halo=CYAN)
  cv.rect(x + 44, 46 + nod, 10, 6, (70, 80, 110))
  cv.rect(x + 52, 45 + nod, 4, 8, CYAN)
  cx = 196 + 3 * math.sin(t * 0.8)
  cv.rect(cx, 26, 40, 22, (95, 105, 135))
  cv.rect(cx + 3, 28, 34, 8, (40, 60, 95))
  cv.rect(cx + 2, 40, 7, 3, (255, 90, 120), 0.85)
  cv.rect(cx + 31, 40, 7, 3, (255, 90, 120), 0.85)
  cv.rect(cx + 3, 48, 7, 5, (25, 25, 30))
  cv.rect(cx + 30, 48, 7, 5, (25, 25, 30))
  for k in range(7):
    if (k + f // 2) % 2 == 0:
      cv.rect(74 + k * 16, 50, 9, 2, MINT, 0.85)
  return cv.img()


def stopped_wait(t, **_):
  """停等：小白坐著晃腳、喝珍奶，喝一口眼睛瞇起來，旁邊冒彩色泡泡。"""
  cv = Canvas()
  f = fidx(t)
  c = t % 8
  sip = 3.0 <= c < 4.5
  x = 60
  draw_xb(cv, x, GROUND, eyes="happy" if sip else blink_eyes(t), mouth="o" if sip else "smile", arms="hold",
          feet="step1" if f % 4 < 2 else "step2", halo=(255, 200, 150))
  cx, cy = x + 48, 32 - (4 if sip else 0)
  cv.rect(cx, cy, 14, 22, (230, 230, 240), 0.3)
  cv.rect(cx + 1, cy + 5, 12, 16, BOBA_TEA)
  for k in range(6):
    cv.rect(cx + 2 + (k * 4) % 10, cy + 15 + (k % 2) * 3, 2, 2, BOBA)
  cv.rect(cx + 7, cy - 7, 2, 9, PINK)
  for k in range(6):
    ph = (t / 3.0 + k / 6) % 1.0
    bx, by = 140 + k * 18 + 4 * math.sin(ph * 7 + k), 60 - ph * 58
    r = 2 + (k % 3)
    for a_ in range(0, 360, 30):
      cv.px(bx + r * math.cos(math.radians(a_)), by + r * math.sin(math.radians(a_)), RAINBOW[k % 6], 0.8 * (1 - ph * 0.5))
  return cv.img()


def jam_sigh(t, **_):
  """塞車：小白嘆一口氣（吐出一朵灰雲）、冒汗，然後握拳打起精神、放彩色星星。"""
  cv = Canvas()
  c = t % 8
  x = 40
  if c < 3:
    draw_xb(cv, x, GROUND + 1, eyes="tired", mouth="o", arms="down", squash=1, halo=(110, 120, 150))
    ph = c / 3
    cloud_puff(cv, x + 54 + ph * 40, 30 - ph * 14, 0.8 + ph * 0.6, GRAY, 0.75 * (1 - ph * 0.5))
    cv.rect(x - 4, 26, 2, 5, SKYBLUE)
    cv.px(x - 3, 25, SKYBLUE)
  elif c < 5:
    draw_xb(cv, x, GROUND, eyes="blink" if c < 3.3 else "open", mouth="flat", arms="down", halo=(110, 120, 150))
  else:
    f = fidx(t)
    draw_xb(cv, x, GROUND + (0 if f % 2 else -3), eyes="happy", mouth="open", arms="up", halo=YELLOW)
    for k in range(5):
      sparkle(cv, 110 + k * 30, 12 + (k % 2) * 16, 2 + (f + k) % 2, RAINBOW[k % 6])
  return cv.img()


def lane_point(t, side="right", **_):
  """換道／轉彎陪伴：小白在一邊用手指方向，青色箭頭往那邊流（搭配原本的文字畫面用）。"""
  cv = Canvas()
  f = fidx(t)
  right = side == "right"
  x = 8 if right else 204
  draw_xb(cv, x, GROUND, eyes="open", mouth="open", arms="point_r" if right else "point_l", look=1 if right else -1,
          halo=CYAN)
  for k in range(4):
    off = (f % 4) * 6
    ax = (80 + k * 34 + off) if right else (176 - k * 34 - off)
    for d in range(10):
      for th in range(3):
        sx = d + th
        cv.px(ax + sx if right else ax - sx, 22 + d, CYAN, 1 - k * 0.15)
        cv.px(ax + sx if right else ax - sx, 42 - d, CYAN, 1 - k * 0.15)
  return cv.img()


def decel_slow(t, **_):
  """慢慢減速：小白往後仰、雙手往前擋，腳底冒小煙，黃色 V 字往下流（溫和的減速，不是急煞）。"""
  cv = Canvas()
  f = fidx(t)
  x = 120 - 10 * ((t % 3) / 3)
  draw_xb(cv, x, GROUND, eyes="surprise" if (t % 3) < 0.5 else "open", mouth="o", arms="brake", look=1,
          halo=YELLOW)
  for k in range(3):
    ph = (t * 1.2 + k / 3) % 1.0
    cloud_puff(cv, x - 8 - ph * 30, 56 - ph * 6, 0.35 + ph * 0.3, (200, 200, 210), 0.6 * (1 - ph))
  for k in range(3):
    yy = 8 + ((f * 2 + k * 14) % 42)
    for d in range(9):
      cv.px(20 + d, yy + d * 0.6, YELLOW, 0.8)
      cv.px(36 - d, yy + d * 0.6, YELLOW, 0.8)
  return cv.img()


def rain_umbrella(t, **_):
  """下雨天：小白撐著彩色雨傘慢慢走，雨絲斜斜落下。"""
  cv = Canvas()
  f = fidx(t)
  for k in range(60):
    x = (rnd(k, 3) * W + t * 30) % W
    y = (rnd(k, 9) * H + t * 90) % H
    cv.px(x, y, (120, 170, 255), 0.7)
    cv.px(x - 1, y - 2, (120, 170, 255), 0.4)
  x = 60 + ((t * 18) % 140)
  draw_xb(cv, x, GROUND, eyes=blink_eyes(t), mouth="smile", arms="hold", feet="step1" if f % 2 else "step2", look=1)
  ux, uy = x + 22, GROUND - 6
  for yy in range(12):
    w = int(math.sqrt(max(0, 1 - ((12 - yy) / 12) ** 2)) * 30)
    for xx in range(-w, w + 1):
      cv.px(ux + xx, uy - 12 + yy, RAINBOW[int((xx + 30) / 10) % 6])
  cv.rect(ux, uy, 2, 14, (220, 220, 230))
  return cv.img()


def thanks(t, **_):
  """謝謝（手動）：小白在左邊鞠躬，彩色愛心整排往上飄。"""
  cv = Canvas()
  c = t % 4
  bow = 1.0 <= c < 2.5
  x = 20
  if bow:
    draw_xb(cv, x, GROUND, eyes="happy", mouth="smile", arms="down", squash=2, halo=PINK)
  else:
    draw_xb(cv, x, GROUND, eyes="happy", mouth="open", arms="up" if c >= 2.5 else "down", halo=PINK)
  for k in range(7):
    ph = (t / 2.0 + k / 7) % 1.0
    hx = 84 + k * 24 + 6 * math.sin(ph * 6 + k)
    heart_px(cv, hx, 62 - ph * 70, [PINK, PURPLE, CYAN, MINT, YELLOW, PINK, PURPLE][k], 2 if k % 2 else 1)
  return cv.img()


def sorry(t, **_):
  """抱歉（手動）：雙手合十點頭，額頭一滴汗，旁邊淡淡的光點。"""
  cv = Canvas()
  f = fidx(t)
  nod = 2 if (f // 3) % 2 else 0
  x = 106
  draw_xb(cv, x, GROUND - 1 + nod, eyes="tired", mouth="wavy", arms="pray", halo=(150, 170, 255))
  cv.rect(x + 46, 24 + nod, 2, 5, SKYBLUE)
  cv.px(x + 47, 23 + nod, SKYBLUE)
  for k in range(4):
    ph = (t / 1.5 + k / 4) % 1.0
    sparkle(cv, 40 + k * 56 + (0 if k < 2 else 40), 14 + (k % 2) * 22 - ph * 6, 1 + int(ph * 2), (180, 200, 255))
  return cv.img()


def hello(t, **_):
  """打招呼：小白從右邊跳進來，大大揮手，彩色星星閃。"""
  cv = Canvas()
  f = fidx(t)
  c = t % 5
  x = 200 - min(c / 1.0, 1.0) * 90
  draw_xb(cv, x, GROUND + (0 if f % 4 < 2 else -2), eyes="wink" if c > 3.5 else "happy", mouth="open",
          arms="wave1" if f % 2 else "wave2", halo=hue(t, 0.4))
  for k in range(6):
    ph = (t * 0.9 + rnd(k, 3)) % 1.0
    sparkle(cv, 14 + k * 46, 8 + (k * 17) % 44, 1 + int(ph * 3), RAINBOW[k % 6])
  return cv.img()


def celebrate(t, **_):
  """抵達／開心：小白連跳，彩帶一直落下，光暈換彩虹色。"""
  cv = Canvas()
  f = fidx(t)
  for k in range(48):
    x = rnd(k, 11) * W
    y = ((t * (20 + rnd(k, 2) * 25) + rnd(k, 8) * H) % (H + 6)) - 4
    col = RAINBOW[k % 6]
    if (f + k) % 2:
      cv.rect(x, y, 3, 1, col)
    else:
      cv.rect(x, y, 1, 3, col)
  jump = [0, -5, -8, -9, -8, -5][f % 6]
  draw_xb(cv, 106, GROUND + jump, eyes="happy", mouth="open", arms="up", feet="step1" if jump else "stand",
          halo=hue(t, 0.8))
  return cv.img()


SCENES = [
  # key, 中文名, 函式, 一輪幾秒, 建議用在
  ("idle_peek", "探頭張望", idle_peek, 12.0, "OP 沒接手（車機待機）"),
  ("idle_wander", "散步打招呼", idle_wander, 14.0, "OP 沒接手（車機待機）"),
  ("idle_clock", "抱時鐘牌", idle_clock, 8.0, "OP 沒接手、要看時間時（取代原本日期時間）"),
  ("idle_nap", "月亮上打瞌睡", idle_nap, 6.0, "停很久／準備熄火"),
  ("cruise_road", "輕快跳步", cruise_road, 18.0, "平穩行駛（天空依時段變色）"),
  ("cruise_music", "聽歌跳跳", cruise_music, 6.0, "平穩行駛（輪播用）"),
  ("cruise_surf", "順滑衝浪", cruise_surf, 9.0, "平穩行駛（輪播用）"),
  ("accel_run", "小跑步", accel_run, 4.0, "省油加速"),
  ("follow_watch", "望遠鏡跟車", follow_watch, 6.0, "跟車"),
  ("decel_slow", "慢慢減速", decel_slow, 3.0, "減速中、前車減速（溫和減速，不是急煞）"),
  ("lane_point", "指方向", lane_point, 2.0, "換道／轉彎（參數 dir=left/right；搭配原本文字）"),
  ("stopped_wait", "喝珍奶等", stopped_wait, 8.0, "停等（停止中、跟車停等）"),
  ("jam_sigh", "嘆氣打起精神", jam_sigh, 8.0, "前方塞車"),
  ("rain_umbrella", "撐傘散步", rain_umbrella, 8.0, "下雨（有雨刷或天氣資料時）"),
  ("thanks", "鞠躬謝謝", thanks, 4.0, "手動字幕：謝謝"),
  ("sorry", "合十抱歉", sorry, 4.0, "手動字幕：抱歉"),
  ("hello", "跳進來揮手", hello, 5.0, "開機／手動打招呼"),
  ("celebrate", "跳起來慶祝", celebrate, 6.0, "抵達目的地／開心時"),
]
