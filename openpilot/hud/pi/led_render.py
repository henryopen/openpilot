"""後窗 LED 畫面繪製（256x64）—— 全部自己畫，不靠控制卡字型。

版面：左 64x64 圖示區（有動畫）＋ 右 192x64 一行大字；全屏警示（前車急煞）整面用字＋閃爍。
每個畫面都是 frame(t) -> Image，t 是秒數，動畫靠 t 算相位。
Windows（開發機 .96）和 PiBar 共用這一份。
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

W, H = 256, 64
ICON = 64

# 開發機是單一字型檔；Pi 的思源黑體是 .ttc，一個檔裡有日／韓／簡／繁，要挑到繁體那一個
_FONT_CANDIDATES = [
  r"C:\Windows\Fonts\NotoSansMonoCJKtc-Bold.otf",
  "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
]
FONT_PATH = next((p for p in _FONT_CANDIDATES if os.path.exists(p)), _FONT_CANDIDATES[0])
_font_cache = {}


def _tc_index(path):
  if not path.lower().endswith(".ttc"):
    return 0
  for i in range(10):
    try:
      if "TC" in (ImageFont.truetype(path, 12, index=i).getname()[0] or ""):
        return i
    except OSError:
      break
  return 0


FONT_INDEX = _tc_index(FONT_PATH)

GREEN = (0, 235, 60)
YELLOW = (255, 190, 0)
AMBER = (255, 130, 0)
RED = (255, 20, 20)
WHITE = (255, 255, 255)
GRAY = (110, 110, 110)
DIM = (45, 45, 45)
BLACK = (0, 0, 0)
CYAN = (0, 200, 255)


def font(size):
  if size not in _font_cache:
    _font_cache[size] = ImageFont.truetype(FONT_PATH, size, index=FONT_INDEX)
  return _font_cache[size]


def fit_text(d, box, text, color, max_size=60, min_size=16):
  """在 box 內用能放下的最大字級置中畫一行字。"""
  x0, y0, x1, y1 = box
  bw, bh = x1 - x0 - 4, y1 - y0 - 2
  s = min_size
  for s in range(max_size, min_size - 1, -2):
    l, t, r, b = d.textbbox((0, 0), text, font=font(s), anchor="lt")
    if r - l <= bw and b - t <= bh:
      break
  f = font(s)
  l, t, r, b = d.textbbox((0, 0), text, font=f, anchor="lt")
  x = x0 + (x1 - x0 - (r - l)) / 2 - l
  y = y0 + (y1 - y0 - (b - t)) / 2 - t
  d.text((x, y), text, font=f, fill=color, anchor="lt")


def _mix(c, k):
  return tuple(int(v * k) for v in c)


# ---------------------------------------------------------------- 圖示（畫在 64x64，ox 是左上角 x）

def ic_chevrons(d, ox, color, t, up=True, n=3, speed=1.6):
  """一串 V 形箭頭往上（加速）或往下（減速）流動。"""
  ph = (t * speed) % 1.0
  gap = 64 / n
  for i in range(n + 1):
    y = (i - ph if up else i + ph) * gap - gap / 2
    if not -18 < y < 64:
      continue
    k = 0.35 + 0.65 * (1 - abs(y - 26) / 46)
    c = _mix(color, max(0.25, min(1.0, k)))
    if up:
      pts = [(ox + 8, y + 18), (ox + 32, y), (ox + 56, y + 18)]
    else:
      pts = [(ox + 8, y), (ox + 32, y + 18), (ox + 56, y)]
    d.line(pts, fill=c, width=8, joint="curve")


def ic_road(d, ox, color, t):
  """透視道路，中線虛線往觀者流動（巡航）。"""
  d.line([(ox + 4, 63), (ox + 27, 4)], fill=color, width=4)
  d.line([(ox + 60, 63), (ox + 37, 4)], fill=color, width=4)
  ph = (t * 1.2) % 1.0
  for i in range(4):
    s = ((i + ph) / 4) ** 1.6          # 0 遠 → 1 近
    y0 = 4 + s * 59
    y1 = y0 + 2 + s * 12
    w = 1 + s * 4
    if y0 < 63:
      d.line([(ox + 32, y0), (ox + 32, min(63, y1))], fill=WHITE, width=int(w))


def ic_car(d, ox, color, t, pulse=False):
  """前車車尾（跟車）。"""
  k = 1.0 if not pulse else 0.6 + 0.4 * (0.5 + 0.5 * math.sin(t * 6))
  c = _mix(color, k)
  d.rounded_rectangle([ox + 6, 16, ox + 58, 50], radius=8, fill=c)
  d.rounded_rectangle([ox + 13, 20, ox + 51, 31], radius=4, fill=BLACK)
  d.rectangle([ox + 8, 36, ox + 17, 42], fill=RED)
  d.rectangle([ox + 47, 36, ox + 56, 42], fill=RED)
  d.rectangle([ox + 22, 38, ox + 42, 44], fill=BLACK)
  d.rectangle([ox + 10, 50, ox + 20, 58], fill=c)
  d.rectangle([ox + 44, 50, ox + 54, 58], fill=c)


def ic_curve(d, ox, t):
  """黃色菱形彎道標誌。"""
  d.polygon([(ox + 32, 1), (ox + 63, 32), (ox + 32, 63), (ox + 1, 32)], fill=YELLOW)
  d.arc([ox + 20, 20, ox + 50, 58], start=180, end=270, fill=BLACK, width=6)
  d.line([(ox + 23, 58), (ox + 23, 38)], fill=BLACK, width=6)
  d.line([(ox + 35, 20), (ox + 40, 20)], fill=BLACK, width=6)
  d.polygon([(ox + 38, 11), (ox + 48, 20), (ox + 38, 29)], fill=BLACK)


def ic_octagon(d, ox, t, text="停"):
  r = 30
  pts = [(ox + 32 + r * math.cos(math.radians(22.5 + 45 * i)), 32 + r * math.sin(math.radians(22.5 + 45 * i))) for i in range(8)]
  d.polygon(pts, fill=RED, outline=WHITE)
  fit_text(d, (ox + 8, 10, ox + 56, 54), text, WHITE, max_size=36)


def ic_foot(d, ox, t):
  """收油：單一個空心往下箭頭，慢慢呼吸。"""
  k = 0.55 + 0.45 * (0.5 + 0.5 * math.sin(t * 4))
  c = _mix(YELLOW, k)
  d.line([(ox + 10, 18), (ox + 32, 42), (ox + 54, 18)], fill=c, width=9, joint="curve")
  d.line([(ox + 18, 52), (ox + 46, 52)], fill=c, width=5)


def ic_warn_turn(d, ox, t, left=True):
  """台灣三角形警告標誌（紅框白底）＋黑色彎箭頭。動畫只做紅框呼吸，不做方向燈式的流水。

  先畫右轉，左轉直接水平鏡像 —— 兩邊保證一模一樣（9/23 左轉曾經畫成直角、右轉看起來是彎的）。"""
  k = 0.65 + 0.35 * (0.5 + 0.5 * math.sin(t * 3.5))
  tile = Image.new("RGB", (ICON, ICON), BLACK)
  g = ImageDraw.Draw(tile)
  g.polygon([(32, 1), (63, 61), (1, 61)], fill=_mix(RED, k))
  g.polygon([(32, 12), (54, 55), (10, 55)], fill=WHITE)
  g.line([(24, 55), (24, 46)], fill=BLACK, width=6)            # 直的一段
  g.arc([24, 37, 42, 55], start=180, end=270, fill=BLACK, width=6)  # 圓心 (33,46)：左端接直線、上端接箭頭
  g.polygon([(43, 37), (33, 29), (33, 45)], fill=BLACK)        # 箭頭朝右
  if left:
    tile = tile.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
  d._image.paste(tile, (ox, 0))


def ic_lane_change(d, ox, t, right=True):
  """兩條車道，箭頭從一條滑到另一條。"""
  ph = (t * 0.7) % 1.0
  for y0 in range(int((t * 30) % 16) - 16, 64, 16):   # 只留中間那條分隔線，讓箭頭當主角
    d.line([(ox + 32, y0), (ox + 32, y0 + 8)], fill=WHITE, width=3)
  e = 0.5 - 0.5 * math.cos(math.pi * min(1.0, ph * 1.3))
  a, b = (ox + 12, ox + 46) if right else (ox + 52, ox + 18)
  x = a + (b - a) * e
  y = 62 - 44 * e
  d.line([(a, 64), (x, y)], fill=CYAN, width=10)
  # 箭頭沿著移動方向
  ux, uy = (b - a), -44.0
  n = math.hypot(ux, uy)
  ux, uy = ux / n, uy / n
  px, py = -uy, ux
  tip = (x + ux * 17, y + uy * 17)
  d.polygon([tip, (x + px * 14, y + py * 14), (x - px * 14, y - py * 14)], fill=CYAN)


def ic_limit(d, ox, t, value, blink=False):
  """速限標誌：白底紅圈黑字。"""
  on = not blink or (t * 1.5) % 1.0 < 0.7
  d.ellipse([ox + 1, 1, ox + 63, 63], fill=RED if on else _mix(RED, 0.3))
  d.ellipse([ox + 9, 9, ox + 55, 55], fill=WHITE)
  fit_text(d, (ox + 11, 12, ox + 53, 52), str(value), BLACK, max_size=34)


WEEK = "一二三四五六日"


def datetime_screen(now, t):
  """OP 沒接手時：左邊小字日期、右邊大字時間，冒號每秒閃一次。"""
  img = Image.new("RGB", (W, H), BLACK)
  d = ImageDraw.Draw(img)
  fit_text(d, (0, 4, 78, 32), f"{now.month}/{now.day}", CYAN, max_size=26)
  fit_text(d, (0, 32, 78, 60), "週" + WEEK[now.weekday()], CYAN, max_size=24)
  hh, mm = f"{now.hour:02d}", f"{now.minute:02d}"
  f = font(58)
  l, tp, r, b = d.textbbox((0, 0), hh + ":" + mm, font=f, anchor="lt")
  x = 78 + (W - 78 - (r - l)) / 2 - l
  y = (H - (b - tp)) / 2 - tp
  d.text((x, y), hh, font=f, fill=WHITE, anchor="lt")
  x2 = x + d.textlength(hh, font=f)
  if (t % 1.0) < 0.5:                        # 冒號每秒閃一次，時與分的位置固定不動
    d.text((x2, y), ":", font=f, fill=WHITE, anchor="lt")
  d.text((x2 + d.textlength(":", font=f), y), mm, font=f, fill=WHITE, anchor="lt")
  return img


# ---------------------------------------------------------------- 組版

def compose(icon, text, color, t, bg=BLACK, text_max=58):
  """icon(d, ox, t) 畫在左邊圖示區，右邊一行大字。"""
  img = Image.new("RGB", (W, H), bg)
  d = ImageDraw.Draw(img)
  if icon:
    icon(d, 0, t)
  fit_text(d, (ICON if icon else 0, 0, W, H), text, color, max_size=text_max)
  return img


def full_alert(text, t, a=RED, b=BLACK, hz=3.0):
  on = (t * hz) % 1.0 < 0.55
  img = Image.new("RGB", (W, H), a if on else b)
  d = ImageDraw.Draw(img)
  fit_text(d, (0, 0, W, H), text, WHITE if on else a, max_size=62)
  return img


# ---------------------------------------------------------------- 狀態機的畫面 → 圖

# control.reason → (圖示, 字)。stoplight 只代表 OP 要在前面停（路口、停止線都算）——
# YOLO 沒上，OP 分不出是不是紅燈，所以 FORBIDDEN 在屏上寫「紅燈」、畫紅綠燈。
_DECEL = {
  "lead0": ("chev", "前車減速"), "lead1": ("chev", "前車減速"), "lead2": ("chev", "前車減速"),
  "weaklead": ("chev", "前車減速"), "stoplight": ("octagon", "前方停車"), "curve": ("curve", "前方彎道"),
}


def draw(screen, t, now_dt=None):
  """led_director.Screen + 這個畫面已顯示幾秒 → 256x64 圖。now_dt 是日期時間畫面用的 datetime。"""
  k, p = screen.key, screen.params
  if k == "clock":
    import datetime
    return datetime_screen(now_dt or datetime.datetime.now(), t)
  if k == "cruise":
    if p.get("lead"):
      return compose(lambda d, o, t: ic_car(d, o, WHITE, t, pulse=True), f"跟車 {round(p['dRel'])}m", WHITE, t)
    return compose(lambda d, o, t: ic_road(d, o, GREEN, t), f"巡航 {p.get('kph', 0)}", WHITE, t)
  if k == "accel":
    return compose(lambda d, o, t: ic_chevrons(d, o, GREEN, t, up=True), "加速中", GREEN, t)
  if k == "coast":
    return compose(ic_foot, "準備減速", YELLOW, t)
  if k == "decel":
    icon, text = _DECEL.get(p.get("reason", ""), ("chev", "減速中"))
    if icon == "octagon":
      return compose(ic_octagon, text, RED, t)
    if icon == "curve":
      return compose(ic_curve, text, YELLOW, t)
    return compose(lambda d, o, t: ic_chevrons(d, o, YELLOW, t, up=False), text, YELLOW, t)
  if k == "stop":
    return compose(ic_octagon, f"停車 {max(0, round(p.get('dist', 0)))}m", RED, t)
  if k == "hard_brake":
    return full_alert("前車急煞", t)
  if k == "stopped":
    if p.get("kind") == "junction":
      return compose(ic_octagon, f"停等 {p.get('secs', 0)}秒", RED, t)
    return compose(lambda d, o, t: ic_car(d, o, GRAY, t), "停等中", WHITE, t)
  if k == "launch":                                # OP 不判斷綠燈：只說車要走了，圖示是往前延伸的路
    return compose(lambda d, o, t: ic_road(d, o, GREEN, t), "起步中", GREEN, t)
  if k == "turn":
    left = p.get("dir") == "left"
    return compose(lambda d, o, t: ic_warn_turn(d, o, t, left=left), "準備左轉" if left else "準備右轉", WHITE, t)
  if k == "lane":
    right = p.get("dir") == "right"
    return compose(lambda d, o, t: ic_lane_change(d, o, t, right=right), "向右切換" if right else "向左切換", CYAN, t)
  if k == "limit_ahead":
    text = "前方速限" if (t % 3.0) < 1.5 else f"{round(p.get('dist', 0))}m後"
    return compose(lambda d, o, t: ic_limit(d, o, t, p.get("limit", 0), blink=True), text, WHITE, t)
  raise KeyError(k)
