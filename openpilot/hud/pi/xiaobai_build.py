#!/usr/bin/env python3
"""把小白動畫先算成圖，放進 xiaobai/ —— 只在開發機跑（要 numpy），PiBar 上的 led_render 只讀圖。

  python xiaobai_build.py            # 重算 xiaobai/*.png
  python xiaobai_build.py --sheet D  # 另外在目錄 D 畫每段的關鍵畫面總覽（給人看）

每段一張直條圖：第 i 格在 y = 64*i。全螢幕的寬 256，左邊圖示位置用的寬 64。都是 6 格／秒、首尾接得起來。

用哪些段（2026-11-03 使用者：「不要日期時間，其它照你建議」）：
  OP 沒接手 → idle_peek、idle_wander 輪流；車機待機 → idle_nap
  平穩行駛 → cruise_road（依車機時間選清晨／白天／黃昏／夜晚）、cruise_music、cruise_surf 輪流，6 秒後接文字頁
  謝謝、抱歉（手動字幕）→ thanks、sorry；led_agent 開起來第一次連上字幕卡 → hello
  跟車、省油加速、停止中／前車停止（我們停著）、前方塞車 → 左邊 64x64 的小小白（mini_*，下面畫），字照舊
夥計包裡其他段（減速、指方向、撐傘、慶祝、抱時鐘牌）沒用：減速／轉彎／換道是安全畫面維持原本的圖示，
車上讀不到下雨，抵達要等導航，日期時間使用者不要。
"""
import argparse
import math
import os

from PIL import Image

import xiaobai_pack as P

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "xiaobai")
ICON = 64
MID = 14          # 小小白放在 64 高的正中間（身體＋腳 36 點高）


def _crop(cv):
  return cv.img().crop((0, 0, ICON, ICON))


def mini_follow(t):
  """跟車：拿望遠鏡盯著右邊（字那邊＝前車的資訊），偶爾點頭。"""
  cv = P.Canvas()
  nod = 1 if (t % 3.0) > 2.5 else 0
  x = 4
  P.draw_xb(cv, x, MID + nod, eyes="open", mouth="flat", arms="hold", look=1, halo=P.CYAN)
  cv.rect(x + 44, MID + 19 + nod, 10, 6, (70, 80, 110))
  cv.rect(x + 52, MID + 18 + nod, 4, 8, P.CYAN)
  return _crop(cv)


def mini_accel(t):
  """省油加速：原地小跑步，綠色速度線往後飛。"""
  cv = P.Canvas()
  f = P.fidx(t)
  for k in range(5):
    y = 6 + k * 12
    ln = 6 + int(P.rnd(k, 2) * 10)
    x = ICON - ((t * 60 + P.rnd(k, 5) * ICON) % (ICON + 20))
    cv.rect(x, y, ln, 1, P.GREEN if k % 3 else P.MINT, 0.8)
  P.draw_xb(cv, 10, MID + [0, -2, 0, -1][f % 4], eyes="open", mouth="open", arms="run1" if f % 2 else "run2",
            feet="step1" if f % 2 else "step2", look=1, halo=P.GREEN)
  return _crop(cv)


def mini_wait(t):
  """停著等：晃腳、喝珍奶，喝一口眼睛瞇起來，杯子上冒兩顆彩色泡泡。"""
  cv = P.Canvas()
  f = P.fidx(t)
  sip = 3.0 <= (t % 8) < 4.5
  x = 2
  P.draw_xb(cv, x, MID, eyes="happy" if sip else P.blink_eyes(t), mouth="o" if sip else "smile", arms="hold",
            feet="step1" if f % 4 < 2 else "step2", halo=(255, 200, 150))
  cx, cy = x + 46, MID + 12 - (4 if sip else 0)
  cv.rect(cx, cy, 14, 22, (230, 230, 240), 0.3)
  cv.rect(cx + 1, cy + 5, 12, 16, P.BOBA_TEA)
  for k in range(6):
    cv.rect(cx + 2 + (k * 4) % 10, cy + 15 + (k % 2) * 3, 2, 2, P.BOBA)
  cv.rect(cx + 7, cy - 7, 2, 9, P.PINK)
  for k in range(2):
    ph = (t / 4.0 + k / 2) % 1.0
    bx, by = 52 + k * 6 + 2 * math.sin(ph * 7 + k), cy - 8 - ph * 22
    for a in range(0, 360, 45):
      cv.px(bx + 2 * math.cos(math.radians(a)), by + 2 * math.sin(math.radians(a)), P.RAINBOW[(k * 3) % 6], 0.8 * (1 - ph))
  return _crop(cv)


def mini_jam(t):
  """塞車：嘆一口氣（吐一朵灰雲）、冒汗，然後打起精神。"""
  cv = P.Canvas()
  c = t % 8
  x = 10
  if c < 3:
    P.draw_xb(cv, x, MID + 1, eyes="tired", mouth="o", arms="down", squash=1, halo=(110, 120, 150))
    ph = c / 3
    P.cloud_puff(cv, x + 34 + ph * 8, MID - 4 - ph * 6, 0.5 + ph * 0.3, P.GRAY, 0.75 * (1 - ph * 0.5))
    cv.rect(x - 4, MID + 2, 2, 5, P.SKYBLUE)
    cv.px(x - 3, MID + 1, P.SKYBLUE)
  elif c < 5:
    P.draw_xb(cv, x, MID, eyes="blink" if c < 3.3 else "open", mouth="flat", arms="down", halo=(110, 120, 150))
  else:
    f = P.fidx(t)
    P.draw_xb(cv, x, MID + (0 if f % 2 else -2), eyes="happy", mouth="open", arms="up", halo=P.YELLOW)
  return _crop(cv)


# 輸出名稱 → (畫一格的函式, 一輪幾秒)
FULL = {
  "idle_peek": (P.idle_peek, 12.0),
  "idle_wander": (P.idle_wander, 14.0),
  "idle_nap": (P.idle_nap, 6.0),
  "cruise_road_dawn": (lambda t: P.cruise_road(t, hour=7), 18.0),
  "cruise_road_day": (lambda t: P.cruise_road(t, hour=12), 18.0),
  "cruise_road_dusk": (lambda t: P.cruise_road(t, hour=17), 18.0),
  "cruise_road_night": (lambda t: P.cruise_road(t, hour=22), 18.0),
  "cruise_music": (P.cruise_music, 6.0),
  "cruise_surf": (P.cruise_surf, 9.0),
  "thanks": (P.thanks, 4.0),
  "sorry": (P.sorry, 4.0),
  "hello": (P.hello, 5.0),
}
MINI = {
  "mini_follow": (mini_follow, 6.0),
  "mini_accel": (mini_accel, 4.0),
  "mini_wait": (mini_wait, 8.0),
  "mini_jam": (mini_jam, 8.0),
}


def build():
  os.makedirs(OUT, exist_ok=True)
  for key, (fn, secs) in {**FULL, **MINI}.items():
    n = int(round(secs * P.FPS))
    frames = [fn(i / P.FPS) for i in range(n)]
    w = frames[0].width
    strip = Image.new("RGB", (w, 64 * n))
    for i, im in enumerate(frames):
      strip.paste(im, (0, 64 * i))
    path = os.path.join(OUT, f"{key}.png")
    strip.save(path, optimize=True)
    print(f"{key:<18} {n:3d} frames {os.path.getsize(path) / 1024:6.1f} KB")


def sheet(out_dir):
  """每段取 4 格排成一張大圖，放大 3 倍給人看。"""
  os.makedirs(out_dir, exist_ok=True)
  keys = list(FULL) + list(MINI)
  rows = []
  for key in keys:
    strip = Image.open(os.path.join(OUT, f"{key}.png")).convert("RGB")
    n = strip.height // 64
    row = Image.new("RGB", (4 * 260, 64), (30, 30, 36))
    for j in range(4):
      i = (n * j) // 4
      row.paste(strip.crop((0, 64 * i, strip.width, 64 * i + 64)), (j * 260, 0))
    rows.append(row)
  big = Image.new("RGB", (4 * 260, 68 * len(rows)), (30, 30, 36))
  for k, row in enumerate(rows):
    big.paste(row, (0, 68 * k))
  big = big.resize((big.width * 3, big.height * 3), Image.Resampling.NEAREST)
  big.save(os.path.join(out_dir, "xiaobai_strips.png"))
  print("sheet", os.path.join(out_dir, "xiaobai_strips.png"), "rows:", ", ".join(keys))


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("--sheet", help="另外畫總覽圖到這個目錄")
  args = ap.parse_args()
  build()
  if args.sheet:
    sheet(args.sheet)
