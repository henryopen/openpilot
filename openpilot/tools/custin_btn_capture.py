#!/usr/bin/env python3
"""CAN 按鈕擷取工具 —— 提示你按，程式抓出是哪個 bit。

用法（在車機上）:
    /data/btn.sh                        # 預設抓 LKAS 鍵
    /data/btn.sh LKAS 車道偏離 音量加    # 一次抓多顆，依序提示

前提: 車輛電門要轉到 ON（不用發動引擎）。車沒通電時 CAN bus 上沒有任何訊號。
"""
import sys
import time
import threading
from collections import defaultdict
from openpilot.cereal import messaging

BASELINE_SEC = 8.0     # 靜置採樣: 建立「平常就會變」的黑名單
WINDOW_SEC = 4.0       # 每次按鍵的錄製窗口
ROUNDS = 3             # 每顆按鈕重複次數
MAX_CHANGES_PER_WINDOW = 6   # 窗口內變化超過這個數 = 週期訊號，不是按鈕


class CanRecorder:
  """背景收 CAN，記錄每個 (bus, addr, bit) 的變化時刻"""

  def __init__(self):
    self.sock = messaging.sub_sock('can', timeout=100)
    self.state: dict = {}
    self.changes = defaultdict(list)
    self.frames = 0
    self._run = False
    self._th = None

  def _loop(self):
    while self._run:
      for msg in messaging.drain_sock(self.sock):
        t = time.monotonic()
        for c in msg.can:
          if c.src > 2:            # 跳過 openpilot 自己送出的 (src >= 128)
            continue
          self.frames += 1
          d = bytes(c.dat)
          v = int.from_bytes(d, 'big')
          key0 = (c.src, c.address)
          prev = self.state.get(key0)
          self.state[key0] = v
          if prev is None or prev == v:
            continue
          diff = prev ^ v
          for i in range(len(d) * 8):
            if (diff >> i) & 1:
              self.changes[(c.src, c.address, i)].append(t)
      time.sleep(0.005)

  def start(self):
    self._run = True
    self._th = threading.Thread(target=self._loop, daemon=True)
    self._th.start()

  def stop(self):
    self._run = False
    if self._th:
      self._th.join(timeout=1)

  def snapshot(self):
    return {k: list(v) for k, v in self.changes.items()}


def count_in(changes, t0, t1):
  """窗口內有變化的 bit -> 變化次數"""
  out = {}
  for k, ts in changes.items():
    n = sum(1 for t in ts if t0 <= t <= t1)
    if n:
      out[k] = n
  return out


def countdown(sec, msg):
  for i in range(int(sec), 0, -1):
    print(f"\r  {msg} ... {i} ", end="", flush=True)
    time.sleep(1)
  print(f"\r  {msg} ... 完成      ")


def capture_button(rec, name, base, idx, total):
  print(f"\n[步驟 {idx}/{total}] 抓「{name}」")
  hits = []
  for r in range(1, ROUNDS + 1):
    input(f"  第 {r}/{ROUNDS} 次 —— 按 Enter 後開始錄 {WINDOW_SEC:.0f} 秒，期間請按一下「{name}」> ")
    t0 = time.monotonic()
    countdown(WINDOW_SEC, "錄製中（現在按！）")
    hits.append(count_in(rec.snapshot(), t0, time.monotonic()))
    print(f"    這次窗口內有 {len(hits[-1])} 個 bit 變動")

  common = set(hits[0])
  for h in hits[1:]:
    common &= set(h)

  cands = []
  for k in common:
    if k in base:                       # 靜置時就在變 -> 不是按鈕
      continue
    per = [h[k] for h in hits]
    if max(per) > MAX_CHANGES_PER_WINDOW:
      continue
    cands.append((sum(per), per, k))
  cands.sort()

  print(f"\n  ── 「{name}」的候選 ──")
  if not cands:
    print("    沒有候選。可能原因:")
    print("      1) 這顆按鈕不上 CAN（純機械 / 只走 LIN）")
    print("      2) 訊號在靜置期間也一直在變，被黑名單濾掉了")
    print("      3) 按的時間落在窗口外，再跑一次")
    return cands

  print(f"    {'bus':<6}{'addr':<9}{'bit':<6}每次變化次數")
  for _tot, per, (src, addr, bit) in cands[:12]:
    print(f"    bus{src:<3}0x{addr:<7x}{bit:<6}{per}")
  _tot, _per, (src, addr, bit) = cands[0]
  print(f"\n    最可能: bus{src} 0x{addr:x} bit{bit}（大端第 {bit // 8} 個 byte 的第 {bit % 8} 位）")
  print(f"    DBC 寫法:  SG_ {name.split('（')[0]}_BTN : {bit}|1@0+ (1,0) [0|1] \"\" XXX")
  return cands


def main():
  buttons = sys.argv[1:] or ["LKAS（車道置中）"]
  print("=" * 66)
  print(" CAN 按鈕擷取工具")
  print("=" * 66)

  rec = CanRecorder()
  rec.start()

  print("\n[檢查] 讀取 CAN ...")
  time.sleep(2.0)
  if rec.frames == 0:
    print("\n  ✗ 收不到任何 CAN frame。")
    print("    請把車輛電門轉到 ON（不用發動引擎），再重跑一次。")
    rec.stop()
    return 1
  print(f"  ✓ 2 秒內收到 {rec.frames} 個 frame，CAN 正常")

  total = 1 + len(buttons)
  print(f"\n[步驟 1/{total}] 建立基線")
  print(f"  接下來 {BASELINE_SEC:.0f} 秒請「不要按任何按鈕」，方向盤、踏板、排檔都不要動。")
  input("  準備好按 Enter 開始 > ")
  base_t0 = time.monotonic()
  countdown(BASELINE_SEC, "靜置採樣中")
  base = count_in(rec.snapshot(), base_t0, time.monotonic())
  print(f"  基線期間有變動的 bit: {len(base)} 個（這些會被排除）")

  for i, name in enumerate(buttons, start=2):
    capture_button(rec, name, base, i, total)

  rec.stop()
  print("\n" + "=" * 66)
  print("完成。把上面的結果貼回給 Claude 即可。")
  return 0


if __name__ == "__main__":
  sys.exit(main())
