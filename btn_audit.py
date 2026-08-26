import sys, glob, os
from collections import defaultdict, Counter
from openpilot.tools.lib.logreader import LogReader

BASE = "/mnt/f/c4sunny/backup_20260826_before_master/realdata"
route = sys.argv[1]
segs = sorted(glob.glob(f"{BASE}/{route}--*/rlog.zst"),
              key=lambda p: int(p.split("--")[-2].split("/")[0]) if p.split("--")[-2].split("/")[0].isdigit() else 0)

btn_count = Counter()
events = []          # (t, type, pressed, 狀態快照)
state = {"avail": None, "cruise_en": None, "sd_en": None, "sd_active": None,
         "vcruise": None, "vego": None, "long_on": None}
prev_snapshot = None
t0 = None

for p in segs:
    for m in LogReader(p):
        w = m.which()
        if w == "carParams":
            state["long_on"] = m.carParams.openpilotLongitudinalControl
        elif w == "selfdriveState":
            state["sd_en"] = m.selfdriveState.enabled
            state["sd_active"] = m.selfdriveState.active
        elif w == "carState":
            cs = m.carState
            if t0 is None: t0 = m.logMonoTime
            state["avail"] = cs.cruiseState.available
            state["cruise_en"] = cs.cruiseState.enabled
            state["vcruise"] = round(cs.vCruise, 1)
            state["vego"] = round(cs.vEgo * 3.6, 1)
            for be in cs.buttonEvents:
                bt = str(be.type)
                btn_count[(bt, be.pressed)] += 1
                events.append(((m.logMonoTime - t0) / 1e9, bt, be.pressed, dict(state)))

print(f"=== {route}  ({len(segs)} 段) ===")
print(f"openpilotLongitudinalControl = {state['long_on']}\n")
print("按鈕事件統計：")
for (bt, pressed), n in sorted(btn_count.items()):
    print(f"  {bt:<18} {'按下' if pressed else '放開'}  {n:>5} 次")

print("\n每種按鈕的前 3 個案例（放開瞬間的狀態）：")
seen = defaultdict(int)
print(f"{'時間s':>8} {'按鈕':<18} {'動作':<4} {'available':>9} {'cruiseEn':>8} {'sdEnabled':>9} {'sdActive':>8} {'vCruise':>7} {'vEgo':>6}")
print("-" * 92)
for t, bt, pressed, st in events:
    key = (bt, pressed)
    if seen[key] >= 3: continue
    seen[key] += 1
    print(f"{t:8.1f} {bt:<18} {'按下' if pressed else '放開':<4} {str(st['avail']):>9} {str(st['cruise_en']):>8} "
          f"{str(st['sd_en']):>9} {str(st['sd_active']):>8} {str(st['vcruise']):>7} {str(st['vego']):>6}")
