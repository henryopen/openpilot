import sys, glob, re
from collections import Counter, defaultdict
from openpilot.tools.lib.logreader import LogReader

BASE = "/mnt/f/c4sunny/backup_20260826_before_master/realdata"
route = sys.argv[1]
segs = sorted(glob.glob(f"{BASE}/{route}--*/rlog.zst"),
              key=lambda p: int(re.search(r"--(\d+)/rlog", p).group(1)))

# 先把時間序列全收起來
tl = []      # (t, kind, payload)
t0 = None
long_on = None
for p in segs:
    for m in LogReader(p):
        w = m.which()
        if t0 is None and w in ("carState", "selfdriveState"): t0 = m.logMonoTime
        if t0 is None: continue
        t = (m.logMonoTime - t0) / 1e9
        if w == "carParams":
            long_on = m.carParams.openpilotLongitudinalControl
        elif w == "selfdriveState":
            tl.append((t, "sd", (m.selfdriveState.enabled, m.selfdriveState.active,
                                 m.selfdriveState.alertText1, m.selfdriveState.alertText2)))
        elif w == "carState":
            cs = m.carState
            tl.append((t, "cs", (cs.cruiseState.available, cs.cruiseState.enabled, round(cs.vCruise,1),
                                 round(cs.vEgo*3.6,1), [(str(b.type), b.pressed) for b in cs.buttonEvents])))
tl.sort(key=lambda x: x[0])

def state_at(t, kind, idx, default=None):
    v = default
    for tt, k, p in tl:
        if tt > t: break
        if k == kind: v = p[idx]
    return v

def window(t, kind, idx, lo, hi):
    out = []
    for tt, k, p in tl:
        if tt < t + lo: continue
        if tt > t + hi: break
        if k == kind: out.append(p[idx])
    return out

releases = [(t, bt) for t, k, p in tl if k == "cs" for bt, pr in p[4] if not pr]
res = defaultdict(lambda: Counter())
alerts = defaultdict(Counter)

for t, bt in releases:
    before = state_at(t - 0.3, "sd", 0)
    after = window(t, "sd", 0, 0.0, 2.0)
    av_b = state_at(t - 0.3, "cs", 0)
    av_a = window(t, "cs", 0, 0.0, 2.0)
    vc_b = state_at(t - 0.3, "cs", 2)
    vc_a = window(t, "cs", 2, 0.0, 1.0)

    if before is False and any(after): res[bt]["→ 啟用 engage"] += 1
    elif before is True and not all(after) and len(after): res[bt]["→ 解除 disengage"] += 1
    elif before is True: res[bt]["維持啟用"] += 1
    else: res[bt]["維持解除"] += 1

    if av_b is True and av_a and not all(av_a): res[bt]["available True→False"] += 1
    if av_b is False and any(av_a): res[bt]["available False→True"] += 1
    if vc_b is not None and vc_a and vc_a[-1] != vc_b: res[bt][f"vCruise 有變"] += 1

    for a in window(t, "sd", 2, 0.0, 1.5):
        if a and a not in ("", "openpilot Unavailable"): alerts[bt][a] += 1

print(f"=== {route} ({len(segs)} 段)  openpilotLongitudinalControl={long_on} ===\n")
print(f"{'按鈕（放開瞬間）':<20} {'結果':<24} {'次數':>5}")
print("-" * 55)
for bt in sorted(res):
    tot = sum(v for k, v in res[bt].items() if k in ("→ 啟用 engage","→ 解除 disengage","維持啟用","維持解除"))
    print(f"{bt:<20} {'（共 '+str(tot)+' 次放開）':<24}")
    for k, v in res[bt].most_common():
        print(f"{'':<20} {k:<24} {v:>5}")
print("\n按鈕後 1.5 秒內出現的畫面提示：")
for bt in sorted(alerts):
    for a, n in alerts[bt].most_common(4):
        print(f"  {bt:<18} {a:<40} {n:>4} 次")
