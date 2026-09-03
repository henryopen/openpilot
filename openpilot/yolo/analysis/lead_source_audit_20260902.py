"""How often the lead flips between radar and vision, and what it costs in speed.

On 2026-09-02 at 14:18 the lead alternated between a radar track (dRel 46 m, vRel -4 km/h)
and a vision estimate (dRel 50 m, vRel -15 km/h) every frame or two, and the planner braked
for the worst of them: 107.8 -> 72.1 km/h with the driver's feet on nothing.

Before changing radard, find out whether that was a one-off. Count every flip while
openpilot is driving, how far the range and closing speed jumped across it, and whether a
firm deceleration followed. The ones that matter are flips where vision claims a much faster
approach than radar - that is what the MPC brakes for.
"""
import glob
import json
import os
from collections import Counter

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
ROOT = r'F:/c4sunny/rlog20260902F'
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])

FIRM = -0.5          # a deceleration the driver feels
LOOKAHEAD = 40       # two seconds of frames after a flip


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'eng': False, 'a': 0., 'reason': '?'}
    while True:
        try:
            e = next(pump)
        except StopIteration:
            break
        except Exception:
            break
        try:
            w = e.which()
        except Exception:
            continue
        if w == 'carState':
            cur['v'] = e.carState.vEgo
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'longitudinalPlan':
            cur['a'] = float(e.longitudinalPlan.aTarget)
        elif w == 'longitudinalPlanSP':
            cur['reason'] = str(e.longitudinalPlanSP.reason)
        elif w == 'radarState':
            ld = e.radarState.leadOne
            out.append({**cur,
                        'present': bool(ld.present),
                        'radar': bool(ld.radar) if ld.present else None,
                        'd': float(ld.dRel) if ld.present else 0.,
                        'y': float(ld.yRel) if ld.present else 0.,
                        'vrel': float(ld.vRel) if ld.present else 0.})
    return out


def main():
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(ROOT, '*--*'))})
    g = Counter()
    flips = []

    for route in routes:
        segs = sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                      key=lambda p: int(p.rsplit('--', 1)[1]))
        for seg in segs:
            rows = read(seg)
            name = os.path.basename(seg)
            prev = None
            for i, r in enumerate(rows):
                if r['eng'] and r['present']:
                    g['lead_frames'] += 1
                    g['radar_frames' if r['radar'] else 'vision_frames'] += 1
                if not (r['eng'] and r['present'] and prev and prev['present'] and prev['eng']):
                    prev = r
                    continue
                if r['radar'] != prev['radar']:
                    g['flips'] += 1
                    dd = r['d'] - prev['d']
                    dv = (r['vrel'] - prev['vrel']) * 3.6
                    after = rows[i:i + LOOKAHEAD]
                    braked = any(x['a'] < FIRM for x in after)
                    v_lost = (r['v'] - min(x['v'] for x in after)) * 3.6 if after else 0.
                    if abs(dd) > 3.0 or abs(dv) > 5.0:
                        g['big_flips'] += 1
                        if braked:
                            g['big_flips_braked'] += 1
                        flips.append({
                            'seg': name, 'to': 'radar' if r['radar'] else 'vision',
                            'd_jump': round(dd, 1), 'vrel_jump_kph': round(dv, 1),
                            'y': round(r['y'], 2), 'kph': round(r['v'] * 3.6, 1),
                            'braked': braked, 'v_lost_kph': round(v_lost, 1),
                            'reason': r['reason'],
                        })
                prev = r

    lf = max(g['lead_frames'], 1)
    print(f"engaged 且有前車的幀 {g['lead_frames']}（{g['lead_frames'] * 0.05 / 60:.1f} 分鐘）")
    print(f"  雷達確認 {g['radar_frames']} ({g['radar_frames'] / lf * 100:.1f}%)   "
          f"只有視覺 {g['vision_frames']} ({g['vision_frames'] / lf * 100:.1f}%)")
    print(f"來源切換 {g['flips']} 次，其中跳幅大的 {g['big_flips']} 次"
          f"（距離>3m 或 相對速>5km/h）")
    print(f"  大跳之後兩秒內出現 <{FIRM} m/s² 的減速：{g['big_flips_braked']} 次"
          f"（{g['big_flips_braked'] / max(g['big_flips'], 1) * 100:.0f}%）")

    if flips:
        worst = sorted(flips, key=lambda f: -abs(f['vrel_jump_kph']))[:20]
        print(f"\n=== 相對速跳最多的 20 次 ===")
        print(f"{'路段':>24} {'切到':>7} {'距離跳m':>8} {'相對速跳kph':>11} {'yRel':>6} "
              f"{'車速':>6} {'有煞':>5} {'掉速':>6} {'原因':>9}")
        for f in worst:
            print(f"{f['seg'][-24:]:>24} {f['to']:>7} {f['d_jump']:8.1f} {f['vrel_jump_kph']:11.1f} "
                  f"{f['y']:6.2f} {f['kph']:6.1f} {'是' if f['braked'] else '-':>5} "
                  f"{f['v_lost_kph']:6.1f} {f['reason']:>9}")

        lost = sorted(f['v_lost_kph'] for f in flips if f['braked'])
        if lost:
            print(f"\n大跳且跟著減速的 {len(lost)} 次：掉速中位 {lost[len(lost) // 2]:.1f} km/h、"
                  f"最大 {lost[-1]:.1f} km/h")

    json.dump({'summary': dict(g), 'flips': flips},
              open(os.path.join(ROOT, 'lead_source_audit.json'), 'w'), ensure_ascii=False, indent=1)
    print(f"\n-> {ROOT}/lead_source_audit.json")


if __name__ == '__main__':
    main()
