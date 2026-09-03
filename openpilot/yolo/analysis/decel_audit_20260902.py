"""Every deceleration of 2026-09-02, and what caused it - lead or no lead, all of it.

The driver reports the car slowing for no reason he can see, including on the motorway with
traffic in front, and has no way to tell what did it. He should not have to guess: the
planner already publishes its own reason on longitudinalPlanSP, so the log can say who took
the speed each time rather than us inferring it.

An event is a sustained request to slow while openpilot is driving. For each one: what the
planner blamed, whether there was a lead and how far, how much speed it actually cost, and
whether the driver overrode it. Sorted by how much speed was lost, because the ones he
notices are the big ones.
"""
import glob
import json
import os
from collections import Counter, defaultdict

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
ROOT = r'F:/c4sunny/rlog20260902F'
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])

DECEL_ON = -0.45          # a request this firm is one the driver can feel
DECEL_OFF = -0.15         # and it ends when the request goes soft
MIN_FRAMES = 20           # lasting a second, so single-frame dips are not events


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'brake': False, 'gas': False, 'eng': False, 'lead': False, 'd_rel': 0.,
           'v_rel': 0., 'reason': '?', 'src': '?', 'a_target': 0., 'v_cruise': 0.}
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
            cs = e.carState
            cur.update(v=cs.vEgo, brake=cs.brakePressed, gas=cs.gasPressed,
                       v_cruise=cs.cruiseState.speed or 0.)
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            lead = e.radarState.leadOne
            cur['lead'] = bool(lead.present)
            cur['d_rel'] = float(lead.dRel) if lead.present else 0.
            cur['v_rel'] = float(lead.vRel) if lead.present else 0.
        elif w == 'longitudinalPlanSP':
            cur['reason'] = str(e.longitudinalPlanSP.reason)
        elif w == 'longitudinalPlan':
            lp = e.longitudinalPlan
            cur['src'] = str(lp.longitudinalPlanSource)
            cur['a_target'] = float(lp.aTarget)
            out.append(dict(cur))
    return out


def main():
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(ROOT, '*--*'))})
    events = []
    frames_by_reason = Counter()
    total = 0

    for route in routes:
        segs = sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                      key=lambda p: int(p.rsplit('--', 1)[1]))
        for seg in segs:
            rows = read(seg)
            name = os.path.basename(seg)
            i = 0
            for r in rows:
                if r['eng']:
                    total += 1
                    frames_by_reason[r['reason']] += 1
            while i < len(rows):
                r = rows[i]
                if not (r['eng'] and r['a_target'] < DECEL_ON):
                    i += 1
                    continue
                j = i
                while j < len(rows) and rows[j]['a_target'] < DECEL_OFF and rows[j]['eng']:
                    j += 1
                if j - i >= MIN_FRAMES:
                    win = rows[i:j]
                    reasons = Counter(w['reason'] for w in win)
                    v0, v1 = win[0]['v'], min(w['v'] for w in win)
                    events.append({
                        'seg': name,
                        'route': route,
                        'secs': round((j - i) * 0.05, 1),
                        'v_from': round(v0 * 3.6, 1),
                        'v_to': round(v1 * 3.6, 1),
                        'lost_kph': round((v0 - v1) * 3.6, 1),
                        'set_kph': round(win[0]['v_cruise'] * 3.6),
                        'min_a': round(min(w['a_target'] for w in win), 2),
                        'reason': reasons.most_common(1)[0][0],
                        'reason_mix': dict(reasons),
                        'had_lead': any(w['lead'] for w in win),
                        'lead_d': round(min((w['d_rel'] for w in win if w['lead']), default=0), 1),
                        'lead_vrel': round(min((w['v_rel'] for w in win if w['lead']), default=0), 1),
                        'driver_braked': any(w['brake'] for w in win),
                        'driver_gas': any(w['gas'] for w in win),
                    })
                i = max(j, i + 1)

    hours = total * 0.05 / 3600
    print(f"openpilot 駕駛中 {hours:.2f} 小時   減速事件 {len(events)} 次\n")
    print('=== engaged 時間，誰在決定速度 ===')
    for k, v in frames_by_reason.most_common():
        print(f"  {k:12} {v * 0.05 / 60:7.1f} 分鐘  ({v / max(total, 1) * 100:4.1f}%)")

    print('\n=== 減速事件，依原因分類 ===')
    by_reason = defaultdict(list)
    for ev in events:
        by_reason[ev['reason']].append(ev)
    print(f"{'原因':10} {'次數':>5} {'有前車':>7} {'無前車':>7} {'掉速中位':>9} {'最大掉速':>9}")
    for reason, evs in sorted(by_reason.items(), key=lambda kv: -len(kv[1])):
        lost = sorted(e['lost_kph'] for e in evs)
        with_lead = sum(1 for e in evs if e['had_lead'])
        print(f"{reason:10} {len(evs):5} {with_lead:7} {len(evs) - with_lead:7} "
              f"{lost[len(lost) // 2]:9.1f} {lost[-1]:9.1f}")

    print('\n=== 掉速最多的 20 次（你會有感的那些）===')
    print(f"{'路段':>24} {'秒':>5} {'從kph':>6} {'到kph':>6} {'掉':>5} {'設定':>5} "
          f"{'原因':>9} {'前車m':>6} {'相對速':>6} {'你踩煞':>6}")
    for ev in sorted(events, key=lambda e: -e['lost_kph'])[:20]:
        print(f"{ev['seg'][-24:]:>24} {ev['secs']:5.1f} {ev['v_from']:6.1f} {ev['v_to']:6.1f} "
              f"{ev['lost_kph']:5.1f} {ev['set_kph']:5.0f} {ev['reason']:>9} "
              f"{ev['lead_d']:6.1f} {ev['lead_vrel']:6.1f} {'是' if ev['driver_braked'] else '-':>6}")

    # the motorway run: route 00000010 is the 14:09 departure
    hw = [e for e in events if e['route'] == '00000010--17de10767f' and e['v_from'] > 60]
    print(f"\n=== 高速公路（時速 60 以上）的減速 {len(hw)} 次 ===")
    print(f"{'路段':>24} {'秒':>5} {'從kph':>6} {'掉':>5} {'原因':>9} {'前車m':>6} {'相對速':>6}")
    for ev in sorted(hw, key=lambda e: -e['lost_kph'])[:15]:
        print(f"{ev['seg'][-24:]:>24} {ev['secs']:5.1f} {ev['v_from']:6.1f} {ev['lost_kph']:5.1f} "
              f"{ev['reason']:>9} {ev['lead_d']:6.1f} {ev['lead_vrel']:6.1f}")

    json.dump({'events': events, 'frames_by_reason': dict(frames_by_reason)},
              open(os.path.join(ROOT, 'decel_audit.json'), 'w'), ensure_ascii=False, indent=1)
    print(f"\n-> {ROOT}/decel_audit.json")


if __name__ == '__main__':
    main()
