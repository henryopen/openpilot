"""What the model knew, and when, at every junction stop of 2026-09-02 - with no lead.

stop_for_lights reads one point of the plan: where it ends and how fast it is going there.
The HUD reads a different one: the first place the speed trajectory falls to rest and stays.
The car's display reads a third: the speed trajectory integrated, which only runs 2.5 s and
is why nothing appears on the pi until about fifteen metres.

The claim worth testing is that the model knows earlier than any of them. So take the real
stops, walk back twenty seconds from the halt, and print every candidate signal side by side
against the true distance still to go (integrated from the wheels). Whichever first says
"stop" at a useful range, without saying it on the drives where nothing happened, is the one
worth arming on.

Lead frames are excluded throughout: with a car in front the planner already has the stop.
"""
import bisect
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

# openpilot's model time grid
T_IDXS = [0., 0.00976562, 0.0390625, 0.08789062, 0.15625, 0.24414062, 0.3515625, 0.47851562,
          0.625, 0.79101562, 0.9765625, 1.18164062, 1.40625, 1.65039062, 1.9140625, 2.19726562,
          2.5, 2.82226562, 3.1640625, 3.52539062, 3.90625, 4.30664062, 4.7265625, 5.16601562,
          5.625, 6.10351562, 6.6015625, 7.11914062, 7.65625, 8.21289062, 8.7890625, 9.38476562,
          10.]


def planned_stop(x, v):
    """First place the plan comes to rest and stays there - the HUD's reading."""
    idx = next((i for i, s in enumerate(v) if s < 1.0), None)
    if idx is None:
        return 0.
    if any(v[i] > 1.0 for i in range(idx, len(v))):     # dips and recovers: traffic, not a stop
        return 0.
    return float(x[idx])


def plan_speed_floor(v):
    """Lowest speed anywhere in the plan, and how far ahead in time it happens."""
    lo = min(v)
    return lo, T_IDXS[v.index(lo)] if lo in v else 0.


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'brake': False, 'eng': False, 'lead': False, 'src': '?'}
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
            cur['brake'] = e.carState.brakePressed
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            cur['lead'] = bool(e.radarState.leadOne.present)
        elif w == 'longitudinalPlan':
            cur['src'] = str(e.longitudinalPlan.longitudinalPlanSource)
        elif w == 'modelV2':
            md = e.modelV2
            x = list(md.position.x)
            v = list(md.velocity.x)
            if not x or not v:
                continue
            lo, lo_t = plan_speed_floor(v)
            out.append({
                'v_ego': cur['v'], 'brake': cur['brake'], 'eng': cur['eng'],
                'lead': cur['lead'], 'src': cur['src'],
                'x_end': x[-1], 'v_end': v[-1],
                'stop_ahead': planned_stop(x, v),
                'v_floor': lo, 'v_floor_t': lo_t,
                'a_des': float(md.action.desiredAcceleration),
            })
    return out


def main():
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(ROOT, '*--*'))})
    stops = []
    g = Counter()

    for route in routes:
        segs = sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                      key=lambda p: int(p.rsplit('--', 1)[1]))
        frames = []
        for seg in segs:
            for f in read(seg):
                f['seg'] = os.path.basename(seg)
                frames.append(f)

        rolling = False
        for i, f in enumerate(frames):
            if f['v_ego'] > 5.:
                rolling = True
            elif f['v_ego'] < 0.5 and rolling:
                rolling = False
                win = frames[max(0, i - 400):i + 1]          # twenty seconds
                if len(win) < 100:
                    continue
                if any(w['lead'] for w in win[-100:]):
                    g['stop_with_lead'] += 1
                    continue
                g['stop_no_lead'] += 1
                # true distance still to go, integrated backwards from the halt
                d = 0.
                for w in reversed(win):
                    w['to_go'] = d
                    d += w['v_ego'] * 0.05
                stops.append(win)

    print(f"無前車的停止 {g['stop_no_lead']} 次（有前車 {g['stop_with_lead']} 次，不看）\n")

    for win in stops:
        last = win[-1]
        print(f"=== {last['seg']}  進場 {max(w['v_ego'] for w in win) * 3.6:.0f} km/h ===")
        print(f"{'還剩m':>7} {'秒':>6} {'kph':>6} {'OP':>3} {'煞':>3} "
              f"{'plan末端m':>9} {'末端速':>7} {'停點m':>7} {'plan最低速':>10} {'最低@s':>7} {'a_des':>6} {'來源':>7}")
        for w in win[::20]:                                   # every second
            if w['to_go'] > 90:
                continue
            print(f"{w['to_go']:7.1f} {-(len(win) - win.index(w)) * 0.05:6.1f} {w['v_ego'] * 3.6:6.1f} "
                  f"{'開' if w['eng'] else '關':>3} {'踩' if w['brake'] else '-':>3} "
                  f"{w['x_end']:9.1f} {w['v_end']:7.2f} {w['stop_ahead']:7.1f} "
                  f"{w['v_floor']:10.2f} {w['v_floor_t']:7.2f} {w['a_des']:6.2f} {w['src']:>7}")
        print()

    # earliest useful range of each candidate, over the real stops
    print('=== 各訊號最早在多遠給出「要停」的線索 ===')
    rows = []
    for win in stops:
        r = {'seg': win[-1]['seg']}
        r['x_end<45'] = next((round(w['to_go'], 1) for w in win
                              if w['x_end'] < 45 and w['v_end'] < 2.0), None)
        r['stop_ahead>0'] = next((round(w['to_go'], 1) for w in win if w['stop_ahead'] > 0), None)
        r['v_floor<2'] = next((round(w['to_go'], 1) for w in win if w['v_floor'] < 2.0), None)
        r['v_floor<half'] = next((round(w['to_go'], 1) for w in win
                                  if w['v_ego'] > 3 and w['v_floor'] < w['v_ego'] * 0.5), None)
        r['brake'] = next((round(w['to_go'], 1) for w in win if w['brake']), None)
        rows.append(r)
        print(f"  {r['seg'].rsplit('--', 1)[1]:>4}  現行(x_end<45&v_end<2)={str(r['x_end<45']):>6} m"
              f"   停點法={str(r['stop_ahead>0']):>6} m"
              f"   plan最低速<2={str(r['v_floor<2']):>6} m"
              f"   最低速<半={str(r['v_floor<half']):>6} m"
              f"   你踩煞車={str(r['brake']):>6} m")

    json.dump(rows, open(os.path.join(ROOT, 'stop_signals.json'), 'w'), ensure_ascii=False, indent=1)
    print(f"\n-> {ROOT}/stop_signals.json")


if __name__ == '__main__':
    main()
