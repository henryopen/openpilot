"""How often does the junction stop fire when there is no junction, and what does it cost?

arm_tradeoff answers this for the arming rule on its own. The gate that matters now is the
other one: arming only takes 5 km/h off the set speed, but committing hands the line to the
MPC and the car brakes for it. So both are counted here, from the real module rather than a
re-implementation of its rules.

A firing is a hit if the car actually came to a halt within twenty seconds of it, the same
ground truth arm_tradeoff uses. Anything else is a false alarm, and its cost is how long the
module held on before letting go.

Runs where the recordings are, on Windows. Nothing here needs the solver.
"""
import glob
import json
import os
import sys
import types
from collections import Counter

import capnp
import zstandard

# The module reads a Param and the clock; neither exists here.
_p = types.ModuleType('openpilot.common.params')


class _Params:
    def __init__(self, *a, **k):
        pass

    def get_bool(self, k):
        return True


_p.Params = _Params
sys.modules['openpilot.common.params'] = _p
_rt = types.ModuleType('openpilot.common.realtime')
_rt.DT_MDL = 0.05
sys.modules['openpilot.common.realtime'] = _rt
sys.path.insert(0, r'E:/Documents/GitHub/openpilot-master')

from openpilot.selfdrive.controls.lib.stop_for_lights import StopForLights

SCHEMA = r'F:/c4sunny/schema_hcop'
ROOT = r'F:/c4sunny/rlog20260902F'
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])

DT = 0.05
LOOK_AHEAD = int(20. / DT)   # a stop this soon after a firing is what the firing was for
HALTED = 0.3                 # and reaching this slow is a stop


def read(seg):
    """One segment, lined up on modelV2 - the message the planner actually runs on.

    The set speed is carState.vCruise: cruiseState.speed belongs to the stock ACC and is
    zero all drive on this car.
    """
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'gas': False, 'lead': None, 'eng': False, 'vc': 50 / 3.6}
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
            kph = float(cs.vCruise)
            cur.update(v=cs.vEgo, gas=cs.gasPressed,
                       vc=(kph / 3.6) if 0. < kph < 250. else 50 / 3.6)
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            cur['lead'] = e.radarState.leadOne
        elif w == 'modelV2':
            out.append((e.modelV2, dict(cur)))
    return out


def stops_after(speeds, i):
    """Did the car come to a halt within the look-ahead?

    Asked plainly, with no "having been rolling" qualifier on it. That qualifier was in the
    first version and it marked a real stop as a false alarm: the module can commit below
    the rolling threshold, and one of these commitments was made at 14.7 km/h at a junction
    the car did then stop at.
    """
    return any(v < HALTED for v in speeds[i:i + LOOK_AHEAD])


def main():
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(ROOT, '*--*'))})
    g = Counter()
    fires = []

    for route in routes:
        segs = sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                      key=lambda p: int(p.rsplit('--', 1)[1]))
        sfl = StopForLights()
        speeds, states = [], []
        for seg in segs:
            name = os.path.basename(seg)
            for md, c in read(seg):
                try:
                    sfl.update(md, c['v'], c['vc'], c['gas'], c['lead'])
                except Exception:
                    g['replay_error'] += 1
                    continue
                speeds.append(c['v'])
                states.append({'seg': name, 'armed': sfl.armed, 'active': sfl.is_active,
                               'eng': c['eng'], 'v': c['v']})
        g['frames'] += len(states)
        g['engaged'] += sum(1 for s in states if s['eng'])

        for what, key in (('arm', 'armed'), ('commit', 'active')):
            was = False
            start = 0
            for i, s in enumerate(states):
                now = s[key]
                if now and not was:
                    start = i
                elif was and not now:
                    fires.append({'what': what, 'seg': states[start]['seg'],
                                  'kph': round(states[start]['v'] * 3.6, 1),
                                  'eng': states[start]['eng'],
                                  'held': (i - start) * DT,
                                  'hit': stops_after(speeds, start)})
                was = now
            if was:   # still on at the end of the route
                fires.append({'what': what, 'seg': states[start]['seg'],
                              'kph': round(states[start]['v'] * 3.6, 1),
                              'eng': states[start]['eng'],
                              'held': (len(states) - start) * DT,
                              'hit': stops_after(speeds, start)})

    hours = g['frames'] * DT / 3600.
    head = f"總幀 {g['frames']}（{hours:.2f} 小時，其中 openpilot 開著 {g['engaged'] * DT / 60:.0f} 分）"
    print(head + f"  重放錯誤 {g['replay_error']}")
    print("")
    hdr = f"{'閘門':<8}{'觸發':>6}{'命中':>6}{'誤報':>6}{'誤報/小時':>10}{'誤報總秒':>10}{'誤報中位秒':>11}{'誤報最長秒':>11}"
    print(hdr)
    for what, label in (('arm', '武裝'), ('commit', '承諾')):
        for only_eng in (False, True):
            got = [f for f in fires if f['what'] == what and (f['eng'] or not only_eng)]
            bad = [f for f in got if not f['hit']]
            held = sorted(f['held'] for f in bad)
            med = held[len(held) // 2] if held else 0.
            name = label + ('（OP 開著）' if only_eng else '（全部）')
            row = f"{name:<8}{len(got):>6}{len(got) - len(bad):>6}{len(bad):>6}"
            rate = f"{len(bad) / max(hours, 1e-9):>10.1f}{sum(held):>10.1f}"
            print(row + rate + f"{med:>11.2f}{max(held, default=0.):>11.1f}")

    out = os.path.join(ROOT, 'stop_gate_cost.json')
    json.dump({'hours': hours, 'fires': fires}, open(out, 'w', encoding='utf-8'),
              ensure_ascii=False, indent=1)
    print("")
    print('-> ' + out)


if __name__ == '__main__':
    main()
