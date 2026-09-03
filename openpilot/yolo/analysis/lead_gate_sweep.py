"""What does it cost to stop treating a car far up the road as the answer to a junction?

The module stands down whenever radard reports a lead, at any range. On 2026-09-03 every
stop had one, so it never acted, and the driver handled the junctions himself. open251021
does not do it that way: a lead only counts when it is close (7 m there, on the decision to
release at a standstill rather than on arming).

So sweep the distance at which a lead is taken to be the answer, run the real module over
both days, and print what each threshold buys and what it costs.

  hits    a firing followed by the car coming to a halt within twenty seconds
  false   one that was not, with the seconds it held on

Run where the recordings are. Nothing here needs the solver.
"""
import glob
import os
import sys
import types

import capnp
import zstandard

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

from openpilot.selfdrive.controls.lib import stop_for_lights as SFL

SCHEMA = r'F:/c4sunny/schema_hcop'
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])

DAYS = [('9/2', r'F:/c4sunny/rlog20260902F'), ('9/3', r'F:/c4sunny/rlog20260903')]
DT = 0.05
LOOK_AHEAD = int(20. / DT)
HALTED = 0.3
THRESHOLDS = [1e9, 80., 50., 30., 15., 7.]


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    out = []
    cur = {'v': 0., 'gas': False, 'lead': None, 'eng': False, 'vc': 50 / 3.6}
    # the last segment of a route is usually cut off mid-message; take what is there
    pump = iter(log.Event.read_multiple_bytes(data))
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
            cur.update(v=float(cs.vEgo), gas=bool(cs.gasPressed),
                       vc=(kph / 3.6) if 0. < kph < 250. else 50 / 3.6)
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            cur['lead'] = e.radarState.leadOne
        elif w == 'modelV2':
            out.append((e.modelV2, dict(cur)))
    return out


def load(root):
    """Every segment of every route on a day, in order, grouped by route."""
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(root, '*--*'))})
    for route in routes:
        segs = sorted(glob.glob(os.path.join(root, route + '--*')),
                      key=lambda p: int(p.rsplit('--', 1)[1]))
        frames = []
        for seg in segs:
            frames.extend(read(seg))
        if frames:
            yield route, frames


def run(cached, limit):
    """Replay the module at one threshold and score every firing."""
    SFL.LEAD_IS_THE_ANSWER = limit
    out = {'arm': [], 'commit': []}
    for _route, frames in cached:
        sfl = SFL.StopForLights()
        speeds, states = [], []
        for md, c in frames:
            try:
                sfl.update(md, c['v'], c['vc'], c['gas'], c['lead'])
            except Exception:
                continue
            speeds.append(c['v'])
            states.append((sfl.armed, sfl.is_active, c['eng']))
        for key, idx in (('arm', 0), ('commit', 1)):
            was, start = False, 0
            for i, st in enumerate(states):
                if st[idx] and not was:
                    start = i
                elif was and not st[idx]:
                    hit = any(v < HALTED for v in speeds[start:start + LOOK_AHEAD])
                    out[key].append((states[start][2], (i - start) * DT, hit))
                was = st[idx]
            if was:
                hit = any(v < HALTED for v in speeds[start:start + LOOK_AHEAD])
                out[key].append((states[start][2], (len(states) - start) * DT, hit))
    return out


def main():
    for label, root in DAYS:
        if not os.path.isdir(root):
            print(f'{label}: 找不到 {root}，跳過')
            continue
        cached = list(load(root))
        frames = sum(len(f) for _, f in cached)
        print("")
        print(f'=== {label}  {frames} 幀（{frames * DT / 60:.1f} 分）===')
        head = f"{'前車算數的距離':>14}{'武裝':>6}{'命中':>6}{'誤報':>6}"
        print(head + f"{'誤報秒(OP開)':>13}{'承諾':>6}{'命中':>6}{'誤報':>6}{'誤報秒(OP開)':>13}")
        for limit in THRESHOLDS:
            r = run(cached, limit)
            cells = []
            for key in ('arm', 'commit'):
                got = r[key]
                bad = [g for g in got if not g[2]]
                bad_eng = [g[1] for g in bad if g[0]]
                cells.append(f"{len(got):>6}{len(got) - len(bad):>6}{len(bad):>6}{sum(bad_eng):>13.1f}")
            name = '不限（現行）' if limit > 1e8 else f'{limit:.0f} m'
            print(f"{name:>14}" + "".join(cells))


if __name__ == '__main__':
    main()
