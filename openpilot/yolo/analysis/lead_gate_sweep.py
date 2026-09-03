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

# The rule that ships is relative: a lead counts only if it is at or before the stop. The
# absolute distances are here to show what a fixed number would have done instead, and
# "any" is what the module did before either.
RELATIVE = object()
# The car follows on time, not on metres: this one is set to relaxed, 1.75 s. So the
# headway gates are the ones worth reading - a fixed distance means a different thing at
# every speed, which is how 48 m at 32 km/h got called an ordinary following distance when
# it is 5.4 s, three times the setting.
T_FOLLOW = 1.75
GATES = [('任何前車（原本）', None),
         ('絕對 50 m', ('m', 50.)), ('絕對 30 m', ('m', 30.)),
         ('時距 2.0 s', ('s', 2.0)), ('時距 2.5 s', ('s', 2.5)),
         ('時距 3.0 s', ('s', 3.0)), ('時距 3.5 s（2×設定）', ('s', 3.5)),
         ('相對：停止點之前', RELATIVE)]
_real_gate = SFL.StopForLights._lead_is_the_answer


def _by_metres(self, lead, stop_ahead, d):
  return lead is not None and lead.present and float(lead.dRel) < d


def _by_headway(self, lead, stop_ahead, secs, v_ego):
  if lead is None or not lead.present:
    return False
  if v_ego < 1.:
    return True                       # crawling: headway is meaningless, a lead is a lead
  return float(lead.dRel) / v_ego < secs


def patch(kind):
  if kind is RELATIVE:
    SFL.StopForLights._lead_is_the_answer = _real_gate
  elif kind is None:
    SFL.StopForLights._lead_is_the_answer = (
        lambda self, lead, stop_ahead: lead is not None and lead.present)
  elif kind[0] == 'm':
    SFL.StopForLights._lead_is_the_answer = (
        lambda self, lead, stop_ahead, d=kind[1]: _by_metres(self, lead, stop_ahead, d))
  else:
    SFL.StopForLights._lead_is_the_answer = (
        lambda self, lead, stop_ahead, t=kind[1]: _by_headway(self, lead, stop_ahead, t, self._v_ego))


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


def real_stops(speeds, leads):
    """Every halt that followed real speed, split by whether anything was in front.

    The split is the whole point: a stop with a car at it is the planner's, by the driver's
    own rule, and covering more of those is not this module doing its job. The ones with
    nothing in front are the ones it exists for.
    """
    out, rolling = [], False
    for i, v in enumerate(speeds):
        if v > 5.:
            rolling = True
        elif v < HALTED and rolling:
            rolling = False
            out.append((i, any(leads[max(0, i - 100):i])))
    return out


def run(cached, kind):
    """Replay the module under one gate and score every firing."""
    patch(kind)
    out = {'arm': [], 'commit': [], 'no_lead': [0, 0], 'with_lead': [0, 0]}
    for _route, frames in cached:
        sfl = SFL.StopForLights()
        speeds, states, leads = [], [], []
        for md, c in frames:
            sfl._v_ego = c['v']      # the headway gates need it; the shipped rule does not
            try:
                sfl.update(md, c['v'], c['vc'], c['gas'], c['lead'])
            except Exception:
                continue
            speeds.append(c['v'])
            leads.append(bool(c['lead'] is not None and c['lead'].present))
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
        # what actually matters: how many of the day's real stops the module was committed
        # to at some point in the twenty seconds before the car came to rest
        for i, had_lead in real_stops(speeds, leads):
            box = out['with_lead'] if had_lead else out['no_lead']
            box[1] += 1
            if any(st[1] for st in states[max(0, i - LOOK_AHEAD):i]):
                box[0] += 1
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
        head = f"{'前車何時算數':>18}{'涵蓋:無前車停止':>16}{'涵蓋:有前車停止':>16}{'誤承諾':>7}"
        print(head + f"{'誤煞秒(OP開)':>13}{'武裝誤報秒':>12}")
        for name, kind in GATES:
            r = run(cached, kind)
            bad = [g for g in r['commit'] if not g[2]]
            bad_eng = sum(g[1] for g in bad if g[0])
            arm_bad = sum(g[1] for g in r['arm'] if not g[2] and g[0])
            a = f"{r['no_lead'][0]}/{r['no_lead'][1]}"
            b = f"{r['with_lead'][0]}/{r['with_lead'][1]}"
            row = f"{name:>18}{a:>16}{b:>16}{len(bad):>7}"
            print(row + f"{bad_eng:>13.1f}{arm_bad:>12.1f}")


if __name__ == '__main__':
    main()
