"""Replay StopForLights over the 2026-09-02 afternoon drives, against the car's own schema.

The module keeps its state to itself - nothing it decides reaches a log - so the only way to
see what it did is to run the real class over the real messages again. The question is not
whether it is enabled but which gate it dies at: never arms, arms and lets go, commits and
loses to another candidate, or never gets the chance because the driver is already braking.
Those have different fixes and guessing between them is what wastes a drive.

Schema note: use the car's own capnp, copied to F:/c4sunny/schema_hcop, with include/c++.capnp
beside it. The 2025-11 viewer copy loads too, but it predates selfdriveState and still calls
leadOne.present "status", so it quietly answers a different question.

Ground truth for "should have stopped" is the driver: rolling above 5 m/s down to a halt with
nothing in front is a junction he handled himself.
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

from openpilot.selfdrive.controls.lib.stop_for_lights import StopForLights  # noqa: E402

SCHEMA = r'F:/c4sunny/schema_hcop'
ROOT = r'F:/c4sunny/rlog20260902F'
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def read(seg):
    """One segment, lined up on modelV2 - the message the planner actually runs on."""
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'gas': False, 'brake': False, 'lead': None, 'src': '?', 'eng': False,
           'vc': 30., 'a': 0.}
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
            cur.update(v=cs.vEgo, a=cs.aEgo, gas=cs.gasPressed, brake=cs.brakePressed,
                       vc=cs.cruiseState.speed or 30.)
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            cur['lead'] = e.radarState.leadOne
        elif w == 'longitudinalPlan':
            cur['src'] = str(e.longitudinalPlan.longitudinalPlanSource)
        elif w == 'modelV2':
            out.append((e.modelV2, dict(cur)))
    return out


def main():
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(ROOT, '*--*'))})
    g = Counter()
    stops = []
    events = []

    for route in routes:
        segs = sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                      key=lambda p: int(p.rsplit('--', 1)[1]))
        sfl = StopForLights()
        hist = []
        rolling = False
        was_armed = was_active = False
        for seg in segs:
            name = os.path.basename(seg)
            for md, c in read(seg):
                try:
                    sfl.update(md, c['v'], c['vc'], c['gas'], c['lead'])
                except Exception as exc:
                    g['replay_error'] += 1
                    if g['replay_error'] < 3:
                        print('replay error:', exc)
                    continue
                g['frames'] += 1
                g['engaged' if c['eng'] else 'manual'] += 1
                if c['eng']:
                    g['eng_src_' + c['src']] += 1

                if sfl.armed and not was_armed:
                    g['armed'] += 1
                    events.append({'seg': name, 'what': 'arm', 'eng': c['eng'],
                                   'kph': round(c['v'] * 3.6, 1)})
                if sfl.is_active and not was_active:
                    g['committed'] += 1
                    events.append({'seg': name, 'what': 'commit', 'eng': c['eng'],
                                   'kph': round(c['v'] * 3.6, 1),
                                   'dist': round(sfl.stop_distance, 1)})
                if was_armed and not sfl.armed and not sfl.is_active:
                    g['let_go'] += 1
                was_armed, was_active = sfl.armed, sfl.is_active

                hist.append({'v': c['v'], 'eng': c['eng'], 'brake': c['brake'], 'src': c['src'],
                             'armed': sfl.armed, 'active': sfl.is_active,
                             'lead': bool(c['lead'] and c['lead'].present)})
                hist = hist[-260:]

                if c['v'] > 5.:
                    rolling = True
                elif c['v'] < 0.5 and rolling:
                    rolling = False
                    win = hist[-200:]
                    if len(win) < 40:
                        continue
                    had_lead = any(h['lead'] for h in win[-100:])
                    g['stop_with_lead' if had_lead else 'stop_no_lead'] += 1
                    if had_lead:
                        continue
                    stops.append({
                        'seg': name,
                        'entry_kph': round(max(h['v'] for h in win) * 3.6, 1),
                        'engaged_frac': round(sum(h['eng'] for h in win) / len(win), 2),
                        'engaged_at_entry': bool(win[0]['eng']),
                        'armed': any(h['armed'] for h in win),
                        'committed': any(h['active'] for h in win),
                        'module_drove': sum(1 for h in win if h['src'] == 'e2e'),
                        'driver_braked': sum(1 for h in win if h['brake']),
                        'brake_first_at': next((round((i - len(win)) * 0.05, 1)
                                                for i, h in enumerate(win) if h['brake']), None),
                        'commit_first_at': next((round((i - len(win)) * 0.05, 1)
                                                 for i, h in enumerate(win) if h['active']), None),
                    })
            g['segments'] += 1

    print(f"段數 {g['segments']}  幀 {g['frames']}  重放錯誤 {g['replay_error']}")
    print(f"openpilot engaged {g['engaged']} 幀 ({g['engaged'] / max(g['frames'], 1) * 100:.0f}%)"
          f"   手動 {g['manual']} 幀")
    print(f"arm {g['armed']}   commit {g['committed']}   let_go {g['let_go']}")
    print(f"停止事件：有前車 {g['stop_with_lead']}   無前車 {g['stop_no_lead']}")
    print('engaged 時的縱向來源:',
          {k[8:]: v for k, v in sorted(g.items()) if k.startswith('eng_src_')})

    print('\n=== 無前車的停止，逐次 ===')
    hdr = f"{'路段':>10} {'進場kph':>8} {'OP佔比':>7} {'進場OP':>7} {'arm':>4} {'commit':>7} " \
          f"{'模組控車':>8} {'駕駛煞車':>8} {'首次煞車s':>9} {'首次commit s':>12}"
    print(hdr)
    for s in stops:
        print(f"{s['seg'].rsplit('--', 1)[1]:>10} {s['entry_kph']:8.1f} "
              f"{s['engaged_frac'] * 100:6.0f}% {'開' if s['engaged_at_entry'] else '關':>7} "
              f"{'是' if s['armed'] else '-':>4} {'是' if s['committed'] else '-':>7} "
              f"{s['module_drove']:8} {s['driver_braked']:8} "
              f"{str(s['brake_first_at']):>9} {str(s['commit_first_at']):>12}")

    eng = [s for s in stops if s['engaged_at_entry']]
    print(f"\n進場時 openpilot 開著的無前車停止: {len(eng)} 次 / 共 {len(stops)} 次")
    for s in eng:
        print(f"  {s['seg']}: arm={s['armed']} commit={s['committed']} "
              f"模組控車{s['module_drove']}幀 駕駛煞車{s['driver_braked']}幀")

    json.dump({'summary': dict(g), 'stops': stops, 'events': events},
              open(os.path.join(ROOT, 'sfl_review.json'), 'w'), ensure_ascii=False, indent=1)
    print(f"\n完整結果 -> {ROOT}/sfl_review.json")


if __name__ == '__main__':
    main()
