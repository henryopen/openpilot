"""The five minutes the driver pointed at: 14:15-14:20 on 2026-09-02, second by second.

He says the car slowed and he could not tell what did it, and that it was not the curve
limiter. Rather than argue from summaries, print the window itself: speed against set speed,
who the planner says is holding the speed, what the lead is doing, and where the driver's
feet were. Wall clock comes from the log's own clocks message - directory mtimes are when
the files were copied, not when they were driven.
"""
import datetime
import glob
import os

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
ROOT = r'F:/c4sunny/rlog20260902F'
ROUTE = '00000010--17de10767f'
TZ = datetime.timezone(datetime.timedelta(hours=8))
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def scan(seg, want_rows):
    """Wall clock of the segment, and optionally every frame in it."""
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return None, []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    t0 = None
    rows = []
    cur = {'v': 0., 'set': 0., 'brake': False, 'gas': False, 'eng': False, 'lead': False,
           'd': 0., 'vrel': 0., 'reason': '?', 'src': '?', 'a': 0., 'steer': 0.}
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
        if w == 'clocks' and t0 is None:
            t0 = datetime.datetime.fromtimestamp(e.clocks.wallTimeNanos / 1e9, TZ)
            if not want_rows:
                return t0, []
        elif w == 'carState':
            cs = e.carState
            cur.update(v=cs.vEgo, set=cs.cruiseState.speed or 0., brake=cs.brakePressed,
                       gas=cs.gasPressed, steer=float(cs.steeringAngleDeg))
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            ld = e.radarState.leadOne
            cur['lead'] = bool(ld.present)
            cur['d'] = float(ld.dRel) if ld.present else 0.
            cur['vrel'] = float(ld.vRel) if ld.present else 0.
        elif w == 'longitudinalPlanSP':
            cur['reason'] = str(e.longitudinalPlanSP.reason)
        elif w == 'longitudinalPlan':
            cur['src'] = str(e.longitudinalPlan.longitudinalPlanSource)
            cur['a'] = float(e.longitudinalPlan.aTarget)
            if want_rows:
                rows.append(dict(cur))
    return t0, rows


def main():
    segs = sorted(glob.glob(os.path.join(ROOT, ROUTE + '--*')),
                  key=lambda p: int(p.rsplit('--', 1)[1]))
    print('=== 各段的實際時間（台灣）===')
    times = {}
    for s in segs[:20]:
        t0, _ = scan(s, False)
        times[s] = t0
        if t0:
            print(f"  {os.path.basename(s).rsplit('--', 1)[1]:>3}  {t0:%H:%M:%S}")

    want = [s for s, t in times.items() if t and 14 * 60 + 14 <= t.hour * 60 + t.minute <= 14 * 60 + 20]
    print(f"\n14:14-14:20 落在 {len(want)} 段\n")

    for s in want:
        t0 = times[s]
        _, rows = scan(s, True)
        if not rows:
            continue
        print(f"=== {os.path.basename(s)}  起 {t0:%H:%M:%S} ===")
        print(f"{'時刻':>8} {'kph':>6} {'設定':>5} {'OP':>3} {'油':>3} {'煞':>3} "
              f"{'aTarget':>8} {'原因':>9} {'來源':>7} {'前車m':>7} {'相對kph':>8} {'方向盤':>7}")
        prev = None
        for i, r in enumerate(rows):
            if i % 20:                                   # once a second
                continue
            t = t0 + datetime.timedelta(seconds=i * 0.05)
            mark = ''
            if prev is not None and (prev - r['v']) * 3.6 > 2.0:
                mark = '  <<< 掉速'
            print(f"{t:%H:%M:%S} {r['v'] * 3.6:6.1f} {r['set'] * 3.6:5.0f} "
                  f"{'開' if r['eng'] else '關':>3} {'踩' if r['gas'] else '-':>3} "
                  f"{'踩' if r['brake'] else '-':>3} {r['a']:8.2f} {r['reason']:>9} "
                  f"{r['src']:>7} {r['d']:7.1f} {r['vrel'] * 3.6:8.1f} {r['steer']:7.1f}{mark}")
            prev = r['v']
        print()


if __name__ == '__main__':
    main()
