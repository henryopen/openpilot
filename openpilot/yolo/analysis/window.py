"""Print a stretch of a drive second by second: speed, who is holding it, and what is ahead.

Written for the question "the car kept starting and stopping behind a motorcycle - was the
radar locked on it at all", which needs the lead's source and range next to the throttle and
brake rather than a summary. Takes a route and segment range so it can be pointed at any
complaint without editing it.

    python openpilot/yolo/analysis/window.py --route 00000010--17de10767f --segments 33-34
    ... --on-device            # run against the car's own logs and schema
    ... --every 4              # a row every 0.2 s instead of every second
"""
import argparse
import datetime
import os

import zstandard

DEVICE_ROOT = '/data/media/0/realdata'
LAPTOP_ROOT = r'F:/c4sunny/rlog20260902F'
LAPTOP_SCHEMA = r'F:/c4sunny/schema_hcop'
TZ = datetime.timezone(datetime.timedelta(hours=8))

ap = argparse.ArgumentParser()
ap.add_argument('--route', required=True)
ap.add_argument('--segments', required=True, help='e.g. 33-34')
ap.add_argument('--on-device', action='store_true')
ap.add_argument('--root', default='')
ap.add_argument('--every', type=int, default=20, help='rows per second of log (20 = 1 s apart)')
ap.add_argument('--min-kph', type=float, default=-1, help='only print rows above this speed')
ap.add_argument('--tracks', action='store_true', help='list every radar track, not just the lead')
args = ap.parse_args()

if args.on_device:
    import sys
    sys.path.insert(0, '/data/openpilot')
    from openpilot.cereal import log
else:
    import capnp
    capnp.remove_import_hook()
    os.chdir(LAPTOP_SCHEMA)
    log = capnp.load('log.capnp', imports=[LAPTOP_SCHEMA])

ROOT = args.root or (DEVICE_ROOT if args.on_device else LAPTOP_ROOT)


def read(seg):
    """Rows on the radar's clock, since the lead is what this is about."""
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return None, []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    t0 = None
    rows = []
    cur = {'v': 0., 'gas': False, 'brake': False, 'eng': False, 'a': 0., 'reason': '?',
           'src': '?', 'prob': 0., 'vx': 0.}
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
        elif w == 'carState':
            cs = e.carState
            cur.update(v=cs.vEgo, gas=cs.gasPressed, brake=cs.brakePressed)
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'longitudinalPlan':
            cur['a'] = float(e.longitudinalPlan.aTarget)
            cur['src'] = str(e.longitudinalPlan.longitudinalPlanSource)
        elif w == 'longitudinalPlanSP':
            cur['reason'] = str(e.longitudinalPlanSP.reason)
        elif w == 'modelV2':
            lv = e.modelV2.leadsV3
            if len(lv):
                cur['prob'] = float(lv[0].prob)
                cur['vx'] = float(lv[0].x[0])
        elif w == 'radarState':
            ld = e.radarState.leadOne
            rows.append({**cur,
                         'lead': bool(ld.present),
                         'radar': bool(ld.radar) if ld.present else False,
                         'd': float(ld.dRel) if ld.present else 0.,
                         'y': float(ld.yRel) if ld.present else 0.,
                         'vrel': float(ld.vRel) if ld.present else 0.,
                         'tid': int(ld.radarTrackId) if ld.present else -1,
                         'ntracks': len(cur.get('pts', [])),
                         'pts': list(cur.get('pts', []))})
        elif w == 'radarTracksSP':
            # every decoded track, not only the one radard was handed
            cur['pts'] = [(int(p.trackId), round(float(p.dRel), 1), round(float(p.yRel), 2),
                           round(float(p.vRel) * 3.6, 1)) for p in e.radarTracksSP.points]
    return t0, rows


def main():
    lo, hi = (int(v) for v in args.segments.split('-'))
    print(f"{'時刻':>8} {'kph':>6} {'OP':>3} {'油':>3} {'煞':>3} {'aTgt':>6} {'原因':>9} | " +
          f"{'lead':>5} {'R/V':>4} {'距m':>6} {'橫m':>6} {'相對kph':>8} {'trkId':>7} {'#trk':>5} | " +
          f"{'視覺prob':>8} {'視覺m':>6}")
    for n in range(lo, hi + 1):
        seg = os.path.join(ROOT, f'{args.route}--{n}')
        t0, rows = read(seg)
        if not rows:
            continue
        print(f'--- 段 {n}  起 {t0:%H:%M:%S} ---' if t0 else f'--- 段 {n} ---')
        for i, r in enumerate(rows):
            if i % args.every:
                continue
            if r['v'] * 3.6 < args.min_kph:
                continue
            t = (t0 + datetime.timedelta(seconds=i * 0.05)) if t0 else None
            stamp = f'{t:%H:%M:%S}' if t else f'{i * 0.05:7.1f}'
            print(f"{stamp:>8} {r['v'] * 3.6:6.1f} {'開' if r['eng'] else '關':>3} " +
                  f"{'踩' if r['gas'] else '-':>3} {'踩' if r['brake'] else '-':>3} " +
                  f"{r['a']:6.2f} {r['reason']:>9} | " +
                  f"{'有' if r['lead'] else '無':>5} {('R' if r['radar'] else 'V') if r['lead'] else '-':>4} " +
                  f"{r['d']:6.1f} {r['y']:6.2f} {r['vrel'] * 3.6:8.1f} {r['tid']:7} {r['ntracks']:5} | " +
                  f"{r['prob']:8.2f} {r['vx']:6.1f}")
            if args.tracks:
                for tid, d, y, vk in sorted(r['pts'], key=lambda t: t[1]):
                    mark = ' <= lead' if tid == r['tid'] else ''
                    print(f"{'':>8}   track {tid:>7}  {d:6.1f} m  y={y:6.2f}  {vk:7.1f} kph{mark}")


if __name__ == '__main__':
    main()
