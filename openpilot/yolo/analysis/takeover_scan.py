"""Find the moments the driver had to step in, and what the car was asking for just before.

A complaint arrives as a time and a feeling. This turns that into a list: every time
openpilot was driving and the brake went down, and every time it asked for acceleration
while sitting close behind something. Both come with the ten seconds leading up to them, so
the reason is on the same screen as the event.

    python openpilot/yolo/analysis/takeover_scan.py --route 00000015--d6a173598d \
        --segments 2-9 --root F:/c4sunny/rlog20260903
"""
import argparse
import datetime
import os

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
TZ = datetime.timezone(datetime.timedelta(hours=8))

ap = argparse.ArgumentParser()
ap.add_argument('--route', required=True)
ap.add_argument('--segments', required=True, help='e.g. 2-9')
ap.add_argument('--root', required=True)
ap.add_argument('--creep-lead', type=float, default=8.0, help='close enough to count as behind something')
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])

DT = 0.05
LEAD_UP = int(10. / DT)


def frames_of(seg_dir):
    """Wall time comes off the clocks message, the way window.py reads it: logMonoTime is
    monotonic since boot and rendering it as a date gives an hour that looks plausible and
    is wrong."""
    path = os.path.join(seg_dir, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    out = []
    t0 = None
    cur = {'v': 0., 'brake': False, 'gas': False, 'eng': False, 'aT': 0., 'src': '?',
           'lead': False, 'dRel': 0., 'vLead': 0., 'radar': False, 'prob': 0., 'stop': False,
           'ts': None}
    for e in log.Event.read_multiple_bytes(data):
        try:
            w = e.which()
        except Exception:
            continue
        if w == 'clocks' and t0 is None:
            t0 = datetime.datetime.fromtimestamp(e.clocks.wallTimeNanos / 1e9, TZ)
        elif w == 'carState':
            cs = e.carState
            cur.update(v=float(cs.vEgo) * 3.6, brake=bool(cs.brakePressed), gas=bool(cs.gasPressed))
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            lead = e.radarState.leadOne
            cur.update(lead=bool(lead.present), dRel=float(lead.dRel), vLead=float(lead.vLead) * 3.6,
                       radar=bool(lead.radar), prob=float(lead.modelProb))
        elif w == 'longitudinalPlan':
            lp = e.longitudinalPlan
            cur.update(aT=float(lp.aTarget), src=str(lp.longitudinalPlanSource), stop=bool(lp.shouldStop))
        elif w == 'modelV2':
            out.append(dict(cur))
    # clocks can arrive after the first few modelV2, so stamp the whole segment afterwards,
    # anchored on its start the way window.py does
    for i, f in enumerate(out):
        f['ts'] = (t0 + datetime.timedelta(seconds=i * DT)) if t0 else None
    return out


def show(frames, i, why):
    t = frames[i]['ts']
    stamp = t.strftime('%H:%M:%S') if t else '??'
    print("")
    print(f"### {why}  @ {stamp}")
    head = f"{'秒':>6} {'kph':>6} {'OP':>3} {'油':>3} {'煞':>3} {'aTgt':>7} {'來源':>8}"
    print(head + f" {'前車':>5} {'R/V':>4} {'距m':>7} {'前車kph':>8} {'prob':>6}")
    for j in range(max(0, i - LEAD_UP), min(len(frames), i + int(3. / DT)), 10):
        f = frames[j]
        rv = ('R' if f['radar'] else 'V') if f['lead'] else '-'
        a = f"{(j - i) * DT:6.1f} {f['v']:6.1f} {'開' if f['eng'] else '關':>3}"
        b = f" {'踩' if f['gas'] else '-':>3} {'踩' if f['brake'] else '-':>3}"
        c = f" {f['aT']:7.2f} {f['src']:>8} {'有' if f['lead'] else '無':>5} {rv:>4}"
        print(a + b + c + f" {f['dRel']:7.1f} {f['vLead']:8.1f} {f['prob']:6.2f}")


def main():
    lo, hi = (int(x) for x in args.segments.split('-'))
    frames = []
    for n in range(lo, hi + 1):
        frames.extend(frames_of(os.path.join(args.root, f'{args.route}--{n}')))
    if not frames:
        raise SystemExit('沒讀到任何 modelV2')
    span = (frames[-1]['ts'] - frames[0]['ts']).total_seconds() / 60
    print(f"{len(frames)} 幀，{frames[0]['ts']:%H:%M:%S} – {frames[-1]['ts']:%H:%M:%S}（{span:.1f} 分）")

    last = None
    for i in range(1, len(frames)):
        a, b = frames[i - 1], frames[i]
        if a['eng'] and not b['eng'] and b['brake']:
            if last is None or (i - last) * DT > 5:
                show(frames, i, '駕駛踩煞車接管')
                last = i

    print("")
    print(f'=== 停等中卻要求加速（OP 開著、車速 < 5 km/h、前車在 {args.creep_lead:.0f} m 內、aTarget > 0.2）===')
    last = None
    for i, f in enumerate(frames):
        if (f['eng'] and f['v'] < 5. and f['lead'] and f['dRel'] < args.creep_lead
                and f['aT'] > 0.2 and abs(f['vLead']) < 3.):
            if last is None or (i - last) * DT > 5:
                t = f['ts'].strftime('%H:%M:%S') if f['ts'] else '??'
                line = f"  {t}  車速 {f['v']:.1f} kph  前車 {f['dRel']:.1f} m（{f['vLead']:+.1f} kph）"
                print(line + f"  aTarget {f['aT']:+.2f}  來源 {f['src']}")
                last = i
    if last is None:
        print('  沒有')


if __name__ == '__main__':
    main()
