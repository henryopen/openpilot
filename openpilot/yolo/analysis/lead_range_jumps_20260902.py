"""Does the lead's range move the way its closing speed says it should?

Behind motorcycles the car kept easing on and off the throttle. The lead was radar-backed
throughout and the track id never changed, but the range moved 3 m in a quarter of a second
while the closing speed read under 2 km/h. Those two cannot both be true of one object: the
stock ACC's slot had swapped to a different machine, and radard passed the new range on as
if the same car had jumped.

So compare, frame to frame, how far the range actually moved against how far vRel says it
should have. A large mismatch is the slot changing its mind. Counting them says whether this
is worth defending against, and the accel at the time says what it costs.
"""
import argparse
import glob
import os
from collections import Counter

import zstandard

DEVICE_ROOT = '/data/media/0/realdata'
LAPTOP_ROOT = r'F:/c4sunny/rlog20260902F'
LAPTOP_SCHEMA = r'F:/c4sunny/schema_hcop'

ap = argparse.ArgumentParser()
ap.add_argument('--on-device', action='store_true')
ap.add_argument('--root', default='')
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

DT = 0.05
WINDOW = 5            # compare across a quarter second: the swaps are quick, not instant
MISMATCH = 1.5        # metres of range movement the closing speed does not account for


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'eng': False, 'a': 0.}
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
        elif w == 'radarState':
            ld = e.radarState.leadOne
            out.append({**cur,
                        'lead': bool(ld.present),
                        'radar': bool(ld.radar) if ld.present else False,
                        'd': float(ld.dRel) if ld.present else 0.,
                        'vrel': float(ld.vRel) if ld.present else 0.,
                        'tid': int(ld.radarTrackId) if ld.present else -1})
    return out


def main():
    segs = []
    for route in sorted({os.path.basename(d).rsplit('--', 1)[0]
                         for d in glob.glob(os.path.join(ROOT, '*--*'))}):
        segs += sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                       key=lambda p: int(p.rsplit('--', 1)[1]))

    g = Counter()
    jumps = []
    for seg in segs:
        rows = read(seg)
        for i, r in enumerate(rows):
            if i < WINDOW:
                continue
            prev = rows[i - WINDOW]
            if not (r['eng'] and r['lead'] and r['radar']):
                continue
            if prev['lead'] and prev['radar'] and prev['tid'] == r['tid']:
                g['pairs'] += 1
                # vRel is the closing speed over the window; average the two ends
                expected = (prev['vrel'] + r['vrel']) / 2 * DT * WINDOW
                actual = r['d'] - prev['d']
                err = actual - expected
                if abs(err) > MISMATCH:
                    g['mismatch'] += 1
                    g['further' if err > 0 else 'nearer'] += 1
                    jumps.append({'seg': os.path.basename(seg), 'from': round(prev['d'], 1),
                                  'to': round(r['d'], 1), 'err': round(err, 1),
                                  'vrel_kph': round(prev['vrel'] * 3.6, 1),
                                  'kph': round(r['v'] * 3.6, 1), 'a': round(r['a'], 2)})

    pairs = max(g['pairs'], 1)
    print(f"雷達 lead 且 trackId 不變的 {WINDOW * DT:.2f} 秒窗口：{pairs}（{pairs * DT / 60:.1f} 分鐘）")
    print(f"距離變化與相對速對不上（>{MISMATCH} m / {WINDOW * DT:.2f}s）：{g['mismatch']} "
          f"({g['mismatch'] / pairs * 100:.2f}%)   每分鐘 {g['mismatch'] / (pairs * DT / 60):.1f} 次")
    print(f"  其中突然變遠：{g['further']}   突然變近：{g['nearer']}")

    if jumps:
        far = sorted((j for j in jumps if j['err'] > 0), key=lambda j: -j['err'])
        near = sorted((j for j in jumps if j['err'] < 0), key=lambda j: j['err'])
        print(f"\n跳最遠的 10 次（車以為前方變空，會放行加速）")
        print(f"{'路段':>22} {'從m':>6} {'到m':>6} {'誤差m':>7} {'vRel':>7} {'車速':>6} {'aTgt':>6}")
        for j in far[:10]:
            print(f"{j['seg'][-22:]:>22} {j['from']:6.1f} {j['to']:6.1f} {j['err']:7.1f} " +
                  f"{j['vrel_kph']:7.1f} {j['kph']:6.1f} {j['a']:6.2f}")
        print(f"\n跳最近的 5 次（車以為前方突然逼近，會急煞）")
        for j in near[:5]:
            print(f"{j['seg'][-22:]:>22} {j['from']:6.1f} {j['to']:6.1f} {j['err']:7.1f} " +
                  f"{j['vrel_kph']:7.1f} {j['kph']:6.1f} {j['a']:6.2f}")

        # what the accel was doing around a jump outward
        acc = [j['a'] for j in far if j['a'] > 0.3]
        print(f"\n變遠的跳動中，當下 aTarget > 0.3（正在放行加速）的有 {len(acc)} 次")


if __name__ == '__main__':
    main()
