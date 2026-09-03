"""Would mapd have had a position to look up, on the drives the driver says showed no limit?

mapd only knows where the car is because mapd_bridge writes LastGPSPosition, and it only
writes when gpsLocationExternal arrives with horizontalAccuracy under 25 m. Everything else
in the chain has now been verified offline - the tiles, the binary, the server's output, the
page's fields - so if the display stayed blank on the road, this is the remaining candidate.

Reads the same rlogs, applies the bridge's own conditions, and reports what fraction of the
drive would have produced a position.
"""
import argparse
import glob
import os
from collections import Counter

import zstandard

DEVICE_ROOT = '/data/media/0/realdata'
LAPTOP_ROOT = r'F:/c4sunny/rlog20260902F'
LAPTOP_SCHEMA = r'F:/c4sunny/schema_hcop'

MIN_ACCURACY = 25.       # mapd_bridge's own threshold

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


def main():
    segs = []
    for route in sorted({os.path.basename(d).rsplit('--', 1)[0]
                         for d in glob.glob(os.path.join(ROOT, '*--*'))}):
        segs += sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                       key=lambda p: int(p.rsplit('--', 1)[1]))

    g = Counter()
    acc_samples = []
    seen_types = set()
    for seg in segs:
        path = os.path.join(seg, 'rlog.zst')
        if not os.path.exists(path):
            continue
        data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
        pump = iter(log.Event.read_multiple_bytes(data))
        while True:
            try:
                e = next(pump)
            except StopIteration:
                break
            except Exception:
                break          # the last segment of a route is always cut short
            try:
                w = e.which()
            except Exception:
                continue
            if 'gps' not in w.lower() and 'location' not in w.lower():
                continue
            seen_types.add(w)
            if w != 'gpsLocationExternal':
                continue
            gps = e.gpsLocationExternal
            g['gps_msgs'] += 1
            acc = float(gps.horizontalAccuracy)
            acc_samples.append(acc)
            if acc < MIN_ACCURACY:
                g['would_write'] += 1
            if getattr(gps, 'hasFix', True):
                g['has_fix'] += 1

    print('log 裡與定位有關的訊息型別:', sorted(seen_types) or '(無)')
    n = max(g['gps_msgs'], 1)
    print(f"gpsLocationExternal 共 {g['gps_msgs']} 筆")
    print(f"  有 fix：{g['has_fix']} ({g['has_fix'] / n * 100:.1f}%)")
    print(f"  精度 < {MIN_ACCURACY:.0f} m（mapd_bridge 才會寫）：{g['would_write']} "
          f"({g['would_write'] / n * 100:.1f}%)")
    if acc_samples:
        acc_samples.sort()
        q = lambda p: acc_samples[int(len(acc_samples) * p)]        # noqa: E731
        print(f"  水平精度：中位 {q(0.5):.2f} m　p90 {q(0.9):.2f} m　最差 {acc_samples[-1]:.1f} m")


if __name__ == '__main__':
    main()
