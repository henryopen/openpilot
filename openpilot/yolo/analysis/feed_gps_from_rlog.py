"""Feed mapd the GPS track from a recorded drive, the way mapd_bridge would have.

mapd is given a position and nothing else; whether it can also name the *next* speed limit
and how far away it is depends on it seeing the car move along a way. Feeding one fixed
coordinate gets the current limit and an empty "ahead" - which is exactly what the driver
reports seeing on the road.

So replay the real positions at the rate the bridge writes them (2 Hz, accuracy under 25 m)
and watch both values. Run mapd alongside:

    /data/media/0/osm/mapd > /tmp/mapd.log 2>&1 &
    PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \
        openpilot/yolo/analysis/feed_gps_from_rlog.py --route 00000010--17de10767f --segments 33-35
"""
import argparse
import json
import os
import sys
import time

import zstandard

sys.path.insert(0, '/data/openpilot')
from openpilot.cereal import log

MIN_ACCURACY = 25.
POS = '/dev/shm/params/d/LastGPSPosition'

ap = argparse.ArgumentParser()
ap.add_argument('--route', required=True)
ap.add_argument('--segments', required=True, help='e.g. 33-35')
ap.add_argument('--root', default='/data/media/0/realdata')
ap.add_argument('--rate', type=float, default=2.0, help='writes per second, as mapd_bridge does')
ap.add_argument('--watch', action='store_true', help='print what mapd produces as we go')
ap.add_argument('--summary', action='store_true',
                help='tally which roads have a limit in the map data and which do not')
args = ap.parse_args()


def positions(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    while True:
        try:
            e = next(pump)
        except StopIteration:
            return
        except Exception:
            return
        try:
            if e.which() != 'gpsLocationExternal':
                continue
        except Exception:
            continue
        gps = e.gpsLocationExternal
        if float(gps.horizontalAccuracy) >= MIN_ACCURACY:
            continue
        yield {'latitude': float(gps.latitude), 'longitude': float(gps.longitude),
               'bearing': float(gps.bearingDeg)}


def read(name):
    try:
        return open(f'/dev/shm/params/d/{name}').read()
    except Exception:
        return ''


def main():
    lo, hi = (int(v) for v in args.segments.split('-'))
    period = 1.0 / args.rate
    n = 0
    roads: dict[str, list] = {}
    for s in range(lo, hi + 1):
        seg = os.path.join(args.root, f'{args.route}--{s}')
        for pos in positions(seg):
            open(POS, 'w').write(json.dumps(pos))
            n += 1
            if args.summary and n % 8 == 0:
                road = read('RoadName')
                if road:
                    limit = float(read('MapSpeedLimit') or 0) * 3.6
                    r = roads.setdefault(road, [0, 0, 0.])
                    r[0] += 1
                    if limit > 0:
                        r[1] += 1
                        r[2] = limit
            if args.watch and n % 4 == 0:
                nxt = read('NextMapSpeedLimit')
                try:
                    j = json.loads(nxt or '{}')
                    ahead = f"{j.get('speedlimit', 0) * 3.6:.0f} km/h @ {j.get('distance', 0):.0f} m"
                except Exception:
                    ahead = nxt[:40]
                print(f"{n:5}  {pos['latitude']:.5f},{pos['longitude']:.5f}  "
                      f"現在={float(read('MapSpeedLimit') or 0) * 3.6:.0f} km/h  "
                      f"下一個={ahead}  {read('RoadName')}", flush=True)
            time.sleep(period)

    print(f'\n' + f'餵完 {n} 個位置，經過 {len(roads)} 條路')
    if args.summary and roads:
        have = [(r, v) for r, v in roads.items() if v[1] > 0]
        none = [(r, v) for r, v in roads.items() if v[1] == 0]
        tot = sum(v[0] for v in roads.values())
        hit = sum(v[0] for _, v in have)
        print(f'有速限資料的取樣點：{hit}/{tot} ({hit / max(tot, 1) * 100:.1f}%)')
        print(f'\n' + f'=== 查得到速限的路（{len(have)} 條）===')
        for r, v in sorted(have, key=lambda x: -x[1][0])[:12]:
            print(f'  {r:24} {v[2]:5.0f} km/h   取樣 {v[0]}')
        print(f'\n' + f'=== 查不到速限的路（{len(none)} 條）===')
        for r, v in sorted(none, key=lambda x: -x[1][0])[:12]:
            print(f'  {r:24} 取樣 {v[0]}')


if __name__ == '__main__':
    main()
