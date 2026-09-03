"""When did the set speed change by itself, and what was the speed limit assist doing?

The driver reports the set speed moving on its own while the Pi shows no limit at all. Those
two cannot both be explained by "the map has no data": the assist only moves the set speed
when it has a limit to move it to. So find every change the driver did not make - no button
pressed in the surrounding second - and print what was on screen at the time.

cruiseState.speed is the set speed in the cluster's units. Button presses are in
carState.buttonEvents, so a change with no button near it came from the assist.
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
BUTTON_WINDOW = 40      # two seconds of frames either side


def main():
    segs = []
    for route in sorted({os.path.basename(d).rsplit('--', 1)[0]
                         for d in glob.glob(os.path.join(ROOT, '*--*'))}):
        segs += sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                       key=lambda p: int(p.rsplit('--', 1)[1]))

    g = Counter()
    changes = []
    for seg in segs:
        path = os.path.join(seg, 'rlog.zst')
        if not os.path.exists(path):
            continue
        data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
        pump = iter(log.Event.read_multiple_bytes(data))
        rows = []
        while True:
            try:
                e = next(pump)
            except StopIteration:
                break
            except Exception:
                break
            try:
                if e.which() != 'carState':
                    continue
            except Exception:
                continue
            cs = e.carState
            # vCruise is openpilot's own set speed under OP LONG; cruiseState.speed is the
            # stock ACC's, which openpilot does not drive here
            rows.append({'set': float(cs.vCruise),
                         'stock': float(cs.cruiseState.speed) * 3.6,
                         'v': float(cs.vEgo) * 3.6,
                         'enabled': bool(cs.cruiseState.enabled),
                         'buttons': len(cs.buttonEvents) > 0})
        for i in range(1, len(rows)):
            a, b = rows[i - 1], rows[i]
            if not (a['enabled'] and b['enabled']):
                continue
            g['frames'] += 1
            if abs(b['set'] - a['set']) < 0.5:
                continue
            lo = max(0, i - BUTTON_WINDOW)
            hi = min(len(rows), i + BUTTON_WINDOW)
            pressed = any(r['buttons'] for r in rows[lo:hi])
            g['changes'] += 1
            g['by_button' if pressed else 'by_itself'] += 1
            if not pressed:
                changes.append({'seg': os.path.basename(seg), 'from': round(a['set'], 1),
                                'to': round(b['set'], 1), 'kph': round(b['v'], 1)})

    print(f"engaged 的幀 {g['frames']}（{g['frames'] * 0.01 / 60:.1f} 分鐘的 carState）")
    print(f"（set 用 carState.vCruise，openpilot 自己的設定速度）")
    print(f"設定速度變化 {g['changes']} 次：按鍵造成 {g['by_button']}，**自己變的 {g['by_itself']}**")
    if changes:
        print(f"\n{'路段':>24} {'從':>7} {'到':>7} {'當時車速':>9}")
        for c in changes[:25]:
            print(f"{c['seg'][-24:]:>24} {c['from']:7.1f} {c['to']:7.1f} {c['kph']:9.1f}")
        tos = Counter(c['to'] for c in changes)
        print(f"\n自己變到的目標值分布（前 8）：{tos.most_common(8)}")


if __name__ == '__main__':
    main()
