"""What the radar gave us, and what each of our own gates threw away.

The near-range threshold was found and relaxed one symptom at a time, which is not the same
as knowing what the decode costs. This walks the whole chain instead: every one of the ten
slots, every radar frame, through each gate in the order radar_interface applies them, and
counts what survives.

  raw          the slot reported anything at all (score > 0 and a range)
  range        rng > CUSTIN_MIN_RANGE, or CUSTIN_PRIMARY_MIN_RANGE on 0x238
  score        SCORE >= CUSTIN_MIN_SCORE
  lateral      |y| < CUSTIN_MAX_ABS_Y, which 0x238 is exempt from
  held         eight consecutive frames without the slot jumping or going quiet
  to radard    and then only 0x238 survives, because CUSTIN_PRIMARY_ONLY is on

Split by range band and by whether the target is moving with traffic or standing still,
because the two are lost at different gates and only one of them is a car to follow.

    python openpilot/yolo/analysis/radar_audit.py --root F:/c4sunny/rlog20260903
    python openpilot/yolo/analysis/radar_audit.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import glob
import os
from collections import Counter, defaultdict, deque

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'

ADDRS = tuple(range(0x238, 0x238 + 3 * 10, 3))
PRIMARY = ADDRS[0]
MIN_RANGE = 2.0            # CUSTIN_MIN_RANGE
PRIMARY_MIN_RANGE = 0.5    # CUSTIN_PRIMARY_MIN_RANGE, added 2026-09-04
MIN_SCORE = 30             # CUSTIN_MIN_SCORE
MAX_ABS_Y = 5.5            # CUSTIN_MAX_ABS_Y
MAX_GAP = 0.2              # CUSTIN_MAX_GAP
MAX_JUMP = 3.0             # CUSTIN_MAX_JUMP
MIN_HITS = 8               # CUSTIN_MIN_HITS

BANDS = [(0, 10), (10, 25), (25, 50), (50, 100)]
MOVING = 15.0              # km/h absolute: above this it is traffic, below it is scenery

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True)
ap.add_argument('--segments', default='', help='e.g. 3-8, default every segment found')
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def be_bits(data, start_bit, length):
    val = 0
    bit = start_bit
    for _ in range(length):
        val = (val << 1) | ((data[bit // 8] >> (bit % 8)) & 1)
        bit = bit + 15 if bit % 8 == 0 else bit - 1
    return val


def signed(val, length):
    return val - (1 << length) if val >= (1 << (length - 1)) else val


def decode(data):
    if len(data) < 8:
        return None
    return {'dist': be_bits(data, 5, 10) * 0.1,
            'lat': signed(be_bits(data, 21, 10), 10) * 0.0475,
            'v_abs': signed(be_bits(data, 37, 11), 11) * 0.02526 + 2.587,
            'score': be_bits(data, 41, 6)}


def band_of(d):
    for lo, hi in BANDS:
        if lo <= d < hi:
            return f'{lo}-{hi} m'
    return '>=100 m'


def main():
    segs = sorted(glob.glob(os.path.join(args.root, '*--*')),
                  key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))
    if args.segments:
        lo, hi = (int(x) for x in args.segments.split('-'))
        segs = [s for s in segs if lo <= int(s.rsplit('--', 1)[1]) <= hi]
    segs = [s for s in segs if os.path.exists(os.path.join(s, 'rlog.zst'))]
    if not segs:
        raise SystemExit(f'{args.root} 底下沒有段')

    stages = ['raw', 'range', 'score', 'lateral', 'held', 'to radard']
    seen = defaultdict(Counter)      # (band, moving) -> stage counts
    per_slot = defaultdict(Counter)  # addr -> stage counts
    other_useful = Counter()         # non-primary slots that survived to 'held'
    frames = 0

    for seg in segs:
        data = zstandard.ZstdDecompressor().stream_reader(
            open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
        hist = {a: deque(maxlen=12) for a in ADDRS}
        hits = dict.fromkeys(ADDRS, 0)
        t = 0.
        raw = {}
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
                pass
            elif w == 'can':
                t += 1 / 33.
                got = False
                for c in e.can:
                    if c.src == 1 and c.address in hist:
                        raw[c.address] = decode(bytes(c.dat))
                        got = True
                if not got:
                    continue
                frames += 1
                held_now = []
                for a in ADDRS:
                    r = raw.get(a)
                    if r is None:
                        continue
                    primary = a == PRIMARY
                    key = (band_of(r['dist']), 'moving' if r['v_abs'] * 3.6 > MOVING else 'static')
                    if r['score'] == 0 or r['dist'] == 0.:
                        continue                       # an empty slot, not a loss
                    seen[key]['raw'] += 1
                    per_slot[a]['raw'] += 1

                    floor = PRIMARY_MIN_RANGE if primary else MIN_RANGE
                    if not r['dist'] > floor:
                        hist[a].clear()
                        hits[a] = 0
                        continue
                    seen[key]['range'] += 1
                    per_slot[a]['range'] += 1

                    if not r['score'] >= MIN_SCORE:
                        hist[a].clear()
                        hits[a] = 0
                        continue
                    seen[key]['score'] += 1
                    per_slot[a]['score'] += 1

                    if not (primary or abs(-r['lat']) < MAX_ABS_Y):
                        hist[a].clear()
                        hits[a] = 0
                        continue
                    seen[key]['lateral'] += 1
                    per_slot[a]['lateral'] += 1

                    # the slot's own history, exactly as CustinSlot keeps it
                    if hist[a] and (t - hist[a][-1][0] > MAX_GAP
                                    or abs(r['dist'] - hist[a][-1][1]) > MAX_JUMP):
                        hist[a].clear()
                        hits[a] = 0
                    hist[a].append((t, r['dist']))
                    hits[a] = min(hits[a] + 1, MIN_HITS)
                    if hits[a] < MIN_HITS:
                        continue
                    seen[key]['held'] += 1
                    per_slot[a]['held'] += 1
                    held_now.append((a, r, key))

                    if primary:
                        seen[key]['to radard'] += 1
                        per_slot[a]['to radard'] += 1

                got_primary = any(a == PRIMARY for a, _, _ in held_now)
                for a, r, _k in held_now:
                    if a != PRIMARY and not got_primary and r['v_abs'] * 3.6 > MOVING:
                        other_useful[band_of(r['dist'])] += 1

    print(f'{len(segs)} 段、{frames} 個雷達訊框（十個槽一起算）')
    print("")
    print('=== 每一道門檻剩下多少 slot-frame（相對「原始有東西」的百分比）===')
    hdr = f"{'距離':>10}{'類型':>8}" + "".join(f"{s:>12}" for s in stages)
    print(hdr)
    for lo, hi in BANDS:
        b = f'{lo}-{hi} m'
        for kind in ('moving', 'static'):
            c = seen[(b, kind)]
            if not c['raw']:
                continue
            cells = "".join(f"{c[s]:>7}{c[s] / c['raw'] * 100:>5.0f}%" for s in stages)
            print(f"{b:>10}{kind:>8}" + cells)

    print("")
    print('=== 各槽位（moving + static 合計）===')
    print(f"{'slot':>8}" + "".join(f"{s:>12}" for s in stages))
    for a in ADDRS:
        c = per_slot[a]
        if not c['raw']:
            continue
        cells = "".join(f"{c[s]:>7}{c[s] / c['raw'] * 100:>5.0f}%" for s in stages)
        print(f"{f'0x{a:x}':>8}" + cells)

    print("")
    print('=== 被 CUSTIN_PRIMARY_ONLY 擋掉的：非 primary 槽穩住了一個在動的目標，')
    print('    而同一刻 primary 槽沒有東西可給 radard ===')
    total = sum(other_useful.values())
    for b, n in sorted(other_useful.items()):
        print(f"  {b:>10}  {n:>7} slot-frame")
    print(f"  {'合計':>10}  {total:>7} slot-frame（{total / max(frames, 1) * 100:.1f}% 的雷達訊框）")


if __name__ == '__main__':
    main()
