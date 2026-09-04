"""What is actually in the radar's ten slots, and how much of it is a car we could follow.

Every previous look at this asked "which slot" or "which threshold". This one asks what the
returns *are*, because the answer decides everything else: the decode's four gates - range,
score, lateral offset, eight consecutive frames - do not ask whether a return is moving, and
the field that says so, V_ABS, has been sitting in the DBC unused since it was reversed.

Returns are grouped into tracks (a slot holding the same object, by the same continuity rule
the decode uses) and each track is classified by its own speed over the ground:

  standing    |V_ABS| < 1.5 m/s        roadside structure, parked cars, signs
  with us     V_ABS >= +1.5 m/s        traffic going our way - the only kind we can follow
  oncoming    V_ABS <= -1.5 m/s        the other carriageway

Then, for each class, how much of it survives our gates and how much reaches radard.

    python openpilot/yolo/analysis/radar_content.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import glob
import os
import statistics
from collections import Counter, defaultdict

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
ADDRS = tuple(range(0x238, 0x238 + 3 * 10, 3))
PRIMARY = ADDRS[0]
TRIGGER = ADDRS[-1]        # RadarInterface emits one RadarData when this one lands
MIN_RANGE, PRIMARY_MIN_RANGE, MIN_SCORE = 2.0, 0.5, 30
MAX_ABS_Y, MAX_GAP, MAX_JUMP, MIN_HITS = 5.5, 0.2, 3.0, 8
STANDING = 1.5             # m/s over the ground
IN_LANE = 1.8              # metres either side of centre

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True)
ap.add_argument('--min-frames', type=int, default=8, help='a track has to last this long to count')
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def be_bits(data, start_bit, length):
    val, bit = 0, start_bit
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


def kind_of(v_abs):
    if v_abs >= STANDING:
        return '同向移動'
    if v_abs <= -STANDING:
        return '對向'
    return '靜止不動'


def main():
    segs = sorted((s for s in glob.glob(os.path.join(args.root, '*--*'))
                   if os.path.exists(os.path.join(s, 'rlog.zst'))),
                  key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))
    if not segs:
        raise SystemExit(f'{args.root} 底下沒有段')

    tracks = []            # one entry per reconstructed track
    frames = 0
    for seg in segs:
        data = zstandard.ZstdDecompressor().stream_reader(
            open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
        cur = dict.fromkeys(ADDRS)         # the track being built on each slot
        v_ego, t = 0., 0.
        first_t = None
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
                v_ego = float(e.carState.vEgo)
            elif w == 'can':
                # the log's own clock, not a frame counter: the ten radar messages arrive
                # spread over more than one can event, so counting events runs the clock
                # about 1.6x fast and both MAX_GAP and every duration come out wrong
                t = e.logMonoTime / 1e9
                got = False
                for m in e.can:
                    if m.src == 1 and m.address in cur:
                        raw[m.address] = decode(bytes(m.dat))
                        if m.address == TRIGGER:
                            got = True
                if not got:
                    continue      # one cycle per trigger message, the way the car does it
                if first_t is None:
                    first_t = t
                frames += 1
                for a in ADDRS:
                    r = raw.get(a)
                    empty = r is None or r['score'] == 0 or r['dist'] == 0.
                    tr = cur[a]
                    broke = empty or (tr is not None and
                                      (t - tr['last_t'] > MAX_GAP
                                       or abs(r['dist'] - tr['last_d']) > MAX_JUMP))
                    if broke and tr is not None:
                        tracks.append(tr)
                        cur[a] = None
                        tr = None
                    if empty:
                        continue
                    if tr is None:
                        tr = cur[a] = {'slot': a, 'n': 0, 'd': [], 'y': [], 'v': [],
                                       'ego': [], 'passed': 0, 'first_t': t,
                                       'last_t': t, 'last_d': r['dist']}
                    tr['n'] += 1
                    tr['d'].append(r['dist'])
                    tr['y'].append(-r['lat'])
                    tr['v'].append(r['v_abs'])
                    tr['ego'].append(v_ego)
                    tr['last_t'], tr['last_d'] = t, r['dist']
                    # the decode's own gates, minus the eight-frame hold which is the track
                    floor = PRIMARY_MIN_RANGE if a == PRIMARY else MIN_RANGE
                    if (r['dist'] > floor and r['score'] >= MIN_SCORE
                            and (a == PRIMARY or abs(-r['lat']) < MAX_ABS_Y)):
                        tr['passed'] += 1
        for a in ADDRS:
            if cur[a] is not None:
                tracks.append(cur[a])

    tracks = [t for t in tracks if t['n'] >= args.min_frames]
    head = f'{len(segs)} 段、{frames} 個雷達訊框，重建出 {len(tracks)} 條 track'
    print(head + f'（至少 {args.min_frames} 幀 = {args.min_frames / 33:.2f} 秒）')

    by = defaultdict(list)
    for tr in tracks:
        v = statistics.median(tr['v'])
        lane = abs(statistics.median(tr['y'])) < IN_LANE
        by[(kind_of(v), '本車道' if lane else '車道外')].append(tr)

    print("")
    print('=== 雷達到底看到什麼（依 track 分類）===')
    print(f"{'種類':>10}{'位置':>8}{'條數':>8}{'佔比':>7}{'中位長度':>10}{'中位距離':>10}{'中位速度':>10}")
    total = len(tracks)
    for kind in ('同向移動', '靜止不動', '對向'):
        for lane in ('本車道', '車道外'):
            g = by[(kind, lane)]
            if not g:
                continue
            secs = statistics.median(t['last_t'] - t['first_t'] for t in g)
            dist = statistics.median(statistics.median(t['d']) for t in g)
            spd = statistics.median(statistics.median(t['v']) for t in g) * 3.6
            row = f"{kind:>10}{lane:>8}{len(g):>8}{len(g) / total * 100:>6.1f}%"
            print(row + f"{secs:>9.2f}s{dist:>9.1f}m{spd:>9.1f}kph")

    print("")
    print('=== 這些 track 有多少能走到 radard ===')
    print(f"{'種類':>10}{'位置':>8}{'條數':>8}{'過我們四道門檻':>16}{'→ 進 radard (0x238)':>20}")
    for kind in ('同向移動', '靜止不動', '對向'):
        for lane in ('本車道', '車道外'):
            g = by[(kind, lane)]
            if not g:
                continue
            # a track counts as usable if it kept the gates for eight frames in a row's worth
            ok = [t for t in g if t['passed'] >= MIN_HITS]
            prim = [t for t in ok if t['slot'] == PRIMARY]
            row = f"{kind:>10}{lane:>8}{len(g):>8}{len(ok):>10}{len(ok) / len(g) * 100:>5.0f}%"
            print(row + f"{len(prim):>9}{len(prim) / len(g) * 100:>4.0f}%")

    print("")
    print('=== 同向移動、在本車道的 track（唯一能拿來跟車的）===')
    print('  條數會騙人：一條長 track 和一條 0.3 秒的碎片各算一條。所以也看時間。')
    g = by[('同向移動', '本車道')]
    if g:
        cnt, secs = Counter(), Counter()
        lives = defaultdict(list)
        for tr in g:
            d = statistics.median(tr['d'])
            b = '0-25 m' if d < 25 else ('25-50 m' if d < 50 else ('50-90 m' if d < 90 else '>=90 m'))
            who = '0x238' if tr['slot'] == PRIMARY else '其他九槽'
            cnt[(b, who)] += 1
            secs[(b, who)] += tr['last_t'] - tr['first_t']
            lives[who].append(tr['last_t'] - tr['first_t'])
        head = f"{'距離':>10}{'0x238 條':>10}{'其他槽 條':>11}{'0x238 秒':>11}{'其他槽 秒':>11}"
        print(head + f"{'其他槽時間佔比':>16}")
        for b in ('0-25 m', '25-50 m', '50-90 m', '>=90 m'):
            pc, oc = cnt[(b, '0x238')], cnt[(b, '其他九槽')]
            ps, os_ = secs[(b, '0x238')], secs[(b, '其他九槽')]
            if pc + oc == 0:
                continue
            row = f"{b:>10}{pc:>10}{oc:>11}{ps:>10.0f}s{os_:>10.0f}s"
            print(row + f"{os_ / (ps + os_) * 100:>15.0f}%")
        print("")
        for who in ('0x238', '其他九槽'):
            v = sorted(lives[who])
            if not v:
                continue
            p90 = v[int(len(v) * 0.9)]
            line = f"  {who} 的 track 長度：中位 {statistics.median(v):.2f}s  p90 {p90:.2f}s"
            print(line + f"  最長 {max(v):.1f}s  總計 {sum(v):.0f}s")


if __name__ == '__main__':
    main()
