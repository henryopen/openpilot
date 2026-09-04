"""When vision sees a car ahead, does the radar have it - and are we handing it over?

radar_audit counts what each gate throws away. This asks the question that decides whether
any of it matters: at the moments vision reports a lead, would radard have found a radar
track to match it, if it were given more than the stock ACC's own slot?

Three gates stand between the radar and a matched lead, and they are counted separately:

  prob      radard only looks at the radar at all when the filtered lead probability is
            over 0.5. Below that the lead is dropped outright, radar or no radar.
  primary   only 0x238 is handed over (CUSTIN_PRIMARY_ONLY), and in town that slot is
            whatever the stock ACC has latched, which is often roadside furniture.
  match     match_vision_to_track's own sanity test, reproduced exactly.

    python openpilot/yolo/analysis/radar_vs_vision.py --root F:/c4sunny/rlog20260903
"""
import argparse
import glob
import os
from collections import Counter, deque

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
ADDRS = tuple(range(0x238, 0x238 + 3 * 10, 3))
PRIMARY = ADDRS[0]
MIN_RANGE, PRIMARY_MIN_RANGE, MIN_SCORE = 2.0, 0.5, 30
MAX_ABS_Y, MAX_GAP, MAX_JUMP, MIN_HITS = 5.5, 0.2, 3.0, 8
RADAR_TO_CAMERA = 1.52
DT_MDL = 0.05
PROB_RC = 0.2                      # radard's asymmetric lead-prob filter
BANDS = [(0, 25), (25, 50), (50, 90)]

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True)
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


def matches(d_rel, y_rel, v_rel, v_ego, lx, ly, lv):
    """match_vision_to_track's sanity test, the shipped one."""
    offset = lx - RADAR_TO_CAMERA
    same = abs(v_rel + v_ego - lv) < 1.5 and abs(y_rel + ly) < 2.0
    dist_sane = abs(d_rel - offset) < max(offset * .07, 2.0) or same
    vel_sane = abs(v_rel + v_ego - lv) < 10 or (v_ego + v_rel > 3)
    return dist_sane and vel_sane


def band_of(d):
    for lo, hi in BANDS:
        if lo <= d < hi:
            return f'{lo}-{hi} m'
    return None


def main():
    segs = sorted((s for s in glob.glob(os.path.join(args.root, '*--*'))
                   if os.path.exists(os.path.join(s, 'rlog.zst'))),
                  key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))
    if not segs:
        raise SystemExit(f'{args.root} 底下沒有段')

    c = Counter()
    for seg in segs:
        data = zstandard.ZstdDecompressor().stream_reader(
            open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
        hist = {a: deque(maxlen=12) for a in ADDRS}
        hits = dict.fromkeys(ADDRS, 0)
        held: dict = {}
        v_ego, t, prob_f = 0., 0., 0.
        raw: dict = {}
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
                t += 1 / 33.
                got = False
                for m in e.can:
                    if m.src == 1 and m.address in hist:
                        raw[m.address] = decode(bytes(m.dat))
                        got = True
                if not got:
                    continue
                held = {}
                for a in ADDRS:
                    r = raw.get(a)
                    if r is None or r['score'] == 0 or r['dist'] == 0.:
                        continue
                    primary = a == PRIMARY
                    floor = PRIMARY_MIN_RANGE if primary else MIN_RANGE
                    ok = (r['dist'] > floor and r['score'] >= MIN_SCORE
                          and (primary or abs(-r['lat']) < MAX_ABS_Y))
                    if not ok:
                        hist[a].clear()
                        hits[a] = 0
                        continue
                    if hist[a] and (t - hist[a][-1][0] > MAX_GAP
                                    or abs(r['dist'] - hist[a][-1][1]) > MAX_JUMP):
                        hist[a].clear()
                        hits[a] = 0
                    hist[a].append((t, r['dist']))
                    hits[a] = min(hits[a] + 1, MIN_HITS)
                    if hits[a] >= MIN_HITS:
                        held[a] = (r['dist'], -r['lat'], r['v_abs'] - v_ego)
            elif w == 'modelV2':
                leads = e.modelV2.leadsV3
                if len(leads) < 1:
                    continue
                ld = leads[0]
                p = float(ld.prob)
                prob_f = p if p > prob_f else prob_f + (DT_MDL / (PROB_RC + DT_MDL)) * (p - prob_f)
                lx, ly, lv = float(ld.x[0]), float(ld.y[0]), float(ld.v[0])
                b = band_of(lx - RADAR_TO_CAMERA)
                if b is None or v_ego < 3.:
                    continue
                c[(b, 'vision 看到')] += 1
                if prob_f <= 0.5:
                    c[(b, '被 prob<=0.5 丟掉')] += 1
                    continue
                c[(b, 'prob 過關')] += 1
                pm = PRIMARY in held and matches(*held[PRIMARY], v_ego, lx, ly, lv)
                am = any(matches(*h, v_ego, lx, ly, lv) for h in held.values())
                if pm:
                    c[(b, '現行：primary 配上')] += 1
                elif am:
                    c[(b, '放行全部才配得上')] += 1
                else:
                    c[(b, '哪個槽都配不上')] += 1

    print(f'{len(segs)} 段')
    print("")
    print('（只看 vision 有 lead、車速 > 3 m/s 的幀）')
    rows = ['vision 看到', '被 prob<=0.5 丟掉', 'prob 過關',
            '現行：primary 配上', '放行全部才配得上', '哪個槽都配不上']
    print(f"{'':>22}" + "".join(f"{f'{lo}-{hi} m':>14}" for lo, hi in BANDS))
    for r in rows:
        cells = []
        for lo, hi in BANDS:
            b = f'{lo}-{hi} m'
            n = c[(b, r)]
            base = c[(b, 'vision 看到')] or 1
            cells.append(f"{n:>8}{n / base * 100:>5.0f}%")
        print(f"{r:>22}" + "".join(cells))


if __name__ == '__main__':
    main()
