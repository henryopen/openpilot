"""What is in the two radar messages per target we have never decoded.

The census says the radar's address range is not ten messages but ten groups of three:
0x238/0x239/0x23a is one target, 0x23b/0x23c/0x23d the next, and so on. The first two
of every group fall silent together - they are the same object - and the third is always
present. Our DBC only ever decoded the first of each group.

So this brute-forces every Motorola field (start bit, length, signed or not) in all three
messages against quantities we can trust independently:

  dist      the first message's range, verified against video to 0.44 m in FIELD_MAP.md
  vrel      the range differentiated over a sliding window - no field needed to compute it
  vabs      vrel + vEgo, the speed over the ground a real V_ABS field would have to match
  lat       our claimed lateral field
  ego       our own speed, to catch fields that are only echoing the car

A field that matches vabs across thousands of samples is real. One that only matches
inside one drive is a fit.

    python openpilot/yolo/analysis/radar_msg_map.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import glob
import os

import capnp
import numpy as np
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
BASE = [0x238 + 3 * k for k in range(10)]
NEEDED = [b + i for b in BASE for i in (0, 1, 2)]
MAX_GAP, MAX_JUMP = 0.2, 3.0
WIN = 7                    # frames each side for the least-squares range rate

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True)
ap.add_argument('--segs', type=int, default=14)
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def word(b):
    return int.from_bytes(b, 'big')


def field(w, p0, ln, signed):
    """Motorola field whose most significant bit sits p0 bits from the top of the payload."""
    v = (w >> np.uint64(64 - p0 - ln)) & np.uint64((1 << ln) - 1)
    v = v.astype(np.int64)
    if signed:
        v = np.where(v >= (1 << (ln - 1)), v - (1 << ln), v)
    return v


def main():
    segs = sorted((s for s in glob.glob(os.path.join(args.root, '*--*'))
                   if os.path.exists(os.path.join(s, 'rlog.zst'))),
                  key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))
    segs = segs[::max(1, len(segs) // args.segs)][:args.segs]

    rows = []              # (wA, wB, wC, dist, vrel, ego)
    for seg in segs:
        data = zstandard.ZstdDecompressor().stream_reader(
            open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
        cur, v_ego = {}, 0.
        live = {b: [] for b in BASE}          # the run being built on each slot
        pump = iter(log.Event.read_multiple_bytes(data))
        while True:
            try:
                e = next(pump)
            except StopIteration:
                break
            except Exception:
                break                     # some segments are cut off mid-message
            try:
                w = e.which()
            except Exception:
                continue
            if w == 'carState':
                v_ego = float(e.carState.vEgo)
            elif w == 'can':
                t = e.logMonoTime / 1e9
                hit = False
                for m in e.can:
                    if m.src == 1 and 0x238 <= m.address <= 0x25e:
                        cur[m.address] = bytes(m.dat)
                        if m.address == 0x255:
                            hit = True
                if not hit or any(a not in cur for a in NEEDED):
                    continue
                for b in BASE:
                    wa = word(cur[b])
                    dist = ((wa >> 48) & 0x3FF) * 0.1        # bits 2-11 from the top
                    run = live[b]
                    broke = dist == 0. or (run and (t - run[-1][0] > MAX_GAP
                                                    or abs(dist - run[-1][1]) > MAX_JUMP))
                    if broke:
                        flush(run, rows)
                        run.clear()
                    if dist == 0.:
                        continue
                    run.append((t, dist, v_ego, wa, word(cur[b + 1]), word(cur[b + 2])))
        for b in BASE:
            flush(live[b], rows)

    print(f'{len(segs)} 段、{len(rows)} 個樣本（每個都在一條連續 track 內，前後各 {WIN} 幀）')
    arr = np.array(rows, dtype=object)
    wa = np.array([r[0] for r in rows], dtype=np.uint64)
    wb = np.array([r[1] for r in rows], dtype=np.uint64)
    wc = np.array([r[2] for r in rows], dtype=np.uint64)
    dist = np.array([r[3] for r in rows])
    vrel = np.array([r[4] for r in rows])
    ego = np.array([r[5] for r in rows])
    del arr

    targets = {'dist 距離': dist, 'vrel 相對速': vrel, 'vabs 對地速': vrel + ego,
               'ego 本車速': ego}

    for name, w in (('A 第一則（我們在解的）', wa), ('B 第二則（沒解過）', wb),
                    ('C 第三則（沒解過）', wc)):
        print("")
        print(f'=== {name} ===')
        for tname, tgt in targets.items():
            best = []
            for ln in (8, 9, 10, 11, 12, 13, 14, 16):
                for p0 in range(65 - ln):
                    for signed in (False, True):
                        v = field(w, p0, ln, signed)
                        if v.std() < 1e-9:
                            continue
                        r = float(np.corrcoef(v, tgt)[0, 1])
                        if not np.isfinite(r):
                            continue
                        best.append((abs(r), r, p0, ln, signed, v))
            best.sort(key=lambda x: -x[0])
            shown, seen = 0, []
            for _ar, r, p0, ln, signed, v in best:
                if any(abs(p0 - q) < 3 for q in seen):      # same field, one bit over
                    continue
                seen.append(p0)
                a, b_ = np.polyfit(v, tgt, 1)
                res = float(np.median(np.abs(a * v + b_ - tgt)))
                sg = '有號' if signed else '無號'
                head = f'  {tname:>10}  r={r:+.3f}  bit {p0:>2}+{ln:<2} {sg}'
                print(head + f'  scale {a:+.5f} offset {b_:+.3f}  中位殘差 {res:.3f}')
                shown += 1
                if shown == 3:
                    break


def flush(run, rows):
    if len(run) < 2 * WIN + 1:
        return
    t = np.array([r[0] for r in run])
    d = np.array([r[1] for r in run])
    for i in range(WIN, len(run) - WIN):
        sl = slice(i - WIN, i + WIN + 1)
        rate = np.polyfit(t[sl] - t[i], d[sl], 1)[0]
        rows.append((run[i][3], run[i][4], run[i][5], d[i], rate, run[i][2]))


if __name__ == '__main__':
    main()
