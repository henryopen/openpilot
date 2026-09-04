"""Calibrate the radar's fields against something outside the radar.

Every field we claim in custin_radar.dbc was fitted to our own data, so a field can look
right and be a fit - the V_ABS boundary is already suspect, because taking one bit more at
the top turns a +26 m/s target into a -25 m/s one.

The model's lead is an independent reference: radard itself compares it to the radar with
`abs(v_rel + v_ego - lead.v) < 1.5`, so lead.v is a speed over the ground, and lead.x a
range. So: whenever vision is sure about a lead, find the slot sitting at that range and
brute-force every field in its three messages against the lead's own speed and offset.

A field that lands on lead.v across thousands of frames and a range of speeds is the real
one, and the fit gives its scale and offset directly.

    python openpilot/yolo/analysis/radar_calibrate.py --root F:/c4sunny/rlog20260902F
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
RADAR_TO_CAMERA = 1.52
MATCH_RANGE = 2.0          # metres between the slot's range and the lead's
MIN_PROB = 0.9             # only frames where vision is sure

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True)
ap.add_argument('--segs', type=int, default=25)
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def field(w, p0, ln, signed):
    v = ((w >> np.uint64(64 - p0 - ln)) & np.uint64((1 << ln) - 1)).astype(np.int64)
    return np.where(v >= (1 << (ln - 1)), v - (1 << ln), v) if signed else v


def scan(w, tgt, label, top=4):
    best = []
    for ln in (8, 9, 10, 11, 12, 13, 14, 16):
        for p0 in range(65 - ln):
            for signed in (False, True):
                v = field(w, p0, ln, signed)
                if v.std() < 1e-9:
                    continue
                r = float(np.corrcoef(v, tgt)[0, 1])
                if np.isfinite(r):
                    best.append((abs(r), r, p0, ln, signed, v))
    best.sort(key=lambda x: -x[0])
    print(f'  --- {label} ---')
    shown, seen = 0, []
    for _a, r, p0, ln, signed, v in best:
        if any(abs(p0 - q) < 2 for q in seen):
            continue
        seen.append(p0)
        sc, off = np.polyfit(v, tgt, 1)
        res = float(np.median(np.abs(sc * v + off - tgt)))
        sg = '有號' if signed else '無號'
        head = f'    r={r:+.4f}  bit {p0:>2}+{ln:<2} {sg}'
        print(head + f'  scale {sc:+.6f} offset {off:+.4f}  中位殘差 {res:.3f}')
        shown += 1
        if shown == top:
            break


def main():
    segs = sorted((s for s in glob.glob(os.path.join(args.root, '*--*'))
                   if os.path.exists(os.path.join(s, 'rlog.zst'))),
                  key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))
    segs = segs[::max(1, len(segs) // args.segs)][:args.segs]

    rows = []
    for seg in segs:
        data = zstandard.ZstdDecompressor().stream_reader(
            open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
        cur, v_ego = {}, 0.
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
                for m in e.can:
                    if m.src == 1 and 0x238 <= m.address <= 0x25e:
                        cur[m.address] = int.from_bytes(bytes(m.dat), 'big')
            elif w == 'modelV2':
                leads = e.modelV2.leadsV3
                if len(leads) < 1 or any(a not in cur for a in NEEDED):
                    continue
                ld = leads[0]
                if float(ld.prob) < MIN_PROB or v_ego < 1.:
                    continue
                lx, ly, lv = float(ld.x[0]) - RADAR_TO_CAMERA, float(ld.y[0]), float(ld.v[0])
                hit = [b for b in BASE
                       if abs(((cur[b] >> 48) & 0x3FF) * 0.1 - lx) < MATCH_RANGE
                       and ((cur[b] >> 48) & 0x3FF) > 0]
                if len(hit) != 1:          # ambiguous, do not guess
                    continue
                b = hit[0]
                rows.append((cur[b], cur[b + 1], cur[b + 2], lv, ly, v_ego, lx))

    print(f'{len(segs)} 段、{len(rows)} 個「視覺很確定的前車 + 唯一一個距離對得上的 slot」樣本')
    wa = np.array([r[0] for r in rows], dtype=np.uint64)
    wb = np.array([r[1] for r in rows], dtype=np.uint64)
    wc = np.array([r[2] for r in rows], dtype=np.uint64)
    lv = np.array([r[3] for r in rows])
    ly = np.array([r[4] for r in rows])
    ego = np.array([r[5] for r in rows])
    lx = np.array([r[6] for r in rows])
    span = f'  前車對地速 {lv.min():.1f} ~ {lv.max():.1f} m/s，本車 {ego.min():.1f} ~ {ego.max():.1f}'
    print(span + f'，橫向 {ly.min():.1f} ~ {ly.max():.1f} m')

    for name, w in (('A 第一則', wa), ('B 第二則', wb), ('C 第三則', wc)):
        print("")
        print(f'=== {name} ===')
        scan(w, lv, '對前車「對地速度」lead.v')
        scan(w, lv - ego, '對「相對速度」lead.v - v_ego', top=2)
        scan(w, ly, '對前車橫向位置 lead.y', top=3)
        scan(w, np.arctan2(ly, lx), '對前車方位角 atan2(y, x)', top=2)


if __name__ == '__main__':
    main()
