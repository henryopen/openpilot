"""Is the radar's lateral field a distance or an angle - and are we reading the right bits?

custin_radar.dbc claims LAT_DIST in metres at bit 21|10; FIELD_MAP.md, reversed a week
earlier from the same car, claims AZIMUTH in degrees at bit 18|11. They cannot both be
right, and calibrating against the vision lead cannot tell them apart because the lead is
always straight ahead.

Geometry can. Drive past a standing object and its lateral offset stays put while its
azimuth opens up. So for every field in the message, over tracks that sweep in from far
to near, score

    F = variance between tracks / variance within a track

under both readings: the value as metres, and the value times range as metres. The real
field is the one that stays still on one object and separates different objects - a high
F. A field that is constant, a counter, or noise cannot score.

    python openpilot/yolo/analysis/radar_lateral.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import glob
import os

import capnp
import numpy as np
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
BASE = [0x238 + 3 * k for k in range(10)]
MAX_GAP, MAX_JUMP = 0.2, 1.5
MIN_LEN, MIN_SWEEP = 25, 12.0      # frames, and metres of range change

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True)
ap.add_argument('--segs', type=int, default=20)
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def field(w, p0, ln, signed):
    v = ((w >> np.uint64(64 - p0 - ln)) & np.uint64((1 << ln) - 1)).astype(np.float64)
    return np.where(v >= (1 << (ln - 1)), v - (1 << ln), v) if signed else v


def main():
    segs = sorted((s for s in glob.glob(os.path.join(args.root, '*--*'))
                   if os.path.exists(os.path.join(s, 'rlog.zst'))),
                  key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))
    segs = segs[::max(1, len(segs) // args.segs)][:args.segs]

    words, dists, tid = [], [], []
    n = 0
    for seg in segs:
        data = zstandard.ZstdDecompressor().stream_reader(
            open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
        cur = {}
        live = {b: [] for b in BASE}
        pump = iter(log.Event.read_multiple_bytes(data))
        while True:
            try:
                e = next(pump)
            except StopIteration:
                break
            except Exception:
                break
            try:
                if e.which() != 'can':
                    continue
            except Exception:
                continue
            t = e.logMonoTime / 1e9
            hit = False
            for m in e.can:
                if m.src == 1 and 0x238 <= m.address <= 0x255:
                    cur[m.address] = int.from_bytes(bytes(m.dat), 'big')
                    if m.address == 0x255:
                        hit = True
            if not hit or any(b not in cur for b in BASE):
                continue
            for b in BASE:
                w = cur[b]
                d = ((w >> 48) & 0x3FF) * 0.1
                run = live[b]
                if d == 0. or (run and (t - run[-1][0] > MAX_GAP
                                        or abs(d - run[-1][2]) > MAX_JUMP)):
                    n = keep(run, words, dists, tid, n)
                    run.clear()
                if d > 0.:
                    run.append((t, w, d))
        for b in BASE:
            n = keep(live[b], words, dists, tid, n)

    w = np.array(words, dtype=np.uint64)
    d = np.array(dists)
    g = np.array(tid)
    print(f'{len(segs)} 段、{n} 條「掃過來」的 track、{len(w)} 個樣本'
          .replace('、', '、'))

    def fratio(y):
        means = np.array([y[g == i].mean() for i in range(n)])
        within = np.array([y[g == i].var() for i in range(n)])
        return float(means.var() / (within.mean() + 1e-12))

    for name, w_ in (('A 第一則', w),):
        for ln in (9, 10, 11, 12):
            best = []
            for p0 in range(65 - ln):
                for signed in (False, True):
                    v = field(w_, p0, ln, signed)
                    if v.std() < 1e-9:
                        continue
                    best.append((fratio(v), 'metres', p0, ln, signed))
                    best.append((fratio(v * d), 'angle', p0, ln, signed))
            best.sort(key=lambda x: -x[0])
            print("")
            print(f'{name}，長度 {ln} bit —— F 越大代表「同一個物體上不動、不同物體之間分得開」')
            for f_, how, p0, ln_, signed in best[:4]:
                sg = '有號' if signed else '無號'
                lab = '直接當公尺' if how == 'metres' else '當角度 × 距離'
                print(f'  F={f_:>8.1f}  bit {p0:>2}+{ln_:<2} {sg}  {lab}')


def keep(run, words, dists, tid, n):
    if len(run) < MIN_LEN:
        return n
    d = [r[2] for r in run]
    if max(d) - min(d) < MIN_SWEEP:
        return n
    for _t, w, dd in run:
        words.append(w)
        dists.append(dd)
        tid.append(n)
    return n + 1


if __name__ == '__main__':
    main()
