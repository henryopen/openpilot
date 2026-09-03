"""Replay the CAN through the real RadarInterface and count slot swaps that kept their id.

0x238 carries whichever target the stock ACC has chosen. When it changes car the range moves
several metres in a frame while REL_SPEED does not, and until now the track id carried on -
so radard's filter, the preferred-track match and the MPC's obstacle all treated it as one
car that teleported.

The parser already spotted these (it throws away the range history on a jump); the change
makes it issue a new id as well. This counts, over the same drives, how often the range moves
in a way the reported speed cannot explain *while the id stays the same*. That number should
be near zero afterwards - not because the swaps stopped, but because they are now announced.

Runs on the car: RadarInterface needs opendbc and the car's DBC.

    PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \
        openpilot/yolo/analysis/verify_slot_swap.py --route 00000010--17de10767f --segments 33-34
"""
import argparse
import os
import sys
from collections import Counter

import zstandard

sys.path.insert(0, '/data/openpilot')

from openpilot.cereal import log, messaging  # noqa: F401
from opendbc.car.hyundai.radar_interface import RadarInterface
from opendbc.car.hyundai.values import HyundaiFlags

ap = argparse.ArgumentParser()
ap.add_argument('--route', required=True)
ap.add_argument('--segments', required=True)
ap.add_argument('--root', default='/data/media/0/realdata')
ap.add_argument('--baseline', action='store_true',
                help='suppress the new id on a swap, to measure what it was like before')
args = ap.parse_args()

if args.baseline:
    # what the parser did before: notice the swap, reset the range history, keep the id
    from opendbc.car.hyundai.radar_interface import CustinSlot
    _real_update = CustinSlot.update
    CustinSlot.update = lambda self, t, d, v: bool(_real_update(self, t, d, v)) and False

DT = 1 / 33.
MISMATCH = 2.0        # metres the reported speed cannot account for


def car_params(seg):
    """CarParams is logged once per segment; RadarInterface needs it for the DBC and flags."""
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
    for e in log.Event.read_multiple_bytes(data):
        try:
            if e.which() == 'carParams':
                return e.carParams
        except Exception:
            continue
    return None


def run(seg, CP):
    """Feed the segment's CAN to a fresh RadarInterface; return its points per frame."""
    ri = RadarInterface(CP)
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
    out = []
    for e in log.Event.read_multiple_bytes(data):
        try:
            w = e.which()
        except Exception:
            continue
        if w != 'can':
            continue
        strings = [(e.logMonoTime, [(c.address, c.dat, c.src) for c in e.can])]
        try:
            rr = ri.update(strings)
        except Exception:
            continue
        if rr is None:
            continue
        out.append([(int(p.trackId), float(p.dRel), float(p.vRel)) for p in rr.points])
    return out


def main():
    lo, hi = (int(v) for v in args.segments.split('-'))
    g = Counter()
    worst = []
    for n in range(lo, hi + 1):
        seg = os.path.join(args.root, f'{args.route}--{n}')
        if not os.path.exists(os.path.join(seg, 'rlog.zst')):
            continue
        CP = car_params(seg)
        if CP is None or not (CP.flags & HyundaiFlags.CUSTIN_RADAR):
            print(f'段 {n}: 沒有 CarParams 或不是 Custin 雷達，跳過')
            continue
        frames = run(seg, CP)
        prev = {}
        for pts in frames:
            for tid, d, vrel in pts:
                if tid in prev:
                    pd, pv = prev[tid]
                    err = (d - pd) - (pv + vrel) / 2 * DT
                    g['pairs'] += 1
                    if abs(err) > MISMATCH:
                        g['unexplained'] += 1
                        worst.append((os.path.basename(seg), tid, round(pd, 1), round(d, 1),
                                      round(err, 1), round(pv * 3.6, 1)))
                prev[tid] = (d, vrel)
            live = {t for t, _, _ in pts}
            for t in list(prev):
                if t not in live:
                    del prev[t]
        g['frames'] += len(frames)

    print(f'{"改前（換人不換 id）" if args.baseline else "改後（換人發新 id）"}：'
          f'重放 {g["frames"]} 個雷達幀，同 trackId 的相鄰讀數 {g["pairs"]} 對')
    print(f'距離變化超出相對速可解釋範圍（>{MISMATCH} m）：{g["unexplained"]} 次')
    if worst:
        print(f'\n{"路段":>22} {"trkId":>9} {"從m":>6} {"到m":>6} {"誤差m":>7} {"vRel kph":>9}')
        for row in sorted(worst, key=lambda r: -abs(r[4]))[:15]:
            print(f'{row[0][-22:]:>22} {row[1]:>9} {row[2]:6.1f} {row[3]:6.1f} {row[4]:7.1f} {row[5]:9.1f}')


if __name__ == '__main__':
    main()
