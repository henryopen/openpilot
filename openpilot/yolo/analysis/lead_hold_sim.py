"""How long should a lead be held through a flicker, and what does holding cost?

LONGITUDINAL.md 4.1 has one piece left: at long range the lead comes from vision alone, and
its confidence flickers. On 2026-09-03 a car at 62-80 m had its filtered modelProb swinging
between 0.09 and 0.85; each dip under 0.5 dropped the lead, the car was judged to have
nothing in front of it for ten seconds, cruise took it from 26 to 53 km/h, and it only
locked on again at 43 m closing at 20 km/h, which needed -3.0 m/s^2 and the driver.

Measuring the radar first showed the radar cannot fix this: when following breaks, the ACC's
slot has lost the car too (lead_recovery.py - 0% still in the primary slot, 4% in any other).
So the fix has to be on the lead itself: keep the last one through the dip.

The question is how long. A dropout is spurious if the lead comes back soon at a distance
that carries on from where it left off - that car never went anywhere, and holding would
have been right. It is genuine if nothing comes back, or what comes back is somewhere else -
and holding would have meant braking for a car that is not there.

Holding on a timer alone loses: a second of it buys back 30 seconds an hour of dropout and
spends 41 holding a car that had gone. The two kinds do separate though - the ones worth
holding go quiet at 40 m with the raw vision probability still around 0.35, the ones that
really left at 61 m with it down at 0.20 - so the hold is gated on the probability not
having collapsed, which is what carolpilot and StarPilot both do with their relaxed branch,
rather than on time alone.

    python openpilot/yolo/analysis/lead_hold_sim.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import glob
import os

import capnp
import numpy as np
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
MIN_SPEED = 3.0
RETURN_WINDOW = 8.0        # seconds to wait for the lead to come back
D_TOL = 6.0                # metres the returning lead may differ from where it was going
MAX_D = 90.                # beyond this the radar has nothing and vision is guessing anyway
HOLD_FLOORS = (0.10, 0.15, 0.20, 0.25, 0.30, 0.40)

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, action='append')
ap.add_argument('--dump', type=int, default=8)
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def frames_of(seg):
    """-> [(t, v_ego, present, radar, dRel, vRel, modelProb, raw_vision_prob)]"""
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
    out = []
    v_ego, raw_prob = 0., 0.
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
        elif w == 'modelV2':
            lv = e.modelV2.leadsV3
            raw_prob = float(lv[0].prob) if len(lv) else 0.
        elif w == 'radarState':
            ld = e.radarState.leadOne
            out.append((e.logMonoTime / 1e9, v_ego, bool(ld.present), bool(ld.radar),
                        float(ld.dRel), float(ld.vRel), float(ld.modelProb), raw_prob))
    return out


def dropouts(fr):
    """-> [(i_start, gap_seconds, spurious, dRel_before, raw_prob_during)]"""
    out = []
    for i in range(1, len(fr)):
        if not (fr[i - 1][2] and not fr[i][2] and fr[i - 1][1] > MIN_SPEED):
            continue
        d0, v0, t0 = fr[i - 1][4], fr[i - 1][5], fr[i - 1][0]
        if d0 > MAX_D:
            continue
        back, spurious = None, False
        for j in range(i, len(fr)):
            dt = fr[j][0] - t0
            if dt > RETURN_WINDOW:
                break
            if fr[j][2]:
                back = dt
                spurious = abs(fr[j][4] - (d0 + v0 * dt)) < D_TOL
                break
        gap = back if back is not None else RETURN_WINDOW
        during = [f[7] for f in fr[i:] if f[0] - t0 <= (back or RETURN_WINDOW)]
        # how long the raw probability stays above each floor, which is how long a hold
        # gated on it would actually last
        held = {}
        for floor in HOLD_FLOORS:
            k = 0.
            for f in fr[i:]:
                dt = f[0] - t0
                if dt > gap or f[7] < floor:
                    break
                k = dt
            held[floor] = k
        out.append((i, gap, spurious, d0, float(np.median(during)) if during else 0., held))
    return out


def main():
    segs = []
    for root in args.root:
        segs += sorted((s for s in glob.glob(os.path.join(root, '*--*'))
                        if os.path.exists(os.path.join(s, 'rlog.zst'))),
                       key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))

    secs = 0.
    events, examples = [], []
    for seg in segs:
        fr = frames_of(seg)
        if len(fr) < 60:
            continue
        secs += fr[-1][0] - fr[0][0]
        for i, gap, spurious, d0, prob, held in dropouts(fr):
            events.append((gap, spurious, d0, prob, held))
            if spurious and gap > 2.0 and len(examples) < args.dump:
                examples.append((os.path.basename(seg), fr[i][0] - fr[0][0], d0, gap, prob))

    hrs = secs / 3600
    gaps = np.array([e[0] for e in events])
    spur = np.array([e[1] for e in events], dtype=bool)
    d0s = np.array([e[2] for e in events])
    probs = np.array([e[3] for e in events])
    print(f'{len(segs)} 段、{hrs:.2f} 小時')
    print("")
    print(f'前車消失事件：{len(events)} 次（{len(events) / hrs:.0f} 次/小時）')
    keep = f'  其中「很快又回來、而且距離接得上」＝ 本來就不該丟：{spur.sum()} 次'
    print(keep + f'（{spur.mean() * 100:.0f}%）')
    print(f'  真的沒了（沒回來或回來的是別台）：{(~spur).sum()} 次')
    print("")
    print(f'{"":>6}{"斷多久 中位":>14}{"p90":>8}{"消失時距離 中位":>18}{"視覺原始信心 中位":>20}')
    for name, m in (('不該丟', spur), ('真的沒了', ~spur)):
        if m.sum():
            left = f'{name:>6}{np.median(gaps[m]):>14.2f}{np.percentile(gaps[m], 90):>8.2f}'
            print(left + f'{np.median(d0s[m]):>18.1f}{np.median(probs[m]):>20.2f}')

    print("")
    print('只看時間的保持（賠本）')
    hdr = f'{"保持秒數":>10}{"救回秒數/小時":>16}{"錯誤秒數/小時":>16}{"划算嗎":>10}'
    print(hdr)
    for hold in (0.5, 1.0, 2.0, 3.0):
        good = float(np.minimum(gaps[spur], hold).sum())
        bad = float(np.minimum(gaps[~spur], hold).sum())
        ratio = good / bad if bad else 99.
        print(f'{hold:>10.1f}{good / hrs:>16.1f}{bad / hrs:>16.1f}{ratio:>10.2f}')

    print("")
    print('改成「視覺原始信心沒崩掉才保持」，上限 3 秒')
    hdr2 = f'{"信心下限":>10}{"救回秒數/小時":>16}{"錯誤秒數/小時":>16}'
    print(hdr2 + f'{"救回:錯誤":>12}{"救回的斷線數":>14}')
    for floor in HOLD_FLOORS:
        h = np.array([min(e[4][floor], 3.0) for e in events])
        good, bad = float(h[spur].sum()), float(h[~spur].sum())
        ratio = good / bad if bad else 99.
        n = int((h[spur] >= gaps[spur] - 0.05).sum())
        print(f'{floor:>10.2f}{good / hrs:>16.1f}{bad / hrs:>16.1f}{ratio:>12.2f}{n:>14}')

    print("")
    print('按距離拆開（錯誤的代價不對稱：遠處抱著＝不加速，近處抱著＝對空氣煞車）')
    hdr3 = f'{"距離帶":>12}{"信心下限":>10}{"救回秒/時":>12}{"錯誤秒/時":>12}'
    print(hdr3 + f'{"救回:錯誤":>12}{"事件數":>9}')
    for lo, hi, name in ((0, 25, '0-25 m'), (25, 45, '25-45 m'), (45, 90, '45-90 m')):
        band = (d0s >= lo) & (d0s < hi)
        for floor in (0.15, 0.20, 0.25):
            h = np.array([min(e[4][floor], 3.0) for e in events])
            g = float(h[spur & band].sum())
            b = float(h[(~spur) & band].sum())
            ratio = g / b if b else 99.
            left = f'{name:>12}{floor:>10.2f}{g / hrs:>12.1f}{b / hrs:>12.1f}'
            print(left + f'{ratio:>12.2f}{int(band.sum()):>9}')

    if examples:
        print("")
        print('斷超過 2 秒、但其實那台車一直都在（這些就是 9/3 那種）')
        for name, off, d0, gap, prob in examples:
            where = f'  {name[-16:]}  段內第 {off:6.1f} 秒'
            print(where + f'  消失時 {d0:5.1f} m  斷 {gap:4.1f} s  視覺信心中位 {prob:.2f}')


if __name__ == '__main__':
    main()
