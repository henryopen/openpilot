"""Where vision is wrong about a lead, and whether the radar already knew better.

Vision losing the lead is only half of it. The other half is that when it has the lead it
can be badly wrong about how fast it is going, and it is worst on exactly the target that
matters most: a stopped car changes size slowly, so the model reads speed off almost no
change at all. On 2026-08-30 a stationary white car was called 4.20 then 7.03 m/s while the
radar had it at -1.10 in slot 0x23b, and radard did not take the radar until 35.8 m, by
which point the car had 2.9 seconds less to slow down.

That car was in 0x23b, not the ACC's own slot, so CUSTIN_PRIMARY_ONLY threw it away. But
letting the radar raise a lead by itself is not on - the signal to noise on a stationary
in-lane return is about 1:1 and it would brake at empty road. What is on is the opposite
arrangement: vision says whether there is something there, the radar says how far and how
fast. So this measures how much that would buy.

For every frame vision is sure about a lead, the best-matching radar return is found across
all ten slots, and vision's own range and speed are scored against it. V_ABS is the
reference because it is the calibrated one - 0.33 m/s against the stock ACC's own report.

    python openpilot/yolo/analysis/vision_vs_radar.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import glob
import os

import capnp
import numpy as np
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
BASE = [0x238 + 3 * k for k in range(10)]
PRIMARY = BASE[0]
RADAR_TO_CAMERA = 1.52
MIN_SCORE = 30
MIN_PROB = 0.5
MIN_SPEED = 3.0
LANE = 2.0                 # metres either side of centre a lead may sit
BANDS = ((0, 25), (25, 45), (45, 70), (70, 100))

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, action='append')
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def decode(word):
    rng = ((word >> 48) & 0x3FF) * 0.1
    lat = (word >> 36) & 0x3FF
    lat = lat - 1024 if lat >= 512 else lat
    raw = (word >> 19) & 0x7FF
    v_abs = ((raw - 2048) if raw >= 1024 else raw) * 0.02526 + 2.587
    return rng, -lat * 0.0475, v_abs, (word >> 12) & 0x3F


def rows_of(seg):
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
    out = []
    cur, v_ego = {}, 0.
    lead_radar = False
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
        elif w == 'radarState':
            lead_radar = bool(e.radarState.leadOne.radar) and bool(e.radarState.leadOne.present)
        elif w == 'can':
            for m in e.can:
                if m.src == 1 and m.address in BASE:
                    cur[m.address] = int.from_bytes(bytes(m.dat), 'big')
        elif w == 'modelV2':
            lv = e.modelV2.leadsV3
            if not len(lv) or any(b not in cur for b in BASE):
                continue
            prob = float(lv[0].prob)
            if prob < MIN_PROB or v_ego < MIN_SPEED:
                continue
            vx = float(lv[0].x[0]) - RADAR_TO_CAMERA
            vy, vv = float(lv[0].y[0]), float(lv[0].v[0])
            best = None
            for b in BASE:
                d, y, v_abs, score = decode(cur[b])
                if d < 0.5 or score < MIN_SCORE or abs(y) > LANE:
                    continue
                if abs(d - vx) > max(0.25 * vx, 4.0):
                    continue
                err = abs(d - vx)
                if best is None or err < best[0]:
                    best = (err, b, d, y, v_abs)
            if best is not None:
                out.append((vx, vy, vv, best[2], best[4], best[1] == PRIMARY, v_ego, lead_radar))
    return out


def main():
    rows = []
    for root in args.root:
        for seg in sorted(glob.glob(os.path.join(root, '*--*'))):
            if os.path.exists(os.path.join(seg, 'rlog.zst')):
                rows += rows_of(seg)

    a = np.array(rows, dtype=float)
    vx, vv = a[:, 0], a[:, 2]
    rd, rv = a[:, 3], a[:, 4]
    prim = a[:, 5].astype(bool)
    used_radar = a[:, 7].astype(bool)
    still = np.abs(rv) < 1.5            # the radar says it is not moving

    print(f'{len(a)} 個「視覺很確定有前車、而且雷達某個槽對得上」的幀')
    who = f'  其中對上的是主槽 0x238：{prim.mean() * 100:.0f}%'
    print(who + f'，其他九槽：{(~prim).mean() * 100:.0f}%')
    print(f'  其中 radard 當下真的用了雷達：{used_radar.mean() * 100:.0f}%')

    print("")
    print('視覺的速度錯多少（以雷達 V_ABS 為準，它對原廠 ACC 誤差 0.33 m/s）')
    hdr = f'{"距離":>10}{"目標":>8}{"幀數":>8}{"速度誤差中位":>14}{"p90":>8}'
    print(hdr + f'{"錯 > 2 m/s":>12}{"距離誤差中位":>14}')
    for lo, hi in BANDS:
        band = (vx >= lo) & (vx < hi)
        for name, m in (('靜止', band & still), ('在動', band & ~still)):
            if m.sum() < 30:
                continue
            e = np.abs(vv[m] - rv[m])
            de = np.abs(vx[m] - rd[m])
            left = f'{f"{lo}-{hi} m":>10}{name:>8}{int(m.sum()):>8}{np.median(e):>14.2f}'
            mid = f'{np.percentile(e, 90):>8.2f}{np.mean(e > 2) * 100:>11.0f}%'
            print(left + mid + f'{np.median(de):>14.1f}')

    print("")
    print('修正會不會修錯：拿「視覺本來就準」的移動目標當試紙')
    print(f'{"距離":>10}{"幀數":>8}{"主槽修正與視覺差 > 3":>22}{"其他槽修正與視覺差 > 3":>24}')
    for lo, hi in BANDS:
        band = (vx >= lo) & (vx < hi) & ~still
        if band.sum() < 30:
            continue
        e = np.abs(vv - rv)
        p_bad = float(np.mean(e[band & prim] > 3) * 100) if (band & prim).sum() else 0.
        o_bad = float(np.mean(e[band & ~prim] > 3) * 100) if (band & ~prim).sum() else 0.
        print(f'{f"{lo}-{hi} m":>10}{int(band.sum()):>8}{p_bad:>21.0f}%{o_bad:>23.0f}%')

    print("")
    print('最危險的那一格：雷達說靜止，視覺卻說它在走')
    for lo, hi in BANDS:
        m = (vx >= lo) & (vx < hi) & still
        if m.sum() < 30:
            continue
        fast = vv[m] > 2.0
        prim_here = prim[m]
        used = used_radar[m]
        line = f'  {lo}-{hi} m：{int(m.sum()):>6} 幀，視覺報 > 2 m/s 的 {fast.mean() * 100:>3.0f}%'
        mid = f'（中位報 {np.median(vv[m]):.1f} m/s）　對上的在主槽 {prim_here.mean() * 100:.0f}%'
        print(line + mid + f'，radard 用了雷達 {used.mean() * 100:.0f}%')


if __name__ == '__main__':
    main()
