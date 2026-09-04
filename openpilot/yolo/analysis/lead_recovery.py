"""Would a tracker across the ten slots actually keep the lead, or is that just tidiness?

The ACC's slot 0x238 is the only real track the radar gives, and it changes car without
saying so - the decode already throws its range history away on a jump and hands radard a
new track id, which costs the Kalman filter, the preferred-track match and the MPC's
obstacle all at once.

The other nine slots are raw detections, so today they are dropped. A tracker would
associate across all ten and could carry the same car through a swap. Whether that is worth
building comes down to one number: when the primary slot loses its object, is that object
sitting in another slot at the time?

Counting every break in the primary slot answers the wrong question: the slot only holds a
real car some of the time, and the rest of the time it flails through clutter, where the
range moves 0.8 m between frames against 0.09 m while genuinely following. Breaks in the
flailing cost nothing because nothing downstream was using it.

So this counts only what following actually loses: stretches where radard held a
radar-matched lead for a second or more, and then lost it while still moving. For each, it
looks for the same object in another slot - close in range once its own speed is allowed
for, close in offset, moving at the same speed - because that is exactly what a tracker
across the ten slots could have carried through.

    python openpilot/yolo/analysis/lead_recovery.py --root F:/c4sunny/rlog20260902F
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
MIN_SCORE = 30
MAX_GAP = 0.2              # seconds without an update before the slot counts as reused
MAX_JUMP = 3.0             # metres of range jump that means a different object
MIN_SPEED = 3.0

LOOK = 0.5                 # seconds to look for the object in another slot
D_TOL = 3.0                # metres of range, on top of what its own speed explains
Y_TOL = 1.5
V_TOL = 2.0

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


def frames_of(seg):
    """-> [(t, v_ego, lead_present, lead_radar, lead_d, {slot: (d, y, v_abs)})]"""
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
    out = []
    cur, v_ego = {}, 0.
    lead = (False, False, 0.)
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
            ld = e.radarState.leadOne
            lead = (bool(ld.present), bool(ld.radar), float(ld.dRel))
        elif w == 'can':
            hit = False
            for m in e.can:
                if m.src == 1 and m.address in (0x253, *BASE):
                    if m.address in BASE:
                        cur[m.address] = int.from_bytes(bytes(m.dat), 'big')
                    if m.address == 0x253:
                        hit = True
            if not hit or any(b not in cur for b in BASE):
                continue
            pts = {}
            for b in BASE:
                d, y, v_abs, score = decode(cur[b])
                if d > 0.5 and score >= MIN_SCORE:
                    pts[b] = (d, y, v_abs)
            out.append((e.logMonoTime / 1e9, v_ego, *lead, pts))
    return out


def main():
    segs = []
    for root in args.root:
        segs += sorted((s for s in glob.glob(os.path.join(root, '*--*'))
                        if os.path.exists(os.path.join(s, 'rlog.zst'))),
                       key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))

    secs = 0.
    follow_time = 0.
    breaks = recovered = radar_still_had = 0
    outages, before_d = [], []
    radar_frames = lead_frames = 0
    for seg in segs:
        fr = frames_of(seg)
        if len(fr) < 60:
            continue
        secs += fr[-1][0] - fr[0][0]
        run_start = None                 # when the current radar-matched lead began
        for i, (t, v_ego, present, radar, _ld_d, _pts) in enumerate(fr):
            if present:
                lead_frames += 1
                if radar:
                    radar_frames += 1
            following = present and radar and v_ego > MIN_SPEED
            if following and run_start is None:
                run_start = (t, i)
            elif not following and run_start is not None:
                held = t - run_start[0]
                if held >= 1.0:
                    follow_time += held
                    # how long until a radar-matched lead comes back
                    back = None
                    for j in range(i, len(fr)):
                        if fr[j][2] and fr[j][3]:
                            back = fr[j][0] - t
                            break
                        if fr[j][0] - t > 5.0:
                            break
                    if back is None or back > 0.2:
                        breaks += 1
                        outages.append(back if back is not None else 5.0)
                        last = fr[i - 1]
                        before_d.append(last[4])
                        # the car we were following. radarState runs at 20 Hz and the radar
                        # at 33, so the frame the lead went out on can already be an empty
                        # slot - walk back for the last time the slot held anything
                        prev_pt, k = None, i - 1
                        while k >= 0 and last[0] - fr[k][0] < 0.5:
                            prev_pt = fr[k][5].get(PRIMARY)
                            if prev_pt is not None:
                                last = fr[k]
                                break
                            k -= 1
                        found = False
                        if prev_pt is not None:
                            for j in range(i, len(fr)):
                                dtj = fr[j][0] - last[0]
                                if dtj > LOOK:
                                    break
                                want = prev_pt[0] + (prev_pt[2] - v_ego) * dtj
                                for b, (d, y, v_abs) in fr[j][5].items():
                                    if b == PRIMARY:
                                        continue
                                    if (abs(d - want) < D_TOL and abs(y - prev_pt[1]) < Y_TOL
                                            and abs(v_abs - prev_pt[2]) < V_TOL):
                                        found = True
                                        break
                                if found:
                                    break
                        recovered += found
                        # was the car still in the primary slot when radard let go? then it
                        # was not the radar that lost it
                        if prev_pt is not None and abs(prev_pt[0] - last[4]) < 3.0:
                            nxt = fr[min(i + 3, len(fr) - 1)][5].get(PRIMARY)
                            if nxt is not None and abs(nxt[0] - last[4]) < 4.0:
                                radar_still_had += 1
                run_start = None

    hrs = secs / 3600
    print(f'{len(segs)} 段、{hrs:.2f} 小時')
    print("")
    pct = radar_frames / max(lead_frames, 1) * 100
    print(f'lead 存在的幀裡有雷達配對的比例：{pct:.1f}% （{radar_frames} / {lead_frames}）')
    print(f'連續跟車（雷達配對的 lead 撐 1 秒以上）總時數：{follow_time / 60:.0f} 分鐘')
    print("")
    print(f'跟車中途斷掉（斷 0.2 秒以上）：{breaks} 次')
    if breaks:
        print(f'  每小時 {breaks / hrs:.1f} 次，平均每 {follow_time / breaks:.0f} 秒跟車就斷一次')
        o = np.array(outages)
        dur = f'  斷多久：中位 {np.median(o):.2f} s，p90 {np.percentile(o, 90):.2f} s'
        print(dur + f'，超過 2 秒的 {np.mean(o > 2) * 100:.0f}%')
        b = np.array(before_d)
        print(f'  斷掉當下前車距離：中位 {np.median(b):.1f} m，p90 {np.percentile(b, 90):.1f} m')
        rec = f'  同一台車還在別的槽裡（追蹤器接得住）：{recovered} 次'
        print(rec + f'（{recovered / breaks * 100:.0f}%）')
        still = f'  斷掉當下雷達主槽其實還看得著那台車：{radar_still_had} 次'
        print(still + f'（{radar_still_had / breaks * 100:.0f}%）← 這些是 radard 丟的，不是雷達丟的')


if __name__ == '__main__':
    main()
