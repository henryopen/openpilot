"""Would holding on to the radar track have stopped the flipping?

get_lead decides fresh every frame: match the vision lead to a radar track, or fall back to
vision's own estimate. Over 2026-09-02 it changed its mind 3704 times in 63 minutes, and the
two answers disagree about closing speed by up to 45 km/h, which the MPC brakes for.

If the track that comes back after a dropout is the one that left, then the track never went
anywhere and the gate simply blinked - holding the previous track through a short gap fixes
it without inventing data. If a different track comes back, holding would be wrong and the
fix has to be elsewhere. radarTrackId is published, so this is answerable from the log.
"""
import glob
import json
import os
from collections import Counter

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
ROOT = r'F:/c4sunny/rlog20260902F'
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])

HOLD_FRAMES = 10          # half a second, the gap we would consider bridging


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    eng = False
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
        if w == 'selfdriveState':
            eng = bool(e.selfdriveState.enabled)
        elif w == 'radarState':
            ld = e.radarState.leadOne
            out.append({
                'eng': eng,
                'present': bool(ld.present),
                'radar': bool(ld.radar) if ld.present else False,
                'tid': int(ld.radarTrackId) if ld.present else -1,
                'd': float(ld.dRel) if ld.present else 0.,
                'vrel': float(ld.vRel) if ld.present else 0.,
            })
    return out


def main():
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(ROOT, '*--*'))})
    g = Counter()
    gaps = []

    for route in routes:
        for seg in sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                          key=lambda p: int(p.rsplit('--', 1)[1])):
            rows = read(seg)
            i = 0
            while i < len(rows):
                r = rows[i]
                if not (r['eng'] and r['present'] and r['radar']):
                    i += 1
                    continue
                # a radar-led stretch; find where it drops to vision
                tid = r['tid']
                j = i + 1
                while j < len(rows) and rows[j]['eng'] and rows[j]['present'] and rows[j]['radar'] \
                        and rows[j]['tid'] == tid:
                    j += 1
                if j >= len(rows) or not rows[j]['eng']:
                    i = j + 1
                    continue
                # how long until radar comes back, and is it the same track?
                k = j
                while k < len(rows) and k - j < 60 and not (rows[k]['present'] and rows[k]['radar']):
                    k += 1
                g['dropouts'] += 1
                if k < len(rows) and rows[k]['present'] and rows[k]['radar']:
                    gap = k - j
                    same = rows[k]['tid'] == tid
                    g['same_track' if same else 'other_track'] += 1
                    if gap <= HOLD_FRAMES:
                        g['short_gap'] += 1
                        if same:
                            g['short_gap_same'] += 1
                    gaps.append({'gap_frames': gap, 'same': same,
                                 'd_before': round(rows[j - 1]['d'], 1),
                                 'd_after': round(rows[k]['d'], 1),
                                 'vrel_vision': round(rows[j]['vrel'] * 3.6, 1),
                                 'vrel_radar_before': round(rows[j - 1]['vrel'] * 3.6, 1)})
                else:
                    g['never_returned'] += 1
                i = max(k, i + 1)

    print(f"雷達段落中斷 {g['dropouts']} 次")
    print(f"  雷達回來時是同一個 track：{g['same_track']} 次"
          f"（{g['same_track'] / max(g['same_track'] + g['other_track'], 1) * 100:.1f}%）")
    print(f"  換成別的 track：{g['other_track']} 次")
    print(f"  一直沒回來（3 秒內）：{g['never_returned']} 次")
    print(f"\n中斷在 {HOLD_FRAMES} 幀（{HOLD_FRAMES * 0.05:.1f} 秒）以內的：{g['short_gap']} 次，"
          f"其中同一個 track {g['short_gap_same']} 次")

    if gaps:
        lens = sorted(x['gap_frames'] for x in gaps)
        print(f"\n中斷長度：中位 {lens[len(lens) // 2] * 0.05:.2f} 秒、"
              f"90 百分位 {lens[int(len(lens) * 0.9)] * 0.05:.2f} 秒、最長 {lens[-1] * 0.05:.2f} 秒")
        short = [x for x in gaps if x['gap_frames'] <= HOLD_FRAMES and x['same']]
        print(f"\n短中斷且同 track 的 {len(short)} 次，掉到視覺時的相對速 vs 雷達原本的：")
        diffs = sorted(abs(x['vrel_vision'] - x['vrel_radar_before']) for x in short)
        if diffs:
            print(f"  差異中位 {diffs[len(diffs) // 2]:.1f} km/h、"
                  f"90 百分位 {diffs[int(len(diffs) * 0.9)]:.1f} km/h、最大 {diffs[-1]:.1f} km/h")
        dd = sorted(abs(x['d_after'] - x['d_before']) for x in short)
        if dd:
            print(f"  中斷前後雷達距離差 中位 {dd[len(dd) // 2]:.1f} m（小=真的是同一台車）")

    json.dump({'summary': dict(g), 'gaps': gaps[:500]},
              open(os.path.join(ROOT, 'lead_stickiness.json'), 'w'), ensure_ascii=False, indent=1)
    print(f"\n-> {ROOT}/lead_stickiness.json")


if __name__ == '__main__':
    main()
