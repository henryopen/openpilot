"""Run the real get_lead over the real messages, with and without the preference, and count.

The change is small but it sits in the loop that decides what to brake for, so it gets the
same treatment as the planner changes: the actual function, fed actual capnp readers, over
the drives that showed the fault. Not a reimplementation.

Every radar track is published (radarTracksSP), so the tracks dict radard builds can be
rebuilt here and the same Track objects updated frame by frame. What is compared is how often
the lead's source changes and how far its closing speed jumps when it does - the thing the
MPC differentiates its plan from.

Run it on the car:

    PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \
        openpilot/yolo/analysis/verify_lead_hold.py --on-device --route 00000010--17de10767f \
        --segments 8-21

radard pulls in messaging and the car interface, so a laptop cannot import it; the car can.
Reading logs off the laptop still works for everything that does not need radard - see the
other scripts here - but this one is verification of code that will drive, so it belongs on
the machine that will run it.
"""
import argparse
import glob
import os
import sys
import types
from collections import Counter

import zstandard

DEVICE_ROOT = '/data/media/0/realdata'
DEVICE_OPENPILOT = '/data/openpilot'
LAPTOP_ROOT = r'F:/c4sunny/rlog20260902F'
LAPTOP_OPENPILOT = r'E:/Documents/GitHub/openpilot-master'
LAPTOP_SCHEMA = r'F:/c4sunny/schema_hcop'
DT_MDL = 0.05

ap = argparse.ArgumentParser()
ap.add_argument('--on-device', action='store_true', help='run against the car\'s own tree and logs')
ap.add_argument('--route', default='', help='limit to one route, e.g. 00000010--17de10767f')
ap.add_argument('--segments', default='', help='limit to a segment range, e.g. 8-21')
ap.add_argument('--root', default='', help='override where the segments live')
args = ap.parse_args()

# radard imports the params store; get_lead does not need it.
_p = types.ModuleType('openpilot.common.params')


class _Params:
    def __init__(self, *a, **k):
        pass

    def get_bool(self, k):
        return False

    def get(self, k, *a, **kw):
        return None


_p.Params = _Params
sys.modules['openpilot.common.params'] = _p
sys.path.insert(0, DEVICE_OPENPILOT if args.on_device else LAPTOP_OPENPILOT)

from openpilot.selfdrive.controls.radard import Track, KalmanParams, get_lead

if args.on_device:
    # the schema openpilot itself loads - no second copy to get out of date
    from openpilot.cereal import log
else:
    import capnp
    capnp.remove_import_hook()
    os.chdir(LAPTOP_SCHEMA)
    log = capnp.load('log.capnp', imports=[LAPTOP_SCHEMA])

ROOT = args.root or (DEVICE_ROOT if args.on_device else LAPTOP_ROOT)


def run(seg, use_preferred):
    """Replay one segment through the real get_lead; return per-frame lead dicts."""
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    kp = KalmanParams(DT_MDL)
    tracks: dict[int, Track] = {}
    prev_tid = -1
    v_ego = 0.
    eng = False
    out = []
    leads_v3 = None
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
            v_ego = e.carState.vEgo
        elif w == 'selfdriveState':
            eng = bool(e.selfdriveState.enabled)
        elif w == 'modelV2':
            lv = e.modelV2.leadsV3
            leads_v3 = lv if len(lv) > 1 else None
        elif w == 'radarTracksSP':
            ids = set()
            for pt in e.radarTracksSP.points:
                i = int(pt.trackId)
                ids.add(i)
                v_lead = float(pt.vRel) + v_ego
                if i not in tracks:
                    tracks[i] = Track(i, v_lead, kp)
                tracks[i].update(float(pt.dRel), float(pt.yRel), float(pt.vRel), v_lead)
            for i in list(tracks):
                if i not in ids:
                    del tracks[i]
            if leads_v3 is None:
                continue
            ld = get_lead(v_ego, True, tracks, leads_v3[0], v_ego, float(leads_v3[0].prob),
                          low_speed_override=True,
                          preferred_track_id=prev_tid if use_preferred else -1)
            prev_tid = int(ld.get('radarTrackId', -1))
            out.append({'eng': eng, 'present': ld.get('present', False),
                        'radar': ld.get('radar', False), 'vrel': ld.get('vRel', 0.),
                        'd': ld.get('dRel', 0.)})
    return out


def score(frames):
    g = Counter()
    jumps = []
    prev = None
    for f in frames:
        if not (f['eng'] and f['present']):
            prev = None
            continue
        g['lead_frames'] += 1
        g['radar' if f['radar'] else 'vision'] += 1
        if prev is not None and f['radar'] != prev['radar']:
            g['flips'] += 1
            jumps.append(abs(f['vrel'] - prev['vrel']) * 3.6)
        prev = f
    jumps.sort()
    return g, jumps


def segments():
    if args.route and args.segments:
        lo, hi = (int(v) for v in args.segments.split('-'))
        want = [os.path.join(ROOT, f'{args.route}--{n}') for n in range(lo, hi + 1)]
        return [s for s in want if os.path.exists(os.path.join(s, 'rlog.zst'))]
    pattern = f'{args.route}--*' if args.route else '*--*'
    segs = []
    for route in sorted({os.path.basename(d).rsplit('--', 1)[0]
                         for d in glob.glob(os.path.join(ROOT, pattern))}):
        segs += sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                       key=lambda p: int(p.rsplit('--', 1)[1]))
    return [s for s in segs if os.path.exists(os.path.join(s, 'rlog.zst'))]


def main():
    segs = segments()
    print(f'重放 {len(segs)} 段（{"車機" if args.on_device else "本機"}）')
    print()

    results = {}
    labels = ('改前（每幀重新決定）', '改後（失配時對原 track 放寬再驗）')
    for label, use_pref in zip(labels, (False, True), strict=True):
        allf = []
        for seg in segs:
            try:
                allf += run(seg, use_pref)
            except Exception as exc:
                print(f'  {os.path.basename(seg)} 失敗: {exc}')
        g, jumps = score(allf)
        results[label] = (g, jumps)
        lf = max(g['lead_frames'], 1)
        print(label)
        print(f'  有前車的幀 {g["lead_frames"]}   雷達 {g["radar"] / lf * 100:.1f}%   視覺 {g["vision"] / lf * 100:.1f}%')
        print(f'  來源切換 {g["flips"]} 次')
        if jumps:
            print(f'  切換時相對速跳幅：中位 {jumps[len(jumps) // 2]:.1f} km/h   最大 {jumps[-1]:.1f} km/h')
        print()

    (a, _), (b, _) = results[labels[0]], results[labels[1]]
    if a['flips']:
        print(f'切換次數 {a["flips"]} → {b["flips"]}（少了 {(a["flips"] - b["flips"]) / a["flips"] * 100:.0f}%）')
    lf_a, lf_b = max(a['lead_frames'], 1), max(b['lead_frames'], 1)
    print(f'雷達佔比 {a["radar"] / lf_a * 100:.1f}% → {b["radar"] / lf_b * 100:.1f}%')


if __name__ == '__main__':
    main()
