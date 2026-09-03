"""Run the real get_lead over the real messages, with and without the preference, and count.

The change is small but it sits in the loop that decides what to brake for, so it gets the
same treatment as the planner changes: the actual function, fed actual capnp readers from
the car's own schema, over the drives that showed the fault. Not a reimplementation.

Every radar track is published (radarTracksSP), so the tracks dict radard builds can be
rebuilt here and the same Track objects updated frame by frame. What is compared is how
often the lead's source changes and how far its closing speed jumps when it does - the
thing the MPC differentiates its plan from.
"""
import glob
import os
import sys
import types
from collections import Counter

import capnp
import zstandard

# radard imports the car interface and the params store; neither is needed for get_lead.
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
sys.path.insert(0, r'E:/Documents/GitHub/openpilot-master')

from openpilot.selfdrive.controls.radard import Track, KalmanParams, get_lead  # noqa: E402

SCHEMA = r'F:/c4sunny/schema_hcop'
ROOT = r'F:/c4sunny/rlog20260902F'
DT_MDL = 0.05
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


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
            pts = e.radarTracksSP.points
            ids = set()
            for pt in pts:
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


def main():
    segs = []
    for route in sorted({os.path.basename(d).rsplit('--', 1)[0]
                         for d in glob.glob(os.path.join(ROOT, '*--*'))}):
        segs += sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                       key=lambda p: int(p.rsplit('--', 1)[1]))
    print(f"重放 {len(segs)} 段，hold = {LEAD_HOLD_FRAMES} 幀（{LEAD_HOLD_FRAMES * 0.05:.1f} 秒）\n")

    results = {}
    for label, use_pref in (('改前（每幀重新決定）', False), ('改後（失配時對原 track 放寬再驗）', True)):
        allf = []
        for seg in segs:
            try:
                allf += run(seg, use_pref)
            except Exception as exc:
                print(f'  {os.path.basename(seg)} 失敗: {exc}')
        g, jumps = score(allf)
        results[label] = (g, jumps)
        lf = max(g['lead_frames'], 1)
        print(f"{label}")
        print(f"  有前車的幀 {g['lead_frames']}   雷達 {g['radar'] / lf * 100:.1f}%   "
              f"視覺 {g['vision'] / lf * 100:.1f}%")
        print(f"  來源切換 {g['flips']} 次")
        if jumps:
            print(f"  切換時相對速跳幅：中位 {jumps[len(jumps) // 2]:.1f} km/h   "
                  f"最大 {jumps[-1]:.1f} km/h")
        print()

    (a, _), (b, _) = results['改前（每幀重新決定）'], results['改後（失配時對原 track 放寬再驗）']
    if a['flips']:
        print(f"切換次數 {a['flips']} → {b['flips']}"
              f"（少了 {(a['flips'] - b['flips']) / a['flips'] * 100:.0f}%）")
    lf_a, lf_b = max(a['lead_frames'], 1), max(b['lead_frames'], 1)
    print(f"雷達佔比 {a['radar'] / lf_a * 100:.1f}% → {b['radar'] / lf_b * 100:.1f}%")


if __name__ == '__main__':
    main()
