"""Does the car ever see a stopped vehicle, and does it see motorcycles?

radard is handed one radar track: 0x238, the target the stock ACC has locked. Everything else
is decoded but withheld, because a guardrail scanned along its length reads as a car holding
station and radard picks it over the real one. That decision was measured and stands - but it
means the answer to "can it see a stopped car" is really "does the stock ACC lock stopped
cars", which has never been checked.

allPoints carries every track, so this compares three things frame by frame: what the radar
as a whole reports standing still, what the primary target is doing, and whether radard
ended up with a lead at all. A target is standing still when its speed over the ground is
near zero - vRel + v_ego - while we are moving.

Motorcycles cannot be told from the radar alone; what can be measured is how the lead behaves
where the model says there is one and the radar disagrees, which is where a small return
would show up.
"""
import argparse
import glob
import os
from collections import Counter

import zstandard

DEVICE_ROOT = '/data/media/0/realdata'
LAPTOP_ROOT = r'F:/c4sunny/rlog20260902F'
LAPTOP_SCHEMA = r'F:/c4sunny/schema_hcop'

ap = argparse.ArgumentParser()
ap.add_argument('--on-device', action='store_true')
ap.add_argument('--root', default='')
args = ap.parse_args()

if args.on_device:
    import sys
    sys.path.insert(0, '/data/openpilot')
    from openpilot.cereal import log
else:
    import capnp
    capnp.remove_import_hook()
    os.chdir(LAPTOP_SCHEMA)
    log = capnp.load('log.capnp', imports=[LAPTOP_SCHEMA])

ROOT = args.root or (DEVICE_ROOT if args.on_device else LAPTOP_ROOT)

MOVING = 4.0          # only ask about stopped targets while we are actually driving
STILL = 2.5           # speed over the ground under this is standing still
LANE_Y = 1.5          # our own lane
# 0x238's slot is never deleted, so its id holds for a whole segment and it is the only
# track present in every frame - and the only one radard is ever handed. Ids are assigned
# as slots appear, so the number itself means nothing; find it per segment.


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'eng': False, 'lead': False, 'lead_radar': False, 'lead_d': 0.,
           'lead_prob': 0., 'vision_lead': False, 'vision_d': 0.}
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
            cur['v'] = e.carState.vEgo
        elif w == 'selfdriveState':
            cur['eng'] = bool(e.selfdriveState.enabled)
        elif w == 'modelV2':
            lv = e.modelV2.leadsV3
            if len(lv):
                cur['vision_lead'] = float(lv[0].prob) > 0.5
                cur['vision_d'] = float(lv[0].x[0])
                cur['lead_prob'] = float(lv[0].prob)
        elif w == 'radarState':
            ld = e.radarState.leadOne
            cur['lead'] = bool(ld.present)
            cur['lead_radar'] = bool(ld.radar) if ld.present else False
            cur['lead_d'] = float(ld.dRel) if ld.present else 0.
        elif w == 'radarTracksSP':
            pts = [{'id': int(p.trackId), 'd': float(p.dRel), 'y': float(p.yRel),
                    'vrel': float(p.vRel)} for p in e.radarTracksSP.points]
            out.append({**cur, 'pts': pts})
    return out


def main():
    segs = []
    for route in sorted({os.path.basename(d).rsplit('--', 1)[0]
                         for d in glob.glob(os.path.join(ROOT, '*--*'))}):
        segs += sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                       key=lambda p: int(p.rsplit('--', 1)[1]))

    g = Counter()
    stopped_events = []
    for seg in segs:
        run = 0
        frames = read(seg)
        seen = Counter()
        for f in frames:
            for p in f['pts']:
                seen[p['id']] += 1
        primary_id = seen.most_common(1)[0][0] if seen else -1
        for f in frames:
            if f['v'] < MOVING:
                run = 0
                continue
            g['frames'] += 1
            g['tracks_total'] += len(f['pts'])

            primary = next((p for p in f['pts'] if p['id'] == primary_id), None)
            in_lane_still = [p for p in f['pts']
                             if abs(p['vrel'] + f['v']) < STILL and abs(p['y']) < LANE_Y and 5 < p['d'] < 80]
            any_still = [p for p in f['pts'] if abs(p['vrel'] + f['v']) < STILL and 5 < p['d'] < 80]

            if any_still:
                g['frames_any_still'] += 1
            if in_lane_still:
                g['frames_lane_still'] += 1
                if f['vision_lead']:
                    g['lane_still_vision_saw'] += 1
                if f['lead']:
                    g['lane_still_had_lead'] += 1
                if f['lead'] and f['lead_radar']:
                    g['lane_still_lead_from_radar'] += 1

            if primary is not None:
                g['frames_primary'] += 1
                if abs(primary['vrel'] + f['v']) < STILL:
                    g['primary_still'] += 1
                    run += 1
                    if run == 20:      # a second of the stock ACC holding a stopped target
                        stopped_events.append({'seg': os.path.basename(seg),
                                               'kph': round(f['v'] * 3.6, 1),
                                               'd': round(primary['d'], 1),
                                               'y': round(primary['y'], 2),
                                               'vision': f['vision_lead'],
                                               'lead': f['lead'],
                                               'lead_radar': f['lead_radar']})
                else:
                    run = 0
            else:
                run = 0

    fr = max(g['frames'], 1)
    print(f"行進中的幀 {fr}（{fr * 0.05 / 60:.1f} 分鐘）   雷達 track 平均 {g['tracks_total'] / fr:.1f} 個/幀")
    print()
    print(f"雷達看到對地靜止目標的幀：{g['frames_any_still']} ({g['frames_any_still'] / fr * 100:.1f}%)")
    print(f"  其中在本車道（|y|<{LANE_Y}m）：{g['frames_lane_still']} ({g['frames_lane_still'] / fr * 100:.1f}%)")
    ls = max(g['frames_lane_still'], 1)
    print(f"    視覺同時也看到 lead：{g['lane_still_vision_saw']} ({g['lane_still_vision_saw'] / ls * 100:.1f}%)")
    print(f"    radard 真的有 lead：{g['lane_still_had_lead']} ({g['lane_still_had_lead'] / ls * 100:.1f}%)")
    print(f"      其中來自雷達：{g['lane_still_lead_from_radar']} ({g['lane_still_lead_from_radar'] / ls * 100:.1f}%)")
    print()
    pf = max(g['frames_primary'], 1)
    print(f"原廠 ACC 的 primary target 存在的幀：{g['frames_primary']} ({g['frames_primary'] / fr * 100:.1f}%)")
    print(f"  primary 是對地靜止的：{g['primary_still']} ({g['primary_still'] / pf * 100:.1f}%)")
    print(f"  持續一秒以上的靜止 primary 事件：{len(stopped_events)} 次")
    for ev in stopped_events[:15]:
        print(f"    {ev['seg'][-20:]:>20} {ev['kph']:5.1f} kph  {ev['d']:5.1f} m  y={ev['y']:5.2f}  "
              f"視覺={'有' if ev['vision'] else '無'}  radard有lead={'是' if ev['lead'] else '否'}  "
              f"來自雷達={'是' if ev['lead_radar'] else '否'}")


if __name__ == '__main__':
    main()
