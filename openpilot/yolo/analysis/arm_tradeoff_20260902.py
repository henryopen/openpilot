"""Cost of arming earlier: how many false alarms each candidate trigger buys.

The stops say the model knows 40-80 m out and the 45 m distance gate is what hides it. That
only matters if arming on the earlier signal does not also fire all over open road. So sweep
the whole eight hours - every frame with no lead - and count how often each rule fires and
whether a real stop followed within twenty seconds.

A firing that leads to a stop is a hit. One that does not is a false alarm, and its cost is
how long it held the speed down before letting go.
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

MIN_SPEED = 15 / 3.6


def planned_stop(x, v):
    idx = next((i for i, s in enumerate(v) if s < 1.0), None)
    if idx is None:
        return 0.
    if any(v[i] > 1.0 for i in range(idx, len(v))):
        return 0.
    return float(x[idx])


RULES = {
    '現行 x_end<45 & v_end<2': lambda f: f['x_end'] < 45 and f['v_end'] < 2.0,
    'v_end<2（拿掉距離限制）': lambda f: f['v_end'] < 2.0,
    'v_floor<2': lambda f: f['v_floor'] < 2.0,
    'v_floor<2 且 x_end<80': lambda f: f['v_floor'] < 2.0 and f['x_end'] < 80,
    'v_floor<2 且 停點>0': lambda f: f['v_floor'] < 2.0 and f['stop_ahead'] > 0,
    'v_floor < 半速': lambda f: f['v_floor'] < f['v_ego'] * 0.5,
}


def read(seg):
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        return []
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    out = []
    cur = {'v': 0., 'lead': False, 'eng': False}
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
        elif w == 'radarState':
            cur['lead'] = bool(e.radarState.leadOne.present)
        elif w == 'modelV2':
            md = e.modelV2
            x, v = list(md.position.x), list(md.velocity.x)
            if not x or not v:
                continue
            out.append({'v_ego': cur['v'], 'lead': cur['lead'], 'eng': cur['eng'],
                        'x_end': x[-1], 'v_end': v[-1], 'v_floor': min(v),
                        'stop_ahead': planned_stop(x, v)})
    return out


def main():
    routes = sorted({os.path.basename(d).rsplit('--', 1)[0]
                     for d in glob.glob(os.path.join(ROOT, '*--*'))})
    frames = []
    for route in routes:
        segs = sorted(glob.glob(os.path.join(ROOT, route + '--*')),
                      key=lambda p: int(p.rsplit('--', 1)[1]))
        for seg in segs:
            frames.extend(read(seg))

    hours = len(frames) * 0.05 / 3600
    # a real stop: speed falls below 0.5 with no lead in the run-up
    stop_at = []
    rolling = False
    for i, f in enumerate(frames):
        if f['v_ego'] > 5.:
            rolling = True
        elif f['v_ego'] < 0.5 and rolling:
            rolling = False
            if not any(w['lead'] for w in frames[max(0, i - 100):i]):
                stop_at.append(i)
    stop_set = sorted(stop_at)

    print(f"總幀 {len(frames)}（{hours:.2f} 小時）   無前車的真實停止 {len(stop_set)} 次\n")
    print(f"{'觸發規則':28} {'觸發次數':>8} {'命中':>5} {'誤報':>5} {'誤報/小時':>9} {'誤報總秒數':>10}")

    results = {}
    for name, rule in RULES.items():
        firing = False
        fires = hit = miss = 0
        false_frames = 0
        start = 0
        for i, f in enumerate(frames):
            # the module only looks when there is no lead and we are above the floor
            ok = (not f['lead']) and f['v_ego'] > MIN_SPEED and rule(f)
            if ok and not firing:
                firing = True
                start = i
                fires += 1
            elif not ok and firing:
                firing = False
                j = bisect_stop(stop_set, start, i + 400)
                if j:
                    hit += 1
                else:
                    miss += 1
                    false_frames += i - start
        results[name] = {'fires': fires, 'hit': hit, 'miss': miss,
                         'false_per_hour': round(miss / hours, 1),
                         'false_seconds': round(false_frames * 0.05, 1)}
        print(f"{name:28} {fires:8} {hit:5} {miss:5} {miss / hours:9.1f} {false_frames * 0.05:10.1f}")

    json.dump(results, open(os.path.join(ROOT, 'arm_tradeoff.json'), 'w'),
              ensure_ascii=False, indent=1)
    print(f"\n-> {ROOT}/arm_tradeoff.json")


def bisect_stop(stops, lo, hi):
    """Is there a real stop between lo and hi frames?"""
    for s in stops:
        if lo <= s <= hi:
            return True
        if s > hi:
            return False
    return False


if __name__ == '__main__':
    main()
