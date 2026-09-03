"""What the car thought was in front of it at 14:18:30-14:19:05, frame by frame.

The speed fell 35 km/h with no input from the driver and the planner blamed the lead. The
lead's distance moved 15 m in one second, which no car does - so either a different object
became the lead, or one object's range jumped. Those are different faults: the first is the
tracker picking up something beside us, the second is the radar.

So print both leads with their lateral offset and whether radar confirmed them, next to the
model's own view. A lead that appears at a large yRel is in another lane.
"""
import datetime
import os

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
SEG = r'F:/c4sunny/rlog20260902F/00000010--17de10767f--10'
TZ = datetime.timezone(datetime.timedelta(hours=8))
capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def fmt_lead(ld):
    if not ld.present:
        return f"{'-':>7} {'-':>6} {'-':>7} {'-':>4} {'-':>5}"
    radar = 'R' if getattr(ld, 'radar', False) else 'V'
    return (f"{ld.dRel:7.1f} {ld.yRel:6.2f} {ld.vRel * 3.6:7.1f} {radar:>4} "
            f"{ld.modelProb:5.2f}")


def main():
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(SEG, 'rlog.zst'), 'rb')).read()
    pump = iter(log.Event.read_multiple_bytes(data))
    t0 = None
    rows = []
    cur = {'v': 0., 'a': 0., 'src': '?', 'steer': 0.}
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
        if w == 'clocks' and t0 is None:
            t0 = datetime.datetime.fromtimestamp(e.clocks.wallTimeNanos / 1e9, TZ)
        elif w == 'carState':
            cur['v'] = e.carState.vEgo
            cur['steer'] = float(e.carState.steeringAngleDeg)
        elif w == 'longitudinalPlan':
            cur['src'] = str(e.longitudinalPlan.longitudinalPlanSource)
            cur['a'] = float(e.longitudinalPlan.aTarget)
        elif w == 'radarState':
            rs = e.radarState
            rows.append((dict(cur), rs.leadOne, rs.leadTwo))

    print(f"段起 {t0:%H:%M:%S}   共 {len(rows)} 幀\n")
    print(f"{'時刻':>8} {'kph':>6} {'aTgt':>6} {'來源':>6} {'方向盤':>6} | "
          f"{'L1 dRel':>7} {'yRel':>6} {'vRel':>7} {'R/V':>4} {'prob':>5} | "
          f"{'L2 dRel':>7} {'yRel':>6} {'vRel':>7} {'R/V':>4} {'prob':>5}")
    prev_d = None
    for i, (c, l1, l2) in enumerate(rows):
        t = t0 + datetime.timedelta(seconds=i * 0.05)
        if not (datetime.time(14, 18, 28) <= t.time() <= datetime.time(14, 19, 2)):
            continue
        if i % 5:                                    # four times a second
            continue
        mark = ''
        if prev_d is not None and l1.present and abs(l1.dRel - prev_d) > 3.0:
            mark = f'   <<< 跳 {l1.dRel - prev_d:+.1f} m'
        print(f"{t:%H:%M:%S} {c['v'] * 3.6:6.1f} {c['a']:6.2f} {c['src']:>6} "
              f"{c['steer']:6.1f} | {fmt_lead(l1)} | {fmt_lead(l2)}{mark}")
        prev_d = l1.dRel if l1.present else None


if __name__ == '__main__':
    main()
