"""Read the radar's own CAN, before anything of ours touches it.

radarTracksSP is already four filters deep - a minimum range, a score threshold, a lateral
gate and eight consecutive good frames - so "nothing there" in that message does not mean
the radar said nothing. This decodes 0x238-0x253 off bus 1 straight out of the log and
prints what each of the ten slots actually reported.

The decode is checked against the log's own radarTracksSP on the primary slot before any
of it is believed: if the hand decode and the car's decode disagree there, nothing below
means anything.

    python openpilot/yolo/analysis/radar_raw.py --route 00000015--d6a173598d --segment 4 \
        --root F:/c4sunny/rlog20260903 --from 16:27:40 --to 16:27:56
"""
import argparse
import datetime
import os

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
TZ = datetime.timezone(datetime.timedelta(hours=8))

ADDRS = tuple(range(0x238, 0x238 + 3 * 10, 3))
RADAR_BUS = 1

ap = argparse.ArgumentParser()
ap.add_argument('--route', required=True)
ap.add_argument('--segment', type=int, required=True)
ap.add_argument('--root', required=True)
ap.add_argument('--from', dest='t_from', default='00:00:00')
ap.add_argument('--to', dest='t_to', default='23:59:59')
ap.add_argument('--every', type=float, default=1.0, help='seconds between printed rows')
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def be_bits(data, start_bit, length):
    """Motorola / big-endian signal extraction, the numbering DBC files use."""
    val = 0
    bit = start_bit
    for _ in range(length):
        val = (val << 1) | ((data[bit // 8] >> (bit % 8)) & 1)
        bit = bit + 15 if bit % 8 == 0 else bit - 1
    return val


def signed(val, length):
    return val - (1 << length) if val >= (1 << (length - 1)) else val


def decode(data):
    """The five fields of custin_radar.dbc, by hand."""
    if len(data) < 8:
        return None
    lat = signed(be_bits(data, 21, 10), 10)
    v_abs = signed(be_bits(data, 37, 11), 11)
    return {
        'flag': be_bits(data, 7, 2),
        'dist': be_bits(data, 5, 10) * 0.1,
        'lat': lat * 0.0475,
        'v_abs': v_abs * 0.02526 + 2.587,
        'score': be_bits(data, 41, 6),
    }


def main():
    seg = os.path.join(args.root, f'{args.route}--{args.segment}', 'rlog.zst')
    data = zstandard.ZstdDecompressor().stream_reader(open(seg, 'rb')).read()

    t0 = None
    n_model = 0
    v_ego = 0.
    raw = {a: None for a in ADDRS}
    logged = []          # what the car's own decode published, for the check
    rows = []
    last_print = -1e9

    for e in log.Event.read_multiple_bytes(data):
        try:
            w = e.which()
        except Exception:
            continue
        if w == 'clocks' and t0 is None:
            t0 = datetime.datetime.fromtimestamp(e.clocks.wallTimeNanos / 1e9, TZ)
        elif w == 'carState':
            v_ego = float(e.carState.vEgo)
        elif w == 'can':
            for c in e.can:
                if c.src == RADAR_BUS and c.address in raw:
                    raw[c.address] = decode(bytes(c.dat))
        elif w == 'radarTracksSP':
            logged.append([(float(p.dRel), float(p.yRel)) for p in e.radarTracksSP.points])
        elif w == 'modelV2':
            if t0 is None:
                continue
            t = t0 + datetime.timedelta(seconds=n_model * 0.05)
            n_model += 1
            ss = t.strftime('%H:%M:%S')
            if not (args.t_from <= ss <= args.t_to):
                continue
            if n_model * 0.05 - last_print < args.every:
                continue
            last_print = n_model * 0.05
            rows.append((ss, v_ego * 3.6, {a: (dict(r) if r else None) for a, r in raw.items()},
                         logged[-1] if logged else []))

    if not rows:
        raise SystemExit('那個時間窗沒有資料')

    # sanity: the primary slot's raw range against what the car published, on the same rows
    checks = [(r[2][0x238]['dist'], d) for r in rows if r[2][0x238]
              for d, _ in r[3] if abs(r[2][0x238]['dist'] - d) < 3.0]
    print(f"解碼自檢：{len(checks)} 個列的 0x238 原始距離在已發布的點裡找得到對應（差 < 3 m）")

    print("")
    hdr = f"{'時刻':>9}{'kph':>6}  " + "".join(f"{f'0x{a:x}':>26}" for a in ADDRS[:5])
    print(hdr)
    print(f"{'':>9}{'':>6}  " + "".join(f"{'dist / lat / v / score':>26}" for _ in ADDRS[:5]))
    for ss, v, slots, _pub in rows:
        cells = []
        for a in ADDRS[:5]:
            r = slots[a]
            if r is None:
                cells.append(f"{'—':>26}")
            else:
                cells.append(f"{r['dist']:6.1f} {r['lat']:6.2f} {r['v_abs']:6.2f} {r['score']:3d}  ")
        print(f"{ss:>9}{v:6.1f}  " + "".join(cells))

    print("")
    print('=== 正前方（|lat| < 1.5 m、2 < dist < 15 m）的原始回波，含被我們濾掉的 ===')
    print(f"{'時刻':>9} {'slot':>7} {'dist':>7} {'lat':>7} {'v_abs':>7} {'score':>6}  {'我們會不會採用':>14}")
    for ss, v, slots, _pub in rows:
        for a in ADDRS:
            r = slots[a]
            if r and abs(r['lat']) < 1.5 and 2. < r['dist'] < 15.:
                why = 'score<30' if r['score'] < 30 else ('不是 0x238，radard 收不到'
                                                          if a != 0x238 else '會')
                print(f"{ss:>9} {f'0x{a:x}':>7} {r['dist']:7.1f} {r['lat']:7.2f} "
                      f"{r['v_abs']:7.2f} {r['score']:6d}  {why:>14}")


if __name__ == '__main__':
    main()
