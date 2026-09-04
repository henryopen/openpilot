"""Would a radar forward-collision system have fired, and how often at nothing?

With openpilot longitudinal the car's own AEB is out of the loop, and openpilot has none:
EventName.aeb exists but nothing raises it. So this replays the drives we already have and
asks what a collision warning would have done, before any of it is written.

The law is the standard one - the deceleration needed to stop in the gap that is left:

    closing = -vRel                      approaching, metres per second
    a_req   = closing^2 / (2 * (dRel - MARGIN))

a_req rises as the room runs out. Following normally it stays low, because braking bleeds
the closing speed off. It only climbs when nobody is slowing down, which is the thing worth
warning about. panda caps what we may ask for at -3.5 m/s^2 (opendbc/safety/modes/hyundai.h),
so anything needing more than that cannot be stopped for - it can only be started earlier.

Three sources are scored against each other, because where the signal is taken from turns
out to matter more than the threshold:

  lead     radarState.leadOne, what the car actually computed - and what a warning written
           inside radard would see. It inherits the lead_prob gate, which drops the lead
           exactly when vision is unsure, which is exactly the case worth warning about.
  slot     the ACC's own radar slot decoded straight off the bus, no lead logic at all
  slot+vis the same, but only where vision agrees there is something at that range

Scored two ways:
  false alarms  a trigger that nothing independent of the radar backs up: within the next
                few seconds the car did not brake hard and the driver did not touch the
                pedal. Judging it by the gap instead would be circular - the gap comes from
                the same slot that fired, and the slot latches guardrails.
  headroom      there is no near-collision anywhere in these logs - every hard brake in them
                is ordinary traffic braking, with the lead 11 to 42 m out and closing at 1
                to 5 m/s. So coverage cannot be measured here. What can is the gap between
                a_req in ordinary driving and the threshold: a threshold well above the
                everyday population is a backstop, one inside it is a nuisance.

    python openpilot/yolo/analysis/fcw_sim.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import glob
import os

import capnp
import numpy as np
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
RADAR_TO_CAMERA = 1.52
MARGIN = 2.5               # metres left over, not a bumper-to-bumper stop
MIN_SPEED = 5.0            # 18 km/h. Below this the car is creeping up to something and the
                           # gap is small enough that a_req explodes on its own - every false
                           # alarm the lead source produced was at 3.2 to 3.9 m/s closing on a
                           # stopped car 4 to 7 m ahead. That speed range belongs to stop and
                           # go, not to a collision system.
MIN_CLOSING = 0.5
HOLD = 3                   # consecutive frames before it counts as a trigger
HARD_BRAKE = -2.5          # what counts as a real event in the logs
LOOKAHEAD = 5.0            # seconds a trigger is allowed to be early by
ONCOMING_MARGIN = 3.0      # closing faster than this above our own speed is oncoming

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, action='append')
ap.add_argument('--dump', type=int, default=10)
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def read(root):
    """-> per segment, a time-ordered table of what the car knew."""
    segs = sorted((s for s in glob.glob(os.path.join(root, '*--*'))
                   if os.path.exists(os.path.join(s, 'rlog.zst'))),
                  key=lambda p: (p.rsplit('--', 2)[0], int(p.rsplit('--', 1)[1])))
    for seg in segs:
        data = zstandard.ZstdDecompressor().stream_reader(
            open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
        rows = []
        v_ego = a_ego = 0.
        brake = gas = False
        vis = (0., 0., 0.)
        vabs_hist: list = []
        prim = (0., 0.)                 # the raw primary slot: range, speed over the ground
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
                c = e.carState
                v_ego, a_ego = float(c.vEgo), float(c.aEgo)
                brake, gas = bool(c.brakePressed), bool(c.gasPressed)
            elif w == 'modelV2':
                lv = e.modelV2.leadsV3
                vis = ((float(lv[0].prob), float(lv[0].x[0]) - RADAR_TO_CAMERA,
                        float(lv[0].v[0])) if len(lv) else (0., 0., 0.))
            elif w == 'can':
                for m in e.can:
                    if m.src == 1 and m.address == 0x238:
                        b = int.from_bytes(bytes(m.dat), 'big')
                        rng = ((b >> 48) & 0x3FF) * 0.1
                        raw = (b >> 19) & 0x7FF
                        va = ((raw - 2048) if raw >= 1024 else raw) * 0.02526 + 2.587
                        score = (b >> 12) & 0x3F
                        if rng > 0.5 and score >= 30:
                            if vabs_hist and abs(rng - prim[0]) > 3.0:
                                vabs_hist = []          # a different object took the slot
                            vabs_hist.append(va)
                            vabs_hist = vabs_hist[-12:]
                            prim = (rng, sorted(vabs_hist)[len(vabs_hist) // 2])
                        else:
                            vabs_hist = []
                            prim = (0., 0.)
            elif w == 'radarState':
                ld = e.radarState.leadOne
                rows.append((e.logMonoTime / 1e9, v_ego, a_ego, brake, gas,
                             bool(ld.present), float(ld.dRel), float(ld.vRel),
                             bool(ld.radar), vis[0], vis[1], prim[0], prim[1] - v_ego))
        if len(rows) > 40:
            yield os.path.basename(seg), rows


def series(rows):
    a = np.array(rows, dtype=object)
    return {'t': np.array([r[0] for r in rows]),
            'v': np.array([r[1] for r in rows]),
            'a': np.array([r[2] for r in rows]),
            'brake': np.array([r[3] for r in rows], dtype=bool),
            'present': np.array([r[5] for r in rows], dtype=bool),
            'd': np.array([r[6] for r in rows]),
            'vrel': np.array([r[7] for r in rows]),
            'radar': np.array([r[8] for r in rows], dtype=bool),
            'vprob': np.array([r[9] for r in rows]),
            'vx': np.array([r[10] for r in rows]),
            'pd': np.array([r[11] for r in rows]),
            'pv': np.array([r[12] for r in rows])} if len(a) else None


def triggers(s, a_warn, source):
    if source == 'lead':
        d, vrel = s['d'], s['vrel']
        live = s['present'] & s['radar']
    else:
        d, vrel = s['pd'], s['pv']
        live = d > 0.5
    closing = -vrel
    gap = np.maximum(d - MARGIN, 0.5)
    a_req = np.where(closing > MIN_CLOSING, closing ** 2 / (2 * gap), 0.)

    live = live & (s['v'] > MIN_SPEED) & (closing > MIN_CLOSING)
    # a target of our own kind cannot approach faster than we drive: closing above our own
    # speed means it is coming the other way. The stock ACC's slot picks oncoming traffic up
    # regularly, and vision often has a lead at a similar range, so both agreeing is not
    # enough on its own - this is what every false alarm at 48-52 m/s closing was.
    live &= closing < s['v'] + ONCOMING_MARGIN
    if source == 'slot+vis':
        live &= (s['vprob'] > 0.5) & (np.abs(s['vx'] - d) < np.maximum(0.25 * d, 4.0))
    fire = live & (a_req > a_warn)
    a_req = np.where(live, a_req, 0.)

    out, run = [], 0
    for i, f in enumerate(fire):
        run = run + 1 if f else 0
        if run == HOLD:
            out.append(i)
    return out, a_req


def real_events(s):
    """Moments the car really did brake hard, whoever caused it."""
    hard = s['a'] < HARD_BRAKE
    t = s['t']
    ev, i = [], 0
    while i < len(hard):
        if hard[i]:
            j = i
            while j + 1 < len(hard) and (hard[j + 1] or t[j + 1] - t[j] < 0.5):
                j += 1
            ev.append(t[i])
            i = j + 1
        else:
            i += 1
    return ev


def main():
    data = []
    for root in args.root:
        label = os.path.basename(root.rstrip('/\\'))
        for name, rows in read(root):
            s = series(rows)
            if s is not None:
                data.append((label, name, s))
    secs = sum(s['t'][-1] - s['t'][0] for _, _, s in data)
    print(f'{len(data)} 段、{secs / 3600:.2f} 小時')

    head = f'{"訊號源":>10}{"門檻 a_req":>11}{"觸發":>8}{"假警報":>8}'
    print("")
    print(head + f'{"假警報/小時":>13}{"真煞車被涵蓋":>15}{"提前中位":>10}')
    keep = None
    for source in ('lead', 'slot', 'slot+vis'):
        for a_warn in (2.0, 2.5, 3.0, 3.5, 4.0):
            n_trig = n_false = covered = n_events = 0
            leads, examples = [], []
            for label, name, s in data:
                trig, _ = triggers(s, a_warn, source)
                ev = real_events(s)
                n_events += len(ev)
                for i in trig:
                    n_trig += 1
                    win = (s['t'] >= s['t'][i]) & (s['t'] <= s['t'][i] + LOOKAHEAD)
                    dd = s['d'] if source == 'lead' else s['pd']
                    vv = s['vrel'] if source == 'lead' else s['pv']
                    if s['a'][win].min() < HARD_BRAKE or s['brake'][win].any():
                        continue
                    n_false += 1
                    if len(examples) < args.dump:
                        examples.append((label, name, s['t'][i] - s['t'][0], dd[i],
                                         -vv[i], s['v'][i]))
                for et in ev:
                    early = [et - s['t'][i] for i in trig if 0 <= et - s['t'][i] <= LOOKAHEAD]
                    if early:
                        covered += 1
                        leads.append(max(early))
            cov = f'{covered}/{n_events}' if n_events else '—'
            med = f'{np.median(leads):.1f}s' if leads else '—'
            row = f'{source:>10}{a_warn:>11.1f}{n_trig:>8}{n_false:>8}'
            print(row + f'{n_false / (secs / 3600):>13.1f}{cov:>15}{med:>10}')
            if source == 'lead' and abs(a_warn - 2.5) < 1e-9:
                keep = examples

    print("")
    print('平常開車時 a_req 到底有多大（門檻要坐在這個分布的上面）')
    print(f'{"訊號源":>10}{"有效幀":>9}{"中位":>8}{"p90":>8}{"p99":>8}{"p99.9":>8}{"最大":>8}')
    for source in ('lead', 'slot', 'slot+vis'):
        pool = []
        for _l, _n, s2 in data:
            _t, a_req = triggers(s2, 99., source)
            pool.append(a_req[a_req > 0])
        v = np.concatenate(pool)
        pcts = f'{np.median(v):>8.2f}{np.percentile(v, 90):>8.2f}{np.percentile(v, 99):>8.2f}'
        print(f'{source:>10}{len(v):>9}' + pcts + f'{np.percentile(v, 99.9):>8.2f}{v.max():>8.1f}')

    print("")
    print('駕駛/OP 開始煞車的那一刻，a_req 是多少（門檻要坐在這個分布上面才算補漏網）')
    onsets = []
    for _l, _n, s2 in data:
        _t, a_req = triggers(s2, 99., 'lead')
        a = s2['a']
        for i in range(1, len(a)):
            if a[i] < -1.5 <= a[i - 1] and s2['v'][i] > MIN_SPEED and a_req[i] > 0:
                onsets.append(a_req[i])
    if onsets:
        v = np.array(onsets)
        pct = f'{np.median(v):.2f}  p90 {np.percentile(v, 90):.2f}  p99 {np.percentile(v, 99):.2f}'
        print(f'  {len(v)} 次煞車起點：中位 {pct}  最大 {v.max():.2f}')

    print("")
    print('那些真的重煞車的當下，手邊有什麼（看有沒有任何設計救得了）')
    hdr = f'{"段":>28}{"秒":>7}{"本車":>7}{"最劇":>7}{"雷達主槽":>12}'
    print(hdr + f'{"接近":>7}{"視覺":>7}{"radard lead":>13}')
    shown = 0
    for _label, name, s2 in data:
        for et in real_events(s2):
            i = int(np.argmin(np.abs(s2['t'] - et)))
            j = max(0, i - 20)                       # a second before it started
            win = (s2['t'] >= et) & (s2['t'] <= et + 2.0)
            pd = f"{s2['pd'][j]:.1f} m" if s2['pd'][j] > 0.5 else '無'
            lead = f"{s2['d'][j]:.1f} m" if s2['present'][j] else '無'
            left = f'{name[-16:]:>28}{et - s2["t"][0]:>7.1f}{s2["v"][j]:>7.1f}'
            mid = f'{s2["a"][win].min():>7.1f}{pd:>12}{-s2["pv"][j]:>7.1f}'
            print(left + mid + f'{s2["vprob"][j]:>7.2f}{lead:>13}')
            shown += 1
            if shown >= 20:
                break
        if shown >= 20:
            break

    if keep:
        print("")
        print('lead 訊號源、門檻 2.5 之下的假警報（可以調影片看）')
        for label, name, off, d, closing, v in keep:
            where = f'  {label} {name}  段內第 {off:6.1f} 秒'
            print(where + f'  距離 {d:5.1f} m  接近 {closing:5.1f} m/s  本車 {v:5.1f} m/s')


if __name__ == '__main__':
    main()
