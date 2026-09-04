"""Replay the logs through the LeadHold that actually shipped, not a copy of the idea.

lead_hold_sim.py chose the thresholds; this checks the code written from them behaves the
way that choice assumed. radard cannot be imported on this machine (no opendbc), so the
class and its constants are lifted out of radard.py by source and run as they are - if the
file changes, this follows it.

    python openpilot/yolo/analysis/lead_hold_verify.py --root F:/c4sunny/rlog20260902F
"""
import argparse
import ast
import glob
import os

import capnp
import zstandard

SCHEMA = r'F:/c4sunny/schema_hcop'
RADARD = os.path.join(os.path.dirname(__file__), '..', '..', 'selfdrive', 'controls', 'radard.py')
DT_MDL = 0.05
RETURN_WINDOW = 8.0
D_TOL = 6.0
MIN_SPEED = 3.0

ap = argparse.ArgumentParser()
ap.add_argument('--root', required=True, action='append')
args = ap.parse_args()

capnp.remove_import_hook()
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])


def load_lead_hold():
    """-> the LeadHold class and its constants, taken from radard.py as written."""
    src = open(RADARD, encoding='utf-8').read()
    tree = ast.parse(src)
    keep = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id.startswith('LEAD_HOLD_') for t in node.targets):
            keep.append(node)
        elif isinstance(node, ast.ClassDef) and node.name == 'LeadHold':
            keep.append(node)
    if len(keep) < 2:
        raise SystemExit('radard.py 裡找不到 LeadHold 或它的常數')
    ns: dict = {'DT_MDL': DT_MDL, 'Any': object}
    exec(compile(ast.Module(body=keep, type_ignores=[]), RADARD, 'exec'), ns)
    consts = {k: v for k, v in ns.items() if k.startswith('LEAD_HOLD_')}
    return ns['LeadHold'], consts


def frames_of(seg):
    data = zstandard.ZstdDecompressor().stream_reader(
        open(os.path.join(seg, 'rlog.zst'), 'rb')).read()
    out = []
    v_ego, raw = 0., 0.
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
        elif w == 'modelV2':
            lv = e.modelV2.leadsV3
            raw = float(lv[0].prob) if len(lv) else 0.
        elif w == 'radarState':
            ld = e.radarState.leadOne
            out.append((e.logMonoTime / 1e9, v_ego, bool(ld.present), float(ld.dRel),
                        float(ld.vRel), raw))
    return out


def spurious_at(fr, i):
    """Did the lead come back soon, carrying on from where it left off?"""
    t0, d0, v0 = fr[i - 1][0], fr[i - 1][3], fr[i - 1][4]
    for j in range(i, len(fr)):
        dt = fr[j][0] - t0
        if dt > RETURN_WINDOW:
            break
        if fr[j][2]:
            return abs(fr[j][3] - (d0 + v0 * dt)) < D_TOL, dt
    return False, RETURN_WINDOW


def main():
    LeadHold, consts = load_lead_hold()
    print('radard.py 裡目前的設定：' + '  '.join(f'{k}={v}' for k, v in sorted(consts.items())))

    segs = []
    for root in args.root:
        segs += sorted(s for s in glob.glob(os.path.join(root, '*--*'))
                       if os.path.exists(os.path.join(s, 'rlog.zst')))

    secs = 0.
    good, bad, over_dist, over_time = 0., 0., 0, 0
    n_good, n_bad = 0, 0
    holds_by_band: dict = {}
    for seg in segs:
        fr = frames_of(seg)
        if len(fr) < 60:
            continue
        secs += fr[-1][0] - fr[0][0]
        hold = LeadHold()
        pending = None                 # (spurious, dRel at the drop) for the dropout we are in
        held = 0.
        for i, (_t, v_ego, present, d_rel, v_rel, raw) in enumerate(fr):
            lead = {'present': present, 'dRel': d_rel, 'vRel': v_rel}
            out = hold.update(lead, raw, v_ego)
            if present:
                if pending is not None:
                    d0 = pending[1]
                    band = '0-25 m' if d0 < 25 else ('25-45 m' if d0 < 45 else '45+ m')
                    holds_by_band.setdefault(band, [0, 0.]) [0] += 1
                    holds_by_band[band][1] += held
                    if d0 >= consts['LEAD_HOLD_MAX_DIST']:
                        over_dist += held > 0
                    pending, held = None, 0.
                continue
            if pending is None and i > 0 and fr[i - 1][2] and fr[i - 1][1] > MIN_SPEED:
                pending = (spurious_at(fr, i)[0], fr[i - 1][3])
            if out['present'] and pending is not None:
                held += DT_MDL
                if held > consts['LEAD_HOLD_MAX_TIME'] + 1e-6:
                    over_time += 1
                if pending[0]:
                    good += DT_MDL
                    n_good += 1
                else:
                    bad += DT_MDL
                    n_bad += 1

    hrs = secs / 3600
    print(f'{len(segs)} 段、{hrs:.2f} 小時')
    print("")
    line = f'保持住的時間：對的 {good / hrs:.1f} 秒/小時，錯的 {bad / hrs:.1f} 秒/小時'
    print(line + f'，比值 {good / max(bad, 1e-9):.2f}')
    print('模擬當時的預測（0–25 m 加 25–45 m）：對 10.3 秒/小時、錯 4.3 秒/小時，比值 2.4')
    print("")
    print('把關有沒有失守')
    print(f'  超過 {consts["LEAD_HOLD_MAX_DIST"]:.0f} m 還在保持：{over_dist} 次（必須是 0）')
    print(f'  超過 {consts["LEAD_HOLD_MAX_TIME"]:.0f} 秒還在保持：{over_time} 幀（必須是 0）')
    print("")
    print(f'{"消失時距離":>12}{"事件":>8}{"平均保持":>10}')
    for band in ('0-25 m', '25-45 m', '45+ m'):
        if band in holds_by_band:
            n, tot = holds_by_band[band]
            print(f'{band:>12}{n:>8}{tot / max(n, 1):>9.2f}s')


if __name__ == '__main__':
    main()
