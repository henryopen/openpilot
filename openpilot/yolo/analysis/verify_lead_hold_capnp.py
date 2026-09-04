"""Run the new longitudinal code against real capnp readers, on the device.

ruff and a local replay cannot see capnp's behaviour: the local schema is older than the
device's, and 2026-09-02 that difference put plannerd into a crash loop for a whole drive
(leadOne.status here, present there). So both new pieces get exercised on the car, against
its own log.capnp and its own rlog:

  LeadHold        every leadOne in the log is turned back into the dict radard builds, run
                  through the hold, and assigned to a real RadarState message - which is
                  where a missing or misspelt field would throw
  the weak-lead   the planner's condition is evaluated over the same frames, reading
  cruise cap      leadsV3 the way the planner reads it

    ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
        PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/verify_lead_hold_capnp.py \
        --route /data/media/0/realdata/<segment>'
"""
import argparse
import glob
import os

from openpilot.cereal import custom, log
from openpilot.selfdrive.controls.lib.longitudinal_planner import (
  WEAK_LEAD_MAX_DIST,
  WEAK_LEAD_MAX_PROB,
  WEAK_LEAD_MIN_DIST,
  WEAK_LEAD_MIN_PROB,
  WEAK_LEAD_MIN_SPEED,
)
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, LeadHold
from openpilot.tools.lib.logreader import LogReader

LEAD_FIELDS = ('dRel', 'yRel', 'vRel', 'vLead', 'vLeadK', 'aLeadK', 'aLeadTau',
               'present', 'modelProb', 'radar', 'radarTrackId')

ap = argparse.ArgumentParser()
ap.add_argument('--route', help='a segment directory; the newest one is used if not given')
args = ap.parse_args()


def newest_segment():
    segs = sorted(glob.glob('/data/media/0/realdata/*--*'),
                  key=lambda p: os.path.getmtime(p))
    for seg in reversed(segs):
        if os.path.exists(os.path.join(seg, 'rlog.zst')) or os.path.exists(os.path.join(seg, 'rlog')):
            return seg
    raise SystemExit('找不到任何有 rlog 的段')


def main():
    seg = args.route or newest_segment()
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        path = os.path.join(seg, 'rlog')
    print(f'段：{seg}')

    # the enum the display reads has to exist in the device's own schema
    reason = custom.LongitudinalPlanSP.Reason.weakLead
    print(f'weakLead 這個 reason 在車機 schema 裡：{reason}')
    print(f'LeadData 的欄位：{log.RadarState.LeadData.schema.fieldnames}')
    missing = [f for f in LEAD_FIELDS if f not in log.RadarState.LeadData.schema.fieldnames]
    if missing:
        raise SystemExit(f'radard 會寫、但車機 schema 沒有的欄位：{missing}')

    hold = LeadHold()
    frames = held_frames = weak_frames = 0
    v_ego, raw_prob, lead_x = 0., 0., 0.
    lead_present = False
    for msg in LogReader(path):
        w = msg.which()
        if w == 'carState':
            v_ego = msg.carState.vEgo
        elif w == 'modelV2':
            lv = msg.modelV2.leadsV3
            if len(lv):
                raw_prob = lv[0].prob
                lead_x = lv[0].x[0] - RADAR_TO_CAMERA
        elif w == 'radarState':
            frames += 1
            ld = msg.radarState.leadOne
            lead_present = bool(ld.present)
            lead = {f: getattr(ld, f) for f in LEAD_FIELDS}
            out = hold.update(lead, raw_prob, v_ego)
            if out['present'] and not lead['present']:
                held_frames += 1
            # the assignment radard does - a bad key or type throws right here
            rs = log.RadarState.new_message()
            rs.leadOne = out

            if not lead_present and v_ego > WEAK_LEAD_MIN_SPEED:
                if (WEAK_LEAD_MIN_PROB <= raw_prob < WEAK_LEAD_MAX_PROB
                        and WEAK_LEAD_MIN_DIST <= lead_x < WEAK_LEAD_MAX_DIST):
                    weak_frames += 1

    print("")
    print(f'{frames} 個 radarState 幀，全部走過 LeadHold 並寫進真的 RadarState，沒有例外')
    print(f'  保持住 lead 的幀：{held_frames}（{held_frames / max(frames, 1) * 100:.1f}%）')
    print(f'  弱前車會擋加速的幀：{weak_frames}（{weak_frames / max(frames, 1) * 100:.1f}%）')


if __name__ == '__main__':
    main()
