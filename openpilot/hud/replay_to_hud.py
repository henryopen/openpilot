"""Replay a segment's messages so the HUD server behaves as if the car were driving.

server.py decides it is onroad from `sm.alive["carState"]` and, offroad, replaces the whole
map block with just the road name. So nothing about the speed limit display can be checked
while the car is parked - which is how it went unverified long enough for the driver to
report never seeing a limit on the road.

This publishes a segment's own messages back onto msgq at their recorded rate, which makes
carState alive and lets the server build the real thing. Pair it with a GPS feed so mapd has
something to look up:

    # one terminal
    PYTHONPATH=/data/openpilot /usr/local/venv/bin/python openpilot/hud/replay_to_hud.py \
        --route 00000010--17de10767f --segment 33
    # and read the stream
    curl -s -N http://127.0.0.1:8902/stream | head -c 2000

Only the services the HUD subscribes to are published, and nothing is sent to the car: this
writes to msgq, which is where the daemons would have put it. Do not run it while driving.
"""
import argparse
import os
import time

import zstandard

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log

# gpsLocationExternal is here so the server can tell a road with no posted limit from no
# fix at all; it is also what mapd reads, so replaying it puts mapd on the recorded road.
SERVICES = ["carState", "selfdriveState", "radarState", "radarTracksSP", "modelV2",
            "carControl", "longitudinalPlan", "longitudinalPlanSP", "controlsState",
            "gpsLocationExternal"]

ap = argparse.ArgumentParser()
ap.add_argument('--route', required=True)
ap.add_argument('--segment', type=int, required=True)
ap.add_argument('--root', default='/data/media/0/realdata')
ap.add_argument('--loops', type=int, default=3, help='times to replay the segment')
args = ap.parse_args()


def main():
    seg = os.path.join(args.root, f'{args.route}--{args.segment}')
    path = os.path.join(seg, 'rlog.zst')
    if not os.path.exists(path):
        raise SystemExit(f'找不到 {path}')

    print('讀取', path, flush=True)
    data = zstandard.ZstdDecompressor().stream_reader(open(path, 'rb')).read()
    msgs = []
    for e in log.Event.read_multiple_bytes(data):
        try:
            w = e.which()
        except Exception:
            continue
        if w in SERVICES:
            msgs.append((e.logMonoTime, w, e.as_builder().to_bytes()))
    if not msgs:
        raise SystemExit('這段沒有 HUD 需要的訊息')
    print(f'{len(msgs)} 筆訊息，{len({m[1] for m in msgs})} 種服務', flush=True)

    pm = messaging.PubMaster(SERVICES)
    for loop in range(args.loops):
        t0_log = msgs[0][0]
        t0_wall = time.monotonic()
        for i, (mono, service, raw) in enumerate(msgs):
            wait = (mono - t0_log) / 1e9 - (time.monotonic() - t0_wall)
            if wait > 0:
                time.sleep(wait)
            pm.send(service, raw)
            if i % 2000 == 0:
                print(f'  第 {loop + 1} 輪 {i}/{len(msgs)}', flush=True)
    print('重放結束', flush=True)


if __name__ == '__main__':
    main()
