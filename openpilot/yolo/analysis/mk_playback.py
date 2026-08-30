#!/usr/bin/env python3
"""Rebuild what the HUD showed over a stretch of a recorded route, as dash.html's playback.json.

Runs the recorded messages through hud/server.py's own assembly so the page is fed what it
was fed on the day, rather than a second implementation of it. Open the page with ?play.

  ./mk_playback.py <route> <seg> /tmp/playback.json
"""
import datetime
import json
import sys

from openpilot.hud import server as S
from openpilot.hud.radar_tracker import TargetTracker
from openpilot.tools.lib.logreader import LogReader

KPH = 3.6
LANE_W = 3.5

route, seg, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
targets = TargetTracker()
latest = {}
wall0 = wallmono = None
frames = []


def kph(x):
  return round((x or 0.0) * KPH)


for m in LogReader(f"/data/media/0/realdata/{route}--{seg}/rlog.zst"):
  w = m.which()
  if w == 'clocks':
    wt = m.clocks.wallTimeNanos / 1e9
    if wt > 1.7e9 and wall0 is None:
      wall0, wallmono = wt, m.logMonoTime / 1e9
  if w in ('carState', 'selfdriveState', 'radarState', 'radarTracksSP', 'carControl',
           'longitudinalPlan', 'controlsState'):
    latest[w] = getattr(m, w)
  if w != 'modelV2':
    continue
  if not all(k in latest for k in ('carState', 'selfdriveState', 'radarState', 'carControl', 'longitudinalPlan')):
    continue
  cs, ss, rs, cc, lp = (latest['carState'], latest['selfdriveState'], latest['radarState'],
                        latest['carControl'], latest['longitudinalPlan'])
  pts = [{"dRel": float(p.dRel), "yRel": float(p.yRel), "vRel": float(p.vRel)}
         for p in latest['radarTracksSP'].points] if 'radarTracksSP' in latest else []
  ct = S._control(cc, lp, latest.get('controlsState', cs))
  others = S._radar_targets(pts, targets, float(cs.vEgo), 0.05)
  leads = [S._lead(rs.leadOne), S._lead(rs.leadTwo)]
  followed = leads[0]["dRel"] if leads[0]["status"] else None
  gate = max(3.0, 0.2 * followed) if followed is not None else 3.0
  leads += [t for t in others if followed is None or abs(t["dRel"] - followed) > gate]

  v = float(cs.vEgoCluster or cs.vEgo or 0.)
  F = {"vEgo": kph(v), "vCruise": kph(float(cs.vCruise) / KPH if 0 < float(cs.vCruise) < 250 else 0),
       "engaged": bool(ss.enabled or ss.active), "avail": bool(cs.cruiseState.available),
       "road": "-", "src": ct["src"],
       "slRaw": 0, "slOfs": 0, "slNext": 0, "slNextD": 0,
       "bsdL": bool(cs.leftBlindspot), "bsdR": bool(cs.rightBlindspot),
       "blkL": False, "blkR": False,
       "blinkL": bool(cs.leftBlinker), "blinkR": bool(cs.rightBlinker),
       "aT": ct["aTarget"], "tq": ct["torque"], "sat": ct["sat"],
       "lDes": ct["curvature"] * v * v, "lAct": float(cs.yawRate) * v,
       "alert1": str(ss.alertText1), "alert2": str(ss.alertText2), "alertStatus": str(ss.alertStatus),
       "stop": ct["stop"], "stopD": round(ct["stopDistance"], 1), "leads": []}
  for i, L in enumerate(leads):
    if not L.get("status"):
      continue
    if i == 1:
      continue                             # leadTwo is another estimate of leadOne, not a second car
    y = -(L.get("yRel") or 0.0)
    if not L.get("oncoming") and L.get("laneN") is not None:
      wgt = min(.85, max(.35, .85 - ((L.get("dRel") or 0) - 8) * (0.5 / 32)))
      y = y * (1 - wgt) + (-L["laneN"] * LANE_W) * wgt
    F["leads"].append({"d": round(L["dRel"], 1), "y": round(y, 2), "v": kph(L["vRel"]),
                       "radar": bool(L.get("radar")), "coast": bool(L.get("coast")),
                       "onc": bool(L.get("oncoming")), "lane": L.get("lane", ""), "main": i == 0,
                       "id": f"r{L['id']}" if L.get("id") is not None else ("lead" if i == 0 else None)})
  md = S._model(m.modelV2)
  F["lanes"], F["edges"], F["path"] = md["lanes"], md["edges"], md["path"]
  F["stopA"] = md["stopAhead"]
  if wall0 is not None:
    tw = wall0 + (m.logMonoTime / 1e9 - wallmono) + 8 * 3600     # the device keeps UTC
    F["clk"] = datetime.datetime.fromtimestamp(tw, datetime.UTC).strftime("%H:%M:%S")
  frames.append(F)

json.dump({"d": S.SAMPLE_X, "label": f"replay {route}--{seg}", "f": frames}, open(out, "w"))
print(f"{len(frames)} frames -> {out}  {frames[0].get('clk')} .. {frames[-1].get('clk')}")
