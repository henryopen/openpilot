#!/usr/bin/env python3
"""hud: stream selected cereal services as Server-Sent Events for an external HUD display.

Runs on the comma device as a standalone systemd service (sp-hud.service), outside of
openpilot's process manager, so it survives branch switches.

Built for sunnypilot, which published its own selfdriveStateSP / longitudinalPlanSP /
liveMapDataSP messages. This branch has none of those, so the same shapes are assembled
here from what master does have, and the page needs no changes:

  road name, speed limit  <- mapd's memory params
  speed limit assist      <- SpeedLimitMode and the set speed
  mads                    <- AlwaysOnLateral and carControl.latActive
  dec                     <- experimentalMode, which picks e2e or acc the same way

  GET /stream  -> SSE, JSON snapshot ~10 Hz onroad / 1 Hz standby
  GET /health  -> {"ok": true}
"""
import bisect
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.relc import RoadEdgeLaneChangeController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.hud.radar_tracker import TargetTracker, relevant

PORT = 8902
SERVICES = ["carState", "selfdriveState", "radarState", "radarTracksSP", "modelV2", "carControl",
            "longitudinalPlan", "controlsState"]

KPH_TO_MS = 1 / 3.6

# modelV2 的線是 33 個不等距點；降取樣成固定距離，HUD 夠用又不會塞爆 SSE
SAMPLE_X = [0, 5, 12, 20, 30, 45, 60, 80, 95, 110]

_lock = threading.Lock()
_snapshot = b'{"standby": true}'
_frame = 0
_model_cache = None


def _resample(line):
  xs, ys = list(line.x), list(line.y)
  if len(xs) < 2:
    return [0.0] * len(SAMPLE_X)
  out = []
  for d in SAMPLE_X:
    if d <= xs[0]:
      out.append(ys[0])
    elif d >= xs[-1]:
      out.append(ys[-1])
    else:
      i = bisect.bisect_left(xs, d)
      x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
      t = (d - x0) / (x1 - x0) if x1 != x0 else 0.0
      out.append(y0 + (y1 - y0) * t)
  return [round(float(v), 3) for v in out]


def _model(md):
  return {
    "d": SAMPLE_X,
    "lanes": [{"y": _resample(l), "p": round(float(p), 3)}
              for l, p in zip(md.laneLines, md.laneLineProbs, strict=True)],
    "edges": [{"y": _resample(e), "s": round(float(st), 3)}
              for e, st in zip(md.roadEdges, md.roadEdgeStds, strict=True)],
    "path": _resample(md.position),
  }


def _car_state(cs):
  cruise = cs.cruiseState
  # With openpilot longitudinal the set speed lives in carState.vCruise (km/h, 255 when
  # unset); cruiseState.speed belongs to the stock ACC and stays 0 on this car.
  v_cruise = float(cs.vCruise)
  cruise_speed = v_cruise * KPH_TO_MS if 0. < v_cruise < 250. else 0.
  return {
    "vEgo": float(cs.vEgo),
    "vEgoCluster": float(cs.vEgoCluster),
    "cruiseEnabled": bool(cruise.enabled),
    "cruiseAvailable": bool(cruise.available),
    "cruiseSpeed": cruise_speed,
    "gasPressed": bool(cs.gasPressed),
    "brakePressed": bool(cs.brakePressed),
    "leftBlinker": bool(cs.leftBlinker),
    "rightBlinker": bool(cs.rightBlinker),
    "standstill": bool(cs.standstill),
    "steeringAngleDeg": float(cs.steeringAngleDeg),
    "leftBlindspot": bool(cs.leftBlindspot),
    "rightBlindspot": bool(cs.rightBlindspot),
    "yawRate": float(cs.yawRate),
  }


def _lat_saturated(cs):
  try:
    lcs = cs.lateralControlState
    branch = getattr(lcs, lcs.which())
    return bool(getattr(branch, "saturated", False))
  except Exception:
    return False


def _stop_distance(speeds):
  """How much further the plan runs before it comes to rest.

  There is no stop line on the wire, so integrate the plan's own speed trajectory.
  It only reaches 2.5s ahead, so this is the distance left to a stop that is already
  being braked for, not the distance to one still up the road.
  """
  t = ModelConstants.T_IDXS
  if len(speeds) < 2:
    return 0.
  dist = 0.
  for i in range(min(len(speeds), len(t)) - 1):
    dist += (speeds[i] + speeds[i + 1]) / 2 * (t[i + 1] - t[i])
    if speeds[i + 1] < 0.1:
      break
  return dist


def _control(cc, lp, cs):
  a = cc.actuators
  return {
    "sat": _lat_saturated(cs),
    "torque": float(a.torque),          # 橫向輸出 [0,1]
    "curvature": float(a.curvature),    # 1/m，配 v^2 可得目標橫向加速度
    "accel": float(a.accel),            # m/s^2
    "aTarget": float(lp.aTarget),
    "src": str(lp.longitudinalPlanSource),   # cruise / lead0 / lead1 / lead2 / e2e
    "hasLead": bool(lp.hasLead),
    "stop": bool(lp.shouldStop),             # the model's own call, via the planner
    "stopDistance": _stop_distance(lp.speeds),
  }


def _selfdrive_state(ss):
  return {
    "enabled": bool(ss.enabled),
    "active": bool(ss.active),
    "state": str(ss.state),
    "alertText1": str(ss.alertText1),
    "alertText2": str(ss.alertText2),
    "alertStatus": str(ss.alertStatus),
    "experimentalMode": bool(ss.experimentalMode),
  }


def _lead(lead):
  return {
    "status": bool(lead.present),   # sunnypilot called this status
    "dRel": float(lead.dRel),
    "vRel": float(lead.vRel),
    "yRel": float(lead.yRel),
    "vLead": float(lead.vLead),
    "radar": bool(lead.radar),      # matched to a radar track, rather than vision alone
    "prob": float(lead.modelProb),
  }


def _radar_targets(points, targets, v_ego, dt):
  """Everything the radar sees, not just the one target being followed.

  radard only ever hands out two leads and they are both in our own lane, so the cars
  either side never reach the display. The decode is opendbc's, the same one that feeds
  the car; what happens here is only the picking and smoothing a display needs.
  """
  try:
    groups = relevant(points, v_ego)
    return [
      {"status": True,
       "id": t.id,                        # stable across frames, so the page can follow one car
       "dRel": round(t.dRel, 1),
       "yRel": round(t.yRel, 1),
       "vRel": round(t.vRel, 1),
       "radar": True,
       "coast": t.misses > 0,             # held over a dropped frame rather than seen right now
       "side": abs(t.yRel) > 1.8,         # in a lane beside us rather than ahead
       "lane": t.lane,
       "laneN": t.lane_n,                 # -2..2, negative to our right
       "oncoming": t.oncoming}
      for t in targets.update(groups, dt)
    ]
  except Exception:
    return []


def _radar_state(rs):
  one, two = _lead(rs.leadOne), _lead(rs.leadTwo)
  # the page reads the first lead off the top level
  return {"leadStatus": one["status"], "dRel": one["dRel"], "vRel": one["vRel"],
          "yRel": one["yRel"], "vLead": one["vLead"], "radar": one["radar"],
          "leads": [one, two]}


def _next_speed_limit(mem_params):
  """mapd writes the limit coming up as JSON: speedlimit in m/s, distance in metres."""
  try:
    j = json.loads(mem_params.get("NextMapSpeedLimit") or "{}")
    return float(j.get("speedlimit") or 0.), float(j.get("distance") or 0.)
  except Exception:
    return 0., 0.


def _sp_shapes(params, mem_params, cs, cc, ss):
  """Rebuild what sunnypilot used to publish, so the page does not have to change."""
  speed_limit = float(mem_params.get("MapSpeedLimit") or 0.)
  offset = params.get("SpeedLimitValueOffset", return_default=True) * KPH_TO_MS
  assisting = params.get("SpeedLimitMode", return_default=True) == 3 and speed_limit > 0.
  ahead, ahead_dist = _next_speed_limit(mem_params)

  return {
    "liveMapDataSP": {
      "roadName": mem_params.get("RoadName") or "",
      "speedLimitValid": speed_limit > 0.,
      "speedLimit": speed_limit,
      "speedLimitAheadValid": ahead > 0. and ahead_dist > 0.,
      "speedLimitAhead": ahead,
      "speedLimitAheadDistance": ahead_dist,
    },
    "longitudinalPlanSP": {
      "dec": {"state": "blended" if ss.experimentalMode else "acc"},
      "speedLimit": {
        "resolver": {
          "speedLimitFinal": speed_limit + offset if speed_limit > 0. else 0.,
          "speedLimitValid": speed_limit > 0.,
        },
        "assist": {"state": "active" if assisting else "inactive"},
      },
    },
    "selfdriveStateSP": {
      "speedLimit": {
        "resolver": {
          "speedLimitValid": speed_limit > 0.,
          "speedLimit": speed_limit,
          "speedLimitOffset": offset,
        },
      },
      "mads": {
        "available": params.get_bool("AlwaysOnLateral"),
        "active": bool(cc.latActive),
        "enabled": bool(cc.latActive),
        "state": "enabled" if cc.latActive else "disabled",
      },
    },
  }


def poll_loop():
  global _snapshot, _frame, _model_cache
  sm = messaging.SubMaster(SERVICES)
  params = Params()
  mem_params = Params("/dev/shm/params")
  targets = TargetTracker()
  last_target_t = time.monotonic()
  edges = RoadEdgeLaneChangeController()
  while True:
    sm.update(1000)
    data = {"ts": time.monotonic(), "alive": {s: bool(sm.alive[s]) for s in SERVICES}}
    onroad = sm.alive["carState"]

    now = time.monotonic()
    radar_points = [{"dRel": float(p.dRel), "yRel": float(p.yRel), "vRel": float(p.vRel)}
                    for p in sm["radarTracksSP"].points]
    data["standby"] = not onroad
    if not onroad:
      edges.reset()
      targets.reset()
    if onroad:
      try:
        data["carState"] = _car_state(sm["carState"])
        data["selfdriveState"] = _selfdrive_state(sm["selfdriveState"])
        data["radarState"] = _radar_state(sm["radarState"])
        dt = max(1e-3, min(0.5, now - last_target_t))
        last_target_t = now
        others = _radar_targets(radar_points, targets, float(sm["carState"].vEgo), dt)
        # radard's leads come first, so the page still treats the followed car as the main one
        followed = data["radarState"]["dRel"] if data["radarState"]["leadStatus"] else None
        data["radarState"]["leads"] += [t for t in others
                                        if followed is None or abs(t["dRel"] - followed) > 3.0]
        data["radarState"]["targets"] = len(others)
        _frame += 1
        if _frame % 2 == 0 or _model_cache is None:   # 20Hz 進來、10Hz 重算，跟輸出頻率對齊
          _model_cache = _model(sm["modelV2"])
        data["modelV2"] = _model_cache
        if sm.updated["modelV2"]:
          edges.update_from_model(sm["modelV2"], float(sm["carState"].vEgo))
        data["modelDataV2SP"] = {"leftLaneChangeEdgeBlock": edges.left_edge_detected,
                                 "rightLaneChangeEdgeBlock": edges.right_edge_detected}
        data["control"] = _control(sm["carControl"], sm["longitudinalPlan"], sm["controlsState"])
        data["control"]["reason"] = mem_params.get("LongPlanReason") or ""
        data.update(_sp_shapes(params, mem_params, sm["carState"], sm["carControl"], sm["selfdriveState"]))
      except Exception as e:
        data["error"] = str(e)
    if not onroad:
      data["liveMapDataSP"] = {"roadName": mem_params.get("RoadName") or ""}
    with _lock:
      _snapshot = json.dumps(data, default=str).encode()
    # offroad nothing publishes (sm.update just times out); onroad cap the loop ~20 Hz
    time.sleep(1.0 if not onroad else 0.05)


class Handler(BaseHTTPRequestHandler):
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt, *args):
    pass

  def do_GET(self):
    if self.path.startswith("/stream"):
      self.send_response(200)
      self.send_header("Content-Type", "text/event-stream")
      self.send_header("Cache-Control", "no-cache")
      self.send_header("Access-Control-Allow-Origin", "*")
      self.send_header("Connection", "keep-alive")
      self.end_headers()
      try:
        while True:
          with _lock:
            payload = _snapshot
          self.wfile.write(b"data: " + payload + b"\n\n")
          self.wfile.flush()
          time.sleep(0.1)
      except (BrokenPipeError, ConnectionResetError, OSError):
        return
    elif self.path.startswith("/health"):
      body = b'{"ok": true}'
      self.send_response(200)
      self.send_header("Content-Type", "application/json")
      self.send_header("Access-Control-Allow-Origin", "*")
      self.send_header("Content-Length", str(len(body)))
      self.end_headers()
      self.wfile.write(body)
    else:
      self.send_response(404)
      self.send_header("Content-Length", "0")
      self.end_headers()


def main():
  threading.Thread(target=poll_loop, daemon=True).start()
  server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
  server.daemon_threads = True
  server.serve_forever()


if __name__ == "__main__":
  main()
