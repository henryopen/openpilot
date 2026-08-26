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
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.hud.radar_tracker import RadarTracker, cluster, relevant

PORT = 8902
SERVICES = ["carState", "selfdriveState", "radarState", "modelV2", "carControl",
            "longitudinalPlan", "controlsState"]

KPH_TO_MS = 1 / 3.6
RADAR_FIRST, RADAR_LAST = 0x238, 0x255   # the front radar's track list, on bus 1

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


class RoadEdgeBlock:
  """Whether the road edge is too close to change lanes into.

  sunnypilot worked this out in relc.py and published it on a custom message, which
  master has no room for. Every input is on stock modelV2 though, and this file already
  reads them, so the display works it out rather than going without. Same thresholds as
  relc.py. Display only, it blocks nothing.
  """
  NEARSIDE_PROB = 0.2
  EDGE_PROB = 0.35
  REACTION_TIME = 1.0
  CLEAR_TIME = 0.3
  MIN_SPEED = 20 * CV.MPH_TO_MS
  EDGE_MARGIN = 1.08
  CLEARANCE = 3.7

  def __init__(self):
    self.detected = [False, False]
    self._edge_timer = [0., 0.]
    self._clear_timer = [0., 0.]

  def reset(self):
    self.detected = [False, False]
    self._edge_timer = [0., 0.]
    self._clear_timer = [0., 0.]

  def update(self, md, v_ego):
    if v_ego < self.MIN_SPEED:
      self.reset()
      return

    edges = md.roadEdges
    # laneLineProbs is four lines: the pair either side of us is [1] and [2], so the
    # outer pair [0] and [3] is what a lane change would cross
    for i, lane_prob in enumerate((md.laneLineProbs[0], md.laneLineProbs[3])):
      edge_prob = min(max(1.0 - md.roadEdgeStds[i], 0.0), 1.0)
      clearance = abs(edges[i].y[0]) - self.EDGE_MARGIN if len(edges) > i and len(edges[i].y) else 0.
      close = edge_prob > self.EDGE_PROB and lane_prob < self.NEARSIDE_PROB and clearance < self.CLEARANCE

      if close:
        self._edge_timer[i] = min(self._edge_timer[i] + DT_MDL, self.REACTION_TIME + self.CLEAR_TIME)
        self._clear_timer[i] = 0.
        if self._edge_timer[i] > self.REACTION_TIME:
          self.detected[i] = True
      else:
        self._clear_timer[i] += DT_MDL
        if self._clear_timer[i] > self.CLEAR_TIME:
          self._edge_timer[i] = 0.
          self.detected[i] = False


def _car_state(cs):
  cruise = cs.cruiseState
  return {
    "vEgo": float(cs.vEgo),
    "vEgoCluster": float(cs.vEgoCluster),
    "cruiseEnabled": bool(cruise.enabled),
    "cruiseAvailable": bool(cruise.available),
    "cruiseSpeed": float(cruise.speed),
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


def _radar_targets(tracker, v_ego):
  """Everything the radar sees, not just the one target being followed.

  radard only ever hands out two leads and they are both in our own lane, so the
  cars either side never reach the display. The tracks are on bus 1 anyway, so read
  them here. This is display only: what actually controls the car is still the
  single target radard picked.
  """
  try:
    return [
      {"status": True,
       "dRel": round(g["dRel"], 1),
       "yRel": round(g["yRel"], 1),
       "vRel": round(g["vRel"], 1),
       "radar": True,
       "side": abs(g["yRel"]) > 1.8,      # in a lane beside us rather than ahead
       "lane": g["lane"],
       "oncoming": g["oncoming"]}
      for g in relevant(cluster(tracker.points(v_ego)), v_ego)
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
  can_sock = messaging.sub_sock("can", timeout=20)
  tracker = RadarTracker()
  edges = RoadEdgeBlock()
  while True:
    sm.update(1000)
    data = {"ts": time.monotonic(), "alive": {s: bool(sm.alive[s]) for s in SERVICES}}
    onroad = sm.alive["carState"]

    now = time.monotonic()
    for msg in messaging.drain_sock(can_sock):
      for c in msg.can:
        if c.src == 1 and RADAR_FIRST <= c.address <= RADAR_LAST:
          tracker.on_can(c.address, bytes(c.dat), now)
    data["standby"] = not onroad
    if not onroad:
      edges.reset()
    if onroad:
      try:
        data["carState"] = _car_state(sm["carState"])
        data["selfdriveState"] = _selfdrive_state(sm["selfdriveState"])
        data["radarState"] = _radar_state(sm["radarState"])
        others = _radar_targets(tracker, float(sm["carState"].vEgo))
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
          edges.update(sm["modelV2"], float(sm["carState"].vEgo))
        data["modelDataV2SP"] = {"leftLaneChangeEdgeBlock": edges.detected[0],
                                 "rightLaneChangeEdgeBlock": edges.detected[1]}
        data["control"] = _control(sm["carControl"], sm["longitudinalPlan"], sm["controlsState"])
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
