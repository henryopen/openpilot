#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from openpilot.cereal import messaging, log
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# Default lead acceleration decay set to 50% at 1s
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1     # Kalman filter states enum

# stationary qualification parameters
V_EGO_STATIONARY = 4.   # no stationary object flag below this speed

RADAR_TO_CAMERA = 1.52  # RADAR is ~ 1.5m ahead from center of mesh frame

# The lead going away is usually not the lead leaving. Over 2.27 hours of this car's own
# logs leadOne vanished 268 times an hour, and 52% of those came back within a second at a
# distance that carried on from where it left off, with the raw vision probability still
# sitting at 0.35 - the model blinking, not a car pulling away. Holding through the blink is
# worth 2.1:1 inside 25 m and 2.9:1 from 25 to 45, counted as dropout seconds recovered
# against seconds spent holding a lead that really had gone. Past 45 m it is 0.9:1, so
# nothing is held out there; LONGITUDINAL.md 7.3 says what is done instead.
#
# The gate is the model's own probability, not the filtered one radard decides with: that
# filter rises instantly and decays slowly, so by the time it admits the lead is gone it can
# no longer tell a blink from a departure. Holding on a timer alone loses - a second of it
# buys back 30 dropout seconds an hour and spends 41 (openpilot/yolo/analysis/lead_hold_sim.py).
LEAD_HOLD_MAX_DIST = 45.
LEAD_HOLD_MIN_PROB = 0.2
LEAD_HOLD_MAX_TIME = 3.0
LEAD_HOLD_MIN_SPEED = 3.0


class KalmanParams:
  def __init__(self, dt: float):
    # Lead Kalman Filter params, calculating K from A, C, Q, R requires the control library.
    # hardcoding a lookup table to compute K for values of radar_ts between 0.01s and 0.2s
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]
    #Q = np.matrix([[10., 0.0], [0.0, 100.]])
    #R = 1e3
    #K = np.matrix([[ 0.05705578], [ 0.03073241]])
    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)
    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float, v_lead: float):
    # relative values, copy
    self.dRel = d_rel   # LONG_DIST
    self.yRel = y_rel   # -LAT_DIST
    self.vRel = v_rel   # REL_SPEED
    self.vLead = v_lead

    # computed velocity and accelerations
    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    # Learn if constant acceleration
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau.x = _LEAD_ACCEL_TAU
    else:
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "present": True,
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    # Radar points closer than 0.75, are almost always glitches on toyota radars
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def __str__(self):
    ret = f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"
    return ret


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track],
                          preferred_track_id: int = -1):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])

    # This isn't exactly right, but it's a good heuristic
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  # if no 'sane' match is found return -1
  # stationary radar points can be false positives
  # The Custin's radar reports a guardrail scanned along its length as a steady target in
  # our own lane, and at 25% the gate is 28 m wide when vision is looking 110 m out, so the
  # rail gets accepted over the real car. Tightening this to 7% cut that from 56% of frames
  # to 4% while the stock ACC's own target still matched on 99%.
  #
  # That gate is a test of vision's distance, and vision's distance is the thing that
  # degrades: over 2026-08-29's two drives it missed by more than the gate on 26% of frames
  # by day and 52% at night, so the radar was thrown away exactly where it was worth most.
  # Speed and lateral position say 'same object' without asking vision how far away it is -
  # this radar reports each track's own speed over the ground - and the guardrail the gate
  # was tightened for sits a lane's width off centre rather than on it. Among the frames the
  # gate rejects, the lateral disagreement is 0.3 m at the median; letting those through
  # raises accepted frames from 77.9% to 91.7% by day and from 57.9% to 79.7% at night, and
  # what it admits disagrees on distance by 3.9 m at the median while agreeing on speed.
  same_object = abs(track.vRel + v_ego - lead.v[0]) < 1.5 and abs(track.yRel + lead.y[0]) < 2.0
  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist)*.07, 2.0]) or same_object
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track

  # The gate is strict enough to blink: over 2026-09-02's drives the lead fell from radar to
  # vision 1905 times in 63 minutes, and 95.1% of the time the track that came back was the
  # one that left, a median of 0.10 s later with its range 0.2 m from where it was. Nothing
  # moved - one frame of camera noise failed the test. Each swap hands the planner a
  # different measurement of the same car, disagreeing on closing speed by a median 6.4 km/h
  # and by as much as 45.8, and the MPC brakes for the difference.
  #
  # So give the track we were already using a second, looser look, the way StarPilot does
  # (selfdrive/controls/radard.py, preferred_track_id): still measured against this frame's
  # vision lead rather than sustained on its own, so a track that has genuinely diverged is
  # still dropped. cnt >= 3 keeps a track that has only just appeared from being preferred.
  preferred = tracks.get(preferred_track_id)
  if preferred is not None and preferred.cnt >= 3:
    pref_same = abs(preferred.vRel + v_ego - lead.v[0]) < 2.5 and abs(preferred.yRel + lead.y[0]) < 3.0
    pref_dist = abs(preferred.dRel - offset_vision_dist) < max([(offset_vision_dist)*.12, 4.0]) or pref_same
    pref_vel = (abs(preferred.vRel + v_ego - lead.v[0]) < 13) or (v_ego + preferred.vRel > 3)
    if pref_dist and pref_vel:
      return preferred
  return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float, lead_prob: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "modelProb": float(lead_prob),
    "present": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, lead_prob: float, low_speed_override: bool = True,
             preferred_track_id: int = -1) -> dict[str, Any]:
  # Determine leads, this is where the essential logic happens
  if len(tracks) > 0 and ready and lead_prob > .5:
    track = match_vision_to_track(v_ego, lead_msg, tracks, preferred_track_id)
  else:
    track = None

  lead_dict = {'present': False}
  if track is not None:
    lead_dict = track.get_RadarState(lead_prob)
  elif (track is None) and ready and (lead_prob > .5):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego, lead_prob)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)

      # Only choose new track if it is actually closer than the previous one
      if (not lead_dict['present']) or (closest_track.dRel < lead_dict['dRel']):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class LeadHold:
  """Carries the last lead through a dip in the model's confidence. See LEAD_HOLD_* above."""

  def __init__(self):
    self.lead: dict[str, Any] | None = None
    self.held = 0.

  def update(self, lead: dict[str, Any], raw_prob: float, v_ego: float) -> dict[str, Any]:
    if lead['present']:
      self.lead = dict(lead)
      self.held = 0.
      return lead

    if (self.lead is None or v_ego < LEAD_HOLD_MIN_SPEED or raw_prob < LEAD_HOLD_MIN_PROB
        or self.held >= LEAD_HOLD_MAX_TIME or self.lead['dRel'] > LEAD_HOLD_MAX_DIST):
      self.lead = None
      self.held = 0.
      return lead

    self.held += DT_MDL
    # let it keep closing at the speed it was last measured doing, so the gap does not freeze
    self.lead['dRel'] = max(self.lead['dRel'] + self.lead['vRel'] * DT_MDL, 0.)
    return dict(self.lead)


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)
    self.lead_prob_filters = [FirstOrderFilter(0.0, 0.2, DT_MDL) for _ in range(2)]
    self.prev_lead_track_ids = [-1, -1]
    self.lead_hold = LeadHold()

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL))+1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen['modelV2']

    if sm.recv_frame['carState'] != self.last_v_ego_frame:
      self.v_ego = sm['carState'].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame['carState']

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]

      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      # create the track if it doesn't exist or it's a new track
      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead)

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime['modelV2']
    self.radar_state.radarErrors = rr.errors

    if len(sm['modelV2'].velocity.x):
      model_v_ego = sm['modelV2'].velocity.x[0]
    else:
      model_v_ego = self.v_ego
    leads_v3 = sm['modelV2'].leadsV3
    if len(leads_v3) > 1:
      for i in range(2):
        # Asymmetric filter on lead prob to keep lead when uncertain
        lead_prob = leads_v3[i].prob
        if lead_prob > self.lead_prob_filters[i].x:
          self.lead_prob_filters[i].x = lead_prob
        else:
          self.lead_prob_filters[i].update(lead_prob)

      lead_one = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
                          self.lead_prob_filters[0].x, low_speed_override=True,
                          preferred_track_id=self.prev_lead_track_ids[0])
      self.radar_state.leadOne = self.lead_hold.update(lead_one, leads_v3[0].prob, self.v_ego)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego,
                                          self.lead_prob_filters[1].x, low_speed_override=False,
                                          preferred_track_id=self.prev_lead_track_ids[1])
      self.prev_lead_track_ids = [int(self.radar_state.leadOne.radarTrackId),
                                  int(self.radar_state.leadTwo.radarTrackId)]

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None

    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


# fuses camera and radar data for best lead detection
def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  # wait for stats about the car to come in from controls
  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  # *** setup messaging
  sm = messaging.SubMaster(['modelV2', 'carState', 'radarTracks'], poll='modelV2')
  pm = messaging.PubMaster(['radarState'])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()

    RD.update(sm, sm['radarTracks'])
    RD.publish(pm)


if __name__ == "__main__":
  main()
