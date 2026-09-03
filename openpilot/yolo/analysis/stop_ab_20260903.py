"""Where does the car come to rest, now the junction is an obstacle for the MPC?

Runs on the device, because the only honest way to ask this is with the real acados solver.
Three controllers over the same recordings, started from the same state:

  cruise   the set speed alone, wound down the 0.65 m/s2 profile the module used to publish
  curve    that, plus the geometric a_target the module used to add as its own candidate
  mpc      the line handed to the solver as a car standing there, which is what it does now

Ground truth is where the car came to rest on the day: distance travelled over the run-in,
integrated off the recorded wheel speed. A controller that travels further than that has
gone past the line, and the number printed is how far past.

The model's plan is replayed as recorded, so once a simulated speed drifts from the one
actually driven, the plan it is being fed is no longer the plan the model would have made.
That is inherent to replaying an open loop, it applies equally to all three, and it is why
these are only ever compared against each other rather than called absolute.

  PYTHONPATH=/data/openpilot /usr/local/venv/bin/python \
      openpilot/yolo/analysis/stop_ab_20260903.py
"""
import argparse
import os
import types

import numpy as np

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (LongitudinalMpc, STOP_DISTANCE,
                                                                           T_IDXS as T_IDXS_MPC)
from openpilot.selfdrive.controls.lib.longitudinal_planner import CONTROL_N_T_IDX, dash_to_true, get_cruise_accel
from openpilot.selfdrive.controls.lib.stop_for_lights import HANDOFF, MAX_DECEL, STOP_GAP, StopForLights
from openpilot.tools.lib.logreader import LogReader

# The seven stops with nothing in front over 2026-09-02, from sfl_review's ground truth:
# rolling above 5 m/s down to a halt with no lead. Four are on the route still on the car.
EVENTS = ["00000013--41c7f635e5--1", "00000013--41c7f635e5--2",
          "00000013--41c7f635e5--7", "00000013--41c7f635e5--28",
          "00000010--17de10767f--1", "00000011--2ae114cdc3--0",
          "00000012--385c105bf2--16"]

ROOT = "/data/media/0/realdata"
RUN_IN = 15.0            # seconds of approach to simulate
ENTRY_SPEED = 5.0        # rolling this fast before the halt is a stop, not a crawl
HALTED = 0.3             # and this slow is the halt
OLD_APPROACH_DECEL = 0.65  # the profile the module published before this change
DEFAULT_V_CRUISE_KPH = 50.

PERSONALITY = log.LongitudinalPersonality.standard
# Only the two fields get_cruise_accel and get_accel_from_plan actually read.
CP = types.SimpleNamespace(steerRatio=16.0, wheelbase=2.8, longitudinalActuatorDelay=0.2)
ACTION_T = CP.longitudinalActuatorDelay + DT_MDL


def read_segment(seg):
  """Every frame the controllers need, in the order they were recorded."""
  path = os.path.join(ROOT, seg)
  if not os.path.exists(os.path.join(path, "rlog.zst")):
    return None
  frames = []
  cs = rs = cc = None
  for msg in LogReader(os.path.join(path, "rlog.zst")):
    try:
      which = msg.which()
    except Exception:
      continue
    if which == "carState":
      cs = msg.carState
    elif which == "radarState":
      rs = msg.radarState
    elif which == "carControl":
      cc = msg.carControl
    elif which == "modelV2" and cs is not None:
      frames.append({"md": msg.modelV2, "v": float(cs.vEgo), "a": float(cs.aEgo),
                     "v_cruise_kph": float(cs.vCruise), "rs": rs, "cc": cc})
  return frames or None


def find_stop(frames):
  """The first halt that follows real speed, and where to start the run-in."""
  seen_speed = False
  for i, f in enumerate(frames):
    if f["v"] > ENTRY_SPEED:
      seen_speed = True
    elif seen_speed and f["v"] < HALTED:
      start = max(0, i - int(RUN_IN / DT_MDL))
      return start, i
  return None, None


def travelled(frames, start, end):
  return sum(f["v"] for f in frames[start:end]) * DT_MDL


def fake_radar():
  none = types.SimpleNamespace(present=False, dRel=0., vLead=0., aLeadK=0., aLeadTau=1.5)
  return types.SimpleNamespace(leadOne=none, leadTwo=none)


def simulate(frames, start, end, controller, d_true):
  """Drive the run-in again under one controller and report where it stops."""
  sfl = StopForLights()
  mpc = LongitudinalMpc()
  v = frames[start]["v"]
  a_prev = frames[start]["a"]
  a_cruise_prev = a_prev
  x = 0.
  v_cruise_kph = frames[start]["v_cruise_kph"]
  if not 0. < v_cruise_kph < 250.:
    v_cruise_kph = DEFAULT_V_CRUISE_KPH
  v_cruise = dash_to_true(v_cruise_kph * CV.KPH_TO_MS)

  worst_a, worst_j, armed_at, committed_at = 0., 0., None, None
  # a stop can take longer than it did on the day if the controller is softer, so run on
  # past the recorded halt rather than stopping the clock where the driver stopped
  for i in range(start, min(len(frames), end + int(20. / DT_MDL))):
    f = frames[i]
    rs = f["rs"] if f["rs"] is not None else fake_radar()
    lead = rs.leadOne
    sfl.update(f["md"], v, v_cruise, False, lead)
    if sfl.armed and armed_at is None:
      armed_at = d_true - x
    if sfl.is_active and committed_at is None:
      committed_at = d_true - x

    cap = sfl.v_cruise_cap
    stop_x = None
    a_min = -4.0
    if sfl.is_active:
      if controller == "mpc":
        stop_x = sfl.obstacle_x(STOP_DISTANCE)
        a_min = MAX_DECEL
      else:
        # what the module published before: a speed wound down a fixed-decel profile
        cap = 0. if sfl.stop_distance <= HANDOFF else (2. * OLD_APPROACH_DECEL *
                                                       (sfl.stop_distance - HANDOFF)) ** 0.5

    mpc.set_weights(True, personality=PERSONALITY)
    mpc.set_cur_state(v, a_prev)
    mpc.update(rs, personality=PERSONALITY, stop_x=stop_x, a_min=a_min)
    v_traj = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, mpc.v_solution)
    a_traj = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, mpc.a_solution)
    a_mpc = get_accel_from_plan(v_traj, a_traj, CONTROL_N_T_IDX, action_t=ACTION_T)

    a_cruise = get_cruise_accel(False, min(v_cruise, cap), v, a_cruise_prev, 0., CP, DT_MDL, 0., True)
    a_cruise_prev = a_cruise
    cands = [a_mpc, a_cruise]
    if controller == "curve" and sfl.is_active:
      # the geometric candidate the module used to add, taken no firmer than MAX_DECEL
      a_geo = float(f["md"].action.desiredAcceleration)
      if sfl.stop_distance > 0. and v > 0.5:
        a_geo = min(a_geo, -v * v / (2. * max(sfl.stop_distance - STOP_GAP, 1.)))
      cands.append(max(a_geo, MAX_DECEL))

    a = min(cands)
    worst_j = min(worst_j, (a - a_prev) / DT_MDL)
    worst_a = min(worst_a, a)
    a_prev = a
    v = max(0., v + a * DT_MDL)
    x += v * DT_MDL
    if v < HALTED:
      return {"stopped": True, "overshoot": x - d_true, "a": worst_a, "j": worst_j,
              "armed_at": armed_at, "committed_at": committed_at}
  return {"stopped": False, "overshoot": x - d_true, "a": worst_a, "j": worst_j,
          "armed_at": armed_at, "committed_at": committed_at}


def main():
  global ROOT
  ap = argparse.ArgumentParser()
  ap.add_argument("--root", default=ROOT)
  args = ap.parse_args()
  ROOT = args.root

  rows = []
  for seg in EVENTS:
    frames = read_segment(seg)
    if frames is None:
      print(f"{seg}: 不在這台機器上，跳過", flush=True)
      continue
    start, end = find_stop(frames)
    if start is None:
      print(f"{seg}: 找不到停止事件，跳過", flush=True)
      continue
    d_true = travelled(frames, start, end)
    entry = frames[start]["v"] * CV.MS_TO_KPH
    print(f"\n{seg}  進場 {entry:.1f} km/h  當天走了 {d_true:.1f} m", flush=True)
    row = {"seg": seg, "entry": entry, "d_true": d_true}
    for controller in ("cruise", "curve", "mpc"):
      r = simulate(frames, start, end, controller, d_true)
      row[controller] = r
      where = f"{r['overshoot']:+.1f} m" if r["stopped"] else "沒停下來"
      arm = "—" if r["armed_at"] is None else f"{r['armed_at']:.0f} m"
      com = "—" if r["committed_at"] is None else f"{r['committed_at']:.0f} m"
      line = f"   {controller:<7} {where:>12}   最大減速 {r['a']:.2f} m/s²"
      print(f"{line}  最大 jerk {r['j']:.2f} m/s³  武裝 {arm}  承諾 {com}", flush=True)
    rows.append(row)

  print("\n=== 總結（正數 = 衝過當天的停止點）===")
  print(f"{'控制器':<8}{'停住':>6}{'中位':>9}{'最差':>9}{'最大減速':>10}")
  for controller in ("cruise", "curve", "mpc"):
    got = [r[controller] for r in rows]
    ok = [g for g in got if g["stopped"]]
    over = sorted(g["overshoot"] for g in ok)
    med = over[len(over) // 2] if over else float("nan")
    worst = max(over) if over else float("nan")
    hardest = min((g["a"] for g in got), default=float("nan"))
    print(f"{controller:<8}{len(ok)}/{len(got):<4}{med:>+8.1f} m{worst:>+8.1f} m{hardest:>9.2f}")


if __name__ == "__main__":
  main()
