"""Where does the car come to rest, now the junction is an obstacle for the MPC?

Runs on the device, because the only honest way to ask this is with the real acados solver.
Three control laws over the same stops, started from the same state:

  cruise   the set speed alone, wound down the 0.65 m/s2 profile the module used to publish
  curve    that, plus the geometric a_target the module used to add as its own candidate
  mpc      the line handed to the solver as a car standing there, which is what it does now

What is measured is the control law and nothing else. The module is first run over the
recording at the speed actually driven, so arming, the tracked point and the moment of
commitment are the real ones; the stop is then driven again from that state, with the point
counted down by the wheels the way the module does it. The question each law answers is the
only one that matters here: given the distance it believes it has, does it stop in it?

Letting the simulated speed run free instead does not work, and was tried first. Most of
these stops were braked by the driver, so a simulation with no driver in it carries its
speed straight through the junction, and every number that falls out is about the trigger
firing late rather than about the controller. The trigger is not what this change touches.

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
# rolling above 5 m/s down to a halt with no lead.
EVENTS = ["00000013--41c7f635e5--1", "00000013--41c7f635e5--2",
          "00000013--41c7f635e5--7", "00000013--41c7f635e5--28",
          "00000010--17de10767f--1", "00000011--2ae114cdc3--0",
          "00000012--385c105bf2--16"]

ROOT = "/data/media/0/realdata"
ENTRY_SPEED = 5.0          # rolling this fast is a stop being made, not a crawl
HALTED = 0.3               # and this slow is a halt
OLD_APPROACH_DECEL = 0.65  # the profile the module published before this change
NO_CAP = 1e9
DEFAULT_V_CRUISE_KPH = 50.

PERSONALITY = log.LongitudinalPersonality.standard
# Only the fields get_cruise_accel and get_accel_from_plan actually read.
CP = types.SimpleNamespace(steerRatio=16.0, wheelbase=2.8, longitudinalActuatorDelay=0.2)
ACTION_T = CP.longitudinalActuatorDelay + DT_MDL


def read_segment(seg):
  """Every frame the module needs, in the order they were recorded."""
  path = os.path.join(ROOT, seg, "rlog.zst")
  if not os.path.exists(path):
    return None
  frames = []
  cs = rs = None
  for msg in LogReader(path):
    try:
      which = msg.which()
    except Exception:
      continue
    if which == "carState":
      cs = msg.carState
    elif which == "radarState":
      rs = msg.radarState
    elif which == "modelV2" and cs is not None:
      frames.append({"md": msg.modelV2, "v": float(cs.vEgo),
                     "v_cruise_kph": float(cs.vCruise), "rs": rs})
  return frames or None


def fake_radar():
  empty = types.SimpleNamespace(present=False, dRel=0., vLead=0., aLeadK=0., aLeadTau=1.5)
  return types.SimpleNamespace(leadOne=empty, leadTwo=empty)


def commitment(frames, v_cruise):
  """Replay the module at the speed actually driven, and take the state where it commits."""
  sfl = StopForLights()
  for f in frames:
    rs = f["rs"] if f["rs"] is not None else fake_radar()
    sfl.update(f["md"], f["v"], v_cruise, False, rs.leadOne)
    if sfl.is_active and f["v"] > ENTRY_SPEED:
      return f["v"], sfl.stop_distance
  return None, None


def stop_from(v0, d0, v_cruise, controller):
  """Drive the stop the module has committed to, under one control law.

  The point is dead reckoned exactly as the module does it, counted down with the wheels.
  Nothing else is fed in: from here the model has no say, and the question is only whether
  the law can stop in the distance the module believes it has.
  """
  mpc = LongitudinalMpc()
  rs = fake_radar()
  v, d, x, t = v0, d0, 0., 0.
  a_prev, a_cruise_prev = 0., 0.
  worst_a, worst_j = 0., 0.
  while v > HALTED and t < 30.:
    stop_x = None
    a_min = -4.0
    if controller == "mpc":
      stop_x = d - STOP_GAP + STOP_DISTANCE
      a_min = MAX_DECEL
      cap = 0. if d <= HANDOFF else NO_CAP          # the approach is the solver's
    else:
      cap = 0. if d <= HANDOFF else (2. * OLD_APPROACH_DECEL * (d - HANDOFF)) ** 0.5

    mpc.set_weights(True, personality=PERSONALITY)
    mpc.set_cur_state(v, a_prev)
    mpc.update(rs, personality=PERSONALITY, stop_x=stop_x, a_min=a_min)
    v_traj = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, mpc.v_solution)
    a_traj = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, mpc.a_solution)
    a_mpc = get_accel_from_plan(v_traj, a_traj, CONTROL_N_T_IDX, action_t=ACTION_T)

    a_cruise = get_cruise_accel(False, min(v_cruise, cap), v, a_cruise_prev, 0., CP, DT_MDL, 0., True)
    a_cruise_prev = a_cruise
    cands = [a_mpc, a_cruise]
    if controller == "curve":
      # the geometric candidate the module used to add, taken no firmer than MAX_DECEL
      a_geo = -v * v / (2. * max(d - STOP_GAP, 1.)) if v > 0.5 else 0.
      cands.append(max(a_geo, MAX_DECEL))

    a = min(cands)
    worst_j = min(worst_j, (a - a_prev) / DT_MDL)
    worst_a = min(worst_a, a)
    a_prev = a
    v = max(0., v + a * DT_MDL)
    x += v * DT_MDL
    d = max(0., d - v * DT_MDL)
    t += DT_MDL
  # where it came to rest against where it should have: STOP_GAP short of the tracked point
  return {"stopped": v <= HALTED, "err": x - (d0 - STOP_GAP), "a": worst_a, "j": worst_j, "t": t}


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
      print(seg + ": 不在這台機器上，跳過", flush=True)
      continue
    v_cruise_kph = next((f["v_cruise_kph"] for f in frames if 0. < f["v_cruise_kph"] < 250.),
                        DEFAULT_V_CRUISE_KPH)
    v_cruise = dash_to_true(v_cruise_kph * CV.KPH_TO_MS)
    v0, d0 = commitment(frames, v_cruise)
    if v0 is None:
      print(seg + ": 模組沒有承諾任何停止，跳過", flush=True)
      continue
    head = f"{seg}  承諾時 {v0 * CV.MS_TO_KPH:.1f} km/h、停止點 {d0:.1f} m 外"
    print("")
    print(head + f"（設定速度 {v_cruise_kph:.0f} km/h）", flush=True)
    row = {"seg": seg}
    for controller in ("cruise", "curve", "mpc"):
      r = stop_from(v0, d0, v_cruise, controller)
      row[controller] = r
      where = f"{r['err']:+.1f} m" if r["stopped"] else "30 秒還沒停"
      line = f"   {controller:<7}{where:>13}   最大減速 {r['a']:.2f} m/s²"
      print(line + f"  最大 jerk {r['j']:.1f} m/s³  費時 {r['t']:.1f} s", flush=True)
    rows.append(row)

  print("")
  print("=== 總結（正數 = 停在該停的位置之後，也就是衝過去）===")
  print(f"{'控制器':<8}{'停住':>7}{'中位':>10}{'最差':>10}{'最大減速':>10}")
  for controller in ("cruise", "curve", "mpc"):
    got = [r[controller] for r in rows]
    ok = [g for g in got if g["stopped"]]
    errs = sorted(g["err"] for g in ok)
    med = errs[len(errs) // 2] if errs else float("nan")
    worst = max(errs) if errs else float("nan")
    hardest = min((g["a"] for g in got), default=float("nan"))
    print(f"{controller:<8}{len(ok)}/{len(got):<5}{med:>+9.1f} m{worst:>+9.1f} m{hardest:>10.2f}")


if __name__ == "__main__":
  main()
