#!/usr/bin/env python3
"""On C3: work out where C3's camera points relative to the car, every drive, by itself - and turn the light
detections into car-frame directions for C4's HUD. DISPLAY ONLY.

The driver (2026-10-18): C3 goes wherever it fits that day (likely the dashboard, lower than C4) and moves
every time he gets in - it cannot be asked to sit the same each time; openpilot calibrates itself and C3
should too.

  up       = C3's accelerometer averaged over the drive (gravity; corners and stops average out)
  forward  = the direction in which C3's accelerometer follows the car's own longitudinal acceleration
             (carState.aEgo from the wheels, sent by C4 over the USB link, c3_relay.py), taken square to "up",
             leaving out the time spent turning (C3's own gyro about "up" above MAX_YAW_RATE)
  car frame: x forward, z down, y = z x x (right) - openpilot's calib frame; C3's camera is fixed to its
             device frame (tici: view_frame_from_device_frame), so this places every pixel in the car frame.

Offline check on two of C4's own drives (fed in as if C4 were C3; the answer is known from openpilot's own
calibration): 1-1.5 deg after 10-15 minutes of driving.

Ready when there has been enough speeding up and slowing down (MIN_MOVING_S moving, aEgo spread MIN_AEGO_STD)
and two halves of the drive agree within MAX_SPLIT_DEG. If gravity swings by more than MOVED_DEG from the drive
average for MOVED_S, C3 has been moved: start over. The last good result is kept in CALIB_FILE and used
(marked "prev") until this drive's is ready.

Lights: c3detect writes its boxes to /dev/shm/c3det/det.json at half resolution (964x604, ar0231 f=2648 at full
res). Each box centre becomes azimuth (+ right) / elevation (+ up) in the car frame; anything below the horizon
(ELEV_MIN_DEG) is dropped - on the dashboard that is where the dashboard and its reflections are, and lights
are above it anyway. Sent to C4 (10.44.0.2:4411) twice a second; state in /dev/shm/c3calib.json.
"""
import json
import math
import os
import socket
import time
from collections import deque

import numpy as np

import cereal.messaging as messaging
from common.transformations.orientation import euler_from_rot

LISTEN = ("0.0.0.0", 4410)
C4 = ("10.44.0.2", 4411)
DET = "/dev/shm/c3det/det.json"
STATE = "/dev/shm/c3calib.json"
CALIB_FILE = "/data/c3link/calib.json"

GRID_HZ = 20.
SMOOTH_S = 0.5            # accelerometer moving average before the fit
WINDOW_S = 900.           # fit over (at most) the last 15 minutes
SOLVE_EVERY = 5.
MOVING_V = 2.0            # m/s
MIN_MOVING_S = 240.       # 10/18 offline: with 138 s it called a 4.6 deg-off result ready
MIN_AEGO_STD = 0.3        # m/s^2
GAIN_RANGE = (0.6, 1.4)   # C3's forward specific force per unit aEgo should be ~1
MIN_R2 = 0.4
MAX_SPLIT_DEG = 1.5
MAX_YAW_RATE = 0.05       # rad/s (~3 deg/s, after the gyro bias): turning, left out of the forward fit.
                          # 10/18 offline: braking into corners correlates the corner's sideways acceleration
                          # with aEgo and pulled "forward" 6 deg off, both halves agreeing (split could not see it)
MOVED_DEG = 4.
MOVED_S = 20.
LAGS_S = np.arange(-0.6, 0.61, 0.05)

FOCAL, CX, CY = 2648. / 2, 1928. / 4, 1208. / 4   # c3detect's half-resolution frame
ELEV_MIN_DEG = -2.0       # below the horizon (with ~1.5 deg of calibration error to spare)
# camera (x right, y down, z forward) from device (x forward, y right, z down)
VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])


def boot_ns():
  return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def unit(v):
  n = np.linalg.norm(v)
  return v / n if n > 0 else v


def angle_deg(a, b):
  return math.degrees(math.acos(max(-1., min(1., float(np.dot(unit(a), unit(b)))))))


def write_json(path, obj):
  tmp = path + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(obj, f, ensure_ascii=False)
  os.replace(tmp, path)


class Calibrator:
  def __init__(self):
    self.acc = deque()          # (t_ns, x, y, z)   C3 accelerometer, C3 clock
    self.gyro = deque()         # (t_ns, x, y, z)   C3 gyro, C3 clock
    self.cs = deque()           # (t_ns, vEgo, aEgo) from C4, on C3's clock
    self.offsets = deque(maxlen=300)
    self.result = None
    self.prev = self.load()
    self.moved_since = None
    self.state = {}
    self.last_reset = None

  @staticmethod
  def load():
    try:
      with open(CALIB_FILE) as f:
        j = json.load(f)
      return np.array(j["car_from_dev"])
    except (OSError, ValueError, KeyError):
      return None

  def reset(self, why):
    self.acc.clear()
    self.gyro.clear()
    self.cs.clear()
    self.result = None
    self.moved_since = None
    self.last_reset = why

  def add_c4(self, pkt, recv_ns):
    self.offsets.append(recv_ns - pkt["s"])
    off = min(self.offsets)             # C4 -> C3 clock, latency floor
    for t, v, a in pkt["cs"]:
      self.cs.append((t + off, v, a))

  def trim(self, now_ns):
    lim = now_ns - int(WINDOW_S * 1e9)
    for q in (self.acc, self.gyro, self.cs):
      while q and q[0][0] < lim:
        q.popleft()

  def solve(self):
    if len(self.acc) < 200 or len(self.cs) < 200:
      return {"status": "collecting", "why": "no data yet" if not self.cs else "too little data"}
    A = np.array(self.acc, dtype=float)
    C = np.array(self.cs, dtype=float)
    t0, t1 = max(A[0, 0], C[0, 0]), min(A[-1, 0], C[-1, 0])
    if t1 - t0 < 30e9:
      return {"status": "collecting", "why": "overlap < 30 s"}
    g = np.arange(t0, t1, 1e9 / GRID_HZ)
    acc = np.stack([np.interp(g, A[:, 0], A[:, i]) for i in (1, 2, 3)], 1)
    k = max(1, int(SMOOTH_S * GRID_HZ))
    acc = np.stack([np.convolve(acc[:, i], np.ones(k) / k, mode="same") for i in range(3)], 1)
    v = np.interp(g, C[:, 0], C[:, 1])

    up = unit(acc.mean(0))                       # specific force at rest points up
    moving = v > MOVING_V
    if len(self.gyro) > 100:
      Gy = np.array(self.gyro, dtype=float)
      yaw = np.stack([np.interp(g, Gy[:, 0], Gy[:, i]) for i in (1, 2, 3)], 1) @ up
      yaw = np.convolve(yaw, np.ones(k) / k, mode="same")
      still = v < 0.1                          # the raw gyro has a bias: what it reads standing still
      yaw -= np.median(yaw[still]) if still.sum() > 5 * GRID_HZ else np.median(yaw)
      moving &= np.abs(yaw) < MAX_YAW_RATE
    moving_s = moving.sum() / GRID_HZ
    out = {"moving_s": round(moving_s), "up": up.round(4).tolist()}
    if moving_s < MIN_MOVING_S:
      return dict(out, status="collecting", why=f"moving {moving_s:.0f}/{MIN_MOVING_S:.0f} s")

    horiz = acc - np.outer(acc @ up, up)         # remove gravity: what is left is the car's own acceleration
    best = None
    for lag in LAGS_S:
      a_ego = np.interp(g - lag * 1e9, C[:, 0], C[:, 2])
      m = moving
      ae = a_ego[m] - a_ego[m].mean()
      h = horiz[m] - horiz[m].mean(0)
      if ae.std() < 1e-3:
        continue
      f = (h * ae[:, None]).sum(0) / (ae ** 2).sum()
      r2 = 1. - ((h - np.outer(ae, f)) @ unit(f)).var() / max(1e-9, (h @ unit(f)).var())
      if best is None or r2 > best[0]:
        best = (r2, lag, f, ae.std())
    if best is None:
      return dict(out, status="collecting", why="no aEgo variation")
    r2, lag, f, ae_std = best
    gain = float(np.linalg.norm(f))
    fwd = unit(f)
    out.update({"lag_ms": round(lag * 1000), "r2": round(r2, 3), "gain": round(gain, 3), "aego_std": round(ae_std, 3)})
    if ae_std < MIN_AEGO_STD:
      return dict(out, status="collecting", why=f"aEgo spread {ae_std:.2f} < {MIN_AEGO_STD}")

    # two halves (alternating minutes) must agree
    idx = np.nonzero(moving)[0]
    half = ((g[idx] - g[0]) // 60e9).astype(int) % 2
    a_ego = np.interp(g - lag * 1e9, C[:, 0], C[:, 2])
    fh = []
    for hsel in (0, 1):
      ii = idx[half == hsel]
      if len(ii) < 20 * GRID_HZ:
        continue
      ae = a_ego[ii] - a_ego[ii].mean()
      h = horiz[ii] - horiz[ii].mean(0)
      fh.append(unit((h * ae[:, None]).sum(0)))
    split = angle_deg(fh[0], fh[1]) if len(fh) == 2 else 99.
    out["split_deg"] = round(split, 2)

    x = fwd
    z = -up
    y = unit(np.cross(z, x))
    x = np.cross(y, z)
    car_from_dev = np.stack([x, y, z])
    rpy = euler_from_rot(car_from_dev.T)          # like openpilot's rpyCalib: device_from_calib = rot_from_euler(rpy)
    out.update({"car_from_dev": car_from_dev.round(5).tolist(), "rpy_deg": np.degrees(rpy).round(2).tolist()})
    ok = (GAIN_RANGE[0] <= gain <= GAIN_RANGE[1] and r2 >= MIN_R2 and split <= MAX_SPLIT_DEG)
    out["status"] = "ok" if ok else "converging"
    if not ok:
      out["why"] = f"gain {gain:.2f} r2 {r2:.2f} split {split:.1f} deg"
    return out

  def check_moved(self, now_ns):
    """C3 knocked or remounted: gravity in the last few seconds far from the drive's average."""
    if len(self.acc) < 2000:
      return False
    A = np.array(self.acc, dtype=float)
    recent = A[A[:, 0] > now_ns - 5e9, 1:]
    if len(recent) < 50:
      return False
    d = angle_deg(recent.mean(0), A[:, 1:].mean(0))
    if d > MOVED_DEG:
      self.moved_since = self.moved_since or now_ns
      if now_ns - self.moved_since > MOVED_S * 1e9:
        return True
    else:
      self.moved_since = None
    return False


def light_dirs(det, car_from_dev):
  """c3detect boxes -> car-frame azimuth / elevation (deg); below the horizon dropped."""
  out, below = [], 0
  dev_from_view = VIEW_FROM_DEVICE.T
  for d in det.get("dets", []):
    u, v = d["x"] + d["w"] / 2, d["y"] + d["h"] / 2
    ray = car_from_dev @ (dev_from_view @ np.array([(u - CX) / FOCAL, (v - CY) / FOCAL, 1.]))
    az = math.degrees(math.atan2(ray[1], ray[0]))
    el = math.degrees(math.atan2(-ray[2], math.hypot(ray[0], ray[1])))
    if el < ELEV_MIN_DEG:
      below += 1
      continue
    out.append({"zh": d.get("zh", d.get("cls")), "cls": d.get("cls"), "conf": d.get("conf"),
                "az": round(az, 1), "el": round(el, 1), "h": d.get("h")})
  return out, below


def horizon_row(car_from_dev):
  """Image row (half res) of the horizon straight ahead, for the screen / debugging."""
  ray_cam = VIEW_FROM_DEVICE @ (car_from_dev.T @ np.array([1., 0., 0.]))
  return round(float(CY + FOCAL * ray_cam[1] / ray_cam[2]), 1) if ray_cam[2] > 0 else None


def main():
  sm = messaging.SubMaster(["accelerometer", "gyroscope"])
  rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  rx.bind(LISTEN)
  rx.setblocking(False)
  tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  cal = Calibrator()
  last_solve = last_send = last_state = 0.
  last_c4 = -1e9
  det_ts = None
  while True:
    sm.update(50)
    if sm.updated["accelerometer"]:
      e = sm["accelerometer"]
      if e.which() == "acceleration" and len(e.acceleration.v) == 3:
        x, y, z = e.acceleration.v
        cal.acc.append((sm.logMonoTime["accelerometer"], x, y, z))
    if sm.updated["gyroscope"]:
      e = sm["gyroscope"]
      if e.which() == "gyroUncalibrated" and len(e.gyroUncalibrated.v) == 3:
        x, y, z = e.gyroUncalibrated.v
        cal.gyro.append((sm.logMonoTime["gyroscope"], x, y, z))
    try:
      while True:
        data, _ = rx.recvfrom(65536)
        cal.add_c4(json.loads(data), boot_ns())
        last_c4 = time.monotonic()
    except (BlockingIOError, ValueError, KeyError):
      pass

    now = time.monotonic()
    now_ns = boot_ns()
    if now - last_solve >= SOLVE_EVERY:
      last_solve = now
      cal.trim(now_ns)
      if cal.check_moved(now_ns):
        cal.reset("C3 moved")
      try:
        r = cal.solve()
      except Exception as ex:  # keep calibrating whatever one solve throws
        r = {"status": "error", "why": repr(ex)}
      if now - last_c4 > 5:
        r["link"] = "no data from C4 (link down or C4 offroad)"
      if r.get("status") == "ok":
        cal.result = np.array(r["car_from_dev"])
        write_json(CALIB_FILE, {"car_from_dev": r["car_from_dev"], "rpy_deg": r["rpy_deg"]})
      r["using"] = "this drive" if cal.result is not None else ("previous drive" if cal.prev is not None else None)
      basis = cal.result if cal.result is not None else cal.prev
      if basis is not None:
        r["horizon_row"] = horizon_row(basis)
      cal.state = r
    # every second: whether C4 is sending carState (= driving) - c3detect only runs while it is
    if now - last_state >= 1.:
      last_state = now
      write_json(STATE, dict(cal.state, driving=now - last_c4 < 5, t=now))

    if now - last_send >= 0.5:
      last_send = now
      basis = cal.result if cal.result is not None else cal.prev
      msg = {"cal": getattr(cal, "state", {}).get("status", "collecting"),
             "using": "this drive" if cal.result is not None else ("previous drive" if cal.prev is not None else None)}
      try:
        with open(DET) as f:
          det = json.load(f)
        if det.get("ts") != det_ts:
          det_ts = det.get("ts")
          msg["fps"] = det.get("fps")
          if basis is not None:
            msg["lights"], msg["below"] = light_dirs(det, basis)
      except (OSError, ValueError):
        pass
      try:
        tx.sendto(json.dumps(msg, ensure_ascii=False).encode(), C4)
      except OSError:
        pass


if __name__ == "__main__":
  main()
