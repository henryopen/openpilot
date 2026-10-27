#!/usr/bin/env python3
"""On C3: work out where C3's camera points relative to the car, every drive, by itself - and turn the light
detections into car-frame directions for C4's HUD. DISPLAY ONLY.

The driver (2026-10-18): C3 goes wherever it fits that day (likely the dashboard, lower than C4) and moves
every time he gets in - it cannot be asked to sit the same each time; openpilot calibrates itself and C3
should too.

  up       = C3's accelerometer averaged over the drive (gravity; corners and stops average out)
  forward  = the direction in which C3's accelerometer follows the car's own longitudinal acceleration
             (carState.aEgo from the wheels, sent by C4 over the USB link from hud/server.py), taken square to "up",
             leaving out the time spent turning (C3's own gyro about "up" above MAX_YAW_RATE)
  car frame: x forward, z down, y = z x x (right) - openpilot's calib frame; C3's camera is fixed to its
             device frame (tici: view_frame_from_device_frame), so this places every pixel in the car frame.

Offline check on two of C4's own drives (fed in as if C4 were C3; the answer is known from openpilot's own
calibration): 1-1.5 deg after 10-15 minutes of driving.

C3's IMU does not publish in the device frame: mounted upright in the car its accelerometer reads gravity on +x
(10/23 drive, 9.78 of 9.8). openpilot's locationd turns both accelerometer and gyro into the device frame with
(-z, -y, -x); done the same here on the way in, so "up" / "forward" / car_from_dev are all device frame, which is
what VIEW_FROM_DEVICE needs. (Until 10/23 the raw axes went straight in - never noticed because C3 had never
finished a calibration; the result would have put the lights ~80 deg off.) Check: the 10/23 drive's average,
turned so, puts the camera 5.8 deg up, the photos from that drive 3-5 deg.

Coarse (10/23, the driver: the lights should work from the start of the drive): until this drive's fit is ready,
"up" is just the accelerometer average and "forward" is taken as C3's own forward (device x, squared to "up") -
pitch and roll right, which decide whether a light is above the horizon; yaw off by however far C3 is turned.
The previous drive's result is preferred over it while its "up" agrees with this drive's within BOOT_MOVED_DEG (C3
not moved since). HUD: "粗標定" -> "已標定".

Ready when there has been enough speeding up and slowing down (MIN_MOVING_S moving, aEgo spread MIN_AEGO_STD)
and two halves of the drive agree within MAX_SPLIT_DEG. The last good result is kept in CALIB_FILE and used
(marked "prev") until this drive's is ready.

C3 moved? Judged only once per boot, by the BOOT_MOVED_DEG check above - never during a drive (10/24, the
driver: a mount that does not change while driving is fixed; only a reboot needs the question). The old in-drive
check (gravity of the last 5 s vs the drive average, 4 deg for 20 s) reset four times on 10/24, every one false:
junction turns and the garage ramp put 1-1.7 m/s2 of the car's own acceleration into the specific force.

Lights: c3lights (C4's band models) writes its boxes to /dev/shm/c3det/det.json at half resolution (964x604,
ar0231 f=2648 at full res), Each box centre becomes
azimuth (+ right) / elevation (+ up) in the car frame; anything below the horizon (ELEV_MIN_DEG) is dropped -
on the dashboard that is where the dashboard and its reflections are, and lights are above it anyway.
Sent to C4 (10.44.0.2:4411) twice a second; state in /dev/shm/c3calib.json.
"""
import json
import math
import os
import socket
import subprocess
import time
from collections import deque

import numpy as np

import cereal.messaging as messaging
from common.transformations.orientation import euler_from_rot

LISTEN = ("0.0.0.0", 4410)
C4 = ("10.44.0.2", 4411)
DET = "/dev/shm/c3det/det.json"
VOICE = "/dev/shm/c3voice.json"   # c3voice.py: what C3's mic heard and what came of it (res)
STATE = "/dev/shm/c3calib.json"
CALIB_FILE = "/data/c3link/calib.json"
CALIB_LOG = "/data/c3link/calib.log"   # every change of status / basis / link, and once a minute while driving
LOG_MAX = 1_000_000                     # bytes, then one rotation to calib.log.1

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
BOOT_MOVED_DEG = 10.      # previous drive's result kept unless this boot's gravity is this far from it. 10/24 the
                          # garage ramp alone was 5.8 deg from the drive's result (4 deg used to reject it)
COARSE_S = 5.             # seconds of accelerometer before the coarse result is offered
# sensord can come up alive but silent ("poll timed out" forever, 10/23: a whole boot with no accelerometer, so no
# calibration), which systemd cannot see. No accelerometer for this long restarts it, at most every SENSORD_RETRY.
SENSORD_SILENT_S = 15.
SENSORD_RETRY_S = 60.
LAGS_S = np.arange(-0.6, 0.61, 0.05)

FOCAL, CX, CY = 2648. / 2, 1928. / 4, 1208. / 4   # c3detect's half-resolution frame
ELEV_MIN_DEG = 0.0        # 10/25: on the Custin's dashboard (and the new finder) 44% of 533 judged lights sat under 3 deg
                          # (median 3.4); 3 -> 0 deg: HUD shows a light in 85% instead of 66% of the frames with one in
                          # front, frames with only false lights 31 -> 51 of 269 (before "twice in a row"). Was 3.0:
                          # 10/18, 82 boxes on C3's own drives judged by eye: lights sat 2.4-11.9 deg up (one at
                          # -0.5), tail lights / scooters / the dashboard around the horizon. >= 3 deg keeps 46 of
                          # 48 real lights and drops 17 of 34 false ones; raising the confidence instead cost lights
REPEAT_AZ_DEG = 2.0       # a light is passed on once seen twice in a row: same colour, within this
LIGHTS_FRESH_S = 3.0      # lights older than this are not shown (the 10/25 finder + colour runs ~1/s on C3, was 2.5/s)
LIGHT_HOLD_S = 2.5        # one cycle without the light does not blank it: the last lights stay this long (HUD flicker)
# camera (x right, y down, z forward) from device (x forward, y right, z down)
VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
LOG_EVERY_S = 60.


def dev_from_imu(x, y, z):
  """C3's accelerometer / gyro axes -> device frame, as openpilot's locationd does."""
  return -z, -y, -x


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


def log_line(text):
  try:
    if os.path.exists(CALIB_LOG) and os.path.getsize(CALIB_LOG) > LOG_MAX:
      os.replace(CALIB_LOG, CALIB_LOG + ".1")
    with open(CALIB_LOG, "a", encoding="utf-8") as f:
      f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} up={time.monotonic():.0f} {text}\n")
  except OSError:
    pass


def car_frame(fwd, up):
  """car_from_dev (rows: car x forward, y right, z down, in device coordinates) from forward and up."""
  z = -unit(up)
  y = unit(np.cross(z, fwd))
  x = np.cross(y, z)
  return np.stack([x, y, z])


class Calibrator:
  def __init__(self):
    self.acc = deque()          # (t_ns, x, y, z)   C3 accelerometer, C3 clock
    self.gyro = deque()         # (t_ns, x, y, z)   C3 gyro, C3 clock
    self.cs = deque()           # (t_ns, vEgo, aEgo) from C4, on C3's clock
    self.offsets = deque(maxlen=300)
    self.result = None
    self.coarse_m = None
    self.prev = self.load()
    self.prev_ok = None         # decided once per boot (first coarse gravity vs prev), then fixed for the drive
    self.boot_deg = None
    self.state = {}

  @staticmethod
  def load():
    try:
      with open(CALIB_FILE) as f:
        j = json.load(f)
      return np.array(j["car_from_dev"])
    except (OSError, ValueError, KeyError):
      return None

  def coarse(self):
    """Pitch and roll from gravity alone, forward taken as C3's own forward (see the top)."""
    if len(self.acc) < 2 or self.acc[-1][0] - self.acc[0][0] < COARSE_S * 1e9:
      return None
    up = unit(np.array(self.acc, dtype=float)[:, 1:].mean(0))
    return car_frame(np.array([1., 0., 0.]), up)

  def basis(self):
    """(car_from_dev, which) the lights are turned with: this drive's fit, else the previous drive's while C3 has
    not moved since (judged once, at the first coarse gravity of this boot), else the coarse one."""
    if self.result is not None:
      return self.result, "this drive"
    if self.prev is not None:
      if self.prev_ok is None and self.coarse_m is not None:
        self.boot_deg = round(angle_deg(self.prev[2], self.coarse_m[2]), 1)
        self.prev_ok = self.boot_deg <= BOOT_MOVED_DEG
      if self.prev_ok is not False:
        return self.prev, "previous drive"
    if self.coarse_m is not None:
      return self.coarse_m, "coarse"
    return None, None

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

    car_from_dev = car_frame(fwd, up)
    rpy = euler_from_rot(car_from_dev.T)          # like openpilot's rpyCalib: device_from_calib = rot_from_euler(rpy)
    out.update({"car_from_dev": car_from_dev.round(5).tolist(), "rpy_deg": np.degrees(rpy).round(2).tolist()})
    ok = (GAIN_RANGE[0] <= gain <= GAIN_RANGE[1] and r2 >= MIN_R2 and split <= MAX_SPLIT_DEG)
    out["status"] = "ok" if ok else "converging"
    if not ok:
      out["why"] = f"gain {gain:.2f} r2 {r2:.2f} split {split:.1f} deg"
    return out


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


def ahead_col(car_from_dev):
  """Image column (half res) of straight ahead - c3lights centres its band on it (10/26: it only looks within 12 deg)."""
  ray_cam = VIEW_FROM_DEVICE @ (car_from_dev.T @ np.array([1., 0., 0.]))
  return round(float(CX + FOCAL * ray_cam[0] / ray_cam[2]), 1) if ray_cam[2] > 0 else None


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
  last_lights = None
  last_seen = 0.
  prev_frame = []
  logged, last_log = None, 0.
  last_acc = last_sensord_kick = time.monotonic()
  while True:
    sm.update(50)
    if sm.updated["accelerometer"]:
      last_acc = time.monotonic()
    elif time.monotonic() - last_acc > SENSORD_SILENT_S and time.monotonic() - last_sensord_kick > SENSORD_RETRY_S:
      last_sensord_kick = time.monotonic()
      print("c3calib: no accelerometer for %.0f s, restarting c3-sensord" % (time.monotonic() - last_acc), flush=True)
      subprocess.run(["sudo", "systemctl", "restart", "c3-sensord"], timeout=30, check=False)
    if sm.updated["accelerometer"]:
      e = sm["accelerometer"]
      if e.which() == "acceleration" and len(e.acceleration.v) == 3:
        cal.acc.append((sm.logMonoTime["accelerometer"], *dev_from_imu(*e.acceleration.v)))
    if sm.updated["gyroscope"]:
      e = sm["gyroscope"]
      if e.which() == "gyroUncalibrated" and len(e.gyroUncalibrated.v) == 3:
        cal.gyro.append((sm.logMonoTime["gyroscope"], *dev_from_imu(*e.gyroUncalibrated.v)))
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
      try:
        r = cal.solve()
      except Exception as ex:  # keep calibrating whatever one solve throws
        r = {"status": "error", "why": repr(ex)}
      if now - last_c4 > 5:
        r["link"] = "no data from C4 (link down or C4 offroad)"
      if r.get("status") == "ok":
        cal.result = np.array(r["car_from_dev"])
        write_json(CALIB_FILE, {"car_from_dev": r["car_from_dev"], "rpy_deg": r["rpy_deg"]})
      cal.coarse_m = cal.coarse()
      basis, r["using"] = cal.basis()
      if basis is not None:
        r["horizon_row"] = horizon_row(basis)
        r["ahead_col"] = ahead_col(basis)
        r["basis_rpy_deg"] = np.degrees(euler_from_rot(basis.T)).round(2).tolist()
      cal.state = r
      # a lasting record (10/23: the drive's calibration could not be checked afterwards, /dev/shm is gone)
      key = (r.get("status"), r.get("using"), "link" not in r, cal.prev_ok)   # prev_ok: the boot decision, logged once
      if key != logged or ("link" not in r and now - last_log >= LOG_EVERY_S):
        logged, last_log = key, now
        log_line(json.dumps({k: r.get(k) for k in ("status", "why", "using", "link", "moving_s", "aego_std", "r2", "gain",
                                                    "split_deg", "lag_ms", "basis_rpy_deg", "horizon_row")}
                            | {"boot_deg": cal.boot_deg, "n_acc": len(cal.acc), "n_c4": len(cal.cs)}, ensure_ascii=False))
    # every second: whether C4 is sending carState (= driving) - c3detect only runs while it is
    if now - last_state >= 1.:
      last_state = now
      fresh = last_lights is not None and now - last_lights[2] < LIGHTS_FRESH_S
      # C4's speed for c3lights (10/26: stopped, it reads the found lights' colour 10x a second)
      v_c4 = round(float(cal.cs[-1][1]), 2) if cal.cs and now - last_c4 < 2 else None
      write_json(STATE, dict(cal.state, driving=now - last_c4 < 5, t=now, v=v_c4, lights=last_lights[0] if fresh else []))

    if now - last_send >= 0.5:
      last_send = now
      basis, using = cal.basis()
      st = cal.state
      # what the driver sees on C4's HUD (he looks at nothing else while driving): calibration progress,
      # C3's temperature, whether the detector runs, and the lights
      msg = {"cal": st.get("status", "collecting"), "why": st.get("why", ""),
             "prog": st.get("moving_s", 0), "need": MIN_MOVING_S,
             "using": using}
      try:
        with open(DET) as f:
          det = json.load(f)
        msg.update({"temp": det.get("temp"), "fps": det.get("fps"), "idle": det.get("idle", False),
                    "det_err": det.get("err", "")})
        if det.get("ts") != det_ts:
          det_ts = det.get("ts")
          if basis is not None and not det.get("idle"):
            lights, below = light_dirs(det, basis)
            # twice in a row (yolod's rule on C4): same colour, same direction as in the previous frame
            seen = [lt for lt in lights if any(p["zh"] == lt["zh"] and abs(p["az"] - lt["az"]) <= REPEAT_AZ_DEG
                                               and abs(p["el"] - lt["el"]) <= REPEAT_AZ_DEG for p in prev_frame)]
            prev_frame = lights
            if seen or last_lights is None or not last_lights[0] or now - last_seen > LIGHT_HOLD_S:
              last_lights = (seen, below, now)
              if seen:
                last_seen = now
            else:
              last_lights = (last_lights[0], below, now)    # held: a missed cycle keeps what was shown
      except (OSError, ValueError):
        msg["det_err"] = "c3detect 沒有輸出"
      try:
        with open(VOICE) as f:
          v = json.load(f)
        msg["voice"] = {"ok": bool(v.get("ok")) and now - float(v.get("t", 0)) < 3., "n": v.get("n", 0),
                        "last": v.get("last", ""), "res": v.get("res", ""),
                        "ago": round(now - float(v["last_t"]), 1) if v.get("last_t") else None}
      except (OSError, ValueError, TypeError):
        msg["voice"] = {"ok": False}
      if last_lights is not None and now - last_lights[2] < LIGHTS_FRESH_S:   # sends go out 2/s
        msg["lights"], msg["below"] = last_lights[0], last_lights[1]
      try:
        tx.sendto(json.dumps(msg, ensure_ascii=False).encode(), C4)
      except OSError:
        pass


if __name__ == "__main__":
  main()
