"""Moving up to a signal: follow the heads, keep the fixed ones, lock OUR head and read it in order (2026-11-03, tl-onboard).

The driver (11/02-11/03): "一個物件在移動中可以慢慢確定他是個我們需要的物件 然後在判斷他顏色" / "紅綠燈的特色是固定的 不會在往前跑
你要用他的特性來排除吊車尾紅燈路邊的行人燈招牌等" / "紅綠燈就是綠轉黃轉紅". Before, every finder cycle stood alone: C4's
tlwatchd took the surest red / green within 12 deg - other roads' heads, a truck's roof lamp, the next junction.
This is the on-car port of the overnight analysis chain (tl-overnight work/tltrack.py follow(), 109 approaches / 2674
frames judged by eye: right 99.6% of its calls, 0 red called go; the car's rule then 72.7% and 14), pure numpy:

  1 track     every finder box gets an ID, matched frame to frame (Hungarian on centre distance, gated by box size /
              speed; once a track's position is known its prediction is the projection of that point)
  2 fixed     each head-like track is triangulated from its rays and the ego motion (C4 speed + C3's gyro): a fixed
              point 4.5-8.5 m up, over the road ahead, near OSM's signal is a signal head; a far head (>~80 m, the
              distance too uncertain) counts when it is fixed, landscape, a vehicle head by the lamp model and at a
              head's elevation for OSM's distance - then OSM gives its distance
              (a lamp riding on a MOVING radar target - C4's radar points - is a vehicle's, not a head: a truck's
              roof lamp fits a fixed point as well as a head does, only at another scale; far ones are tested against
              both: fixed at OSM's distance vs on the radar target - see RADAR_FIT)
  3 ours      among the heads: the one over our lane ahead, nearest, landscape preferred; kept until passed (a valid
              head 8 m nearer takes over: the near side of the junction)
  4 read      both models on that head's box (red lamp lit = red unless a turn arrow + its blinker says go; green
              needs the lamp model's own green lamp; models in flat conflict -> nothing; pedestrian / non-vehicle box ->
              nothing)
  5 order     only green -> yellow -> red -> green (green -> red allowed); a yellow from red / from nothing is not
              believed; going green takes two readings in a row

The call goes to C4 (c3calib -> c3_relay -> tlwatchd): tlwatchd uses it for the light (HUD, light_ahead, the yellow
stop) while this has a head locked, its old reading otherwise.

Inputs per finder cycle (c3lights moving / every find): every box the finder gave (full res), its lightcls
probabilities, lamp-model reads (LAMP_IDX picks the boxes), the frame's rows (lit_cell), the ego pose (EgoIntegrator in
c3calib -> /dev/shm/c3ego.json), and from C4: OSM distance, blinkers, radar points.
"""
import math

import numpy as np

FOCAL, CX, CY = 2648., 1928. / 2, 1208. / 2      # full resolution, tici road camera (c3calib's at half res x2)
VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
CAM_H = 1.36                                      # m (tlwatchd C3_CAM_H)
CLS = ['green', 'green_arrow', 'off', 'pedestrian', 'red', 'yellow']   # c3lights CLS_NAMES
GO_LAMPS = ('green', 'straight', 'left', 'right')

# ---- the analysis chain's constants (tltrack.py, P3 final); unchanged
RMS_OK = 1.0          # deg: a real head fits a fixed point to 0.2-0.7 deg
SIG_OK = 0.2          # fit's distance known to 20% (1 sigma)
HEAD_H = (4.5, 8.5)   # m
OSM_SHORT, OSM_PAST = 30., 35.
LANE_Y = 12.
LANE_Y_FAR = 9.       # m: |y| of heads > FAR_X (P3: a truck's roof lamp 101 m / 10.9 m right, 1029_173336) - only
                      # without radar (radar None / RADAR_CHECK off): with it, the radar fit tells the truck (leg 2)
FAR_X = 80.
LOST_HOLD_S = 1.0
TOP_EXIT_PX = 8.       # full-res row: ours last seen this near the top = gone over us (no LOST_HOLD_S)
WIDTH_K = 1.0
UPRIGHT_COST = 2.0
PASS_BLOCK_M = 30.    # m: ours gone out over the top (gone_over) while its fixed point is still ahead (heads leave the
PASS_BLOCK_S = 6.     # picture ~16 m before the car passes under them): until it is passed (x < 2 m, at most
                      # PASS_BLOCK_S) only a head this much further on may be locked - the next junction's, not a side
                      # head of the same junction (TASK3 leg 4: 1029_141509 / 142426 locked another movement's red
                      # the moment ours left the top; the deployed version kept ours, uncalled, instead); 0: off
PASS_GO_RED = True    # ... only a head reading red while ours left it called go (one junction's heads for one
                      # movement show one colour: a red one beside our go is another movement's; ours yellow -> a red
                      # one, or red -> red, is the same signal's far-side head - 1029_162439, 142928)
UPRIGHT_SIDE_Y = 5.   # m: an upright head this far to the side is no candidate (TASK3: 1029_142409/142426's red on the
                      # left pole is another movement's; our heads there are landscape over the road); 0: off
SWITCH_NEARER_M = 8.
REFIT_EVERY = 2
MAX_MISS = 4
# TASK3 leg 2: the per-frame numbers above / below were tuned on the 0.5 s replays; live C3 cycles every ~0.65 s
CYCLE_REF = 0.5       # s
GATE_TIME = False     # Tracker gate x max(1, cycle / CYCLE_REF) (the heads move further between two cycles)
SEQ_GAP_S = 0.        # s: a pending go survives an unread cycle (None) this long after its first go (0: None resets)
FIT_SPAN_S = 0.       # s: a fit is tried from 3 observations when they span this (0: only from 4, as before)
SEQ_WARM_S = 1.5      # s: a newly locked head's own reads this far back (before the lock, when the lamp model read it)
                      # go into its Sequence first - the greens / red seen before the lock (0: it starts empty)
KEEP_OURS = True      # ours still followed (misses <= 1) but no candidate this cycle (near / overhead its fit's rms grows,
                      # OSM's distance already the next junction's): kept against the candidates by where it was the
                      # last time it was one, as an ok one is (else the best candidate wins)
PRED_EGO_R = 60.      # m: a track seen once is looked for where the car's own motion (C3 gyro + vEgo) moves a fixed
                      # point this far along its ray (0: where it was - a turning car moved far heads past the gate)
OSM_MAX = 160.        # tlwatchd APPROACH_M: the analysis had OSM only inside it (approach.jsonl)
MOVING = 3.0          # m/s

# ---- 11/03 tl-onboard: a lamp riding on a moving radar target is a vehicle's (see the top, 2). Being in front of a
# moving target is not enough (a lead car hides nothing above its roof, and a head far ahead is "inside" the lead's
# bearing and under 4.6 m when put at the lead's range): the lamp has to MOVE WITH it - put at that target's range the
# lamp keeps one height and one lateral offset over seconds, while a fixed head behind it grows (h - 1.36 ~ d / X).
# That test alone (rides_vehicle, leg 1) took real heads too - a head far ahead and a lamp on a car moving ahead at
# about our speed give nearly the same rays: called 57.5 -> 50-53%, and 2 frames red called go at 1031_091150. Leg 2
# (rides_fit): OSM gives the fixed head its scale, so the two hypotheses can be compared; far heads only. C4 sends the
# radar points (hud/server.py 'tl.rad').
RADAR_CHECK = True    # leg 2 (RADAR_FIT + RADAR_HOLD_UNDECIDED): 0 heads lost, 1029_173336 out without LANE_Y_FAR
RADAR_MOVING = 3.0    # m/s: |vRel + vEgo| above this - the target moves (a car standing at the light is left alone)
RADAR_HALF_W = 1.6    # m: half a vehicle's width (+ margin) around the radar point's bearing
RADAR_AZ_MARGIN = 1.0  # deg
RADAR_TOP = 4.8       # m: a lamp at most this high at the target's range can sit on it (trucks / buses ~3.5-4.2 m)
RADAR_RANGE = (3., 120.)
RADAR_WIN_S = 6.0     # s of the track's observations looked at
RADAR_SPAN_S = 2.0    # ... spanning at least this, with
RADAR_MIN_OBS = 4     # RADAR_MIN_OBS on the same target; less -> undecided: no NEW lock on it yet
RADAR_D_JUMP = 4.0    # m: the same target from one observation to the next (range change vs its vRel)
RADAR_HOLD_UNDECIDED = True    # 'undecided' blocks a new lock (with rides_vehicle it cost the most heads; with
                               # rides_fit, far only, it costs none and takes the truck's first 4 frames)
RADAR_H_SLOPE = 0.12  # m/s: |slope| of the lamp's height put at the target's range - riding on it ~0
RADAR_H_STD = 0.30    # m (+1% of the range): its spread
RADAR_Y_STD = 0.40    # m (+1% of the range): the lateral offset's spread
# tl-onboard leg 2: the two hypotheses fitted on the same observations - fixed at OSM's range (as fixed_far places it;
# OSM fixes the scale a fixed-point fit alone cannot tell from a lamp moving with us) vs a fixed offset on the chained
# radar target (its measured d(t), y(t)). 'ride' only when riding explains it clearly better.
RADAR_FIT = True
RADAR_OSM_WIN = (-25., 20.)  # m around OSM's distance the fixed head may be
RADAR_OX = (0., 12., 1.)     # m: the lamp ahead of the radar point (a truck's cab roof is up to ~10 m ahead of its rear)
RADAR_RATIO = 2.5            # rms_fixed >= RATIO * rms_ride ...
RADAR_RMS_MIN = 0.15         # deg ... and rms_fixed above this

# ---- 11/04 TASK3 (the driver 11/03: "沒法用地圖輔助就好提高辨燈的準確度嗎" -> "直接做第一項"): between two finder cycles
# (GPU, ~0.45 s, the CPU idle meanwhile) c3lights re-finds the followed heads on NEWER frames - the same finder on the
# CPU over a small crop around where each head should be now (Follower.local_targets: its fixed point projected with the
# pose, else its last two observations extrapolated) - and they come back as extra observations (Follower.step_local):
# more rays per second -> 4 observations / enough parallax sooner, a missed finder cycle bridged. Appearance (vehicle /
# pedestrian / landscape medians) stays on the finder's own reads (a local find has no lamp-model read); a local
# observation is never read for the light unless the lamp model was run on it (LOCAL_READ, ours only).
LOCAL_MAX = 3         # crops per local pass (one crop serves every target inside it)
LOCAL_S = 192         # px, full res: the crop (the small finder's input, models/c3light_ft1n_192x192_raw.onnx)
LOCAL_EDGE = 24       # px: a target is served by a crop when its predicted centre is this far inside the crop's edge
LOCAL_CONF = 0.15     # the small finder's class-9 score (c3lights DET_CONF)
LOCAL_SIZE = 0.9      # |log| size change vs the track's last box, as Tracker
LOCAL_AGE_S = 1.5     # s: tracks seen (finder or local) within this are looked for
LOCAL_READ = False    # ours re-found locally: lamp model + read it (else geometry only)

# ---- lamp-model reads per cycle (c3lights reads the 2 biggest boxes for det.json; the chain wants the lamp model on
# every box it may track: on 230 approaches 2.4 boxes a frame after dedupe, 90% <= 5)
LAMP_AZ = 40.         # deg
LAMP_EL_MIN = -1.     # deg (a fit is only tried above -1, see Follower)
LAMP_MAX = 6


class Cam:
  """Pixel (full res) <-> car-frame ray (x forward, y right, z down); R = car_from_dev (c3calib's basis)."""
  def __init__(self, car_from_dev):
    self.R = np.asarray(car_from_dev, float)
    self.dev_from_view = VIEW_FROM_DEVICE.T

  def ray(self, u, v):
    r = self.R @ (self.dev_from_view @ np.array([(u - CX) / FOCAL, (v - CY) / FOCAL, 1.]))
    return r / np.linalg.norm(r)

  def azel(self, u, v):
    r = self.ray(u, v)
    return math.degrees(math.atan2(r[1], r[0])), math.degrees(math.atan2(-r[2], math.hypot(r[0], r[1])))

  def pix(self, p):
    c = VIEW_FROM_DEVICE @ (self.R.T @ np.asarray(p, float))
    if c[2] <= 1e-6:
      return None
    return CX + FOCAL * c[0] / c[2], CY + FOCAL * c[1] / c[2]


# ---------------------------------------------------------------- one box
class Det:
  __slots__ = ('box', 'u', 'v', 'w', 'h', 'det', 'cls', 'lamp', 'az', 'el', 'ray', 'local')

  def __init__(self, box, det, cls, lamp, cam, local=False):
    self.local = local     # re-found by a local pass (TASK3), not the finder's full band
    self.box = list(box)
    x1, y1, x2, y2 = self.box
    self.u, self.v, self.w, self.h = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
    self.det, self.cls, self.lamp = det, cls, lamp or {}
    self.ray = cam.ray(self.u, self.v)
    self.az = math.degrees(math.atan2(self.ray[1], self.ray[0]))
    self.el = math.degrees(math.atan2(-self.ray[2], math.hypot(self.ray[0], self.ray[1])))

  def p_red(self):
    return max(self.lamp.get('red', 0.), self.cls.get('red', 0.))

  def p_go(self):
    return max([self.lamp.get(k, 0.) for k in GO_LAMPS] + [self.cls.get('green', 0.), self.cls.get('green_arrow', 0.)])

  def p_yellow(self):
    return max(self.lamp.get('yellow', 0.), self.cls.get('yellow', 0.))

  def p_ped(self):
    return max(self.lamp.get('pedestrian', 0.), self.cls.get('pedestrian', 0.))

  def p_vehicle(self):
    return self.lamp.get('vehicle', 0.5 if self.p_ped() < .5 else 0.)

  def landscape(self):
    return self.w >= 1.3 * self.h


def iou(a, b):
  ix = max(0., min(a[2], b[2]) - max(a[0], b[0]))
  iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
  i = ix * iy
  u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i
  return i / u if u > 0 else 0.


def dedupe(boxes, scores, nms=0.5):
  """Indices of the boxes kept: the finder head is NMS-free -> the best of any pair overlapping IoU >= nms or one inside
  the other (tltrack.dets_of). Order: by score, best first."""
  keep = []
  for i in sorted(range(len(boxes)), key=lambda i: -scores[i]):
    b = boxes[i]
    ok = True
    for j in keep:
      o = boxes[j]
      inside = (min(b[2], o[2]) - max(b[0], o[0]) > 0.8 * min(b[2] - b[0], o[2] - o[0]) and
                min(b[3], o[3]) - max(b[1], o[1]) > 0.8 * min(b[3] - b[1], o[3] - o[1]))
      if iou(b, o) >= nms or inside:
        ok = False
        break
    if ok:
      keep.append(i)
  return keep


def lamp_idx(boxes, scores, cam, must=()):
  """Which boxes the lamp model should read this cycle for the chain: the kept (deduped) boxes in front above
  LAMP_EL_MIN, biggest first, at most LAMP_MAX - plus `must` (the locked head's box)."""
  keep = dedupe(boxes, scores)
  cand = []
  for i in keep:
    x1, y1, x2, y2 = boxes[i]
    az, el = cam.azel((x1 + x2) / 2, (y1 + y2) / 2)
    if el >= LAMP_EL_MIN and abs(az) <= LAMP_AZ:
      cand.append(i)
  cand.sort(key=lambda i: -(boxes[i][3] - boxes[i][1]))
  out = cand[:LAMP_MAX]
  for i in must:
    if i not in out:
      out.append(i)
  return out


# ---------------------------------------------------------------- Hungarian (scipy.optimize.linear_sum_assignment)
def assign(cost):
  """Minimum-cost assignment of a rectangular cost matrix -> (rows, cols), as linear_sum_assignment (shortest
  augmenting path, Jonker-Volgenant style)."""
  cost = np.asarray(cost, float)
  tr = cost.shape[0] > cost.shape[1]
  if tr:
    cost = cost.T
  n, m = cost.shape                       # n <= m
  u, v = np.zeros(n + 1), np.zeros(m + 1)
  p = np.zeros(m + 1, int)                # p[j] = row (1-based) assigned to column j; column 0 is the virtual one
  way = np.zeros(m + 1, int)
  for i in range(1, n + 1):
    p[0] = i
    j0 = 0
    minv = np.full(m + 1, np.inf)
    used = np.zeros(m + 1, bool)
    while True:
      used[j0] = True
      i0 = p[j0]
      cur = cost[i0 - 1] - u[i0] - v[1:]
      free = ~used[1:]
      upd = free & (cur < minv[1:])
      minv[1:][upd] = cur[upd]
      way[1:][upd] = j0
      cand = np.where(free, minv[1:], np.inf)
      j1 = int(np.argmin(cand)) + 1
      delta = cand[j1 - 1]
      for j in range(m + 1):
        if used[j]:
          u[p[j]] += delta
          v[j] -= delta
        else:
          minv[j] -= delta
      j0 = j1
      if p[j0] == 0:
        break
    while True:
      j1 = way[j0]
      p[j0] = p[j1]
      j0 = j1
      if j0 == 0:
        break
  rows, cols = [], []
  for j in range(1, m + 1):
    if p[j]:
      rows.append(p[j] - 1)
      cols.append(j - 1)
  rows, cols = np.array(rows, int), np.array(cols, int)
  if tr:
    rows, cols = cols, rows
  o = np.argsort(rows)
  return rows[o], cols[o]


# ---------------------------------------------------------------- ego motion
class Ego:
  """The car's pose over time from arrays (t s, v m/s, north / east m, heading rad clockwise from north) - built by
  EgoIntegrator (live: c3calib) or by the analysis' Ego (offline replay)."""
  def __init__(self, t, v, n, e, hd):
    self.t, self.v, self.n, self.e, self.hd = (np.asarray(a, float) for a in (t, v, n, e, hd))

  def speed(self, t):
    return float(np.interp(t, self.t, self.v))

  def pose(self, t):
    return float(np.interp(t, self.t, self.n)), float(np.interp(t, self.t, self.e)), float(np.interp(t, self.t, self.hd))

  def poses(self, ts):
    return np.interp(ts, self.t, self.n), np.interp(ts, self.t, self.e), np.interp(ts, self.t, self.hd)

  def to_car(self, w, t):
    n, e, h = self.pose(t)
    dn, de = w[0] - n, w[1] - e
    return np.array([math.cos(h) * dn + math.sin(h) * de, -math.sin(h) * dn + math.cos(h) * de, w[2]])

  def to_world(self, p, t):
    n, e, h = self.pose(t)
    return np.array([n + math.cos(h) * p[0] - math.sin(h) * p[1], e + math.sin(h) * p[0] + math.cos(h) * p[1], p[2]])


class EgoIntegrator:
  """Live dead reckoning on C3 (c3calib): heading from C3's gyro about the car's down axis (car_from_dev @ gyro, bias =
  what it reads while the car stands, learnt at every stop), distance from C4's vEgo. Kept KEEP_S."""
  KEEP_S = 30.
  STILL_V = 0.05

  def __init__(self):
    self.t, self.v, self.n, self.e, self.hd = [], [], [], [], []
    self.bias, self.nb = 0., 0
    self.h, self.N, self.E = 0., 0., 0.
    self.t_last, self.v_last = None, 0.
    self.gt_last = None

  def gyro(self, t, yaw_rate, v_now):
    """yaw_rate: rad/s about the car's down axis (+ = turning right), raw."""
    if v_now is not None and v_now < self.STILL_V:
      self.nb = min(self.nb + 1, 2000)
      self.bias += (yaw_rate - self.bias) / self.nb
    if self.gt_last is not None and 0 < t - self.gt_last < 0.5:
      self.h += (yaw_rate - self.bias) * (t - self.gt_last)
    self.gt_last = t

  def speed(self, t, v):
    """A C4 vEgo sample at t (C3 clock)."""
    if self.t_last is not None and 0 < t - self.t_last < 1.:
      dt = t - self.t_last
      self.N += v * dt * math.cos(self.h)
      self.E += v * dt * math.sin(self.h)
    self.t_last, self.v_last = t, v
    if not self.t or t - self.t[-1] >= 0.1:            # 10 Hz, as the analysis had C4's table
      for a, x in ((self.t, t), (self.v, v), (self.n, self.N), (self.e, self.E), (self.hd, self.h)):
        a.append(x)
      while self.t and self.t[-1] - self.t[0] > self.KEEP_S:
        for a in (self.t, self.v, self.n, self.e, self.hd):
          a.pop(0)

  def dump(self):
    return {'t': [round(x, 3) for x in self.t], 'v': [round(x, 3) for x in self.v], 'n': [round(x, 3) for x in self.n],
            'e': [round(x, 3) for x in self.e], 'hd': [round(x, 5) for x in self.hd], 'bias': round(self.bias, 6)}


# ---------------------------------------------------------------- 1 tracking
class Track:
  _next = 0
  MAX_OBS = 400

  def __init__(self, det, t, i):
    Track._next += 1
    self.id = Track._next
    self.obs = [(t, i, det)]
    self.misses = 0
    self.fix = None
    self.rad = []                  # per observation: (t, el, az, [(d, y, vr) moving radar targets near its bearing])
    self.rides = None              # rides_vehicle: 'ride' / 'undecided' / None (fixed, or no moving target at it)
    self.rfit = None               # rides_fit: (rms fixed at OSM, rms riding) deg
    self.rds = []                  # SEQ_WARM_S: (t, read) before it was locked
    self.cand_world = None         # KEEP_OURS: where it was the last cycle it was a candidate

  @property
  def last(self):
    return self.obs[-1][2]

  @property
  def fobs(self):
    """The finder's own observations (appearance: lamp-model / lightcls medians, landscape) - local finds left out."""
    return [o for o in self.obs if not o[2].local]

  @property
  def flast(self):
    fo = self.fobs
    return fo[-1][2] if fo else self.last

  def add(self, t, i, det):
    self.obs.append((t, i, det))
    if len(self.obs) > self.MAX_OBS:
      self.obs.pop(0)

  def predict(self, t, ego):
    if self.fix is not None and self.fix.get('ok'):
      p = ego.to_car(self.fix['world'], t)
      if p is not None and p[0] > 1.:
        px = ego.cam.pix(p)
        if px is not None:
          return px
    d = self.last
    if PRED_EGO_R and len(self.obs) == 1 and d.ray[0] > 0.05 and t > self.obs[-1][0]:
      p = ego.to_car(ego.to_world(d.ray / d.ray[0] * PRED_EGO_R, self.obs[-1][0]), t)
      if p[0] > 1.:
        px = ego.cam.pix(p)
        if px is not None:
          return px
    if len(self.obs) >= 2 and ego.speed(t) > 0.5:
      (t0, _, d0), (t1, _, d1) = self.obs[-2], self.obs[-1]
      if t1 > t0:
        k = (t - t1) / (t1 - t0)
        return d1.u + k * (d1.u - d0.u), d1.v + k * (d1.v - d0.v)
    return d.u, d.v


class Tracker:
  def __init__(self, max_miss=MAX_MISS):
    self.max_miss = max_miss
    self.tracks = []
    self.t_prev = None

  def step(self, t, i, dets, ego):
    live = self.tracks
    assigned = {}
    gk = max(1., (t - self.t_prev) / CYCLE_REF) if (GATE_TIME and self.t_prev is not None) else 1.
    self.t_prev = t
    if live and dets:
      fast = ego.speed(t) > 0.5
      cost = np.full((len(live), len(dets)), 1e6)
      for a, tr in enumerate(live):
        pu, pv = tr.predict(t, ego)
        lw, lh = tr.last.w, tr.last.h
        gate = (max(1.2 * max(lw, lh), 12.) * (1 + tr.misses) + (25. if fast else 0.)) * gk
        for b, d in enumerate(dets):
          dist = math.hypot(d.u - pu, d.v - pv)
          size = max(abs(math.log(max(d.h, 1) / max(lh, 1))), WIDTH_K * abs(math.log(max(d.w, 1) / max(lw, 1))))
          if dist <= gate and size < 0.9:
            cost[a, b] = dist / gate + 0.5 * size
      ra, rb = assign(cost)
      for a, b in zip(ra, rb, strict=True):
        if cost[a, b] < 1e5:
          assigned[b] = live[a]
    used = set()
    for b, d in enumerate(dets):
      tr = assigned.get(b)
      if tr is None:
        tr = Track(d, t, i)
        self.tracks.append(tr)
      else:
        tr.add(t, i, d)
        tr.misses = 0
      used.add(tr.id)
    keep = []
    for tr in self.tracks:
      if tr.id not in used:
        tr.misses += 1
      if tr.misses <= self.max_miss:
        keep.append(tr)
    self.tracks = keep
    return assigned


# ---------------------------------------------------------------- local passes (TASK3)
def local_crops(targets, S=LOCAL_S, n_max=LOCAL_MAX, W=1928, H=1208):
  """Crop windows for a local pass, targets [(track, u, v)] in priority order -> [(x0, y0, k, [targets])]: k x k
  pixels averaged into one (a head too big for S at full res - near heads are 100-250 px wide - is looked for in a
  k*S window brought down to S); a target inside an existing window of its k (LOCAL_EDGE in) shares it."""
  crops = []
  for tg in targets:
    tr, pu, pv = tg
    k = max(1, math.ceil(1.8 * max(tr.last.w, tr.last.h) / S))
    R = S * k
    for c in crops:
      if c[2] == k and c[0] + LOCAL_EDGE * k <= pu <= c[0] + R - LOCAL_EDGE * k and c[1] + LOCAL_EDGE * k <= pv <= c[1] + R - LOCAL_EDGE * k:
        c[3].append(tg)
        break
    else:
      if len(crops) < n_max and R <= min(W, H):
        crops.append([int(round(min(max(pu - R / 2, 0), W - R))), int(round(min(max(pv - R / 2, 0), H - R))), k, [tg]])
  return crops


def local_match(targets, boxes, t, ego):
  """The boxes a crop gave (full res, deduped) -> {track id: box index}: Tracker's cost (distance from the prediction
  within its gate + size change) and Hungarian."""
  if not targets or not boxes:
    return {}
  fast = ego.speed(t) > 0.5
  cost = np.full((len(targets), len(boxes)), 1e6)
  for a, (tr, pu, pv) in enumerate(targets):
    lw, lh = tr.last.w, tr.last.h
    gate = max(1.2 * max(lw, lh), 12.) * (1 + tr.misses) + (25. if fast else 0.)
    for b, (x1, y1, x2, y2) in enumerate(boxes):
      dist = math.hypot((x1 + x2) / 2 - pu, (y1 + y2) / 2 - pv)
      size = max(abs(math.log(max(y2 - y1, 1) / max(lh, 1))), WIDTH_K * abs(math.log(max(x2 - x1, 1) / max(lw, 1))))
      if dist <= gate and size < LOCAL_SIZE:
        cost[a, b] = dist / gate + 0.5 * size
  ra, rb = assign(cost)
  return {targets[a][0].id: int(b) for a, b in zip(ra, rb, strict=True) if cost[a, b] < 1e5}


# ---------------------------------------------------------------- 2 fixed point (scipy least_squares, soft_l1, numpy)
F_SCALE = math.radians(0.3)


def _resid_jac(w, N, E, H, az, el):
  dn, de = w[0] - N, w[1] - E
  c, s = np.cos(H), np.sin(H)
  x, y = c * dn + s * de, -s * dn + c * de
  rho2 = dn * dn + de * de
  rho = np.sqrt(rho2)
  z = w[2]
  f = np.empty(2 * len(N))
  f[0::2] = np.arctan2(y, x) - az
  f[1::2] = np.arctan2(-z, rho) - el
  J = np.zeros((2 * len(N), 3))
  q = x * x + y * y
  J[0::2, 0] = (x * -s - y * c) / q
  J[0::2, 1] = (x * c - y * s) / q
  k = rho2 + z * z
  J[1::2, 0] = z * dn / np.maximum(rho, 1e-9) / k
  J[1::2, 1] = z * de / np.maximum(rho, 1e-9) / k
  J[1::2, 2] = -rho / k
  return f, J


def _cost(f):
  return float(np.sum(F_SCALE ** 2 * 2. * (np.sqrt(1. + (f / F_SCALE) ** 2) - 1.))) * 0.5


def soft_l1_fit(w0, N, E, H, az, el, iters=100):
  """argmin of the soft_l1 cost (f_scale 0.3 deg) by IRLS Levenberg-Marquardt -> (w, f, J robust-scaled, cost)."""
  w = np.array(w0, float)
  f, J = _resid_jac(w, N, E, H, az, el)
  cost = _cost(f)
  lam = 1e-3
  for _ in range(iters):
    z = (f / F_SCALE) ** 2
    wt = 1. / np.sqrt(1. + z)                       # rho'
    A = J.T @ (J * wt[:, None])
    g = J.T @ (wt * f)
    done = False
    for _ in range(20):
      try:
        step = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
      except np.linalg.LinAlgError:
        lam *= 10.
        continue
      wn = w + step
      fn, Jn = _resid_jac(wn, N, E, H, az, el)
      cn = _cost(fn)
      if np.isfinite(cn) and cn <= cost:
        small = abs(cost - cn) <= 1e-10 * max(cost, 1e-20) or np.linalg.norm(step) <= 1e-9 * (1. + np.linalg.norm(w))
        w, f, J, cost = wn, fn, Jn, cn
        lam = max(lam / 3., 1e-9)
        done = small
        break
      lam *= 4.
    else:
      break
    if done:
      break
  js = (1. + (f / F_SCALE) ** 2) ** -0.75          # scipy scale_for_robust_loss_function: sqrt(rho' + 2 rho'' f^2)
  return w, f, J * js[:, None], cost


def triangulate(track, ego, min_obs=3, min_travel=3., max_obs=30, warm=None):
  fo = track.fobs
  t_from = fo[-max_obs][0] if len(fo) >= max_obs else -1e18     # max_obs finder observations + the local ones since
  obs = [o for o in track.obs if o[0] >= t_from]
  if len(obs) < min_obs:
    return None
  ts = np.array([o[0] for o in obs])
  N, E, H = ego.poses(ts)
  travel = math.hypot(N[-1] - N[0], E[-1] - E[0])
  if travel < min_travel:
    return None
  rays = np.array([o[2].ray for o in obs])
  az = np.arctan2(rays[:, 1], rays[:, 0])
  el = np.arctan2(-rays[:, 2], np.hypot(rays[:, 0], rays[:, 1]))
  best = None
  if warm is not None:
    best = soft_l1_fit(warm, N, E, H, az, el)
  else:
    for r0 in (15., 40., 90.):
      p0 = rays[0] / rays[0][0] * r0 if rays[0][0] > 0.05 else rays[0] * r0
      w0 = ego.to_world(p0, ts[0])
      sol = soft_l1_fit(w0, N, E, H, az, el)
      if best is None or sol[3] < best[3]:
        best = sol
  w, f, J, _ = best
  if not (np.all(np.isfinite(w)) and np.all(np.isfinite(f))):
    return None                                  # a degenerate fit (live: a pose glitch) - no position
  p = ego.to_car(w, ts[-1])
  rms = math.degrees(float(np.sqrt(np.mean(f ** 2))))
  try:
    cov = np.linalg.inv(J.T @ J) * max(float(np.mean(f ** 2)), math.radians(0.15) ** 2)
    q = float(cov[0, 0] + cov[1, 1])
    sig = math.sqrt(q) if q > 0 and math.isfinite(q) else float('inf')   # near-singular: unknown, not a crash
  except np.linalg.LinAlgError:
    sig = float('inf')
  par = math.degrees(float(max(np.ptp(az), np.ptp(el))))
  x = float(p[0])
  return {'ok': x > 0 and rms < RMS_OK and par > 0.8 and sig < SIG_OK * x, 'sig': sig, 'world': w, 'x': x, 'y': float(p[1]),
          'height': CAM_H - float(p[2]), 'rms_deg': rms, 'parallax_deg': par, 'travel': travel}


def el_range(osm_m):
  """tlwatchd.c3_el_range."""
  near, far = max(osm_m - 20., 8.), osm_m + 15.
  return (math.degrees(math.atan((3.5 - CAM_H) / far)) - 1.0, math.degrees(math.atan((7.5 - CAM_H) / near)) + 1.0)


def head_ok(tr, fix):
  if fix is None or not fix['ok'] or not HEAD_H[0] <= fix['height'] <= HEAD_H[1] or abs(fix['y']) > 14.:
    return False
  fo = tr.fobs
  veh = np.median([o[2].p_vehicle() for o in fo])
  ped = np.median([o[2].p_ped() for o in fo])
  return veh >= 0.5 and ped < 0.5


def fixed_far(tr, osm_m):
  fx = tr.fix
  if fx is None or fx['rms_deg'] >= RMS_OK or fx['parallax_deg'] <= 0.8 or not 0 < fx['x'] < 600:
    return False
  d = tr.last
  lo, hi = el_range(osm_m)
  if not lo <= d.el <= hi:
    return False
  fo = tr.fobs
  if np.median([o[2].w >= 1.3 * o[2].h for o in fo[-6:]]) < 0.5:
    return False
  veh = np.median([o[2].p_vehicle() for o in fo])
  ped = np.median([o[2].p_ped() for o in fo])
  return veh >= 0.5 and ped < 0.5


def near_targets(det, radar, v_ego):
  """Moving radar targets the box could sit on: within its bearing (half a vehicle's width either side + a margin),
  the box under RADAR_TOP at the target's range. radar = [(dRel, yRel left +, vRel)] (C4's radarTracksSP points)."""
  out = []
  for d, y, vr in radar or []:
    if not RADAR_RANGE[0] <= d <= RADAR_RANGE[1] or abs(vr + v_ego) < RADAR_MOVING:
      continue
    az_t = math.degrees(math.atan2(-y, d))                 # + right, as Det.az
    half = math.degrees(math.atan2(RADAR_HALF_W, d)) + RADAR_AZ_MARGIN
    if abs(det.az - az_t) <= half and CAM_H + d * math.tan(math.radians(det.el)) <= RADAR_TOP:
      out.append((d, y, vr))
  return out


def rides_vehicle(tr):
  """Whether the track's lamp moves with a moving radar target it sits in front of now: chained target by target back
  from the newest observation (range continuity) over RADAR_WIN_S; with >= RADAR_MIN_OBS spanning RADAR_SPAN_S, put at
  that target's range its height and lateral offset stay put (spread, no height trend - a fixed head behind grows)
  -> 'ride'; a chain too short to tell -> 'undecided'; else None."""
  obs = [o for o in tr.rad if o[0] >= tr.rad[-1][0] - RADAR_WIN_S] if tr.rad else []
  if not obs or not obs[-1][3]:
    return None
  short = False
  for start in obs[-1][3]:
    chain, (tp, dp, vp) = [(obs[-1], start)], (obs[-1][0], start[0], start[2])
    for o in reversed(obs[:-1]):
      want = dp - vp * (tp - o[0])
      c = min(o[3], key=lambda x: abs(x[0] - want), default=None)
      if c is not None and abs(c[0] - want) <= RADAR_D_JUMP:
        chain.append((o, c))
        tp, dp, vp = o[0], c[0], c[2]
    if len(chain) < RADAR_MIN_OBS or chain[0][0][0] - chain[-1][0][0] < RADAR_SPAN_S:
      short = True
      continue
    t = np.array([o[0] for o, _ in chain])
    d = np.array([c[0] for _, c in chain])
    h = CAM_H + d * np.tan(np.radians([o[1] for o, _ in chain]))
    y = d * np.tan(np.radians([o[2] for o, _ in chain]))
    slope = float(np.polyfit(t - t.mean(), h, 1)[0])
    dm = float(d.mean())
    if abs(slope) <= RADAR_H_SLOPE and h.std() <= RADAR_H_STD + 0.01 * dm and y.std() <= RADAR_Y_STD + 0.01 * dm:
      return 'ride'
  return 'undecided' if short else None


def _chains(obs):
  """rides_vehicle's target chains: from each target at the newest observation back by range continuity."""
  for start in obs[-1][3]:
    chain, (tp, dp, vp) = [(obs[-1], start)], (obs[-1][0], start[0], start[2])
    for o in reversed(obs[:-1]):
      want = dp - vp * (tp - o[0])
      c = min(o[3], key=lambda x: abs(x[0] - want), default=None)
      if c is not None and abs(c[0] - want) <= RADAR_D_JUMP:
        chain.append((o, c))
        tp, dp, vp = o[0], c[0], c[2]
    yield chain


def fit_fixed_osm(t, az, el, ego, osm):
  """rms (deg) of the best fixed point whose range at t[-1] is within RADAR_OSM_WIN of osm (az, el rad)."""
  N, E, H = ego.poses(t)
  n1, e1, h1 = ego.pose(float(t[-1]))
  c1, s1 = math.cos(h1), math.sin(h1)
  best = float('inf')
  for x0 in np.arange(max(osm + RADAR_OSM_WIN[0], 15.), osm + RADAR_OSM_WIN[1] + 0.1, 2.5):
    yz = np.array([x0 * math.tan(az[-1]), -x0 * math.tan(el[-1])])
    for _ in range(4):                                     # Gauss-Newton on (y, z) at t[-1]
      w = np.array([n1 + c1 * x0 - s1 * yz[0], e1 + s1 * x0 + c1 * yz[0], yz[1]])
      f, J = _resid_jac(w, N, E, H, az, el)
      Jyz = np.stack([J[:, 0] * -s1 + J[:, 1] * c1, J[:, 2]], 1)
      try:
        yz = yz - np.linalg.lstsq(Jyz, f, rcond=None)[0]
      except np.linalg.LinAlgError:
        break
    w = np.array([n1 + c1 * x0 - s1 * yz[0], e1 + s1 * x0 + c1 * yz[0], yz[1]])
    f, _ = _resid_jac(w, N, E, H, az, el)
    if np.all(np.isfinite(f)):
      best = min(best, math.degrees(float(np.sqrt(np.mean(f ** 2)))))
  return best


def fit_ride(az, el, d, y_left):
  """rms (deg) of the best fixed offset (ox ahead, oy right, height <= RADAR_TOP) on the radar target (az, el rad)."""
  best = float('inf')
  for ox in np.arange(*RADAR_OX):
    r = d + ox
    oy = float(np.mean(np.tan(az) * r + y_left))
    hh = min(float(np.mean(np.tan(el) * r)), RADAR_TOP - CAM_H)
    if abs(oy) > RADAR_HALF_W + 1.:
      continue
    f = np.concatenate([np.arctan2(-y_left + oy, r) - az, np.arctan2(hh, r) - el])
    best = min(best, math.degrees(float(np.sqrt(np.mean(f ** 2)))))
  return best


def rides_fit(tr, ego, osm):
  """RADAR_FIT's test -> 'ride' / 'undecided' (no chain long enough) / None; tr.rfit = (rms_fixed, rms_ride) kept."""
  obs = [o for o in tr.rad if o[0] >= tr.rad[-1][0] - RADAR_WIN_S] if tr.rad else []
  tr.rfit = None
  if osm is None or osm <= FAR_X or not obs or not obs[-1][3]:
    return None                                  # near: OSM can still be the junction being crossed (1031_090536)
  short = False
  for chain in _chains(obs):
    if len(chain) < RADAR_MIN_OBS or chain[0][0][0] - chain[-1][0][0] < RADAR_SPAN_S:
      short = True
      continue
    chain = chain[::-1]
    t = np.array([o[0] for o, _ in chain])
    az = np.radians([o[2] for o, _ in chain])
    el = np.radians([o[1] for o, _ in chain])
    d = np.array([c[0] for _, c in chain])
    yl = np.array([c[1] for _, c in chain])
    # the fixed head at osm now = at osm + the distance driven since each observation: put osm at the chain's end
    rf = fit_fixed_osm(t, az, el, ego, osm)
    rr = fit_ride(az, el, d, yl)
    tr.rfit = (round(rf, 3), round(rr, 3))
    if rf >= RADAR_RMS_MIN and rf >= RADAR_RATIO * rr:
      return 'ride'
  return 'undecided' if short else None


# ---------------------------------------------------------------- 4 reading
def head_group(dets, seed, gap=1.6):
  grp = [seed]
  changed = True
  while changed:
    changed = False
    for d in dets:
      if d in grp:
        continue
      for g in grp:
        hh = max(g.h, d.h)
        if abs(d.v - g.v) < 0.7 * hh and (max(d.box[0], g.box[0]) - min(d.box[2], g.box[2])) < gap * hh:
          grp.append(d)
          changed = True
          break
  return grp


def reading(det, group=None, blink=None):
  """tltrack.reading (P3 final: RED_LAMP, LAMP_RED, LAMP_GO, MODEL_CONFLICT, HEAD_RED, ARROW_BLINK all on)."""
  r, g, y = det.p_red(), det.p_go(), det.p_yellow()
  lg = max(det.lamp.get(k, 0.) for k in GO_LAMPS) if det.lamp else 0.
  cg = max(det.cls.get('green', 0.), det.cls.get('green_arrow', 0.))
  lr, cr = det.lamp.get('red', None), det.cls.get('red', 0.)
  if lr is not None and ((lr >= 0.5 and lg < 0.3 and cr < 0.2 and cg >= 0.5) or
                         (cr >= 0.5 and cg < 0.3 and lr < 0.2 and lg >= 0.5)):
    return None
  red_head = (r >= 0.5 and (r >= g or det.lamp.get('red', 0.) >= 0.5)) or any(
      d is not det and d.p_red() >= 0.5 and d.p_red() >= d.p_go() for d in (group or []))
  if red_head and det.lamp and det.lamp.get('red', 0.) < 0.3 and not any(
      d is not det and d.lamp.get('red', 0.) >= 0.3 for d in (group or [])):
    return None
  if red_head:
    if blink is not None:
      bl, br = blink
      arrows = set()
      for d in (group or [det]):
        arrows.update(k for k in ('left', 'straight', 'right') if d.lamp.get(k, 0.) >= 0.5)
      if ('left' in arrows and bl) or ('right' in arrows and br) or ('straight' in arrows and not bl and not br):
        return 'go'
    return 'red'
  if g >= 0.5:
    if det.lamp and max(det.lamp.get(k, 0.) for k in GO_LAMPS) < 0.3:
      return None
    if blink is not None:
      hs = group or [det]
      ball = any(d.lamp.get('green', 0.) >= 0.5 or d.lamp.get('straight', 0.) >= 0.5 for d in hs)
      turns = {k for d in hs for k in ('left', 'right') if d.lamp.get(k, 0.) >= 0.5}
      if len(turns) == 1 and not ball:
        bl, br = blink
        return 'go' if ('left' in turns and bl) or ('right' in turns and br) else None
    return 'go'
  if y >= 0.5:
    return 'yellow'
  return None


def lit_cell(crop):
  """Where along a landscape head the brightest blob sits, 0 left .. 1 right (None: nothing lit). crop = RGB uint8 of
  the box (zero-padded outside the frame, as PIL's crop); luminance as PIL's convert('L')."""
  if crop is None or crop.size == 0:
    return None
  c = crop.astype(np.int64)
  p = ((c[..., 0] * 19595 + c[..., 1] * 38470 + c[..., 2] * 7471 + 0x8000) >> 16).astype(np.float32)
  if p.max() < 120:
    return None
  m = p >= max(120., 0.8 * p.max())
  xs = np.nonzero(m)[1]
  return float(xs.mean() / max(p.shape[1] - 1, 1))


def crop_box(rgb, r0, box):
  """PIL-style crop of a full-res box out of rows starting at r0 (outside the given rows / frame: zeros)."""
  x1, y1, x2, y2 = (int(round(v)) for v in box)
  if x2 <= x1 or y2 <= y1:
    return None
  out = np.zeros((y2 - y1, x2 - x1, 3), np.uint8)
  sy1, sy2 = max(y1 - r0, 0), min(y2 - r0, rgb.shape[0])
  sx1, sx2 = max(x1, 0), min(x2, rgb.shape[1])
  if sy2 > sy1 and sx2 > sx1:
    out[sy1 + r0 - y1:sy2 + r0 - y1, sx1 - x1:sx2 - x1] = rgb[sy1:sy2, sx1:sx2]
  return out


class Sequence:
  OK = {(None, 'red'), (None, 'go'), ('go', 'yellow'), ('yellow', 'red'), ('red', 'go'), ('go', 'red')}

  def __init__(self, n_go=2):
    self.call, self.n_go, self.pend, self.npend = None, n_go, None, 0
    self.red_cell = None
    self.pend_t = None

  def feed(self, r, cell=None, t=None):
    if r == 'red' and cell is not None:
      self.red_cell = cell if self.red_cell is None else 0.8 * self.red_cell + 0.2 * cell
    if r == 'yellow' and cell is not None and self.red_cell is not None and abs(cell - self.red_cell) < 0.2:
      r = 'red'
    if r is None and SEQ_GAP_S and self.pend == 'go' and t is not None and self.pend_t is not None and \
       t - self.pend_t <= SEQ_GAP_S:
      return self.call                           # an unread cycle between two greens (SEQ_GAP_S)
    if r is None or r == self.call:
      self.pend, self.npend = None, 0
      return self.call
    if (self.call, r) not in self.OK:
      return self.call
    need = self.n_go if r == 'go' else 1
    if self.pend != r:
      self.pend_t = t
    self.npend = self.npend + 1 if self.pend == r else 1
    self.pend = r
    if self.npend >= need:
      self.call, self.pend, self.npend = r, None, 0
    return self.call


# ---------------------------------------------------------------- 3 + 5: the follower
class Follower:
  """tltrack.follow() one frame at a time. step() -> dict for det.json 'trk' (and the replay)."""
  def __init__(self, radar_check=RADAR_CHECK):
    self.trk = Tracker()
    self.ours, self.seq, self.ours_world, self.lost_since = None, Sequence(), None, None
    self.i = 0
    self.radar_check = radar_check
    self.last_t = None
    self.warm = None                             # SEQ_WARM_S: the new lock's reads before it, until its first read
    self.passing = None                          # PASS_BLOCK_M: (world point, t, its call) of ours gone over the top

  def reset(self):
    self.__init__(self.radar_check)

  def step(self, t, dets, ego, osm=None, blink=None, radar=None, crop=None, lamp_fn=None):
    """t: frame time (ego's clock); dets: [Det]; ego: Ego (cam attached as ego.cam); osm: tlwatchd's OSM distance
    ahead or None - used only <= OSM_MAX, as the analysis had it (tlwatchd's approach log); blink (left, right);
    radar: [(dRel, yRel, vRel)] or None; crop(box) -> RGB of the box (lit_cell); lamp_fn(box) -> the lamp model's
    read of a box: the head we read gets one if lamp_idx left it out (its LAMP_GO / LAMP_RED checks need it)."""
    if self.last_t is not None and t - self.last_t > 5.:
      self.reset()                               # a gap (C3 was idle / not reading): start over
    self.last_t = t
    self.i += 1
    self.trk.step(t, self.i, dets, ego)
    return self._decide(t, self.i, dets, ego, osm, blink, radar, crop, lamp_fn)

  def step_local(self, t, found, dets, ego, osm=None, blink=None, radar=None, crop=None, lamp_fn=None):
    """A local pass (TASK3) at t: found {track id: Det (local=True)} - the followed heads re-found in their crops - as
    those tracks' next observation; the others are left as they are (no miss counted: they were not looked for).
    dets: every box the pass found (head_group for ours). Then the same refit / lock / read as step(); ours is read
    only when its local Det has a lamp-model read (LOCAL_READ)."""
    if self.last_t is None or t <= self.last_t or t - self.last_t > 5.:
      return None                                # out of order / after a gap: the finder cycle comes first
    self.last_t = t
    self.i += 1
    for tr in self.trk.tracks:
      d = found.get(tr.id)
      if d is not None:
        tr.add(t, self.i, d)
        tr.misses = 0
    return self._decide(t, self.i, dets, ego, osm, blink, radar, crop, lamp_fn, local=True)

  def local_targets(self, t, ego, n_max=LOCAL_MAX * 3):
    """The tracks a local pass looks for at t, most wanted first: ours, then head-like tracks seen lately in front
    (landscape before upright, nearer straight ahead first) -> [(track, u, v)] predicted centre (full res)."""
    out = []
    for tr in self.trk.tracks:
      if tr.misses > 1 or t - tr.obs[-1][0] > LOCAL_AGE_S:
        continue
      d, fl = tr.last, tr.flast
      if d.el < LAMP_EL_MIN or abs(d.az) > LAMP_AZ:
        continue
      fo = tr.fobs[-8:]
      if max(np.median([o[2].p_vehicle() for o in fo]), fl.p_red(), fl.p_go()) < 0.3 or \
         np.median([o[2].p_ped() for o in fo]) >= 0.5:
        continue
      if tr.fix is not None and tr.fix.get('ok') and ego.to_car(tr.fix['world'], t)[0] < 2.:
        continue                                 # passed under it
      pu, pv = tr.predict(t, ego)
      if not (-LOCAL_EDGE < pu < 2 * CX + LOCAL_EDGE and -LOCAL_EDGE < pv < 2 * CY + LOCAL_EDGE):
        continue
      upright = np.median([o[2].h > 1.3 * o[2].w for o in fo[-6:]]) > 0.5
      out.append(((tr is not self.ours, upright, abs(d.az)), tr, pu, pv))
    out.sort(key=lambda x: x[0])
    return [(tr, pu, pv) for _, tr, pu, pv in out[:n_max]]

  def _decide(self, t, i, dets, ego, osm, blink, radar, crop, lamp_fn, local=False):
    v = ego.speed(t)
    osm_now = osm if (osm is not None and osm <= OSM_MAX) else None
    trk = self.trk
    for tr in trk.tracks:
      short = len(tr.obs) < 4 and not (FIT_SPAN_S and len(tr.obs) == 3 and tr.obs[-1][0] - tr.obs[0][0] >= FIT_SPAN_S)
      if short or tr.misses or tr.obs[-1][1] != i or (tr.fix is not None and len(tr.obs) % REFIT_EVERY):
        continue
      fl = tr.flast
      if tr.last.el < -1. or max(np.median([o[2].p_vehicle() for o in tr.fobs[-8:]]), fl.p_red(), fl.p_go()) < 0.3:
        continue
      tr.fix = triangulate(tr, ego, warm=tr.fix['world'] if tr.fix is not None and tr.fix['ok'] else None)
    if self.radar_check:
      for tr in trk.tracks:
        if not tr.misses and tr.obs[-1][1] == i:
          tr.rad = [o for o in tr.rad if o[0] >= t - RADAR_WIN_S] + [(t, tr.last.el, tr.last.az, near_targets(tr.last, radar, v))]
          tr.rides = rides_fit(tr, ego, osm_now) if RADAR_FIT else rides_vehicle(tr)
    ours = self.ours
    no_osm = osm_now is None and v >= MOVING
    if no_osm:
      ours = None
    gone_over = False
    if ours is not None and ours in trk.tracks and ours.obs[-1][1] != i and t - ours.obs[-1][0] > LOST_HOLD_S and \
       ours.fix is not None and ours.fix.get('ok'):
      px = ego.cam.pix(ego.to_car(ours.fix['world'], t))
      if px is None or px[1] < TOP_EXIT_PX:     # unseen LOST_HOLD_S while out over the top (below): passed under it
        self.passing = (ours.fix['world'], t, self.seq.call) if PASS_BLOCK_M else None
        ours, gone_over = None, True
    if ours is not None and (ours not in trk.tracks or (ours.fix and ego.to_car(ours.fix['world'], t)[0] < 2.)):
      # lost for good, or passed under it. Out over the top of the picture / passed: that junction is being crossed and
      # its call says nothing about the next one - no LOST_HOLD_S of it (11/03 tl-onboard: 1029_172749 held the
      # overhead green 1 s into the next junction's red, with the live pose a beat later to lock the next head)
      gone_over = ours.last.box[1] < TOP_EXIT_PX or (ours.fix is not None and ego.to_car(ours.fix['world'], t)[0] < 2.)
      if not gone_over and ours.fix is not None and ours.fix.get('ok'):
        # lost where its fixed point is already out over the top of the picture: passed under it unseen, as
        # TOP_EXIT_PX (TASK3: 1031_091150 at a 0.455 / 0.55 / 0.7 s cycle - the near junction's low head left the top
        # between two finder frames, LOST_HOLD_S then carried its green into the next junction's red)
        px = ego.cam.pix(ego.to_car(ours.fix['world'], t))
        gone_over = px is None or px[1] < TOP_EXIT_PX
      if gone_over and PASS_BLOCK_M and ours.fix is not None and ours.fix.get('ok'):
        self.passing = (ours.fix['world'], t, self.seq.call)
      ours = None
    x_pass = None
    if self.passing is not None:
      x_pass = ego.to_car(self.passing[0], t)[0]
      if x_pass < 2. or t - self.passing[1] > PASS_BLOCK_S or ours is not None:
        self.passing, x_pass = None, None
    if ours is not None and self.radar_check and ours.rides == 'ride':
      ours = None                                # it rides on a moving vehicle after all
    for tr in (trk.tracks if SEQ_WARM_S else []):
      if tr.obs[-1][1] == i and tr is not ours:
        d = tr.obs[-1][2]
        rc = None
        if d.lamp and not d.local and d.p_vehicle() >= 0.5 and d.p_ped() < 0.5:
          rc = reading(d, head_group(dets, d) if d in dets else [d], blink)
        tr.rds = [x for x in tr.rds if x[0] >= t - SEQ_WARM_S] + [(t, rc)]
    cands = []
    for tr in ([] if no_osm else trk.tracks):
      if tr.misses > 1 or tr.fix is None:
        continue
      if tr.fix['ok'] and head_ok(tr, tr.fix):
        p = ego.to_car(tr.fix['world'], t)
      elif osm_now is not None and fixed_far(tr, osm_now):
        d = tr.last
        p = np.array([osm_now, osm_now * math.tan(math.radians(d.az)), 0.])
      else:
        continue
      lane_far = LANE_Y_FAR if (radar is None or not self.radar_check) else 0.
      if p[0] < 3. or abs(p[1]) > LANE_Y or (lane_far and p[0] > FAR_X and abs(p[1]) > lane_far) or tr.last.box[1] < 4:
        continue
      if osm_now is not None and not osm_now - OSM_SHORT <= p[0] <= osm_now + OSM_PAST:
        continue
      if x_pass is not None and tr.fix['ok'] and p[0] < x_pass + PASS_BLOCK_M and \
         (not PASS_GO_RED or (self.passing[2] == 'go' and
                              (tr.last.p_red() >= 0.5 or any(x[1] == 'red' for x in getattr(tr, 'rds', []))))):
        continue                                 # the junction being passed: not ours any more, nor its side heads
                                                 # (its own fit's range; a fixed_far one sits at OSM's, not its own)
      if self.radar_check and (tr.rides == 'ride' or (RADAR_HOLD_UNDECIDED and tr.rides == 'undecided' and tr is not ours
                                                       and not (tr.last.p_go() >= 0.5 and tr.last.p_red() < 0.5))):
        continue                                 # on a moving vehicle / maybe (unless lit green - no vehicle's lamp is):
                                                 # no new lock until it is told
      upright = np.median([o[2].h > 1.3 * o[2].w for o in tr.fobs[-6:]]) > 0.5
      if upright and UPRIGHT_SIDE_Y and abs(p[1]) > UPRIGHT_SIDE_Y:
        continue
      tr.cand_world = ego.to_world(p, t)
      cands.append((abs(p[1]) / 3. + p[0] / 40. + (UPRIGHT_COST if upright else 0.), p[0], tr))
    if cands:
      cost, x, tr = min(cands, key=lambda c: c[0])
      w_ours = None
      if ours is not None and ours.fix and ours.fix['ok']:
        w_ours = ours.fix['world']
      elif KEEP_OURS and ours is not None and ours.misses <= 1:
        # its own fit even if not ok (a fixed_far candidate sat at OSM's distance: maybe the next junction's), else
        # where it was the last time it was a candidate (KEEP_OURS)
        w_ours = ours.fix['world'] if (ours.fix is not None and ours.fix['x'] > 0) else ours.cand_world
      if w_ours is not None:
        x_ours = ego.to_car(w_ours, t)[0]
        near = min(cands, key=lambda c: c[1] + (1e3 if c[0] - abs(c[1]) / 40. >= UPRIGHT_COST else 0.))
        if near[2] is not ours and near[1] < x_ours - SWITCH_NEARER_M:
          tr = near[2]
        else:
          tr = ours
      if tr is not ours:
        same = self.ours_world is not None and np.linalg.norm(tr.fix['world'][:2] - self.ours_world[:2]) < 3.
        if not same:
          self.seq = Sequence()
          self.warm = [x for x in tr.rds if x[0] < t] if SEQ_WARM_S else None
        ours = tr
    self.ours = ours
    if ours is not None and ours.fix and ours.fix['ok']:
      self.ours_world = ours.fix['world']
    rd, cell, pos, box = None, None, None, None
    if ours is not None:
      if ours.obs[-1][1] == i:
        d = ours.obs[-1][2]
        box = d.box
        if not d.lamp and lamp_fn is not None:
          d.lamp = lamp_fn(d.box) or {}
        rd = reading(d, head_group(dets, d) if d in dets else [d], blink)
        if d.p_vehicle() < 0.5 or d.p_ped() >= 0.5 or (d.local and not d.lamp):
          rd = None                              # a local find without the lamp model: position only
        if d.landscape() and rd in ('red', 'yellow') and crop is not None:
          cell = lit_cell(crop(d.box))
      if ours.fix:
        p = ego.to_car(ours.fix['world'], t)
        pos = (round(float(p[0]), 1), round(float(p[1]), 1), round(ours.fix['height'], 1))
    # a local pass without a read of ours says nothing about the light: not fed (a None would also cut short the two
    # greens in a row a go needs)
    if self.warm is not None and not (local and rd is None):
      if rd is not None:                         # SEQ_WARM_S: the same read in a row just before the lock counts
        n = 0
        while n < len(self.warm) and self.warm[-1 - n][1] == rd:
          n += 1
        for th, rc in self.warm[len(self.warm) - n:]:
          self.seq.feed(rc, None, th)
      self.warm = None
    call = self.seq.call if (local and rd is None) else self.seq.feed(rd, cell, t)
    if ours is not None:
      self.lost_since = None
    else:
      self.lost_since = self.lost_since if self.lost_since is not None else t
      if t - self.lost_since > LOST_HOLD_S or gone_over:
        self.seq, call = Sequence(), None
    return {'ours': ours.id if ours else None, 'pos': pos, 'read': rd, 'call': call, 'box': box,
            'az': round(ours.last.az, 1) if ours else None, 'el': round(ours.last.el, 1) if ours else None,
            'n_trk': len(trk.tracks), 'osm': osm_now}

  def ours_near(self, t, ego, boxes):
    """Indices of this cycle's boxes the locked head may be (within its tracker gate of the prediction) - lamp_idx's
    must, so the head we read always gets the lamp model even when bigger boxes fill LAMP_MAX."""
    tr = self.ours
    if tr is None:
      return []
    pu, pv = tr.predict(t, ego)
    gate = max(1.2 * max(tr.last.w, tr.last.h), 12.) * (1 + tr.misses) + (25. if ego.speed(t) > 0.5 else 0.)
    return [i for i, b in enumerate(boxes) if math.hypot((b[0] + b[2]) / 2 - pu, (b[1] + b[3]) / 2 - pv) <= gate]
