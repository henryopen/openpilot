#!/usr/bin/env python3
"""Live road conditions ahead from TDX (the transport ministry's open data) for the HUD and the rear LED.

DISPLAY ONLY - the driver (2026-10-18): this can only be shown, never act on the car. Nothing here is read
by the planner or anything else that controls; it writes STATE and the HUD server passes it on.

The driver asked for three of TDX's live feeds once his member key existed (2026-10-18): road events
(accidents, works, closed lanes), live section speeds, and on the freeways the vehicle detectors' lane
speeds. Fetched when the device happens to be online, skipped quietly when it is not ("上車有網路就抓沒網路
就不要抓" - he did not want the network checked or managed).

The member key allows 5 calls a minute, so:
- the section shapes and names, the freeway detector positions and the county outlines are fetched while
  parked (offroad, usually on the office WiFi) and cached in CACHE for a day (the outlines for a month);
- on the road one round a minute of at most three calls, for whatever we are on: the freeway's sections,
  events and detectors; the provincial roads' sections and events; or the county's sections (TDX has them
  for 10 counties, not New Taipei) and events.

"Ahead" is followed along TDX's own section polylines, which run in the direction of travel: the section
under us (within ON_SECTION_M, heading within HEADING_DEG), then whichever section starts where it ends,
up to PATH_M. Events and detectors count when they lie on that path (and, where TDX gives a direction,
it is ours). Off the sections (county roads without live data) an event counts only when it is on the road
mapd says we are on and straight ahead.

The credentials are in CRED on the device (not in git); /data/tdx_traffic_off stops it.
"""
import gzip
import json
import math
import os
import re
import ssl
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from datetime import UTC, datetime

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

CRED = "/data/tdx_credentials.json"
CACHE = "/data/tdx_cache"
STATE = "/dev/shm/tdx_traffic.json"
OFF_FLAG = "/data/tdx_traffic_off"
API = "https://tdx.transportdata.tw/api/basic/"
TOKEN_URL = "https://tdx.transportdata.tw/auth/realms/TDXConnect/protocol/openid-connect/token"
UA = "Mozilla/5.0 (openpilot c4 tdx_traffic)"   # without one the WAF answers with an HTML rejection page

CALLS_PER_MIN = 4       # TDX member key: 5 a minute; one spare
LIVE_EVERY = 60.        # s between rounds of live calls
STATIC_MAX_AGE = 86400.         # s: section shapes / names, detector positions
CITY_MAX_AGE = 30 * 86400.      # s: county outlines
LIVE_MAX_AGE = 15 * 60.         # s: a live reading older than this (by its own collection time) is dropped
ON_SECTION_M = 30.
HEADING_DEG = 45.
CHAIN_M = 120.          # the next section starts within this of where this one ends (freeway: 90% within 82 m)
CHAIN_DEG = 60.
END_HEADING_M = 50.
PATH_M = 5000.          # how far ahead the path is followed
AHEAD_M = 3000.         # how far ahead anything is reported
EVENT_PATH_M = 60.
VD_PATH_M = 40.
VD_MIN_M = 150.         # a detector closer than this is where we already are
STRAIGHT_M, STRAIGHT_LAT_M = 1500., 40.   # off the sections: events straight ahead on our road
SLOW_KPH = {"Freeway": 40, "Highway": 30, "City": 15}   # a section ahead this slow is worth saying
SLOW_MARGIN_KPH = 15    # ... and this much slower than the one we are on
MIN_HEADING_SPEED = 3.  # m/s; standing still the GPS heading means nothing
GPS_FRESH = 5.          # s
WRITE_EVERY = 2.        # s

# where TDX has what (its swagger enums, 2026-10-18)
LIVE_CITIES = {"YilanCounty", "ChanghuaCounty", "YunlinCounty", "PingtungCounty", "Keelung", "Taipei", "Kaohsiung",
               "Taichung", "Tainan", "Taoyuan"}
EVENT_CITIES = {"Taipei", "NewTaipei", "Taoyuan", "Taichung", "Tainan", "Kaohsiung", "Keelung", "MiaoliCounty",
                "ChiayiCounty", "PingtungCounty", "YilanCounty", "KinmenCounty"}
EVENT_KIND = {1: "事故", 2: "施工", 3: "壅塞", 4: "特殊事件", 5: "管制", 6: "災變"}
COMPASS = {"N": 0, "NE": 45, "E": 90, "SE": 135, "S": 180, "SW": 225, "W": 270, "NW": 315}
DIR_ZH = {"北向": 0, "東北向": 45, "東向": 90, "東南向": 135, "南向": 180, "西南向": 225, "西向": 270, "西北向": 315}

KX, KY = 111320. * math.cos(math.radians(23.7)), 110540.   # deg -> m, near enough anywhere in Taiwan
GRID = 500.             # m, spatial index cell


def xy(lat, lon):
  return lon * KX, lat * KY


def angdiff(a, b):
  return abs((a - b + 180.) % 360. - 180.)


def heading_of(dx, dy):
  return math.degrees(math.atan2(dx, dy)) % 360.


_NUM = re.compile(r"(-?\d+(?:\.\d+)?)\s+(-?\d+(?:\.\d+)?)")


def wkt_points(s):
  """[(lon, lat), ...] from a POINT / LINESTRING, in order."""
  return [(float(a), float(b)) for a, b in _NUM.findall(s or "")]


def wkt_rings(s):
  """The rings of a (MULTI)POLYGON, each [(lon, lat), ...]."""
  body = (s or "").replace("(", " ( ").replace(")", " ) ")
  rings, cur, depth = [], None, 0
  for tok in re.split(r"\s*,\s*|\s+", body):
    if tok == "(":
      depth += 1
      cur = []
    elif tok == ")":
      if cur:
        rings.append(cur)
      cur = None
      depth -= 1
    elif cur is not None and tok:
      cur.append(tok)
  out = []
  for r in rings:
    pts = [(float(r[i]), float(r[i + 1])) for i in range(0, len(r) - 1, 2)]
    if len(pts) >= 3:
      out.append(pts)
  return out


def thin(pts, step):
  """Every point at least step apart (in the units of pts), keeping the last."""
  if len(pts) < 3:
    return pts
  out = [pts[0]]
  for p in pts[1:-1]:
    if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= step:
      out.append(p)
  out.append(pts[-1])
  return out


def parse_time(s):
  try:
    return datetime.fromisoformat(s).timestamp()
  except (TypeError, ValueError):
    return None


def wall_now():
  """Seconds since the epoch: TDX stamps its data with the wall clock, so that is the clock to compare with."""
  return datetime.now(UTC).timestamp()


def wall_ok():
  """The device clock is only trusted once it has been set (it boots in the past)."""
  return datetime.now(UTC).year >= 2025


# ------------------------------------------------------------------ TDX client
class Tdx:
  def __init__(self, cred):
    self.cred = cred
    self.tok, self.tok_until = None, 0.
    self.calls = deque()
    self.blocked_until = 0.
    self.ctx = ssl.create_default_context()
    self.ctx.verify_flags &= ~ssl.VERIFY_X509_STRICT   # TDX's chain lacks a key identifier newer Pythons insist on

  def _token(self):
    if self.tok and time.monotonic() < self.tok_until - 60:
      return self.tok
    body = urllib.parse.urlencode({"grant_type": "client_credentials", **self.cred}).encode()
    req = urllib.request.Request(TOKEN_URL, body, {"Content-Type": "application/x-www-form-urlencoded", "User-Agent": UA})
    with urllib.request.urlopen(req, timeout=10, context=self.ctx) as r:
      j = json.load(r)
    self.tok, self.tok_until = j["access_token"], time.monotonic() + float(j.get("expires_in", 3600))
    return self.tok

  def budget(self):
    now = time.monotonic()
    while self.calls and now - self.calls[0] > 60.:
      self.calls.popleft()
    return 0 if now < self.blocked_until else CALLS_PER_MIN - len(self.calls)

  def get(self, path, timeout=60, **q):
    q.setdefault("$format", "JSON")
    url = API + urllib.parse.quote(path) + "?" + urllib.parse.urlencode(q)
    req = urllib.request.Request(url, headers={"authorization": "Bearer " + self._token(), "User-Agent": UA,
                                               "Accept-Encoding": "gzip"})
    self.calls.append(time.monotonic())
    try:
      with urllib.request.urlopen(req, timeout=timeout, context=self.ctx) as r:
        b = r.read()
        if r.headers.get("Content-Encoding") == "gzip":
          b = gzip.decompress(b)
    except urllib.error.HTTPError as e:
      if e.code == 429:
        self.blocked_until = time.monotonic() + float(e.headers.get("Retry-After") or 60)
      if e.code == 401:
        self.tok = None
      raise
    return json.loads(b)


def items(j):
  """TDX wraps lists in an object with update times; the list is the one list in it."""
  if isinstance(j, list):
    return j
  return next((v for v in j.values() if isinstance(v, list)), []) if isinstance(j, dict) else []


# ------------------------------------------------------------------ cache
def cache_path(name):
  return os.path.join(CACHE, name + ".json")


def cache_load(name, max_age):
  p = cache_path(name)
  try:
    if wall_now() - os.path.getmtime(p) > max_age and wall_ok():
      return None, True
    with open(p, encoding="utf-8") as f:
      return json.load(f), False
  except (OSError, ValueError):
    return None, True


def cache_save(name, obj):
  os.makedirs(CACHE, exist_ok=True)
  tmp = cache_path(name) + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(obj, f, ensure_ascii=False, separators=(",", ":"))
  os.replace(tmp, cache_path(name))


# what each cached thing is fetched from, and how it is shrunk before it is kept
def _shapes(j):
  return {s["SectionID"]: [[round(la, 6), round(lo, 6)] for lo, la in thin(wkt_points(s.get("Geometry")), 0.0001)]
          for s in items(j) if s.get("Geometry")}


def _sections(j):
  return {s["SectionID"]: {"name": s.get("SectionName") or s.get("RoadName") or "", "road": s.get("RoadName") or ""}
          for s in items(j)}


def _vds(j):
  out = {}
  for v in items(j):
    links = v.get("DetectionLinks") or [{}]
    out[v["VDID"]] = {"lat": v.get("PositionLat"), "lon": v.get("PositionLon"), "road": v.get("RoadName") or "",
                      "bearing": COMPASS.get(links[0].get("Bearing") or "", None)}
  return out


def _cities(j):
  out = []
  for c in items(j):
    rings = [[[round(la, 5), round(lo, 5)] for lo, la in thin(r, 0.003)] for r in wkt_rings(c.get("Geometry"))]
    rings = [r for r in rings if len(r) >= 3]
    if rings:
      lats = [p[0] for r in rings for p in r]
      lons = [p[1] for r in rings for p in r]
      out.append({"city": c.get("City"), "name": c.get("CityName"), "bb": [min(lats), min(lons), max(lats), max(lons)],
                  "rings": rings})
  return out


def static_specs(auth):
  """(cache name, API path, shrink, max age) for an authority's static data."""
  if auth == "Freeway":
    return [("shape_Freeway", "v2/Road/Traffic/SectionShape/Freeway", _shapes, STATIC_MAX_AGE),
            ("sect_Freeway", "v2/Road/Traffic/Section/Freeway", _sections, STATIC_MAX_AGE),
            ("vd_Freeway", "v2/Road/Traffic/VD/Freeway", _vds, STATIC_MAX_AGE)]
  if auth == "Highway":
    return [("shape_Highway", "v2/Road/Traffic/SectionShape/Highway", _shapes, STATIC_MAX_AGE),
            ("sect_Highway", "v2/Road/Traffic/Section/Highway", _sections, STATIC_MAX_AGE)]
  city = auth.split(":", 1)[1]
  return [(f"shape_{city}", f"v2/Road/Traffic/SectionShape/City/{city}", _shapes, STATIC_MAX_AGE),
          (f"sect_{city}", f"v2/Road/Traffic/Section/City/{city}", _sections, STATIC_MAX_AGE)]


CITY_SPEC = ("cities", "V3/Map/District/Boundary/City", _cities, CITY_MAX_AGE)


def live_path(auth):
  return "v2/Road/Traffic/Live/" + ("City/" + auth.split(":", 1)[1] if auth.startswith("City:") else auth)


def event_path(auth):
  return "v1/Traffic/RoadEvent/LiveEvent/" + ("City/" + auth.split(":", 1)[1] if auth.startswith("City:") else auth)


# ------------------------------------------------------------------ geometry on the sections
class Sections:
  """One authority's sections: polylines in local metres, indexed on a grid, for finding ourselves and what follows."""

  def __init__(self, shapes, names):
    self.pts = {}
    self.cum = {}
    self.names = names or {}
    self.grid = {}
    self.starts = {}
    for sid, ll in shapes.items():
      p = [xy(la, lo) for la, lo in ll]
      if len(p) < 2:
        continue
      c = [0.]
      for a, b in zip(p, p[1:], strict=False):
        c.append(c[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
      self.pts[sid], self.cum[sid] = p, c
      for i in range(len(p) - 1):
        for cell in self._cells(p[i], p[i + 1]):
          self.grid.setdefault(cell, set()).add((sid, i))
      self.starts.setdefault((int(p[0][0] // GRID), int(p[0][1] // GRID)), []).append(sid)

  @staticmethod
  def _cells(a, b):
    x0, x1 = sorted((a[0], b[0]))
    y0, y1 = sorted((a[1], b[1]))
    return {(i, j) for i in range(int(x0 // GRID), int(x1 // GRID) + 1) for j in range(int(y0 // GRID), int(y1 // GRID) + 1)}

  def _near(self, x, y):
    i, j = int(x // GRID), int(y // GRID)
    out = set()
    for di in (-1, 0, 1):
      for dj in (-1, 0, 1):
        out |= self.grid.get((i + di, j + dj), set())
    return out

  def _seg_hit(self, sid, i, x, y):
    (ax, ay), (bx, by) = self.pts[sid][i], self.pts[sid][i + 1]
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    t = 0. if ll == 0 else min(1., max(0., ((x - ax) * dx + (y - ay) * dy) / ll))
    px, py = ax + t * dx, ay + t * dy
    return math.hypot(x - px, y - py), self.cum[sid][i] + t * math.sqrt(ll), heading_of(dx, dy)

  def locate(self, x, y, hd):
    """(section, metres along it) of the section under us going our way, or None."""
    best = None
    for sid, i in self._near(x, y):
      d, s, h = self._seg_hit(sid, i, x, y)
      if d <= ON_SECTION_M and angdiff(h, hd) <= HEADING_DEG and (best is None or d < best[0]):
        best = (d, sid, s)
    return None if best is None else (best[1], best[2])

  def _end_heading(self, sid, start):
    """Heading over the first / last END_HEADING_M of the section: its very first or last little segment, at an
    interchange, can point anywhere."""
    p, c = self.pts[sid], self.cum[sid]
    if start:
      k = next((i for i in range(1, len(p)) if c[i] >= END_HEADING_M), len(p) - 1)
      a, b = p[0], p[k]
    else:
      k = next((i for i in range(len(p) - 2, -1, -1) if c[-1] - c[i] >= END_HEADING_M), 0)
      a, b = p[k], p[-1]
    return heading_of(b[0] - a[0], b[1] - a[1])

  def _next(self, sid, seen):
    ex, ey = self.pts[sid][-1]
    he = self._end_heading(sid, False)
    i, j = int(ex // GRID), int(ey // GRID)
    road = self.names.get(sid, {}).get("road", "")
    best = None
    for di in (-1, 0, 1):
      for dj in (-1, 0, 1):
        for s2 in self.starts.get((i + di, j + dj), []):
          if s2 in seen:
            continue
          sx, sy = self.pts[s2][0]
          if math.hypot(sx - ex, sy - ey) > CHAIN_M:
            continue
          turn = angdiff(self._end_heading(s2, True), he)
          if turn > CHAIN_DEG:
            continue
          key = (self.names.get(s2, {}).get("road", "") != road, turn)   # same road first, then the straightest
          if best is None or key < best[0]:
            best = (key, s2)
    return None if best is None else best[1]

  def path(self, sid, along):
    """What lies ahead from `along` metres into `sid`: [(section, metres from us to its start, to its end)]
    and the polyline [(x, y, metres from us)]."""
    secs, line = [], []
    c, p = self.cum[sid], self.pts[sid]
    secs.append((sid, -along, c[-1] - along))
    for k in range(len(p)):
      if c[k] >= along:
        line.append((p[k][0], p[k][1], c[k] - along))
    off = c[-1] - along
    seen = {sid}
    cur = sid
    while off < PATH_M:
      nxt = self._next(cur, seen)
      if nxt is None:
        break
      seen.add(nxt)
      cn, pn = self.cum[nxt], self.pts[nxt]
      secs.append((nxt, off, off + cn[-1]))
      line += [(q[0], q[1], off + cq) for q, cq in zip(pn, cn, strict=False)]
      off += cn[-1]
      cur = nxt
    return secs, line


def on_path(line, x, y, max_d):
  """Metres along the path to the point nearest (x, y), if within max_d of it, else None."""
  best = None
  for (ax, ay, sa), (bx, by, sb) in zip(line, line[1:], strict=False):
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    t = 0. if ll == 0 else min(1., max(0., ((x - ax) * dx + (y - ay) * dy) / ll))
    d = math.hypot(x - ax - t * dx, y - ay - t * dy)
    if d <= max_d and (best is None or d < best[0]):
      best = (d, sa + t * (sb - sa))
  return None if best is None else best[1]


def city_of(cities, lat, lon):
  for c in cities or []:
    la1, lo1, la2, lo2 = c["bb"]
    if not (la1 <= lat <= la2 and lo1 <= lon <= lo2):
      continue
    inside = False
    for r in c["rings"]:
      for (ya, xa), (yb, xb) in zip(r, r[1:] + r[:1], strict=False):
        if (ya > lat) != (yb > lat) and lon < xa + (lat - ya) * (xb - xa) / (yb - ya):
          inside = not inside
    if inside:
      return c["city"]
  return None


# ------------------------------------------------------------------ reading the live feeds
def live_speeds(j, now_wall):
  out = {}
  for s in items(j):
    t = parse_time(s.get("DataCollectTime"))
    if now_wall and t and now_wall - t > LIVE_MAX_AGE:
      continue
    sp = s.get("TravelSpeed")
    if isinstance(sp, (int, float)) and sp > 0:
      out[s["SectionID"]] = int(sp)
  return out


def live_vds(j, now_wall):
  out = {}
  for v in items(j):
    if v.get("Status") != 0:
      continue
    t = parse_time(v.get("DataCollectTime"))
    if now_wall and t and now_wall - t > LIVE_MAX_AGE:
      continue
    lanes = []
    for lf in v.get("LinkFlows") or []:
      for ln in lf.get("Lanes") or []:
        sp = ln.get("Speed")
        if isinstance(sp, (int, float)) and sp > 0:
          lanes.append((ln.get("LaneID", 0), int(sp)))
    if lanes:
      out[v["VDID"]] = [s for _, s in sorted(lanes)]
  return out


def _event_roads(loc):
  roads = []
  fe = loc.get("FreeExpressHighway") or {}
  if fe.get("Road"):
    roads.append(fe["Road"])
  for rw in (loc.get("CityRoad") or {}).get("Roadways") or []:
    if rw.get("Road"):
      roads.append(rw["Road"])
  for r in (loc.get("Intersection") or {}).get("Roads") or []:
    if isinstance(r, dict) and r.get("Road"):
      roads.append(r["Road"])
    elif isinstance(r, str):
      roads.append(r)
  if (loc.get("Address") or {}).get("Road"):
    roads.append(loc["Address"]["Road"])
  return roads


def live_events(j, now_wall):
  out = []
  for e in items(j):
    if now_wall:
      exp = parse_time(e.get("ExpireTime"))
      eff = parse_time(e.get("EffectiveTime"))
      if (exp and exp < now_wall) or (eff and eff > now_wall):
        continue
    pts = wkt_points(e.get("Positions"))
    if not pts:
      continue
    loc = e.get("Location") or {}
    dirn = (loc.get("FreeExpressHighway") or {}).get("Direction") or ""
    imp = e.get("Impact") or {}
    text = (imp.get("Description") or "").strip()
    out.append({"lat": pts[0][1], "lon": pts[0][0], "kind": EVENT_KIND.get(e.get("EventType"), "狀況"),
                "title": (e.get("EventTitle") or "").strip(), "text": text if len(text) <= 16 else "",
                "dir": DIR_ZH.get(dirn), "roads": _event_roads(loc), "id": e.get("EventID")})
  return out


# ------------------------------------------------------------------ the daemon
class TrafficAhead:
  def __init__(self):
    self.tdx = None
    self.cred_mtime = None
    self.sections = {}        # auth -> Sections
    self.vds = None
    self.cities = None
    self.live = {}            # auth -> (monotonic fetched, {section: kph})
    self.vd_live = (0., {})
    self.events = {}          # auth -> (monotonic fetched, [event])
    self.last_round = -1e9
    self.online = None
    self.fail_until = 0.

  # --- credentials and network
  def _client(self):
    try:
      m = os.path.getmtime(CRED)
    except OSError:
      self.tdx = None
      return None
    if self.tdx is None or m != self.cred_mtime:
      with open(CRED) as f:
        c = json.load(f)
      self.tdx, self.cred_mtime = Tdx({"client_id": c["client_id"], "client_secret": c["client_secret"]}), m
    return self.tdx

  def _call(self, path, **q):
    """One API call, or None when offline / out of budget. Failures back off a minute."""
    tdx = self._client()
    if tdx is None or time.monotonic() < self.fail_until or tdx.budget() <= 0:
      return None
    try:
      j = tdx.get(path, **q)
      if self.online is not True:
        cloudlog.info("tdx_traffic: online")
      self.online = True
      return j
    except (urllib.error.URLError, OSError, ValueError, KeyError) as e:
      if self.online is not False:
        cloudlog.info("tdx_traffic: offline or failing (%s)", e)
      self.online = False
      self.fail_until = time.monotonic() + 60.
      return None

  # --- static data
  def _static(self, spec, fetch):
    """(object, fetched just now) - from the cache, or from TDX when the cache is stale and fetching is allowed."""
    name, path, shrink, max_age = spec
    obj, stale = cache_load(name, max_age)
    if stale and fetch:
      j = self._call(path)
      if j is not None:
        obj = shrink(j)
        cache_save(name, obj)
        return obj, True
    return obj, False

  def ensure(self, auth, fetch):
    """The authority's sections (loaded from cache, fetched if stale and allowed); rebuilt only when new."""
    specs = static_specs(auth)
    have = auth in self.sections
    if have and not fetch:
      return self.sections[auth]
    shapes, new_s = self._static(specs[0], fetch)
    names, new_n = self._static(specs[1], fetch and shapes is not None)
    if auth == "Freeway":
      v, new_v = self._static(specs[2], fetch and shapes is not None)
      if v is not None and (self.vds is None or new_v):
        self.vds = {k: dict(d, xy=xy(d["lat"], d["lon"])) for k, d in v.items() if d.get("lat")}
    if shapes and (not have or new_s or new_n):
      self.sections[auth] = Sections(shapes, names)
    return self.sections.get(auth)

  def prefetch(self):
    """Parked: keep the freeway / provincial road sections, the detectors and the county outlines fresh."""
    if self.cities is None or cache_load(CITY_SPEC[0], CITY_SPEC[3])[1]:
      self.cities = self._static(CITY_SPEC, True)[0]
    for auth in ("Freeway", "Highway"):
      self.ensure(auth, True)

  # --- where we are
  def where(self, x, y, hd, city):
    """(auth, section, along) for the section under us, trying freeway, provincial road, then the county."""
    auths = ["Freeway", "Highway"] + ([f"City:{city}"] if city in LIVE_CITIES else [])
    for auth in auths:
      s = self.ensure(auth, False)
      if s is None and auth.startswith("City:"):
        s = self.ensure(auth, True)
      if s is not None:
        hit = s.locate(x, y, hd)
        if hit:
          return auth, hit[0], hit[1]
    return None, None, None

  def round(self, auth, city, now):
    """One round of live calls for what we are on."""
    wall = wall_now() if wall_ok() else None
    if auth:
      j = self._call(live_path(auth))
      if j is not None:
        self.live[auth] = (now, live_speeds(j, wall))
      j = self._call(event_path(auth))
      if j is not None:
        self.events[auth] = (now, live_events(j, wall))
      if auth == "Freeway":
        j = self._call("v2/Road/Traffic/Live/VD/Freeway")
        if j is not None:
          self.vd_live = (now, live_vds(j, wall))
    elif city in EVENT_CITIES:
      j = self._call(event_path(f"City:{city}"))
      if j is not None:
        self.events[f"City:{city}"] = (now, live_events(j, wall))

  @staticmethod
  def _fresh(entry, now):
    return entry is not None and now - entry[0] < 3 * LIVE_EVERY

  def report(self, auth, sid, along, x, y, hd, road, city, now):
    out = {"auth": auth.split(":")[0] if auth else None, "events": [], "ahead": [], "slow": None, "vd": None}
    line = None
    if auth:
      secs, line = self.sections[auth].path(sid, along)
      names = self.sections[auth].names
      out["road"] = names.get(sid, {}).get("name", "")
      live = self.live.get(auth)
      if self._fresh(live, now):
        sp = live[1]
        out["speed"] = sp.get(sid)
        out["age"] = round(now - live[0])
        for s2, d0, _d1 in secs[1:]:
          if d0 > AHEAD_M:
            break
          if s2 in sp:
            out["ahead"].append({"d": round(d0), "speed": sp[s2], "name": names.get(s2, {}).get("name", "")})
        slow_kph = SLOW_KPH[out["auth"]]
        cur = out.get("speed")
        for a in out["ahead"]:
          if a["speed"] < slow_kph and (cur is None or a["speed"] <= cur - SLOW_MARGIN_KPH):
            out["slow"] = a
            break
      vd = self.vd_live
      if auth == "Freeway" and self.vds and self._fresh(vd, now):
        best = None
        for vid, lanes in vd[1].items():
          v = self.vds.get(vid)
          if v is None or (v["bearing"] is not None and angdiff(v["bearing"], hd) > 90.):
            continue
          vx, vy = v["xy"]
          if abs(vx - x) > AHEAD_M + 200 or abs(vy - y) > AHEAD_M + 200:
            continue
          d = on_path(line, vx, vy, VD_PATH_M)
          if d is not None and VD_MIN_M <= d <= AHEAD_M and (best is None or d < best["d"]):
            best = {"d": round(d), "lanes": lanes}
        out["vd"] = best
    # events on the path (or straight ahead on our road off the sections)
    cand = []
    for entry in self.events.values():
      if not self._fresh(entry, now):
        continue
      for e in entry[1]:
        ex, ey = xy(e["lat"], e["lon"])
        if abs(ex - x) > PATH_M or abs(ey - y) > PATH_M:
          continue
        if e["dir"] is not None and angdiff(e["dir"], hd) > 90.:
          continue
        d = None
        if line:
          d = on_path(line, ex, ey, EVENT_PATH_M)
        elif road and any(road in r or r in road for r in e["roads"]):
          dx, dy = ex - x, ey - y
          h = math.radians(hd)
          al, lat_off = dx * math.sin(h) + dy * math.cos(h), dx * math.cos(h) - dy * math.sin(h)
          if 0 < al <= STRAIGHT_M and abs(lat_off) <= STRAIGHT_LAT_M:
            d = al
        if d is not None and 0 <= d <= AHEAD_M:
          cand.append({"d": round(d), "kind": e["kind"], "text": e["text"], "title": e["title"][:24]})
    out["events"] = sorted(cand, key=lambda e: e["d"])[:3]
    return out


def write_state(obj):
  obj["t"] = time.monotonic()
  tmp = STATE + ".tmp"
  with open(tmp, "w", encoding="utf-8") as f:
    json.dump(obj, f, ensure_ascii=False)
  os.replace(tmp, STATE)


def main():
  sm = messaging.SubMaster(["gpsLocationExternal"])
  mem_params = Params("/dev/shm/params")
  ta = TrafficAhead()
  pos, hd, gps_t = None, None, -1e9
  last_write, last_prefetch = -1e9, -1e9
  city, city_at = None, None
  cloudlog.info("tdx_traffic: start (display only)")
  while True:
    sm.update(1000)
    now = time.monotonic()
    if os.path.exists(OFF_FLAG):
      if now - last_write > WRITE_EVERY:
        write_state({"status": "off"})
        last_write = now
      time.sleep(5.)
      continue
    g = sm["gpsLocationExternal"]
    if sm.updated["gpsLocationExternal"] and g.horizontalAccuracy < 25. and not (g.latitude == 0 and g.longitude == 0):
      pos, gps_t = (float(g.latitude), float(g.longitude)), now
      if g.speed > MIN_HEADING_SPEED:
        hd = float(g.bearingDeg)
    driving = pos is not None and now - gps_t < GPS_FRESH

    if not os.path.exists(CRED):
      if now - last_write > WRITE_EVERY:
        write_state({"status": "nocred"})
        last_write = now
      time.sleep(5.)
      continue

    if not driving:
      if now - last_prefetch > 300.:
        ta.prefetch()
        last_prefetch = now
      if now - last_write > WRITE_EVERY:
        write_state({"status": "nogps", "online": ta.online})
        last_write = now
      continue

    if ta.cities is None:
      ta.cities = cache_load(CITY_SPEC[0], 1e12)[0]
    x, y = xy(*pos)
    if city_at is None or math.hypot(x - city_at[0], y - city_at[1]) > 2000.:
      city, city_at = city_of(ta.cities, *pos), (x, y)
    auth, sid, along = ta.where(x, y, hd, city) if hd is not None else (None, None, None)
    if now - ta.last_round >= LIVE_EVERY:
      ta.last_round = now
      ta.round(auth, city, now)
    if now - last_write >= WRITE_EVERY:
      road = mem_params.get("RoadName") or ""
      try:
        rep = ta.report(auth, sid, along, x, y, hd if hd is not None else 0., road, city, now) if hd is not None else {}
      except Exception:
        cloudlog.exception("tdx_traffic: report")
        rep = {}
      write_state(dict(rep, status="ok" if ta.online else "offline", online=ta.online, city=city))
      last_write = now


if __name__ == "__main__":
  main()
