"""The planner's side of road memory (selfdrive/mapd/road_memory.py): mapd_bridge writes the speed this stretch has
been driven at before (v_allow, already slowing at A_COMFORT for a bin ahead); this turns it into a ceiling on the
cruise acceleration, the same way curve_speed and light_ahead do. It can only take speed off, never add it.

a_limit = (v_allow - v_ego) / TAU, clipped to [A_MIN, A_MAX]: 1 m/s under the allowed speed it lets through 0.67
m/s^2 (no more winding up hard between two bends), at the allowed speed it holds, over it slows. A_MIN is the
firmest it asks for; harder is for the stop / lead logic.

Shadow unless ON_FLAG exists (2026-10-30): a day left out, the other days' memory answered on 4% of the road
openpilot drove (11 days of logs; only 10.6% of it had two other days there) and on 0.56% of the free road it
would have slowed - a median 7, at most 18 km/h - below that day's speed. So it learns and is looked up, the HUD
shows what it would do and the log has each start / stop, but the car is only held to it once the flag is made.
"""
import json
import os
import time

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.mapd.road_memory import OUT_FILE

STALE = 1.5          # s: older means mapd_bridge is not writing (no fix, no memory here)
TAU = 1.5            # s
A_MIN = -1.2         # m/s^2
A_MAX = 2.0
ACT_FILE = '/dev/shm/road_mem_act.json'   # for the HUD: limiting or not, written on a change
ON_FLAG = '/data/road_mem_on'             # absent: shadow (see the module doc)


class RoadMemoryLimit:
  def __init__(self):
    self.a_limit = None
    self.v_allow = None
    self.state = {}
    self.read_t = 0.
    self.frame = 0
    self.live = os.path.exists(ON_FLAG)
    self.active = False
    self.a_shadow = None

  def _read(self, now):
    if now - self.read_t >= 0.25:
      self.read_t = now
      try:
        with open(OUT_FILE) as f:
          self.state = json.load(f)
      except (OSError, ValueError):
        self.state = {}

  def update(self, v_ego, engaged, a_cruise):
    """a_cruise: what cruise would do without this. Sets a_limit (None = no say; always None in shadow) and whether
    it is (or in shadow would be) the one limiting."""
    self.frame += 1
    if self.frame % 20 == 0:
      self.live = os.path.exists(ON_FLAG)
    now = time.monotonic()
    self._read(now)
    self.a_limit, self.a_shadow, self.v_allow = None, None, None
    st = self.state
    if engaged and st.get('v_allow') is not None and now - float(st.get('t', 0)) <= STALE:
      self.v_allow = float(st['v_allow'])
      self.a_shadow = min(max((self.v_allow - v_ego) / TAU, A_MIN), A_MAX)
      if self.live:
        self.a_limit = self.a_shadow
    active = self.a_shadow is not None and self.a_shadow < a_cruise
    if active != self.active:
      self.active = active
      cloudlog.event("road_memory", active=active, live=self.live, v_allow=self.v_allow, v_ego=round(v_ego, 2),
                     a=self.a_shadow, a_cruise=round(a_cruise, 2), cap=st.get('cap'), d=st.get('d'), days=st.get('days'))
      try:
        tmp = ACT_FILE + '.tmp'
        with open(tmp, 'w') as f:
          json.dump({'active': active, 'live': self.live, 'v_allow': self.v_allow, 'cap': st.get('cap'), 'd': st.get('d'),
                     'days': st.get('days'), 't': now}, f)
        os.replace(tmp, ACT_FILE)
      except OSError:
        pass
