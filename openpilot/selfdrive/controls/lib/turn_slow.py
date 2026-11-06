"""Slow for a signalled turn the car is still too fast to be told about (2026-11-01).

The turn desire only comes under desire_helper's TURN_TT_SPEED (30 km/h), so a turn signalled at 40 got no turn in the
model's plan and nothing from curve_speed: on 10/31 cruise carried the car into the junction 6 times and the driver
braked 5 of them. The driver, 11/01: GPS, the route and the map may be used for this, but only with the blinker on.

So: with one blinker on, desire_helper holding the lane change (or already giving the turn) for that side, and over
V_TURN, slow to V_TURN by D_MARGIN short of the junction - the route's step when navigating, else OSM's junction on that
side (selfdrive/mapd/turn_ahead). Once under TURN_TT_SPEED the turn desire follows, the model plans the turn and
curve_speed takes it from there. Braking only once A_START is needed, building up at ONSET_J, at most A_MAX; lifting
the throttle from A_LIFT. Off with the blinker, the accelerator, a junction passed, or OFF_FLAG; and for the rest of the
blinker once the driver turns the wheel himself (a lane change caught as a turn: the hold stops the automatic one, so
he steers it - simulated, those 11 of 89 lost ~20 km/h otherwise).

Which blinkers count is desire_helper's hold (see TT_HOLD_* there): on 21 drives held out (10/12-10/27), a blinker at
25 km/h or more with OSM's junction on that side 4-11 s ahead was a turn 30 of 50 times it fired (30 of 30 at that
junction) and a lane change 11 of 98."""
import json
import os
import time

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.mapd.turn_ahead import TurnAheadReader

TURN_HOLD_FILE = '/dev/shm/turn_hold.json'   # desire_helper's go / hold / side
OFF_FLAG = '/data/turn_slow_off'
V_TURN = 25 * CV.KPH_TO_MS      # under desire_helper's TURN_TT_SPEED, so the turn desire follows
D_MARGIN = 30.                  # m: be down to V_TURN this far before the junction (about 4 s at V_TURN)
A_START = 0.3                   # m/s^2 needed before braking
A_LIFT = 0.05                   # m/s^2 needed: stop gaining speed
A_MAX = 1.5
ONSET_J = 1.5                   # m/s^3
PASSED_M = -5.
NAV_M = 300.                    # the route's turn on that side this near is the one
READ_S = 0.2


class TurnSlow:
  def __init__(self):
    self.ahead = TurnAheadReader()
    self.hold, self.hold_read_t = {}, 0.
    self.off, self.off_check = False, 0
    self.is_active = False
    self.a_target = np.inf
    self.a_cmd = 0.
    self.d = None
    self.d_t = 0.
    self.dropped = False            # the driver steered during this blinker

  def _read_hold(self, now):
    if now - self.hold_read_t >= READ_S:
      self.hold_read_t = now
      try:
        with open(TURN_HOLD_FILE, encoding='utf-8') as f:
          self.hold = json.load(f)
      except (OSError, ValueError):
        self.hold = {}
    return self.hold

  def reset(self):
    self.is_active = False
    self.a_target = np.inf
    self.a_cmd = 0.
    self.d = None
    self.d_t = -1.                  # take the distance again from the next read, carried on from the file's time

  def update(self, v_ego, left_blinker, right_blinker, gas_pressed, steering_pressed=False):
    if self.off_check % 100 == 0:
      self.off = os.path.exists(OFF_FLAG)
    self.off_check += 1
    one = left_blinker != right_blinker
    if not one:
      self.dropped = False
    elif steering_pressed:
      self.dropped = True
    if self.off or not one or self.dropped or gas_pressed or v_ego < V_TURN * 0.9:
      self.reset()
      return
    side = 'L' if left_blinker else 'R'
    now = time.monotonic()
    h = self._read_hold(now)
    if h.get('side') != side or not (h.get('hold') or h.get('go')):
      self.reset()
      return
    ta = self.ahead.get()
    nav = ta.get('nav')
    if nav and nav[0] == side and float(nav[1]) <= NAV_M:     # following a route: its turn, not OSM's junction
      d_new = float(nav[1])
    else:
      d_new = ta.get(side) if ta.get('osm') else None
    # the file comes twice a second: carry the distance on with the speed in between
    t_file = float(ta.get('t', 0.)) if ta else 0.
    if d_new is not None and t_file != self.d_t:
      self.d, self.d_t = d_new - v_ego * max(now - t_file, 0.), t_file
    elif self.d is not None:
      self.d -= v_ego * DT_MDL
    if self.d is None or self.d < PASSED_M:
      self.reset()
      return
    need = (v_ego ** 2 - V_TURN ** 2) / (2 * max(self.d - D_MARGIN, 5.))
    if need >= A_START or (self.a_cmd > 0. and need > A_LIFT):
      self.a_cmd = min(min(need, A_MAX), self.a_cmd + ONSET_J * DT_MDL)
      self.a_target = -self.a_cmd
      self.is_active = True
    elif need > A_LIFT:
      self.a_cmd = 0.
      self.a_target = 0.
      self.is_active = True
    else:
      self.reset()
