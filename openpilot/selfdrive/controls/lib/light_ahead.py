"""Coming up to a traffic signal with nothing in front: ease off early, and let a known green go.

The driver (2026-10-14): with no car ahead the car brakes too late for a red, and above 60 km/h it does
not manage at all. Over 8/26-10/13 it stopped itself for a light with nothing in front 22 times against
156 by the driver; from 40-60 km/h its braking peaked at -2.26 m/s^2 (driver -1.94), from above 60 never.
The light is only readable from about 40 m (light classifier, median 41 m), and from 50 km/h a stop at
the driver's 1.5 m/s^2 needs 65 m - so waiting to see the red is always late at speed.

What is known earlier is that a signal is there: the OSM list, through tlwatchd (yolo/tlwatchd.py),
which writes the metres to the signal ahead and its read of the light to /dev/shm/tlwatch.json. The
driving model cannot tell red from green, but on the way into a red its own plan slows well before
the stop - its lowest planned speed under 0.6 x ours at a median 104 m on 40-55 km/h approaches -
and it does that on the way through only 2 of 129 greens. A signal on the map makes that intent
believable where on its own it is not (it also slows for nothing in alleys and car parks).

So, with nothing within LEAD_IGNORE in front, no indicator on (a turn is the corner logic's), engaged,
and the watcher's file fresh:
  green known        nothing - carry on
  red known, or the model means to slow (INTENT_FRAC, for INTENT_FRAMES)
                     slow down evenly towards V_FLOOR by STOP_ZONE short of the signal, no firmer than
                     A_EARLY_MAX; the stop itself stays with stop_for_lights / the junction handoff
  otherwise          nothing. Until 2026-10-28 this coasted (A_COAST, -0.25) from 150 m out whenever the light
                     was not yet known green. Taken out, the driver: "滑行為什麼是 -0.25 不是 0，這樣真的讓人坐得很
                     不舒服". On 10/28's two drives it coasted 41 times (163 s) and the car stopped at that signal
                     in none of them; it cut straight from +0.5..+0.9 to -0.25 in 11 (7-10 m/s^3), switched on and
                     off with each signal on the map (13:37:46-13:39:18 seven times), and -0.25 is not coasting on
                     this car: off the throttle it slows 0.10-0.24, so the SCC tops it up with the brakes (TCS13
                     BrakeLight on 33% / 68% of -0.20..-0.30 requests at 80-93 / 40-60 km/h).
Simulated on the recorded approaches with the classifier's real calls: 40-55 km/h reds start slowing
at a median 104 m and need 0.77 m/s^2 instead of braking late; 2 of 129 greens get a needless slowdown.

Yellow (2026-10-28, the driver: "黃燈你認為可以就做吧"): C3 passes one only where a light turned yellow (tlwatchd calls
it 'yellow'). At the first yellow of a signal it is decided once, and held until that signal is passed or the call is
no longer yellow / red: stop if braking to STOP_ZONE short of the signal needs at most YELLOW_STOP_A, else carry on
as for a green - a yellow too close to stop for is driven through, as a driver does. A stop is then the red's: slow
towards V_FLOOR_RED, and under MIN_SPEED at least no throttle (10/28 14:36:56: a yellow read as green at 24 km/h,
the car pulled from 25 to 33 km/h through it and braked at about -2.2 when the red was seen 2.5 s late). On that
drive's 11 yellows 7 would have been stopped for (needing 0.15-1.8 m/s^2), 4 driven through (9-11 m out, needing
4.8-7.9).

It is a limit on cruise, like the corner: the planner takes the lower of the two, so it can only ever
take speed off. Off unless FLAG exists, so it can ride along in shadow while the watcher's calls are
checked against real drives; `mode` says what it would be doing either way.
"""
import json
import os
import time

import numpy as np

from openpilot.common.constants import CV

FLAG = '/data/light_ahead_on'
STATE = '/dev/shm/tlwatch.json'
OUT = '/dev/shm/light_ahead.json'  # what it is doing, for the HUD: written on a change and once a second
STALE = 1.0                      # s: an older file means the watcher is not running
STOP_ZONE = 15.0                 # m: the OSM node is mid-junction, the stop line about this far short
A_COAST = -0.25                  # m/s^2, off the throttle
A_EARLY_MAX = -1.2               # firmest early slowdown; anything harder is the stop logic's call
V_FLOOR = 20 * CV.KPH_TO_MS      # the model's intent alone does not slow below this
V_FLOOR_RED = 10 * CV.KPH_TO_MS  # a red seen: slow further, the stop logic still makes the stop
MIN_SPEED = 25 * CV.KPH_TO_MS
LEAD_IGNORE = 80.0               # m: a car nearer than this sets the pace
INTENT_FRAC = 0.6
INTENT_FRAMES = 20               # 1 s at 20 Hz. 0.5 s let a dip the model takes crossing a junction slow us for
                                 # the next signal (10/14 18:53: -1.2 at 61 m on a green)
YELLOW_STOP_A = 2.0              # m/s^2: a yellow this gentle to stop for is stopped for
SLOW_START_MIN = 25.0            # m: a slowdown only starts this far out. Nearer is inside the junction, where an
                                 # 'early' slowdown is neither early nor gentle (10/14 17:16: -1.2 at 9 m); the stop
                                 # logic owns that. Replayed: 10/14 needless slows 3 -> 1 (with INTENT_FRAMES 20),
                                 # 8/26-10/13 greens slowed 2 -> 1 of 129; the reds it drops all needed > 1.2 m/s^2


class LightAhead:
  def __init__(self):
    self.enabled = os.path.exists(FLAG)
    self.frame = 0
    self.state = None
    self.intent_frames = 0
    self.a_limit = None
    self.mode = ''               # '' or 'slow' - what it is (or in shadow would be) doing
    self.slowing = False         # it was slowing on the last tick, for this same signal
    self._osm_prev = None
    self.a_shadow = None
    self.yellow = None           # 'stop' / 'go' for the signal being approached, decided at its first yellow
    self._out_mode, self._out_t = None, 0.

  def _read(self):
    try:
      with open(STATE) as f:
        s = json.load(f)
      return s if time.monotonic() - float(s.get('t', 0)) < STALE else None
    except (OSError, ValueError, TypeError):
      return None

  def update(self, md, v_ego, lead, blinker, engaged):
    self._decide(md, v_ego, lead, blinker, engaged)
    now = time.monotonic()
    if self.mode != self._out_mode or now - self._out_t > 1.0:
      self._out_mode, self._out_t = self.mode, now
      try:
        tmp = OUT + '.tmp'
        with open(tmp, 'w') as f:
          json.dump({'t': now, 'mode': self.mode, 'on': self.enabled, 'yellow': self.yellow,
                     'a': None if self.a_shadow is None else round(self.a_shadow, 2)}, f)
        os.replace(tmp, OUT)
      except OSError:
        pass

  def _decide(self, md, v_ego, lead, blinker, engaged):
    self.frame += 1
    if self.frame % 20 == 0:
      self.enabled = os.path.exists(FLAG)
    if self.frame % 5 == 0:
      self.state = self._read()
    # a slowdown already under way carries on inside SLOW_START_MIN - but not into the next signal's
    osm_now = self.state.get('osm_m') if self.state is not None else None
    passed = osm_now is not None and self._osm_prev is not None and osm_now > self._osm_prev + 25.
    self.slowing = self.mode == 'slow' and not passed
    self._osm_prev = osm_now
    light = self.state.get('light') if self.state is not None else None
    if passed or osm_now is None or light not in ('yellow', 'red'):
      self.yellow = None
    self.a_limit, self.mode, self.a_shadow = None, '', None

    vmin = min(md.velocity.x) if len(md.velocity.x) else v_ego
    self.intent_frames = self.intent_frames + 1 if (v_ego > 1. and vmin < INTENT_FRAC * v_ego) else 0

    s = self.state
    if (not engaged or s is None or s.get('mode') != 'approach' or s.get('osm_m') is None
        or blinker or (lead.present and float(lead.dRel) < LEAD_IGNORE)):
      return
    light = s.get('light')
    if light == 'yellow' and self.yellow is None:
      need = v_ego ** 2 / (2. * max(float(s['osm_m']) - STOP_ZONE, 1.))
      self.yellow = 'stop' if need <= YELLOW_STOP_A else 'go'
    if light == 'yellow':
      light = 'red' if self.yellow == 'stop' else 'go'
    if v_ego < MIN_SPEED:
      if light == 'red' and self.yellow == 'stop':
        self.mode, self.a_shadow = 'slow', A_COAST     # slow already: at least no more throttle into it
        if self.enabled:
          self.a_limit = A_COAST
      return
    if light == 'go':
      return
    d = max(float(s['osm_m']) - STOP_ZONE, 5.)
    if (light == 'red' or self.intent_frames >= INTENT_FRAMES) and (self.slowing or float(s['osm_m']) >= SLOW_START_MIN):
      v_floor = V_FLOOR_RED if light == 'red' else V_FLOOR
      a = -(v_ego ** 2 - v_floor ** 2) / (2. * d)
      self.mode, a = 'slow', float(np.clip(a, A_EARLY_MAX, A_COAST))
    else:
      return
    self.a_shadow = a
    if self.enabled:
      self.a_limit = a
