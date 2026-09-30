"""The lead as the planner sees it: believe it is slowing at once, believe it is pulling away slowly.

The driver on 2026-09-29: accelerating and then braking straight after is what makes the ride
uncomfortable. Over 09-16..09-29, following at 10-30 km/h with a lead inside 25 m, openpilot went from
over +0.3 to under -0.5 within 2 s 1.5-3.0 times a minute - behind a car 10-17 m ahead running about
2 km/h faster, it would go to +0.4-0.6 and be back to -0.8..-1.7 as soon as that car eased off. It is
the MPC chasing the lead's speed: a lead a little faster, or accelerating for a moment, is planned
around as if it will keep doing so.

So for everything the planner reads off radarState: the lead's aLeadK counts only when it is braking
(a positive one is taken as zero), and its speed is followed at once when it drops but only through a
RISE_TAU lag when it rises. Neither can make the planner brake later or less: a slower lead is seen as
it is, and a faster one only later.

Closed-loop simulation on the device - the real LongitudinalPlanner, the car's response fitted from
09-27..09-29 (aTarget -> aEgo, R^2 0.86-0.91 by speed), leads moving as they were recorded - over 146
following windows (73 min, 5-130 km/h), 50 stops behind a lead and 39 events where the lead braked
harder than -2.5 (driver's pedal ignored, openpilot left to it throughout). The simulation reproduces
the recorded rate of accelerate-then-brake (0.74 against 0.71 a minute; 1.55/1.47 under 30 km/h,
0.52/0.52 at 30-60, 0.18/0.18 above). Against the code as it was:
  accelerate-then-brake  0.74 -> 0.40 a minute (under 30 km/h 1.55 -> 0.69, 30-60 0.52 -> 0.43,
                         60+ 0.18 -> 0.11); +/- swings 4.6 -> 3.7 a minute
  lead braking hard      closest approach 0.57 -> 0.63 m, none of the 39 closer by more than 0.5 m,
                         12 of them further by more than that
  stopping               resting gap 5.35 -> 5.46 m median, closest 4.51 -> 4.54
  cost                   following gap 2.02 -> 2.13 s median, mean speed 49.8 -> 49.6 km/h
What was tried and not taken, each for the same reason - the lead braking hard came out closer: the
MPC's X_EGO_OBSTACLE_COST 30 -> 10 (0.57 -> 0.00-0.10 m, 11 events closer; 30 is what starts the
braking about 0.2 s sooner), that only while nothing is urgent (6 closer), and A_CHANGE_COST/J_EGO_COST
raised (18 closer). RISE_TAU 2.0 does a little more (0.38) but follows further back and stops softer.
"""
from openpilot.common.realtime import DT_MDL

RISE_TAU = 1.0   # s


class _Lead:
  def __init__(self, base, **over):
    self._base = base
    self._over = over

  def __getattr__(self, k):
    over = self.__dict__["_over"]
    return over[k] if k in over else getattr(self.__dict__["_base"], k)


class _Radar:
  def __init__(self, base, lead_one, lead_two):
    self._base = base
    self.leadOne = lead_one
    self.leadTwo = lead_two

  def __getattr__(self, k):
    return getattr(self.__dict__["_base"], k)


class LeadView:
  def __init__(self):
    self.v_lead = {}

  def _lead(self, key, lead):
    if not lead.present:
      self.v_lead.pop(key, None)
      return lead
    v = float(lead.vLead)
    f = self.v_lead.get(key, v)
    f = v if v < f else f + (v - f) * min(DT_MDL / RISE_TAU, 1.)
    self.v_lead[key] = f
    v_ego = v - float(lead.vRel)
    return _Lead(lead, vLead=f, vRel=f - v_ego, aLeadK=min(float(lead.aLeadK), 0.))

  def update(self, radar_state):
    return _Radar(radar_state, self._lead("one", radar_state.leadOne), self._lead("two", radar_state.leadTwo))
