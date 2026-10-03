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
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_T_FOLLOW, get_safe_obstacle_distance, get_stopped_equivalence_factor

RISE_TAU = 1.0   # s

# Seeing the lead brake sooner (2026-09-30). The driver, after two FCWs on 09-29: why can it not brake
# earlier. Over the 39 leads that braked past -2.5 on 09-26..09-29, counted from when the lead's own
# speed started falling faster than 1 m/s^2, the radar's aLeadK got there a median 0.80 s later (up to
# 2.9 s), openpilot's command 0.90 s, the car itself 1.21 s: the Kalman estimate is the wait, and the
# planner brakes as soon as it has it. So the lead's raw speed is differenced over RAW_WIN, and once
# that has been under RAW_THR for RAW_N frames running it stands in for aLeadK when it is the harsher
# of the two. A jump in dRel over RAW_JUMP_M is a different car, and starts the difference again.
# Same simulation as above: the 39 leads braking hard, closest approach 0.63 -> 2.76 m (08:15:54), 21 of
# them further off by more than 0.5 m and none closer; stops 5.46 -> 5.54 m. Following, braking past
# -1.5 goes 0.48 -> 0.69 a minute - of the 11 new ones, the lead in 10 slowed by more than 1 m/s^2
# within 2 s and in the other by 5 km/h: sooner, not phantom. Accelerate-then-brake 0.40 -> 0.48.
RAW_WIN = 0.3      # s
RAW_THR = -1.0     # m/s^2
RAW_N = 2          # frames
RAW_JUMP_M = 2.0   # m

# ... but only when the gap already needs it (2026-09-30). The driver: braking sooner will bring the
# stop-and-go back. Driving himself on 09-26..09-29, when the lead's raw speed dropped the same way (and
# he was not already braking) he was on the brake within 3 s 8% of the time if the lead then lost under
# 3 km/h and 37% if over 10 km/h - he lets most of them go; the above takes every one. So the raw decel
# counts only when the gap is under RAW_GAP_FRAC of the MPC's own follow distance or the lead would be
# reached in under RAW_TTC s; otherwise aLeadK is waited for, as before. Same closed loop, 235 windows:
#                          without raw   raw, always   raw, gated
#   accelerate-then-brake     0.40          0.48          0.41   a minute (under 30 km/h 0.69/0.78/0.69)
#   braking past -1.0         1.19          1.34          1.25   a minute
#   39 leads braking hard     0.63          2.76          2.76   m closest (08:15:54)
#                                                               4 of 39 up to 0.67 m nearer than always,
#                                                               all at 11-21 m; none nearer than without
#   stops                     5.46          5.54          5.51   m
# Tighter (0.8, 0.7 of the gap) takes braking past -1.5 from 0.67 to 0.62/0.55 a minute but 08:15:54 to
# 2.59/2.30 m and up to 1.1 m off 6-7 of the 39; a TTC alone (4 s) changed nothing against without.
RAW_GAP_FRAC = 0.9

# ... and only from the radar's own speed (2026-10-03). After 10-02 the hard braking left over was the lead's
# data jumping: of OP's 34 brakings past -2.0 for a lead that day, 7 had a lead whose speed leapt about while
# it came from vision (not all of them phantom) - e.g. 09:11:13 at 87 km/h, the radar car at 53 m doing
# 72 km/h read by vision as 45 m and 60-65, and the planner went to -3.4. Vision's speed is noisy frame to
# frame, and the difference above turns that noise into a lead braking: over 10-01..10-02, with the window
# kept to one source and no range jump, it fires 13.7% of the time on a vision lead against 4.0% on radar,
# and below -3.0 5.30% against 0.42%. So it is taken from the radar alone, and restarted whenever the lead
# changes source so a radar/vision step is never read as a deceleration; a vision lead keeps aLeadK, the
# model's own estimate. Closed loop (follow_sim, radard re-run on each log), 10-01 82 min / 10-02 91 min
# following: braking past -2.0 49 -> 32 / 41 -> 32, past -1.0 1.19 -> 1.11 / 1.47 -> 1.30 a minute,
# accelerate-then-brake 0.39 -> 0.41 / 0.53 -> 0.48, following gap unchanged (3.02 / 2.96 s median). The
# brakings past -2.0 those two days with the lead's speed jumping: still past -2.0 in 11 of 27 against 19.
# Safety: the 39 leads braking hard on 09-26..09-29 and the 50 stops unchanged to the centimetre; of the 96
# hard brakings on 10-01..10-02, 6 came nearer by 0.6-1.1 m, the nearest 7.7 m at 37 km/h.
# Tried with it and not taken: remembering the radar track through vision frames (radard's preferred track
# is forgotten the first frame the lead falls to vision) - radar/vision switches 13.8 -> 21.1 a minute and
# no fewer hard brakings; and also keeping that track for 0.3 s against vision - switches down to 5.8 but
# braking past -2.0 for 45 s against 22 over 10-01, the track and vision disagreeing for minutes at a time.
RAW_RADAR_ONLY = True
RAW_TTC = 3.0      # s

# A band around the gap the MPC keeps, for the MPC only (2026-10-02). The driver on 09-23: driving is not
# only speeding up and slowing down, there can be stretches that do neither, like a deadzone - do not go up
# and down with a lead that is going up and down. What went in then (long_deadzone) holds small requests
# at the output; the MPC behind it still works to the exact follow distance and the lead's every change of
# speed, so its requests leave that window (+0.35 / -0.45) and the car speeds up and brakes again - 1.22
# times a minute over 10-01, nearly all behind a lead. So here the MPC is shown the lead with a band taken
# out: a gap up to GAP_FAR_FRAC (at least GAP_FAR_MIN) longer than its follow distance reads as exactly the
# follow distance, and beyond it only the excess counts, so nothing steps. The band is on the far side
# only: what it gives up is acceleration, never braking. None of it when it matters: the lead braking
# (aLeadK under GAP_A, the raw decel above included), closing to within GAP_TTC, or under GAP_MIN_V.
# Everything else - the hold behind a stopped car, the junction, the braking urgency - reads the lead as is.
# Closed loop over 10-01's 164 following windows (82 min), none / 0.4 / 0.6 / 0.8: accelerate-then-brake
# 0.80 / 0.45 / 0.38 / 0.37 a minute; braking past -1.0 1.66 / 1.33 / 1.18 / 1.13; time neither speeding
# up nor slowing (|a| < 0.15) 39 / 46 / 47 / 48%; +/- swings 4.0 / 3.1 / 3.0 / 2.8 a minute; following
# gap median 2.63 / 2.91 / 3.03 / 3.11 s, its P5 1.76 -> 1.81 and the shortest 0.41 s throughout. The 39
# hard-braking leads: none nearer by more than 0.15 m (08:15:54 2.68 -> 2.71 m); stops 5.68 -> 5.79 m.
# A band on the lead's speed as well (shown at ours within 0.8-1.0 m/s) cut a little more but a lead
# closing slowly was then seen too late: shortest gap 0.41 -> 0.00-0.07 s, 08:15:54 to 0.07-1.50 m; a near
# side of 0.1 took the shortest gap to 0.16 s with 7 of the 39 nearer. Neither is here.
GAP_BAND = True
# A lead that is leaving is followed (2026-10-04). The driver, after 10-02: the car ahead kept speeding up and
# went a long way off while we dawdled, and the car behind sounded its horn - a person goes after a lead that
# is fast and pulling away. The two things above that keep the MPC from chasing a lead's every change of pace,
# RISE_TAU and the gap band, hold it back just as much here: over 09-23..10-02, with the lead pulling away
# (faster than us, accelerating over 0.4), it did +0.64..+0.80 at 10-80 km/h and the car +0.26..+0.47.
# So a lead faster than us by CHASE_DV and accelerating (aLeadK over CHASE_A) is taken as leaving: its speed
# is believed at once and the band is off, and the planner lifts its ceiling (longitudinal_planner CHASE_*).
# A lead easing a little faster or slower than us is still not chased. Results are with CHASE_* there; the
# radar-only condition went in after them and can only make chasing rarer.
CHASE_DV = 1.5         # m/s, 0 = off
CHASE_A = 0.2          # m/s^2


def is_chase(lead):
  # radar only: vision's speed is too noisy to say a lead is leaving (see RAW_RADAR_ONLY above)
  if CHASE_DV <= 0. or not lead.present or not lead.radar:
    return False
  v_ego = float(lead.vLead) - float(lead.vRel)
  return float(lead.vLead) > v_ego + CHASE_DV and float(lead.aLeadK) > CHASE_A
GAP_FAR_FRAC = 0.6
GAP_FAR_MIN = 4.0      # m
GAP_TTC = 5.0          # s
GAP_A = -0.3           # m/s^2
GAP_MIN_V = 5 / 3.6    # m/s


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
    self.hist = {}
    self.cnt = {}
    self.last_d = {}
    self.last_src = {}

  def _raw_decel(self, key, lead):
    d = float(lead.dRel)
    src = bool(lead.radar)
    src_changed = RAW_RADAR_ONLY and key in self.last_src and src != self.last_src[key]
    self.last_src[key] = src
    if (key in self.last_d and abs(d - self.last_d[key]) > RAW_JUMP_M) or src_changed:
      self.hist[key] = []
      self.cnt[key] = 0
    self.last_d[key] = d
    h = self.hist.setdefault(key, [])
    h.append(float(lead.vLead))
    k = int(round(RAW_WIN / DT_MDL))
    if len(h) > k + 1:
      h.pop(0)
    slope = (h[-1] - h[-1 - k]) / RAW_WIN if len(h) > k else 0.
    self.cnt[key] = self.cnt.get(key, 0) + 1 if slope < RAW_THR else 0
    if RAW_RADAR_ONLY and not src:
      return 0.
    return slope if self.cnt[key] >= RAW_N else 0.

  @staticmethod
  def _gap_needs_it(lead):
    d = float(lead.dRel)
    v_lead = float(lead.vLead)
    v_ego = v_lead - float(lead.vRel)
    want = get_safe_obstacle_distance(v_ego, get_T_FOLLOW()) - get_stopped_equivalence_factor(v_lead)
    closing = v_ego - v_lead
    return (want > 0. and d < RAW_GAP_FRAC * want) or (closing > 0.1 and d / closing < RAW_TTC)

  def _lead(self, key, lead):
    if not lead.present:
      for s in (self.v_lead, self.hist, self.cnt, self.last_d, self.last_src):
        s.pop(key, None)
      return lead
    v = float(lead.vLead)
    f = self.v_lead.get(key, v)
    leaving = is_chase(lead)
    f = v if (v < f or leaving) else f + (v - f) * min(DT_MDL / RISE_TAU, 1.)
    self.v_lead[key] = f
    v_ego = v - float(lead.vRel)
    a_raw = self._raw_decel(key, lead)
    if a_raw < 0. and not self._gap_needs_it(lead):
      a_raw = 0.
    a_lead = min(float(lead.aLeadK), 0., a_raw)
    return _Lead(lead, vLead=f, vRel=f - v_ego, aLeadK=a_lead, leaving=leaving)

  def update(self, radar_state):
    return _Radar(radar_state, self._lead("one", radar_state.leadOne), self._lead("two", radar_state.leadTwo))

  @staticmethod
  def _band(lead):
    if not GAP_BAND or not lead.present or getattr(lead, "leaving", False):
      return lead
    d = float(lead.dRel)
    v_lead = float(lead.vLead)
    v_ego = v_lead - float(lead.vRel)
    closing = v_ego - v_lead
    if v_ego < GAP_MIN_V or float(lead.aLeadK) < GAP_A or (closing > 0.1 and d / closing < GAP_TTC):
      return lead
    want = get_safe_obstacle_distance(v_ego, get_T_FOLLOW()) - get_stopped_equivalence_factor(v_lead)
    if want <= 0.:
      return lead
    hi = max(GAP_FAR_FRAC * want, GAP_FAR_MIN)
    if d <= want:
      return lead
    return _Lead(lead, dRel=max(want, d - hi))

  def for_mpc(self, radar):
    """What the MPC is given: the view from update(), with the gap band (GAP_*) taken out."""
    return _Radar(radar, self._band(radar.leadOne), self._band(radar.leadTwo))
