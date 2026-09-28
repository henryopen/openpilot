"""A car closer than the MPC would like is not by itself a reason to brake.

A passenger on 2026-09-28: when a car cuts in we slow down for it when lifting off and watching
would do, and brake only if it turns out to be needed. The MPC brakes whenever the gap is short of
its target (about t_follow * v + STOP_DISTANCE) - even for a car pulling away. Over 09-23..09-28,
of 262 cut-ins where lifting off would have kept at least 0.8 s of gap with the lead holding its
speed, OP's car went past -0.5 m/s^2 in 48% (P25 of its deepest -1.02); the driver's own 14, driving
himself, 21% (P25 -0.43). By how the lead was moving: slower by over 3 km/h OP asked for a median
-0.90, about the same speed -0.48, faster by over 3 km/h 0.00 but still under -0.5 a third of the time.

So while following a radar lead above MIN_SPEED, if the lead holding its speed (or braking as it is)
cannot bring it inside T_MIN of us, the MPC's braking is held to COAST - about what lifting off gives
on the flat (get_coast_accel) - and it decides again every frame. The moment that stops being true -
we are already inside T_MIN, the lead's closing speed or braking would need NEED_ON or more to keep
T_MIN, or it is not a radar lead - the MPC's accel goes straight through, and stays through for
HOLD_S before this may hold it again, so a lead closing in does not get a coast in the middle of it.

Replayed against the real leads of the 331 stretches over those days where OP braked past -0.5 for a
lead (8 s each; the lead's own motion does not depend on us): closest gap median 1.74 -> 1.78 s, P10
1.24 -> 1.23, TTC P10 5.2 -> 5.3 s; deepest braking median -1.05 -> -0.88, and 19% of them never go
past -0.5; lifting off 76% of the time. One stretch closer than OP by over 0.15 s at under 1.0 s:
09-26 17:29:11, 60 km/h, a lead 6.8 km/h slower at 33 m - 0.88 s against 1.04, with the simulation
braking weaker than the MPC does once it takes over.
"""
from openpilot.common.constants import CV

MIN_SPEED = 20 * CV.KPH_TO_MS   # below, stop-and-go is the MPC's and the standstill hold's
COAST = -0.3                    # m/s^2, lifting off on the flat
T_MIN = 1.0                     # s of gap lifting off must keep
D_MIN = 5.0                     # m, and never less than this
NEED_ON = 0.3                   # m/s^2 needed to keep T_MIN: at or above, brake as planned
LEAD_BRAKING = -0.5             # m/s^2, a radar lead decelerating harder than this is braking
HOLD_S = 1.0                    # s the MPC keeps full say after it has needed it


def need_to_keep(d_rel: float, v_ego: float, v_lead: float, a_lead: float, d_min: float) -> float:
  """Deceleration that keeps d_min to a lead holding its speed, or braking as it is to a stop."""
  room = d_rel - d_min
  need = 0.0
  if v_lead < v_ego:
    need = (v_ego - v_lead) ** 2 / (2.0 * max(room, 0.5))
  if a_lead < LEAD_BRAKING:
    stop_lead = v_lead ** 2 / (2.0 * -a_lead)
    need = max(need, v_ego ** 2 / (2.0 * max(room + stop_lead, 0.5)))
  return need


class LeadCoast:
  def __init__(self, dt: float):
    self.dt = dt
    self.hold = 0.0
    self.active = False

  def update(self, a_target: float, v_ego: float, lead, eligible: bool) -> float:
    """eligible: the MPC's leadOne is what set a_target this frame."""
    self.active = False
    if not eligible or v_ego < MIN_SPEED or not (lead.present and lead.radar):
      self.hold = 0.0
      return a_target
    d_min = max(T_MIN * v_ego, D_MIN)
    need = need_to_keep(float(lead.dRel), v_ego, float(lead.vLead), float(lead.aLeadK), d_min)
    if lead.dRel < d_min or need >= NEED_ON:
      self.hold = HOLD_S
      return a_target
    if self.hold > 0.0:
      self.hold = max(self.hold - self.dt, 0.0)
      return a_target
    self.active = a_target < COAST
    return max(a_target, COAST)
