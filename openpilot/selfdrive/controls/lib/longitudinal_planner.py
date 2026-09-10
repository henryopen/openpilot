#!/usr/bin/env python3
import math
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.cereal import custom
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE as MPC_STOP_DISTANCE
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (get_T_FOLLOW, get_safe_obstacle_distance,
                                                                            get_stopped_equivalence_factor)
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.curve_speed import CurveSpeedControl
from openpilot.selfdrive.controls.lib.stop_for_lights import StopForLights, MAX_DECEL as STOP_MAX_DECEL
from openpilot.selfdrive.controls.lib.junction_handoff import JunctionHandoff
from openpilot.common.swaglog import cloudlog

# Eco from 18 km/h up, ours below it. open251021's eco curve pulls harder than stock off
# the line (1.6 against 1.2) and much softer everywhere above walking pace; the request was
# for the soft half only, so the first two points hold the values this car already had and
# the rest are eco's, whose 5 m/s breakpoint is exactly the 18 km/h asked for.
#
# Eco is flat at 0.5 from 36 to 72 km/h and then steps to 0.3, which wastes the three
# breakpoints sitting inside that range and leaves a cliff at 72. Driven on 2026-09-05 the
# band was called too eager, and the log agrees it is felt rather than met occasionally: of
# the 42 s spent accelerating between 36 and 72 km/h, 69% was commanded at 0.40 or above and
# the median was 0.41. The three points now taper into the 0.3 at 90 instead, which at a
# 50 km/h set speed takes 50 -> 60 from 6.0 s to 8.2 s. The same ceiling caps closing on a
# lead that pulls away, which was 27% of that band, and that is the cost of the change.
#
# 2026-09-08: that taper went too far - the driver now calls the same band flat. The taper
# took 8% off 30 km/h but 30% off 54 and 36% off 72, so what was cut hardest is the part
# above where the complaint was. Driving free of a lead that day, the plan sat within 8% of
# this ceiling for 33% of the frames at 30-35 km/h and 54% at 45-50, while the same drive
# reached 0.72-0.82 under MPC and e2e, which do not read this array - so the car does that
# much already and only cruise is held down. Only the 10 m/s point moves, to halfway back
# towards eco's 0.5; 54 km/h and above stay where the taper put them, so the flat 0.5-to-72
# stretch that was called too eager does not come back.
#                     0    10km/h  18    36    54    72    90   144
A_CRUISE_MAX_BP =   [0.,   2.8,   5.,   10.,  15.,  20.,  25., 40.]
A_CRUISE_MAX_VALS = [1.2,  1.17,  1.0,  0.48, 0.35, 0.32, 0.3, 0.2]
# With nothing close ahead the ceiling above is what holds the car back, not the MPC. Over
# the 2026-09-09 drive, with the nearest lead beyond the gate below, the plan sat within 8%
# of this ceiling for 75% of the frames at 15-25 km/h, 82% at 25-36, 80% at 36-45, 70% at
# 45-54 and 92% at 54-72. With a lead inside the gate the same figures are 38/46/31/29% -
# there the MPC is in charge and raising this would change nothing. So the ceiling only
# lifts when the road ahead is actually clear.
#
# Pinned to the 2026-08-29 drive, which the driver called too eager: every point here is at
# or below what that drive allowed, checked value by value (0.94/0.81/0.67/0.50/0.47/0.45 at
# 20/25/30/36/45/54 km/h against 0.94/0.81/0.67/0.50/0.50/0.50). That matters because on
# 2026-09-05 a change was made by taking "half the suggestion" without checking, and it came
# out above the too-eager values across 24-42 km/h.
#
# What 08-29 actually delivered, free of a close lead, against 2026-09-09: 0.68 vs 0.54 at
# 25-36 km/h, 0.43 vs 0.35 at 36-45, 0.37 vs 0.27 at 45-54 - but 0.24 vs 0.26 at 54-72 and
# 0.31 vs 0.26 at 72-90. So the gap that was felt is 36-54 km/h; above that today's car is
# already the quicker of the two and the flat 0.5 was never being used. Only 36 and 54 move.
# 40 -> 60 km/h goes from 14.6 s to 12.1 s (08-29 was 11.1 s), 50 -> 70 from 16.3 to 13.4.
A_CRUISE_MAX_VALS_FREE = [1.2, 1.17, 1.0, 0.50, 0.45, 0.35, 0.3, 0.2]
# How much more room than the MPC is asking for before the road counts as clear. A fixed
# distance was considered and measured worse: the MPC's target gap is
# v^2/(2*COMFORT_BRAKE) - v_lead^2/(2*COMFORT_BRAKE) + t_follow*v + STOP_DISTANCE, so 50 m
# is 3.3x the target behind a lead holding 20 km/h and only 0.9x behind one that has
# stopped at 40 km/h - it would be loosest exactly where the risk is. As a share of frames
# the ratio also lands where it is wanted: over the 2026-09-09 drive it opens 37% of
# 15-30 km/h and 46% of 30-45 against a fixed 50 m's 17% and 28%, while at 100-130 km/h it
# opens 79% against 99%. Of the frames each lets through, 85% are cruise-led under the
# ratio against 72% under 50 m, so less of it is spent raising a ceiling nothing is on.
FREE_LEAD_MARGIN = 1.2
# Jerk keeps its own breakpoints. It shares the acceleration curve's in stock, and adding
# points there would silently make the two arrays different lengths.
J_CRUISE_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
# Stopped behind a car that has not moved, do not creep the last metre in to STOP_DISTANCE.
# This car stops a systematic 1.5-2 m short of the 6 m target - 40 stops on 2026-09-06 sat
# at 3.6-4.9 m - so the MPC spends the wait asking for the gap it did not get. Frames
# stopped inside 10 m creep 5.3% of the time, but split by where it stopped: 0.5% inside
# 3 m, 2.3% from 3 to 5, 13.7% from 5 to 6, 38.3% from 6 to 7 and 60.4% beyond that, with
# aTarget reaching +1.36. Requiring the radar's own vRel to say the lead is not leaving
# keeps this off a real pull-away: of 3709 stopped frames where the lead was opening at
# 0.3 m/s or more, this holds back none of them. The radar is required because vision's
# range on a stopped car is what creates the false gap in the first place.
STANDSTILL_CREEP_SPEED = 0.5  # m/s
STANDSTILL_CREEP_DIST = 9.0  # m
STANDSTILL_CREEP_VREL = 0.3  # m/s
# Comfort jerk for tracking the set speed. A plain proportional law on the speed error
# (gain 1.0) saturates at max_accel or A_CRUISE_MIN for any error over ~1.2 m/s, so it holds
# full accel or full decel until the last 4 km/h and then drops off abruptly. Shaping the
# target as sqrt(2*j*error) makes it taper as the set speed is approached, in both
# directions. This only limits the cruise candidate; MPC braking for a lead is unaffected
# because the candidates are resolved with min().
J_CRUISE_COMFORT = 0.16
# Measured on this car: holding a set speed swings about +/-1 km/h on the cluster, crossing
# the set speed six to eight times in fifteen seconds. 0.25 m/s is 0.9 km/h, so it covers
# that swing while leaving any steady-state offset under 1 km/h.
V_CRUISE_DEADZONE = 0.25
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
PlanReason = custom.LongitudinalPlanSP.Reason
PLAN_REASONS = {LongitudinalPlanSource.cruise: PlanReason.cruise,
                LongitudinalPlanSource.lead0: PlanReason.lead,
                LongitudinalPlanSource.lead1: PlanReason.lead,
                LongitudinalPlanSource.e2e: PlanReason.e2e}

# holding cruise back for a lead the model is unsure about, see where it is used below
RADAR_TO_CAMERA = 1.52
WEAK_LEAD_MIN_PROB, WEAK_LEAD_MAX_PROB = 0.2, 0.5
WEAK_LEAD_MIN_DIST, WEAK_LEAD_MAX_DIST = 45., 100.
WEAK_LEAD_MIN_SPEED = 5.
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# The driver sets MAX by what the dash shows, but openpilot controls on true wheel speed,
# so cruise and speed limit targets are dash speeds and are converted back here.
#
# Refitted 2026-09-10 over 22915 moving samples of route 00000026--ee332da948 above
# 40 km/h. The previous constants came from a single route and had the gain too high and
# the offset too low; below 50 km/h the two errors cancel, but the gain error grows with
# speed and the driver noticed it above 90:
#
#   set 100  ->  old target 91.4 true  ->  reached 97.9 on the dash (measured 98.0)
#   set 110  ->  old target 100.8 true ->  reached 107.5 on the dash (measured 108.0)
#
# Residual against the whole route: 1.51 km/h at the median and 2.64 at p95 for the old
# constants, 0.34 and 1.04 for these. Fitted above 40 km/h because that is where set
# speeds live and the relation is not quite linear at walking pace; the low end barely
# moves anyway (40 -> 35.4 becomes 35.0, 50 is unchanged, 60 -> 54.1 becomes 54.5).
#
# The dash reads high by law, and the driver's ask is that its number match the number he
# set, so this deliberately runs the car about 2-3 km/h faster in true terms at 100-120.
# Re-run the regression if the tyres or wheel size change.
DASH_GAIN = 1.0272
DASH_OFFSET_KPH = 4.00
DASH_MAX_KPH = 200.  # above this the value is a sentinel rather than a speed, pass it through


def dash_to_true(v_target: float) -> float:
  v_kph = v_target * CV.MS_TO_KPH
  if v_kph <= 0. or v_kph > DASH_MAX_KPH:
    return v_target
  return max((v_kph - DASH_OFFSET_KPH) / DASH_GAIN, 0.) * CV.KPH_TO_MS


def true_to_dash(v_true: float) -> float:
  """The inverse, for showing a target next to the number the driver set it by."""
  v_kph = v_true * CV.MS_TO_KPH
  if v_kph <= 0. or v_kph > DASH_MAX_KPH:
    return v_true
  return (v_kph * DASH_GAIN + DASH_OFFSET_KPH) * CV.KPH_TO_MS


def get_max_accel(v_ego, lead_free=False):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS_FREE if lead_free else A_CRUISE_MAX_VALS)


def lead_is_far(lead, v_ego, t_follow):
  """Is the nearest lead far enough that cruise, not the MPC, is what limits acceleration?"""
  if not lead.present:
    return True
  # the gap the MPC is working to, in the same terms it uses: a lead moving with us needs
  # less room than one that has stopped, so this tightens by itself when the lead slows
  target_gap = get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(min(lead.vLead, v_ego))
  return lead.dRel > FREE_LEAD_MARGIN * target_gap

# Ease off when the set speed itself is low. Pulling the full 1.2 m/s2 away from a stop
# feels abrupt when the target is 40 km/h, in a way the same acceleration toward 100 km/h
# does not - the car is asking for most of its authority to cover a small gap. Ported from
# FrogPilot's get_max_accel_low_speeds; its CITY_SPEED_LIMIT is 15 m/s.
_LOW_SET_SPEED_BP = [0., 7.5, 15.]


def scale_for_set_speed(max_accel, v_cruise):
  return float(np.interp(v_cruise, _LOW_SET_SPEED_BP, [max_accel / 4, max_accel / 2, max_accel]))

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle,
                     lead_free=False):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego, lead_free)

  if not e2e:
    max_accel = scale_for_set_speed(max_accel, v_cruise)
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  # Ignore the last fraction of a km/h. The law above is proportional all the way to zero
  # error, so holding a set speed means commanding acceleration continuously in one
  # direction and then the other, which is felt as 50 -> 51 -> 49 -> 51. Subtracting the
  # deadzone rather than zeroing inside it keeps the response continuous at the edge.
  speed_error = v_cruise - v_ego
  if abs(speed_error) <= V_CRUISE_DEADZONE:
    speed_error = 0.0
  else:
    speed_error -= math.copysign(V_CRUISE_DEADZONE, speed_error)
  comfort_accel = min(abs(speed_error), math.sqrt(2. * J_CRUISE_COMFORT * abs(speed_error)))
  target_accel = np.clip(math.copysign(comfort_accel, speed_error), A_CRUISE_MIN, max_accel)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.plan_reason = PlanReason.cruise
    self.curve_speed = CurveSpeedControl()
    # StopForLights is left in the tree but no longer driven: the junction is handed to the
    # model now rather than braked for here, and two stopping laws would fight each other.
    self.stop_for_lights = StopForLights()
    self.junction = JunctionHandoff()
    self.a_cruise = init_a
    self.v_cruise_dash = 0.
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = dash_to_true(v_cruise_kph * CV.KPH_TO_MS)
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    # Arm the model for the junction rather than braking for it here. In experimental mode
    # the model is already in the mix, so there is nothing to arm.
    self.stop_for_lights.reset()
    if sm['selfdriveState'].experimentalMode:
      self.junction.reset()
    else:
      self.junction.update(sm['modelV2'], sm['carState'], v_ego, sm['radarState'].leadOne)

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # committed to a junction stop: give the solver a car standing at the line, so the same
    # thing that stops us behind a lead stops us here. It is only ever another obstacle, and
    # the nearest one wins, so this cannot make the car go faster than it otherwise would.
    stop_x = self.stop_for_lights.obstacle_x(MPC_STOP_DISTANCE) if self.stop_for_lights.is_active else None
    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality,
                    stop_x=stop_x, a_min=STOP_MAX_DECEL if stop_x is not None else ACCEL_MIN)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    lead_free = lead_is_far(sm['radarState'].leadOne, v_ego, get_T_FOLLOW(sm['selfdriveState'].personality))
    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle, lead_free)
    # ease off before a corner the model can see. it is a limit on cruise rather than a
    # separate plan source, so it just takes the lower of the two.
    self.curve_speed.update(sm, not long_control_off, sm['carState'].gasPressed, v_ego, sm['carState'].aEgo)
    curve_limited = False
    if self.curve_speed.is_active:
      curve_limited = self.curve_speed.a_target < self.a_cruise
      self.a_cruise = min(self.a_cruise, self.curve_speed.a_target)

    # What cruise is really working to, in the units the dash shows so it can sit beside the
    # driver's MAX. On its own the set speed says nothing the driver cannot already read, so
    # the number is only worth showing when something has taken speed off it. The corner is
    # the one limiter here that computes a speed rather than an acceleration - the rest
    # reach the car as a_cruise - and its target only means anything while it is active,
    # holding the last corner's value otherwise.
    v_cruise_shown = v_cruise
    if self.curve_speed.is_active:
      v_cruise_shown = min(v_cruise_shown, self.curve_speed.v_target)
    self.v_cruise_dash = true_to_dash(v_cruise_shown)

    # A lead the model is unsure about, too far out to hold as a lead. On 2026-09-03 a car
    # 62 to 80 m ahead had its probability swinging between 0.09 and 0.85; every dip dropped
    # it, the road was judged clear for ten seconds, cruise wound 26 km/h up to 53, and it
    # locked on again at 43 m closing at 20 km/h, which needed -3.0 m/s^2 and the driver.
    # Holding a full lead that far out is a coin toss (0.9:1, see radard's LEAD_HOLD_*), but
    # simply not speeding up is a bounded thing to be wrong about: over 2.27 hours this
    # holds cruise back for 43 s an hour and a real lead turns up within six seconds 82% of
    # the time, so 8 s an hour of not accelerating at nothing.
    weak_lead = False
    if not sm['radarState'].leadOne.present and v_ego > WEAK_LEAD_MIN_SPEED:
      leads = sm['modelV2'].leadsV3
      if len(leads):
        lead_x = leads[0].x[0] - RADAR_TO_CAMERA
        weak_lead = (WEAK_LEAD_MIN_PROB <= leads[0].prob < WEAK_LEAD_MAX_PROB
                     and WEAK_LEAD_MIN_DIST <= lead_x < WEAK_LEAD_MAX_DIST)
    if weak_lead:
      self.a_cruise = min(self.a_cruise, 0.)

    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if sm['selfdriveState'].experimentalMode or self.junction.active:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])

    # The model plans a stop but brakes at about two thirds of what reaching it takes, and
    # the shortfall compounds. Hold it to the deceleration the distance it says it has needs.
    # This can only ever brake harder, and it is only reachable when the handoff is armed -
    # which needs no lead, no turn desire, and the model's own plan collapsing to a stop.
    if self.junction.a_floor < min(output_a_target, 0.0):
      output_a_target = self.junction.a_floor
      self.mpc.source = LongitudinalPlanSource.e2e

    # name what set this accel, so the display can say so: the junction stop reports as the
    # mpc's e2e obstacle outside experimental mode, and the curve limiter hides inside cruise
    reason = PLAN_REASONS.get(self.mpc.source, PlanReason.cruise)
    if reason == PlanReason.cruise and curve_limited:
      reason = PlanReason.curve
    elif reason == PlanReason.cruise and weak_lead:
      reason = PlanReason.weakLead
    elif reason == PlanReason.e2e and not sm['selfdriveState'].experimentalMode:
      reason = PlanReason.stopLight
    self.plan_reason = reason
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    # See STANDSTILL_CREEP_* above. Stateless on purpose: the lead opening up clears it on
    # the same frame, so there is nothing to get stuck in.
    lead = sm['radarState'].leadOne
    if (v_ego < STANDSTILL_CREEP_SPEED and lead.present and lead.radar
        and lead.dRel < STANDSTILL_CREEP_DIST and lead.vRel < STANDSTILL_CREEP_VREL
        and not sm['carState'].gasPressed):
      output_a_target = min(output_a_target, 0.0)

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    sp_send = messaging.new_message('longitudinalPlanSP')
    sp_send.valid = plan_send.valid
    sp_send.longitudinalPlanSP.reason = self.plan_reason
    sp_send.longitudinalPlanSP.vCruise = float(self.v_cruise_dash)
    sp_send.longitudinalPlanSP.modelHandoff = bool(self.junction.active)
    pm.send('longitudinalPlanSP', sp_send)
