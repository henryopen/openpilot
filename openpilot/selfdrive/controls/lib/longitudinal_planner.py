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
#                     0    10km/h  18    36    54    72    90   144
A_CRUISE_MAX_BP =   [0.,   2.8,   5.,   10.,  15.,  20.,  25., 40.]
A_CRUISE_MAX_VALS = [1.2,  1.17,  1.0,  0.5,  0.5,  0.5,  0.3, 0.2]
# Jerk keeps its own breakpoints. It shares the acceleration curve's in stock, and adding
# points there would silently make the two arrays different lengths.
J_CRUISE_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
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
# so this car ran about 7% faster than the number set. Regression over 81753 samples of
# route 00000004--422ccab765 (vEgo vs vEgoCluster): dash = 1.071*true + 2.07 km/h. Cruise
# and speed limit targets are dash speeds and are converted back here. Re-run the
# regression if the tyres or wheel size change.
DASH_GAIN = 1.071
DASH_OFFSET_KPH = 2.07
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


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

# Ease off when the set speed itself is low. Pulling the full 1.2 m/s2 away from a stop
# feels abrupt when the target is 40 km/h, in a way the same acceleration toward 100 km/h
# does not - the car is asking for most of its authority to cover a small gap. Ported from
# FrogPilot's get_max_accel_low_speeds; its CITY_SPEED_LIMIT is 15 m/s.
_LOW_SET_SPEED_BP = [0., 7.5, 15.]


def scale_for_set_speed(max_accel, v_cruise):
  return float(np.interp(v_cruise, _LOW_SET_SPEED_BP, [max_accel / 4, max_accel / 2, max_accel]))

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

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

    # what cruise is actually aiming at, which is not what the driver set whenever the
    # junction stop or a forced decel has taken speed off it. Nothing downstream could say
    # so, which is why the driver saw the car slow for no visible reason.
    self.v_cruise_dash = true_to_dash(v_cruise)

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

    self.a_cruise = get_cruise_accel(sm['selfdriveState'].experimentalMode, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    # ease off before a corner the model can see. it is a limit on cruise rather than a
    # separate plan source, so it just takes the lower of the two.
    self.curve_speed.update(sm, not long_control_off, sm['carState'].gasPressed, v_ego, sm['carState'].aEgo)
    curve_limited = False
    if self.curve_speed.is_active:
      curve_limited = self.curve_speed.a_target < self.a_cruise
      self.a_cruise = min(self.a_cruise, self.curve_speed.a_target)

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
    pm.send('longitudinalPlanSP', sp_send)
