#!/usr/bin/env python3
import math
import os
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.cereal import custom, log
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
from openpilot.selfdrive.controls.lib.light_ahead import LightAhead
from openpilot.selfdrive.controls.lib.road_memory_limit import RoadMemoryLimit
from openpilot.selfdrive.controls.lib.long_deadzone import LongDeadzone
from openpilot.selfdrive.controls.lib.standstill_hold import StandstillHold, ARM_SPEED as STANDSTILL_HOLD_ARM_SPEED
from openpilot.selfdrive.controls.lib.lead_view import LeadView, is_chase
from openpilot.selfdrive.controls.lib.stop_for_lights import StopForLights, MAX_DECEL as STOP_MAX_DECEL, STOPPED as STOPPED_SPEED
from openpilot.selfdrive.controls.lib.junction_handoff import JunctionHandoff
from openpilot.selfdrive.keypad import KeypadReader
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
#
# 2026-09-17: both curves reshaped, because the shape was backwards. The driver's own foot
# over 10 engaged minutes of throttle with no brake and nothing inside 40 m, against what
# the free curve was handing out at the same speed:
#
#      km/h      his median / p75      free curve
#      0-10        0.29 / 0.70            1.19        system 4x him
#     10-18        0.41 / 0.68            1.09        system 2.7x him
#     18-27        0.56 / 0.92            0.88        about right
#     27-36        0.57 / 0.80            0.62        about right
#     45-54        0.60 / 0.80            0.46        him 1.3x system
#     54-63        1.28 / 1.40            0.45        him 2.8x system
#     63-72        1.36 / 1.51            0.45        him 3.0x system
#     72-81        1.49 / 1.54            0.44        him 3.4x system
#     90-110       1.20 / 1.37            0.40        him 3.0x system
#
# He accelerates gently from rest and hard once moving; the curves did the opposite. The
# two complaints - "in town with the lead far away it accelerates hard enough to feel
# unsafe" and "on the on-ramp it is too weak to merge" - are the two ends of that one
# error, not two problems.
#
# Two caveats this is set conservatively against. The high-speed sample is small (906
# frames over 54-110 km/h, about 9 seconds) and self-selected, because the reason to press
# the throttle is thinking it is too slow. And 2026-09-14 measured 54-72 km/h at a median
# of 0.52 on a larger sample where today says 1.28. So the new values move towards the
# foot without arriving at it: 0.68 at 72 km/h against his 1.36, which is still half.
#
# Monotonic-decreasing is given up on purpose. The previous note kept it so the car would
# not accelerate harder at 72 than at 54, but the driver does exactly that (0.57 at 36,
# 1.36 at 72), so the constraint described a preference nobody has.
#
# Integrated through the ceiling, 0-30 km/h goes 8.5 -> 9.7 s, 0-50 19.4 -> 18.9,
# 40-70 18.2 -> 13.3 and 61-107 30.1 -> 19.9.
#
# 2026-09-18: the pull-away was still 2.3-2.7x the driver's own. Over the 40 segments of
# 09-18, accelerating frames with no brake, body acceleration median / p90:
#
#      km/h        system          him
#      0-10      0.87 / 1.52    0.32 / 0.76
#     10-18      0.69 / 0.92    0.30 / 0.65
#     18-27      0.47 / 0.82    0.46 / 0.86   <- already his, leave it
#     27-36      0.33 / 0.63    0.56 / 0.68
#
# The ceiling is the whole of it below 18 km/h: 57% of those frames have aTarget sitting
# on aCruiseMax and none above it, so nothing else is in the way. The body then overshoots
# the request by about 60% (229 frames past 1.0 m/s2 while aTarget asked 0.91), which is
# why the ceiling is felt harder than it reads.
#
# The first two points come down to the driver's own p90; the 18 km/h point only moves a
# little because 18-27 km/h already matches him and this is the endpoint the 10-18 band
# interpolates to. Above 36 km/h nothing changes - the merging complaint lives there.
#                     0    10km/h  18    36    54    72    90   144
A_CRUISE_MAX_BP =   [0.,   2.8,   5.,   10.,  15.,  20.,  25., 40.]
A_CRUISE_MAX_VALS = [0.72, 0.68,  0.78, 0.48, 0.40, 0.38, 0.34, 0.26]
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
#
# 2026-09-14: 72 km/h and above lift, asked for after an interchange ramp took 45 s to go
# 61 -> 107 km/h. That stretch was 92% cruise-led with the command sitting at 0.27-0.32, so
# it was this array and nothing else. The claim above that "above 54 today's car is already
# the quicker" was read off 08-29, a drive that never had a clear road at those speeds; it
# does not say what the band should be.
#
# The anchor is the driver's own foot on the same two drives: with the throttle pressed and
# no brake he took 0.52 m/s^2 at the median through 54-72 km/h and 0.51 through 72-90, a
# quarter of the time over 0.92, peaking at 1.33. Under openpilot the same bands got 0.24
# and 0.22 at the median. The car is not short of the power either - left to itself it
# reached 0.90 and 0.92 in those bands - so the ceiling was the whole of it.
#
# Set below that median rather than at it, and kept monotonic: 54 km/h stays where 09-09 put
# it and 72 joins it rather than passing it, or the car would accelerate harder at 72 than at
# 54. Interpolated, 90 km/h goes 0.300 -> 0.420 and 105 km/h 0.272 -> 0.387 (+42%); holding
# the ceiling throughout, 61 -> 105 km/h takes 29 s against 38. Only the free curve moves:
# behind a lead the MPC is in charge and this is not
# read, and the driver has not asked for anything there.
#
# 2026-09-17: reshaped with the curve above - see the measurement of the driver's own foot
# there. This one keeps the gap over A_CRUISE_MAX_VALS at every breakpoint, and takes the
# whole of the increase in the 36-144 km/h range where the merging complaint lives.
#
# 2026-09-18: the first three points follow the curve above down. They were identical to it
# below 18 km/h and still are - the lead being far does not change how the driver pulls away
# from rest, and 1084 frames of that drive had a lead present while this curve was in charge
# (median 26 m), so leaving them high would have kept the pull-away everywhere it was felt.
#
# 2026-10-07: the driver: with the road clear, the low-speed pick-up should be stronger. Over 10-01..10-06
# (this curve unchanged since 09-18), nothing within 40 m, body acceleration median / p75 against his own foot:
#
#      km/h      openpilot (asked)        him
#      0-10      0.69 / 1.11 (0.69)    0.48 / 0.75    <- already above him, the 09-18 complaint: leave it
#     10-18      0.55 / 0.66 (0.74)    0.47 / 0.68
#     18-27      0.51 / 0.63 (0.71)    0.58 / 0.88
#     27-36      0.51 / 0.64 (0.62)    0.62 / 0.93
#     36-45      0.50 / 0.63 (0.58)    0.61 / 0.98
#
# The ceiling is the limit there (the request sits on it 86-91% of the time), and from 18 km/h the body gives
# only 70-85% of what is asked. So 18-72 km/h is lifted for about 0.68 at the body - between his median and
# his p75 - and 0-10 km/h, where openpilot is already the quicker, does not move.
#
# 2026-10-15: the driver: the 10 km/h point 0.68 -> 0.80 (pulling away with the road clear is still slow).
#                          0    10km/h  18    36    54    72    90   144
A_CRUISE_MAX_VALS_FREE = [0.72, 0.80, 0.95, 0.80, 0.75, 0.75, 0.64, 0.50]
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
# Going after a lead that is leaving (2026-10-04) - see lead_view CHASE_*. When it is, and we are no nearer
# than the follow distance, the ceiling rises to CHASE_VALS: about the driver's own P75 when he accelerates
# (09-23..10-02, 10-18 / 18-27 / 27-36 / 36-50 km/h: 0.71 / 0.74 / 0.86 / 0.77, P90 0.93-1.14), where
# openpilot's P90 sat at 0.65-0.70 on the ceiling. Unchanged from 0 to 18 km/h, the launch curve.
# Closed loop over 10-01 / 10-02 following (173 min): with the lead pulling away, our acceleration
# +0.362 -> +0.422 and the time spent more than 3x the follow distance behind 8.6 -> 6.6 s a minute, mean
# speed 43.87 -> 44.04 km/h, gap 2.99 -> 2.91 s; accelerate-then-brake 0.29 -> 0.37 a minute (0.40 before
# COAST_TTC). The 39 leads braking hard: one nearer by 0.5 m, 18:41:25 10.3 -> 8.1 m at 5 km/h; stops
# 5.79 -> 5.75 m. The ceiling alone (no lead_view change) gave +0.381 - the MPC then held it back.
CHASE_ON = True
CHASE_NEED_LEAVING = True
CHASE_DV = 0.5               # m/s
# ... and off the line too (2026-10-17). The driver: when the car ahead pulls away fast from a stop, the gap
# opens so wide the car behind thinks we are not moving. Over 10-08..10-16, the third of launches behind a
# lead with the quickest lead (its first 5 s at 0.9-1.1 m/s^2), ours was 0.52-0.55: at +5 s it was at
# 18-22 km/h and we at 9-10, 20-23 m further off after 8 s (P75 29-34) - the driver himself 13-18 m. Up to
# 18 km/h the chase ceiling was the launch curve (0.72-0.80), so leaving or not made no difference there.
# Now a lead that is leaving (same test as above: radar, faster by CHASE_DV and accelerating) is followed
# off the line at up to 1.2. Closed loop (follow_sim3, body refitted 10-08..10-16 by speed band; 10-14/16
# launches, following, 116 hard-braking leads, 50 stops): those launches +8 s 17.7 -> 16.3 m (P75 29.4 ->
# 26.9), +5 s 10.7 -> 11.8 km/h; following, stops and accelerate-then-brake unchanged; of the 116 leads
# braking hard 6 came nearer by 0.5-3 m, the nearest of them still 9.4 m off (closest overall 1.54 m both).
# 1.0 did about half as much. /data/launch_chase_off brings back the curve as it was.
CHASE_BP = [0., 2.5, 5., 10., 15., 20., 25., 40.]
CHASE_VALS = [1.2, 1.2, 1.14, 0.85, 0.80, 0.70, 0.64, 0.50]
CHASE_BP_OLD = [0., 5., 10., 15., 20., 25., 40.]
CHASE_VALS_OLD = [0.72, 0.78, 0.85, 0.80, 0.70, 0.64, 0.50]
LAUNCH_CHASE_OFF_FLAG = '/data/launch_chase_off'
# Jerk keeps its own breakpoints. It shares the acceleration curve's in stock, and adding
# points there would silently make the two arrays different lengths.
J_CRUISE_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
# Stopped behind a car that has not moved: standstill_hold.StandstillHold. It replaced the
# stateless STANDSTILL_CREEP_* gate (09-06, vRel 0.3 -> 0.5 on 09-20) on 2026-09-26 - see there.
# The keypad's GO (2026-10-11): the driver says there is nothing to stop for. Stopped at a light
# with nothing ahead, that it is green; moving, that the model is stopping for nothing - what the
# CEM override on the distance button did on open251021. Two things stop the car with no lead:
# stop_for_lights' point, which the accelerator clears, and the junction handoff, which keeps the
# model's stop in charge for as long as the model plans one, and the model can not tell red from
# green. A tap of the accelerator ends the first and drives through the second; GO has no foot on
# the pedal, so it ends the first and keeps the second off for GO_DISTANCE, and from a standstill
# pulls away on standstill_hold's ramp. Leads and the radar are untouched. If the car has not
# moved off after GO_WAIT stopped, the press is spent; another press starts it over. Not behind
# a stopped car: there the 1 m rule stays (driver, 2026-10-04 and 2026-10-11). Unlike open251021,
# where the override held until the button was pressed again, this runs out by itself, so a
# forgotten press can not carry through the next red light. GO_DISTANCE and GO_WAIT are my
# choice, not measured.
GO_DISTANCE = 50.            # m
GO_WAIT = 3.0                # s
# Comfort jerk for tracking the set speed. A plain proportional law on the speed error
# (gain 1.0) saturates at max_accel or A_CRUISE_MIN for any error over ~1.2 m/s, so it holds
# full accel or full decel until the last 4 km/h and then drops off abruptly. Shaping the
# target as sqrt(2*j*error) makes it taper as the set speed is approached, in both
# directions. This only limits the cruise candidate; MPC braking for a lead is unaffected
# because the candidates are resolved with min().
J_CRUISE_COMFORT = 0.16

# Braking builds up rather than arriving all at once (2026-10-01). The driver, after 10-01: it brakes hard
# first and then lighter, and that is what is uncomfortable. Over 10-01, OP's braking for a lead reached
# its deepest a median 0.90 s in with the first 0.5 s at 0.69 of it (48% of stretches over 0.7, "hard
# first"); his own braking 1.05 s and 0.59 (33%). The MPC's weights do not move that: in closed loop over
# 10-01's 164 following windows, X_EGO_OBSTACLE_COST 15/10, J_EGO_COST 10 and A_CHANGE_COST 400 all left
# hard-first at 48-53%, and every one of them let 09-29 08:15:54 come closer (2.76 m -> 0.21-1.85 m).
# So the deepening itself is limited to BRAKE_ONSET_J - unless it is urgent: a lead reached within
# BRAKE_ONSET_TTC, or nearer than BRAKE_ONSET_GAP of the follow distance, a stop committed to, the
# junction floor, an FCW, or under BRAKE_ONSET_MIN_V. The MPC starts each solve from this output, so
# what is not taken early it asks for later: light first, firmer after, the way a person brakes.
# Closed loop over 10-01's 164 following windows, 1.0 / 1.5 / 2.5: hard first 50% -> 43 / 44 / 45%, first
# 0.5 s at 0.70 -> 0.64 / 0.65 / 0.67 of the deepest, braking past -1.0 1.77 -> 1.73 / 1.67 / 1.66 a
# minute; the 39 leads braking hard: 08:15:54 stays at 2.76 m, 3 / 2 / 1 of them nearer by more than 0.5 m
# (all 10 m or more away, at most 1.3 m nearer); stops 5.68 m unchanged. The corner braking has its own
# build-up in curve_speed (_V2_ONSET_J), which this does not change.
# Urgency was then loosened from 4 s / 0.8 to 3 s / 0.7 (same windows): hard first 43% -> 41%, first 0.5 s
# 0.64 -> 0.62; 08:15:54 2.76 -> 2.68 m and none of the 39 nearer by more than 0.5 m. 2.5 s / 0.6 went to
# 39% but took 08:15:54 to 1.95 m and 4 of them nearer.
BRAKE_ONSET_J = 1.5          # m/s^3, 0 = off
BRAKE_ONSET_TTC = 3.0        # s
BRAKE_ONSET_GAP = 0.7        # of the follow distance
BRAKE_ONSET_MIN_V = 10 * CV.KPH_TO_MS

# No throttle towards a lead we are already closing on (2026-10-03). The driver: seeing the car ahead is not
# fast, a person does not press the throttle; as it comes near they press the brake slowly down to the stop;
# if it is fast and pulls away, they go after it - smooth, not throttle one moment and brake the next. Over
# 10-01..10-02 (188 min following), accelerating then braking within 2 s came 1.57 times a minute by the
# car's own acceleration against 1.04 when he drives, and in 42% of them the lead was already slower than
# us when it sped up. So with a lead slower than us by COAST_CLOSING that we would reach the follow distance
# of within COAST_TTC, the throttle is not used: at most the speed is held, and the MPC brakes as before
# when it is needed. A faster lead pulling away is followed as before. Closed loop over 10-01 / 10-02
# following (82 / 91 min), off / 8 / 15 / 30 s: accelerate-then-brake by the car 0.40 / 0.34 / 0.29 / 0.28
# a minute, time neither speeding up nor slowing 47.5 -> 48.6% and 49.6 -> 51.8% at 15 s, mean speed
# 43.88 -> 43.87 km/h, following gap 2.98 -> 2.99 s. The 39 leads braking hard: none nearer by 0.5 m,
# 08:15:54 2.71 -> 2.51 m; stops unchanged (5.79 m); of 96 hard brakings on 10-01..02, 2 came 0.9-1.4 m
# nearer, both 12 m or more away. 30 s added one 0.7 m nearer at 7 m for 0.01 a minute less.
COAST_TTC = 15.0             # s, 0 = off
COAST_CLOSING = 0.3          # m/s
# A new lead after the old one moves out of the lane (2026-10-17). The driver: the car in front moves over,
# we speed up to close on the next one, it turns out slower, and we brake straight away. Over 10-08..10-17
# the lead's range jumped by more than 6 m (a different car) 1074 times while following; 174 of them were
# followed within 8 s by speeding up past +0.2 and then braking past -0.3. In those the new car was 45 m
# off, two in three times seen by vision only, its range jumping again a median 9 times in the next 6 s,
# and it read 1-3 km/h faster than us at first and 1 km/h slower 5 s on: its speed is not known yet, and
# the planner closes a 45 m gap on a guess. So for CUT_S after such a jump acceleration is held to CUT_A -
# unless the new car is plainly leaving: the radar's test (is_chase), or the range itself opening by more
# than CUT_AWAY a second since the jump, which vision measures far better than speed. It only ever takes
# acceleration away. Closed loop (follow_sim3, 259 such jumps, 116 hard-braking leads, 50 stops): speed up
# then brake after the jump 85% -> 40% of those that did it and 15% -> 9% of the rest; 12 s on the gap is
# 2 m longer and the mean speed 1-2 km/h lower; a new car that was leaving 67.8 -> 70.7 m (without the range
# test 76.7); the 116 hard-braking leads none nearer, closest 1.54 -> 2.26 m; stops unchanged.
# /data/cut_gentle_off turns it off.
CUT_JUMP_M = 6.0             # m: the lead's range growing by more than this in a frame = a new car
CUT_S = 5.0                  # s
CUT_A = 0.2                  # m/s^2
CUT_AWAY = 1.0               # m/s the range opens by on average since the jump: the new car is leaving
CUT_AWAY_MIN_S = 0.5         # s of range before that is believed
CUT_MIN_V = 10 * CV.KPH_TO_MS
CUT_GENTLE_OFF_FLAG = '/data/cut_gentle_off'
# ... and over once there has been no lead at all for CUT_LOST_S (2026-10-23). On a rainy night vision kept seeing cars
# that were not there - reflections, oncoming headlights - their range jumping, each jump starting the 5 s again, and
# the cap ran on with nothing ahead: 14 s of a 211 s drive held to 0.2 with no lead. A real new car blinks out too, so
# not at once: closed loop (follow_sim3b, 193 lead changes from 10-08..10-23, 116 hard-braking leads, 50 stops),
# speed-up-then-brake after a change 41% as it was -> 53 / 46 / 42% ending after 0.5 / 1.0 / 2.0 s gone; hard-braking
# leads and stops identical in all four; 10-23's changes 1-4 km/h quicker in three of six.
CUT_LOST_S = 2.0
# A shorter follow gap while the lane change is under way (2026-10-24, after StarPilot 7.0's lane_change_gap.py). The
# driver: changing lanes it often slows down. Over 10-12..10-23, 29 lane changes engaged at 40 km/h or more with no
# pedal, 18 lost more than 3 km/h: moving across, the MPC meets the next lane's car inside its 1.5 s gap and brakes
# to open it - and for the first 2-3 s that car is vision's alone (this radar reports the one target the stock ACC
# holds, still in the old lane), its speed read a median 4 km/h, often 10-20, below what the radar then measures.
# So while laneChangeStarting/Finishing (not preLaneChange - a blinker above 20 km/h at a junction is that too)
# t_follow comes down to LC_T_FOLLOW at LC_RAMP_IN /s and goes back at LC_RAMP_OUT /s once it is over; at once if
# the lead brakes LC_ABORT_A or harder, the blind spot on that side is taken, a pedal is pressed, or under
# LC_MIN_V. StarPilot goes back at 4 /s; that only moves the braking to just after the lane change.
# Closed loop (follow_sim3b, 60 lane changes 10-12..10-23 + 116 hard-braking leads + 50 stops), against today:
# lane changes slowing > 3 km/h 22 -> 19, time braking past -0.3 108 -> 92 s, past -1.0 27.2 -> 27.5 s, mean speed
# after +1.0 km/h, headways under 0.8 s 5 -> 5; one hard-braking lead nearer, 10-14 18:46:17 25.4 -> 22.5 m;
# stops unchanged. (StarPilot's 4 /s back: past -1.0 29.4 s; 0.75 s: one more headway under 0.8 s.)
LC_T_FOLLOW = 1.0            # s
LC_RAMP_IN = 0.6             # s of t_follow per s
LC_RAMP_OUT = 0.1            # s of t_follow per s
LC_ABORT_A = -0.8            # m/s^2, the lead's aLeadK
LC_MIN_V = 30 * CV.KPH_TO_MS
LC_STATES = (log.LaneChangeState.laneChangeStarting, log.LaneChangeState.laneChangeFinishing)
# Measured on this car: holding a set speed swings about +/-1 km/h on the cluster, crossing
# the set speed six to eight times in fifteen seconds. 0.25 m/s is 0.9 km/h, so it covers
# that swing while leaving any steady-state offset under 1 km/h.
V_CRUISE_DEADZONE = 0.25
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
PlanReason = custom.LongitudinalPlanSP.Reason
AccelLimit = custom.LongitudinalPlanSP.AccelLimit
PLAN_REASONS = {LongitudinalPlanSource.cruise: PlanReason.cruise,
                LongitudinalPlanSource.lead0: PlanReason.lead,
                LongitudinalPlanSource.lead1: PlanReason.lead,
                LongitudinalPlanSource.e2e: PlanReason.e2e}

# holding cruise back for a lead the model is unsure about, see where it is used below
RADAR_TO_CAMERA = 1.52
WEAK_LEAD_MIN_PROB, WEAK_LEAD_MAX_PROB = 0.2, 0.5
# How long the doubt has to last before it is allowed to take the throttle away. On
# 2026-09-15 this fired 189 times on one drive for 35.1 s in total: a median of 0.10 s
# each - two frames - with 177 of the 189 under half a second and 90% of them less than
# three seconds apart, at a median 35 km/h. That is not a lead being held back from, it is
# the model's probability crossing 0.2 and 0.5 while the car is trying to accelerate, and
# every crossing zeroed a_cruise for a tenth of a second. The driver felt exactly that:
# quick behind a lead, sluggish on an empty road, which is backwards from the tables
# (A_CRUISE_MAX_VALS_FREE is >= A_CRUISE_MAX_VALS at every breakpoint) and only makes
# sense once you see that both this and stop_for_lights can only fire with no lead.
# Ten frames keeps the twelve episodes that lasted long enough to mean something and
# drops the other 177. The reason for the check is unchanged - see the comment at the
# use site - so the exit is immediate: doubt that clears should give the throttle back
# on the same frame, it is only the onset that needs to prove itself.
WEAK_LEAD_MIN_FRAMES = 10
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

# No easing off for a low set speed any more (FrogPilot's get_max_accel_low_speeds, here from 08-29 to
# 09-30: x0.5 at a 27 km/h set speed, full only from 54). It came before the curves above were taken
# from the driver's own foot on 09-17/18, so in town it took them down twice. On 2026-09-30, set to 35
# on the dash, it was what held the ceiling 78% of the time with nothing in front, and the driver: with
# no car ahead it is still slow to pick up. Against his own acceleration with nothing ahead (09-24..30,
# median / P75): 5-15 km/h 0.59/0.91, 15-25 0.66/0.95, 25-35 0.69/0.97, 35-50 0.45/0.84; that day's
# ceiling was 0.47 / 0.50 / 0.51 / 0.49 with it and is 0.71 / 0.74 / 0.66 / 0.59 without.

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle,
                     lead_free=False, chase=False, chase_old=False):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego, lead_free)
  if chase and not e2e:
    bp, vals = (CHASE_BP_OLD, CHASE_VALS_OLD) if chase_old else (CHASE_BP, CHASE_VALS)
    max_accel = max(max_accel, float(np.interp(v_ego, bp, vals)))
  # Which of the four things below ended up being the ceiling. The number on its own does
  # not say - the driver asked to see it because a car sitting on its ceiling looks the
  # same whichever one put it there, and the answer decides what to go and change.
  source = AccelLimit.e2e if e2e else (AccelLimit.free if lead_free else AccelLimit.lead)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    if a_x_allowed < max_accel - 1e-4:
      source = AccelLimit.lateral
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      if coast_limit < max_accel - 1e-4:
        source = AccelLimit.coast
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

  # the ceiling comes back too: on an open road it is what holds the car back, and from the
  # outside that is indistinguishable from the car simply choosing not to accelerate. The
  # display cannot re-derive it - it is the speed curve, the set-speed scaling, the lateral
  # budget and the coast limit, and a copy of that would drift the way the follow distance
  # did - so whoever applies it is the one that reports it.
  return target_accel, float(max_accel), source


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.plan_reason = PlanReason.cruise
    self.weak_lead_frames = 0
    self.curve_speed = CurveSpeedControl()
    self.light_ahead = LightAhead()
    self.road_memory = RoadMemoryLimit()
    self.deadzone = LongDeadzone(dt)
    self.standstill_hold = StandstillHold(dt)
    self.lead_view = LeadView()
    self.launch_chase_old = os.path.exists(LAUNCH_CHASE_OFF_FLAG)
    self.cut_gentle_off = os.path.exists(CUT_GENTLE_OFF_FLAG)
    self.flag_frame = 0
    self.cut_frames = 0       # frames left of the hold after the lead became a different car - see CUT_*
    self.cut_d0 = 0.
    self.cut_since = 0
    self.cut_prev_d = -1.
    self.cut_lost = 0         # frames in a row with no lead at all - see CUT_LOST_S
    self.lc_t_follow = None   # the follow gap the lane-change ramp is at, None when not in use - see LC_*
    # Driven again as of 2026-09-13. It was parked in 09-06 on the grounds that two stopping
    # laws would fight each other, but the two are not both stopping laws: the handoff takes
    # speed off on the way in and never says where to stop, and this says where to stop and
    # writes no deceleration of its own. Measured over the 9/9 route, below.
    self.stop_for_lights = StopForLights()
    self.junction = JunctionHandoff()
    self.keypad = KeypadReader(("go",))
    self.go_left = 0.         # m still to cover with the junction handoff off, after a GO
    self.go_wait = 0.
    self.a_cruise = init_a
    self.a_cruise_max = 0.
    self.a_cruise_max_source = AccelLimit.free
    self.v_cruise_dash = 0.
    self.follow_distance = 0.
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def _lane_change_t_follow(self, sm, lead, v_ego, reset_state):
    """The MPC's follow gap: the personality's, or less while a lane change is under way - see LC_*."""
    base = get_T_FOLLOW(sm['selfdriveState'].personality)
    meta, cs = sm['modelV2'].meta, sm['carState']
    direction = meta.laneChangeDirection
    cur = base if self.lc_t_follow is None else self.lc_t_follow
    if (reset_state or cs.gasPressed or cs.brakePressed or v_ego < LC_MIN_V
        or (direction == log.LaneChangeDirection.left and cs.leftBlindspot)
        or (direction == log.LaneChangeDirection.right and cs.rightBlindspot)
        or (lead.present and float(lead.aLeadK) <= LC_ABORT_A)):
      cur = base
    elif meta.laneChangeState in LC_STATES:
      cur = max(LC_T_FOLLOW, cur - LC_RAMP_IN * self.dt)
    else:
      cur = min(base, cur + LC_RAMP_OUT * self.dt)
    self.lc_t_follow = None if cur >= base else cur
    return cur

  def update(self, sm):
    self.flag_frame += 1
    if self.flag_frame % 20 == 0:
      self.launch_chase_old = os.path.exists(LAUNCH_CHASE_OFF_FLAG)
      self.cut_gentle_off = os.path.exists(CUT_GENTLE_OFF_FLAG)
    # the lead believed slowing at once and pulling away slowly - see lead_view
    radar = self.lead_view.update(sm['radarState'])
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = dash_to_true(v_cruise_kph * CV.KPH_TO_MS)
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    # The keypad's GO - see GO_DISTANCE
    for _ in self.keypad.poll():
      cs = sm['carState']
      if (sm['selfdriveState'].enabled and not sm['selfdriveState'].experimentalMode
          and not cs.brakePressed and not self.standstill_hold.active):
        self.go_left, self.go_wait = GO_DISTANCE, GO_WAIT
        if v_ego < STOPPED_SPEED:
          self.standstill_hold.launch()
      else:
        cloudlog.event("keypad go refused", enabled=bool(sm['selfdriveState'].enabled), v_ego=float(v_ego),
                       brake=bool(cs.brakePressed), held_behind_lead=bool(self.standstill_hold.active))
    going = self.go_left > 0. and sm['selfdriveState'].enabled
    if going:
      self.go_left -= v_ego * DT_MDL
      if v_ego < STOPPED_SPEED:
        self.go_wait -= DT_MDL
        if self.go_wait <= 0.:
          self.go_left = 0.
    else:
      self.go_left = 0.

    # Arm the model for the junction rather than braking for it here. In experimental mode
    # the model is already in the mix, so there is nothing to arm.
    if sm['selfdriveState'].experimentalMode or going:
      self.junction.reset()
    else:
      self.junction.update(sm['modelV2'], sm['carState'], v_ego, radar.leadOne)

    # And give the empty junction its stop point. Experimental mode already stops for one,
    # and there is nothing to stop for until we are driving.
    if sm['selfdriveState'].experimentalMode or not sm['selfdriveState'].enabled:
      self.stop_for_lights.reset()
    else:
      self.stop_for_lights.update(sm['modelV2'], v_ego, v_cruise, sm['carState'].gasPressed or going,
                                  radar.leadOne)

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
    t_follow = self._lane_change_t_follow(sm, radar.leadOne, v_ego, reset_state)
    # the MPC alone gets the lead with a band round its follow distance - see lead_view GAP_*
    self.mpc.update(self.lead_view.for_mpc(radar), personality=sm['selfdriveState'].personality,
                    stop_x=stop_x, a_min=STOP_MAX_DECEL if stop_x is not None else ACCEL_MIN, t_follow=t_follow)

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

    lead_free = lead_is_far(radar.leadOne, v_ego, t_follow)
    # the same gap in the MPC's own terms, for anything that wants to show where it is
    # trying to sit. Kept here rather than worked out again downstream, which is how the
    # HUD ended up drawing it nine metres short at a 20 km/h closing speed
    lead_one = radar.leadOne
    self.follow_distance = (get_safe_obstacle_distance(v_ego, t_follow)
                            - get_stopped_equivalence_factor(float(lead_one.vLead))) if lead_one.present else 0.
    self.a_cruise, self.a_cruise_max, self.a_cruise_max_source = get_cruise_accel(
      sm['selfdriveState'].experimentalMode, v_cruise, v_ego,
      self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
      accel_coast, self.allow_throttle, lead_free,
      chase=(CHASE_ON and lead_one.present and float(lead_one.vLead) > v_ego + CHASE_DV
             and float(lead_one.dRel) >= self.follow_distance
             and (not CHASE_NEED_LEAVING or is_chase(sm['radarState'].leadOne))),
      chase_old=self.launch_chase_old)
    # ease off before a corner the model can see. it is a limit on cruise rather than a
    # separate plan source, so it just takes the lower of the two.
    self.curve_speed.update(sm, not long_control_off, sm['carState'].gasPressed, v_ego, sm['carState'].aEgo)
    curve_limited = False
    if self.curve_speed.is_active:
      curve_limited = self.curve_speed.a_target < self.a_cruise
      self.a_cruise = min(self.a_cruise, self.curve_speed.a_target)

    # a traffic signal ahead with nothing in front: ease off early - see light_ahead. Off unless its
    # flag file exists; a fault in it must never cost the plan, so it just stands down.
    try:
      _cs = sm['carState']
      self.light_ahead.update(sm['modelV2'], v_ego, radar.leadOne, _cs.leftBlinker or _cs.rightBlinker,
                              sm['selfdriveState'].enabled and not reset_state)
      if self.light_ahead.a_limit is not None:
        self.a_cruise = min(self.a_cruise, self.light_ahead.a_limit)
    except Exception:
      cloudlog.exception("light_ahead failed")

    # how fast this stretch has been driven before - see road_memory_limit. Only ever a ceiling; a fault stands it down.
    try:
      self.road_memory.update(v_ego, sm['selfdriveState'].enabled and not reset_state, self.a_cruise)
      if self.road_memory.a_limit is not None:
        self.a_cruise = min(self.a_cruise, self.road_memory.a_limit)
    except Exception:
      cloudlog.exception("road_memory failed")

    # What cruise is really working to, in the units the dash shows so it can sit beside the
    # driver's MAX. On its own the set speed says nothing the driver cannot already read, so
    # the number is only worth showing when something has taken speed off it. The corner is
    # the one limiter here that computes a speed rather than an acceleration - the rest
    # reach the car as a_cruise - and its target only means anything while it is active,
    # holding the last corner's value otherwise.
    v_cruise_shown = v_cruise
    if self.curve_speed.is_active:
      v_cruise_shown = min(v_cruise_shown, self.curve_speed.v_target)
    if self.road_memory.active and self.road_memory.v_allow is not None:
      v_cruise_shown = min(v_cruise_shown, self.road_memory.v_allow)
    self.v_cruise_dash = true_to_dash(v_cruise_shown)

    # A lead the model is unsure about, too far out to hold as a lead. On 2026-09-03 a car
    # 62 to 80 m ahead had its probability swinging between 0.09 and 0.85; every dip dropped
    # it, the road was judged clear for ten seconds, cruise wound 26 km/h up to 53, and it
    # locked on again at 43 m closing at 20 km/h, which needed -3.0 m/s^2 and the driver.
    # Holding a full lead that far out is a coin toss (0.9:1, see radard's LEAD_HOLD_*), but
    # simply not speeding up is a bounded thing to be wrong about: over 2.27 hours this
    # holds cruise back for 43 s an hour and a real lead turns up within six seconds 82% of
    # the time, so 8 s an hour of not accelerating at nothing.
    weak_lead_now = False
    if not radar.leadOne.present and v_ego > WEAK_LEAD_MIN_SPEED:
      leads = sm['modelV2'].leadsV3
      if len(leads):
        lead_x = leads[0].x[0] - RADAR_TO_CAMERA
        weak_lead_now = (WEAK_LEAD_MIN_PROB <= leads[0].prob < WEAK_LEAD_MAX_PROB
                         and WEAK_LEAD_MIN_DIST <= lead_x < WEAK_LEAD_MAX_DIST)
    # the onset has to hold, the release does not - see WEAK_LEAD_MIN_FRAMES
    self.weak_lead_frames = self.weak_lead_frames + 1 if weak_lead_now else 0
    weak_lead = self.weak_lead_frames >= WEAK_LEAD_MIN_FRAMES
    if weak_lead:
      self.a_cruise = min(self.a_cruise, 0.)

    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if sm['selfdriveState'].experimentalMode or self.junction.active:
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])

    # a_min bounds what the solver may plan for the junction stop, but the accel handed on
    # is read back off the trajectory, and that can ask for more than the solve was allowed.
    # Replaying 9/9 it reached -2.84 against a bound of -1.50, on frames where the stop was
    # 14 m off at 27 km/h and the geometry really did need 2.03 - open loop, so the car in
    # the log never slowed and the point kept counting down towards it. A closed loop should
    # not get there, but the bound is the whole reason this may commit on a guess, so hold
    # the stop to it wherever the approach came from. Only the stop: a lead that is nearer
    # brakes on its own terms.
    if stop_x is not None and self.mpc.source == LongitudinalPlanSource.e2e:
      output_a_target = max(output_a_target, STOP_MAX_DECEL)

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

    # Stopped behind a car that has not gone: stay stopped, and in the stopping state, until it
    # has moved 1 m on its own - see standstill_hold.
    # Still rolling when it holds: brake with a request rather than should_stop, whose StopReq this car
    # answers with its own -0.2 stop - see standstill_hold's "The stop distance is the line".
    held = self.standstill_hold.update(not reset_state, v_ego, radar.leadOne, sm['carState'].gasPressed)
    if held:
      output_a_target = min(output_a_target, 0.0)
      if v_ego < STANDSTILL_HOLD_ARM_SPEED:
        self.output_should_stop = True
    else:
      output_a_target = min(output_a_target, self.standstill_hold.launch_cap(v_ego))
    output_a_target = min(output_a_target, self.standstill_hold.gap_limit(not reset_state, v_ego, radar.leadOne, held))

    # no throttle towards a lead we are already closing on - see COAST_*
    if COAST_TTC > 0. and not reset_state and output_a_target > 0. and self.plan_reason in (PlanReason.cruise, PlanReason.lead):
      lead = radar.leadOne
      if lead.present:
        closing = v_ego - float(lead.vLead)
        if closing > COAST_CLOSING and (float(lead.dRel) - self.follow_distance) / closing < COAST_TTC:
          output_a_target = 0.

    # A steady foot between small corrections - see long_deadzone. Only for what cruise or a
    # lead asked; a corner, a junction, the model's stop or a planned stop go straight through.
    # v_desired_filter below integrates this output, so the MPC starts its next solve from the
    # speed actually being held rather than the one it had wanted.
    # Not while light_ahead is easing off for a signal either: its coast (-0.25) sits inside the band,
    # and on a replay of 10/13 the band held 0 there instead - the coast never happened.
    # a different car ahead: do not close on it before its speed is known - see CUT_*
    lead = radar.leadOne
    d_now = float(lead.dRel) if lead.present else -1.
    if lead.present and self.cut_prev_d > 0. and d_now - self.cut_prev_d > CUT_JUMP_M:
      self.cut_frames, self.cut_d0, self.cut_since = int(CUT_S / self.dt), d_now, 0
    else:
      self.cut_frames = max(self.cut_frames - 1, 0)
      self.cut_since += 1
    self.cut_prev_d = d_now
    self.cut_lost = 0 if lead.present else self.cut_lost + 1
    if self.cut_lost * self.dt >= CUT_LOST_S:
      self.cut_frames = 0
    if self.cut_frames > 0 and not self.cut_gentle_off and not reset_state and v_ego > CUT_MIN_V and output_a_target > CUT_A:
      away = (lead.present and self.cut_since * self.dt >= CUT_AWAY_MIN_S
              and (d_now - self.cut_d0) / (self.cut_since * self.dt) > CUT_AWAY)
      if not away and not (lead.present and is_chase(sm['radarState'].leadOne)):
        output_a_target = CUT_A

    dz_eligible = (self.plan_reason in (PlanReason.cruise, PlanReason.lead)
                   and not self.output_should_stop and not self.junction.active and not reset_state
                   and self.light_ahead.a_limit is None)
    output_a_target = self.deadzone.update(output_a_target, v_ego, dz_eligible)

    # let braking build up - see BRAKE_ONSET_*
    if BRAKE_ONSET_J > 0. and not reset_state and output_a_target < min(a_prev, 0.0) and v_ego >= BRAKE_ONSET_MIN_V:
      lead = radar.leadOne
      closing = v_ego - float(lead.vLead) if lead.present else 0.
      urgent = (stop_x is not None or self.junction.a_floor < 0. or self.fcw or
                (lead.present and ((closing > 0.1 and float(lead.dRel) / closing < BRAKE_ONSET_TTC) or
                                   (self.follow_distance > 0. and float(lead.dRel) < BRAKE_ONSET_GAP * self.follow_distance))))
      if not urgent:
        output_a_target = max(output_a_target, min(a_prev, 0.0) - BRAKE_ONSET_J * self.dt)

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
    sp_send.longitudinalPlanSP.followDistance = float(self.follow_distance)
    sp_send.longitudinalPlanSP.aCruiseMax = float(self.a_cruise_max)
    sp_send.longitudinalPlanSP.aCruiseMaxSource = self.a_cruise_max_source
    pm.send('longitudinalPlanSP', sp_send)
