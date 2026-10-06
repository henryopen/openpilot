import math
import os
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.nn_feedforward import load_model
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.swaglog import cloudlog
from openpilot.common.pid import PIDController

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3

# The P term on its own asks for far more than the controller can deliver. Measured over the
# 09-18 drives its magnitude averages 3.34 against a PID output limit of latAccelFactor
# (2.80 on this car) and peaks at 12.66. Past that limit the output sits at full lock, so a
# sign change in the error swings it stop to stop and the wheel hunts instead of settling -
# in four big-corner unwinds the output made 25 full-amplitude reversals at 0.4-1.2 Hz while
# the setpoint and the measurement it was tracking made none between them.
# Capping P's share gives the loop a linear range again: replayed over those same unwinds it
# takes the reversals to 2. Anything below 1.0 buys no further reduction and only costs
# steering effort, so this is the knee.
# It only bites on big corners - |p| clears 1.0 on 81% of frames past 60 degrees of steering
# and 95% past 120, against 6-12% everywhere else, so straights and ordinary corners are
# untouched and this is not the 09-15 lower-KP attempt in disguise.
P_MAX_CONTRIB = 1.0

# On a straight, P at twilsonco's strength (2026-10-04). Over 10-01..10-02 (197 min lateral) the driver took
# the wheel 3.66 times a minute and 57% of those were on straight road; in 71-78% of them his hand was on the
# wheel first (60+, under the takeover line) and the controller then pushed back against it - one-sided, about
# 3.6x its usual straight-road torque, with P the larger part (0.37 against the feedforward's 0.17 at
# 15-30 km/h). The wheel also hunted by itself on straights, P again (its 1 s variation 3.4x the
# feedforward's). twilsonco keeps P in torque space with a low-speed curvature term; in our terms that is
# 1 + LSF(v)^2 / v^2 with LSF [12, 4, 1, 0] at [0, 10, 20, 30] m/s - a third of KP_INTERP at 15-36 km/h.
# Taken alone it costs corners, where the feedforward still leaves ~10% for P to carry, so it is used only
# while the request is straight: KP_INTERP above STRAIGHT_KP_BLEND[1] of requested lateral accel, these below
# STRAIGHT_KP_BLEND[0], linear between. Keyed on the request, not the measurement.
# Closed loop (real LatControlTorque, the car's torque limits, a steering model fitted to 10-01/02 that
# reproduces the logs' straight-road torque, P and hand push-back within 15% and corner tracking 0.91-1.01
# against 0.93-1.01), 225 windows / 75 min, now -> this at 15-30 / 30-50 / 50-80 km/h: pushing back
# against a resting hand 67.7 / 31.4 / 30.3 -> 33.3 / 22.4 / 24.8 counts, straight-road torque variation
# 28.3 / 21.6 / 17.1 -> 16.8 / 16.7 / 15.2; corners 0.91 / 0.93 / 0.98 -> 0.90 / 0.93 / 0.98 of the request,
# big corners 98% unchanged; straight-road 3 s drift (open loop, the plan not re-centring) 0.07 / 0.07 /
# 0.09 -> 0.10 / 0.10 / 0.11 m. Also dropping the error-keyed friction on straights cut push-back further
# (27.7 / 18.3 / 21.4) but drifted more (0.10 / 0.12 / 0.14 m); P everywhere at this strength cost corners
# 0.86 / 0.88 of the request.
KP_STRAIGHT_INTERP = [126.4, 52.8, 28.0, 11.2, 3.56, 1.64, 1.16, 1.03, 1.0]   # on INTERP_SPEEDS
STRAIGHT_KP_BLEND = [0.15, 0.4]   # m/s^2 of requested lateral accel: straight -> corner

# Friction in corners, keyed on where the request is going (2026-10-06). The driver: can turning in and
# unwinding not be smoother. With the neural feedforward carrying a corner there was no friction term at all
# - the network's part leaves get_friction out, and the linear part that has it is blended away - so where
# the wheel has to start or change direction against the rack's stiction it waits for P. Over 10-01..10-05,
# turning in and unwinding on its own, openpilot's wheel stalled at least once in 44% / 50% of them against
# 37% / 34% when the driver turns it. So in a corner (STRAIGHT_KP_BLEND's weight) friction is added in the
# direction the request is moving - twilsonco's way, not the error's - at CURVE_JERK_FRICTION of the car's
# coefficient. Closed loop (the steering model of KP_STRAIGHT_INTERP, 36 corners openpilot took alone):
# stalls turning in 0.68 -> 0.52 a corner (at least one 36 -> 30%), unwinding 0.65 -> 0.56 (29 -> 25%);
# over 225 ordinary windows corner error at 30-50 / 50-80 km/h 0.051 -> 0.042 / 0.034 -> 0.028, straights
# and push-back against a resting hand unchanged, big corners 98 -> 97%. Full strength unwound better but
# turned in no better and erred more past 80 km/h; ungated it shook the wheel on straights (30-50 km/h
# 8.4 -> 15.5 swings a minute), the request's direction there being noise.
CURVE_JERK_FRICTION = 0.5

# In a corner, P and I see the request averaged over the frames either side of the one they track (2026-10-06).
# The driver: a 30 degree turn goes in as 10, 10, 10, not in one movement. Over 10-01..10-05, turning in on its
# own, openpilot's wheel went in 2.7 bursts of ~10 degrees (6.6-8.8 past 20 km/h) where the driver's own come
# in 13-19; past 20 km/h the request moved in 0.6-1.1 bursts a second and the wheel in 1.6-1.9, the torque
# pulling back 2.5-3 times a second. Band-passed 1.2-6 Hz, P's ripple follows the request's (0.73) as much as
# the wheel's (0.77), the two the same size: the model's frame-to-frame noise goes through P into the rack. The
# tracked frame is lat_delay back in the buffer, so the frames after it are already there and the average is
# centred - no lag added, unlike the 09-24 low-pass on the feedforward, which changed nothing. Closed loop with
# a steering model that sticks (static over kinetic friction, fitted to 10-01..10-05), 36 corners openpilot took
# alone: torque ripple turning in 16.4 -> 12.6, bursts 0.84 -> 0.65 a second, torque reversals unwinding 2.28 ->
# 1.58 a second, corner error unchanged or lower. Gated on STRAIGHT_KP_BLEND's weight, so straights are untouched.
CURVE_SETPOINT_HALF_S = 0.25
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

# Low speed runs on comma's own gains: error = setpoint - measurement, integrator frozen
# below 5 m/s. From 09-06 to 09-25 this file scaled the error up at low speed (StarPilot's
# LOW_SPEED curve, then openpilot's older one 20% higher) and let the integrator run down to
# 0.3 m/s, both so junction turns would get round. That made P 40-70% hotter under 40 km/h
# and the wheel hunted on straight roads with no hand on it: on the same 35 stretches of road
# (~100 m GPS cells) driven both before and after, 0.13 -> 2.58 back-and-forth swings a
# minute, at 2.3 Hz and ~20 degrees of wheel, with the model's request flat. Junction turns
# still needed the driver 92% of the time with it in, so the driver chose the straights
# (09-25). Measurements: E:/Temp/drive0925 (split.py, cellcmp.py, timeline.py).

# Presence of this file puts the lateral feedforward back on the linear conversion. Written
# by the HUD's toggle, because the driver cannot SSH into the car from the driver's seat.
NNFF_OFF_FLAG = '/data/nnff_off'

# The low-speed gain above - KP_INTERP times the low-speed error scale - is there so a junction
# turn can force the rack round a stationary tyre. On a near-straight road it is far more than
# the few degrees of trim there need, and the wheel hunts: over the 09-23 drives there were 70
# stretches (259 s) of the wheel swinging back and forth at under 35 km/h with no hand on it,
# 54 of them with the request under 0.1 m/s^2, 67 with the wheel inside 30 degrees and none
# past 90. Split into its terms, the swing is P's: setpoint 0.018, P 0.566, I 0.009,
# feedforward 0.061 (detrended std, medians), and never saturated.
# So P is scaled down only while the road is straight. "Straight" is the larger of the
# requested curvature and the curvature the wheel is actually at, so a turn and the unwind out
# of it keep the full gain: 0.003 1/m is about 7 degrees of wheel, 0.008 about 20. On those
# drives every frame past 60 degrees and every frame over 50 km/h keeps 1.0. Speed fades it
# out between 30 and 50 km/h, where the hunting stops. The integrator still sees the full
# error, so the long-run trim on a straight is unchanged.
# The 0.5 is not derived: a plant model fitted to these drives could not reproduce the wheel
# (free-run R^2 -0.08), so it is set to be compared on the road with the HUD switch below.
STRAIGHT_P_SCALE = 0.5
STRAIGHT_CURV_BP = [0.003, 0.008]   # 1/m
STRAIGHT_SPEED_BP = [30 / 3.6, 50 / 3.6]
# Presence of this file keeps the full gain on straights too - the HUD's switch, as above.
STRAIGHT_P_OFF_FLAG = '/data/straight_p_off'

# The widest |lateral acceleration| the model was trained on. Requests stay well inside it;
# this is a guard, not a working limit.
NN_ACCEL_LIMIT = 3.1

# Below this request the linear conversion is used, above it the network, and in between the
# two are crossfaded. Straight-line driving does not need the network - the constant is only
# wrong by the time the rack is being forced round, and it is the straights that the driver
# feels as "it will not centre". Replayed over the 09-16 drives, on frames asking for less
# than 0.1 m/s^2 at over 30 km/h: the linear conversion puts 5.5% of frames above the 60
# counts this rack needs before the wheel moves, the network 23.5%. Four times as much
# correction on a straight road, in the right direction but harder than the car needs.
NN_BLEND_LO = 0.15
NN_BLEND_HI = 0.50

# The network under-asks at junction speeds. It was trained with |torque| >= 0.95 thrown out,
# and a slow, tight turn is exactly where the rack needs that much, so the turns that needed
# the most torque were never in the training set and the ones that remained taught it less.
# Measured on 09-16/18/20/23/24, turning with no hand on the wheel and the lateral
# acceleration steady, torque actually sent over what the network asked for:
#
#      km/h      5-10    10-15   15-20   20-25   25-30
#      09-23     2.13    1.91    1.39    1.39    1.30
#      09-24     2.66    1.39    1.40    1.19    0.89
#      five-drive range  1.71-2.66  1.07-1.91  1.14-1.40  1.19-1.49  0.89-1.41
#
# The cap on P (P_MAX_CONTRIB) and an integrator frozen whenever a hand rests on the wheel
# leave nothing else to make that up, so the car stops short of the curvature it asked for.
# 1.4 is where 10-20 km/h sits across all five drives (after it: 0.82-1.00 at 15-20), faded
# out by 25 km/h and above that nothing changes. It is conservative below 10 km/h, where the
# gap is friction rather than gain (still 1.22-1.90 after).
# It is faded in on the turn being asked for - the request alone, not the feedforward, which
# has friction added and crosses NN_BLEND_LO on a straight. Keyed on the feedforward, the
# first version changed 33.8% of low-speed frames with the wheel under 10 degrees (p99 41.5
# counts), and those straights are where the wheel already hunts. Under 0.3 m/s^2 requested
# nothing changes; junction turns ask for 0.5-1.1.
NN_LOW_SPEED_GAIN_BP = [0.0, 20.0 / 3.6, 25.0 / 3.6]  # m/s
NN_LOW_SPEED_GAIN_V = [1.4, 1.4, 1.0]
NN_LOW_SPEED_REQ_BP = [0.3, 0.6]  # m/s^2 of requested lateral acceleration: gain faded 0 -> full
# Presence of this file turns the low-speed gain off, re-read once a second like the others.
NN_LOW_SPEED_OFF_FLAG = '/data/nnff_lowspeed_off'

# Planned jerk comes out of a difference between buffer entries, so it carries the high
# frequency of the request straight through. It is worth clipping before the friction term
# sees it, but it is NOT worth leading the setpoint with: replaying this car's own drives,
# setpoint += jerk * lat_delay doubled the frame-to-frame movement of the feedforward
# (p99 10.5 -> 20.8 counts) to buy 1 count of median feedforward. StarPilot carries that
# lead, but with a small-signal deadzone alongside it that there is nothing here to size.
MAX_LAT_JERK = 2.5  # m/s^3

# The second feedforward model (2026-10-01), <fingerprint>_v2.json, 11 inputs - retrained the way twilsonco
# trains NNFF. The driver: the steering comes in a piece at a time, nothing like a person turning the wheel,
# and it has never been right. On 09-30, turning in at 15 km/h and up, the wheel stopped about once per
# turn-in (0.3-0.6 when he turns it himself); in the 0.3 s before each stop the error fell by 0.10, P by
# 0.63 and the friction term by 0.14 - a quarter of the time flipping to push against the turn - while
# the request kept rising. Friction here is keyed on the error, so it lets go exactly when the wheel
# is moving and has to be re-broken. The first model could not carry a turn on its own either: trained
# with every frame at the torque limit thrown out, it under-asked in slow turns, hence the 1.4 below.
# v2 is trained on 1096 segments of 09-16..09-30 with those frames kept (a loss that only minds asking
# too little), balanced over speed x lateral accel, mirrored, held monotonic in lateral accel, and
# given what twilsonco's model has and ours did not: road roll, and where the plan goes 0.3-1.5 s on.
# Its friction is its own - smooth in the requested direction - so get_friction is left out of its
# part, and so is the 1.4. Held-out 09-29/30, fed exactly what it gets here (the setpoint, its history,
# the plan), feedforward against the torque actually sent, RMSE: 10-15 km/h 0.092 -> 0.083, 25-35
# 0.074 -> 0.064, 50-80 0.060 -> 0.039, 80-130 0.063 -> 0.029; frame-to-frame p99 on straights
# 0.018 -> 0.025. The blend is unchanged, so near-straight driving is the linear conversion as before.
# /data/nnff_v1 at start-up keeps the first model and its path.
NN2_PAST_S = (0.3, 0.2, 0.1)
NN2_FUTURE_S = (0.3, 0.6, 1.0, 1.5)
NN2_JERK_HZ = 0.5      # the rate input is filtered this way in training; the raw rate made it jump 3x per frame

# Coming out of a turn the integrator holds wind-up from the turn itself, and letting it keep
# integrating through the unwind makes the wheel come back late. Freezing it on the setpoint
# falling was tried on 2026-09-06 and taken back out on 09-07: the rate is a difference
# between consecutive frames divided by dt, and at 100 Hz the setpoint's own frame-to-frame
# noise is already 4.77 m/s^3 at p90, so a -1.0 threshold sits inside the noise and fired on
# 5.9% of frames with no turn in sight. Over that drive the integrator term fell to a fifth
# of what it had been the day before (0.0775 -> 0.0141 at the median on low-speed straights)
# while P had to make up the difference, and the output at 10-30 km/h reached its limit.
# There is no threshold that works here: -10 is the first value clear of the noise and it
# fires on 0.1% of frames, which is nothing. Left out until there is a measurement of the
# unwind that is not a one-frame difference.

class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    # Neural feedforward, trained on this car's logs. One latAccelFactor cannot cover a rack
    # whose measured factor runs 1.44 at 15 km/h to 8.48 at 110. Falls back to the linear
    # conversion when there is no model for the car, or when the flag file is present -
    # a file rather than a param key so that turning it off needs no rebuild, and re-read
    # once a second so the HUD button takes effect without a restart.
    self.nn_model = load_model(CP.carFingerprint)
    self.nn = None if os.path.isfile(NNFF_OFF_FLAG) else self.nn_model
    self.nn_recheck_frames = int(round(1.0 / self.dt))
    self.nn_recheck = self.nn_recheck_frames
    self.nn_past_frames = [int(round(t / self.dt)) for t in (0.3, 0.2, 0.1)]
    self.nn_v2 = self.nn_model is not None and self.nn_model.input_size == 11
    self.nn2_jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * NN2_JERK_HZ), self.dt)
    self.nn2_prev_setpoint = 0.0
    self.setpoint_half_frames = int(round(CURVE_SETPOINT_HALF_S / self.dt))
    self.plan = None   # modelV2, handed over by controlsd each frame
    cloudlog.info(f"lateral feedforward: {('neural v2' if self.nn_v2 else 'neural') if self.nn else 'linear'}")
    self.straight_p = not os.path.isfile(STRAIGHT_P_OFF_FLAG)
    cloudlog.info(f"straight-road P scale: {'on' if self.straight_p else 'off'}")
    self.nn_low_speed = not os.path.isfile(NN_LOW_SPEED_OFF_FLAG)
    cloudlog.info(f"neural feedforward low-speed gain: {'on' if self.nn_low_speed else 'off'}")

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def _nn_feedforward(self, lateral_accel, v_ego, lateral_jerk, request):
    """Feedforward torque for a requested lateral acceleration, in this file's sign convention.

    Two things this has to get right, both of which were wrong when it first went on the car:

    Sign. The model was trained on carOutput.actuatorsOutput.torque, which is what goes out on
    CAN - already negated relative to the output_torque this file works in, because update()
    returns -output_torque. So the model's answer is negated here. Measured on the drive that
    ran without this: the correlation between requested lateral acceleration and the torque
    actually sent went from -0.786 to +0.153, i.e. the car pushed against the turn.

    Range. The model has seen |lateral acceleration| up to 3.13 m/s^2 and nothing outside it.
    The request is a physical quantity and stays inside that - 0.03% of frames exceed 3 - but
    it is clipped anyway, because a network has no reason to behave outside its training set.
    """
    linear = self.torque_from_lateral_accel(lateral_accel, self.torque_params)
    blend = float(np.clip((abs(lateral_accel) - NN_BLEND_LO) / (NN_BLEND_HI - NN_BLEND_LO), 0.0, 1.0))
    if blend == 0.0:
      return linear

    buf = self.lat_accel_request_buffer
    past = [float(np.clip(buf[max(len(buf) - 1 - n, 0)], -NN_ACCEL_LIMIT, NN_ACCEL_LIMIT))
            for n in self.nn_past_frames]
    inputs = [v_ego, float(np.clip(lateral_accel, -NN_ACCEL_LIMIT, NN_ACCEL_LIMIT)),
              float(np.clip(lateral_jerk, -MAX_LAT_JERK, MAX_LAT_JERK))] + past
    gain = 1.0
    if self.nn_low_speed:
      fade = float(np.interp(abs(request), NN_LOW_SPEED_REQ_BP, [0.0, 1.0]))
      gain = 1.0 + (float(np.interp(v_ego, NN_LOW_SPEED_GAIN_BP, NN_LOW_SPEED_GAIN_V)) - 1.0) * fade
    return (1.0 - blend) * linear + blend * gain * -self.nn.evaluate(inputs)

  def _plan_future(self, fallback):
    """The plan's lateral acceleration NN2_FUTURE_S ahead, or fallback where there is no plan."""
    m = self.plan
    try:
      w, vx = m.orientationRate.z, m.velocity.x
      n = len(ModelConstants.T_IDXS)
      if len(w) != n or len(vx) != n:
        return [fallback] * len(NN2_FUTURE_S)
      la = [float(w[i]) * float(vx[i]) for i in range(n)]   # capnp readers do not slice
      return [float(np.interp(t, ModelConstants.T_IDXS, la)) for t in NN2_FUTURE_S]
    except AttributeError:
      return [fallback] * len(NN2_FUTURE_S)

  def _nn2_feedforward(self, lateral_accel, v_ego, setpoint, jerk, delay_frames, roll):
    """v2: see NN2_* above. lateral_accel is what the linear part and the blend have always used."""
    linear = self.torque_from_lateral_accel(lateral_accel, self.torque_params)
    blend = float(np.clip((abs(lateral_accel) - NN_BLEND_LO) / (NN_BLEND_HI - NN_BLEND_LO), 0.0, 1.0))
    if blend == 0.0:
      return linear
    buf = self.lat_accel_request_buffer
    clip = lambda x: float(np.clip(x, -NN_ACCEL_LIMIT, NN_ACCEL_LIMIT))  # noqa: E731
    past = [clip(buf[max(len(buf) - delay_frames - int(round(t / self.dt)), 0)]) for t in NN2_PAST_S]
    future = [clip(x) for x in self._plan_future(setpoint)]
    inputs = [v_ego, clip(setpoint), float(np.clip(jerk, -MAX_LAT_JERK, MAX_LAT_JERK)), roll * ACCELERATION_DUE_TO_GRAVITY] + past + future
    return (1.0 - blend) * linear + blend * -self.nn.evaluate(inputs)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    self.nn_recheck -= 1
    if self.nn_recheck <= 0:
      self.nn_recheck = self.nn_recheck_frames
      nn = None if os.path.isfile(NNFF_OFF_FLAG) else self.nn_model
      if (nn is None) != (self.nn is None):
        cloudlog.info(f"lateral feedforward switched to {'neural' if nn else 'linear'}")
      self.nn = nn
      sp = not os.path.isfile(STRAIGHT_P_OFF_FLAG)
      if sp != self.straight_p:
        cloudlog.info(f"straight-road P scale switched {'on' if sp else 'off'}")
      self.straight_p = sp
      ls = not os.path.isfile(NN_LOW_SPEED_OFF_FLAG)
      if ls != self.nn_low_speed:
        cloudlog.info(f"neural feedforward low-speed gain switched {'on' if ls else 'off'}")
      self.nn_low_speed = ls
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = float(np.clip(self.jerk_filter.update(raw_lateral_jerk), -MAX_LAT_JERK, MAX_LAT_JERK))

    setpoint = expected_lateral_accel
    nn2_jerk = self.nn2_jerk_filter.update((setpoint - self.nn2_prev_setpoint) / self.dt)
    self.nn2_prev_setpoint = setpoint

    current_kp = np.interp(CS.vEgo, INTERP_SPEEDS, KP_INTERP)
    # see KP_STRAIGHT_INTERP: P at twilsonco's strength while the request is straight
    w_curve = float(np.interp(abs(setpoint), STRAIGHT_KP_BLEND, [0.0, 1.0]))
    kp_straight = float(np.interp(CS.vEgo, INTERP_SPEEDS, KP_STRAIGHT_INTERP))
    p_blend = (kp_straight + w_curve * (current_kp - kp_straight)) / max(current_kp, 1e-3)
    # see CURVE_SETPOINT_HALF_S
    half = min(self.setpoint_half_frames, delay_frames - 1)
    error_setpoint = setpoint
    if half > 0:
      buf = self.lat_accel_request_buffer
      c = len(buf) - delay_frames
      error_setpoint = float(np.mean([buf[i] for i in range(max(c - half, 0), c + half + 1)]))
    error = (setpoint + w_curve * (error_setpoint - setpoint)) - measurement

    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    # Friction above keeps the full error: it saturates at FRICTION_THRESHOLD (0.3) and the cap
    # below sits at 0.06 at 16 km/h, so sharing one clipped error would throw most of the
    # friction compensation away and make the rack slower to break away, not faster.
    error = float(np.clip(error, -P_MAX_CONTRIB / max(current_kp, 1e-3), P_MAX_CONTRIB / max(current_kp, 1e-3)))

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      p_scale = p_blend
      if self.straight_p:
        road_curvature = max(abs(desired_curvature), abs(measured_curvature))
        straight = 1.0 - float(np.interp(road_curvature, STRAIGHT_CURV_BP, [0.0, 1.0]))
        slow = float(np.interp(CS.vEgo, STRAIGHT_SPEED_BP, [1.0, 0.0]))
        p_scale = 1.0 - (1.0 - STRAIGHT_P_SCALE) * straight * slow
      if self.nn is None:
        output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator,
                                          p_scale=p_scale)
        output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)
        ff_torque = self.torque_from_lateral_accel(ff, self.torque_params)
      else:
        # The network converts a requested lateral acceleration to the torque this rack needs
        # for it, which is a statement about the car and only holds for accelerations the car
        # can actually make. The PID's output is not that - it is a control signal, and on the
        # drive this first went out on it reached 4150 m/s^2 against a training range of 3.13,
        # so 36% of frames were extrapolation. Feed the network the request, which is physical,
        # and leave the feedback on the linear conversion where any magnitude is meaningful.
        if self.nn_v2:
          ff_torque = self._nn2_feedforward(ff, CS.vEgo, setpoint, nn2_jerk, delay_frames, params.roll)
        else:
          ff_torque = self._nn_feedforward(ff, CS.vEgo, desired_lateral_jerk, future_desired_lateral_accel)
        feedback_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=0.0,
                                            freeze_integrator=freeze_integrator, p_scale=p_scale)
        output_torque = ff_torque + self.torque_from_lateral_accel(feedback_lataccel, self.torque_params)
        # see CURVE_JERK_FRICTION
        output_torque += CURVE_JERK_FRICTION * w_curve * get_friction(desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD,
                                                                      self.torque_params) / max(self.torque_params.latAccelFactor, 0.1)
        output_torque = float(np.clip(output_torque, -self.steer_max, self.steer_max))
        output_lataccel = feedback_lataccel

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      # with the neural feedforward the PID carries no feedforward term of its own, so log the
      # request that went to the network instead - same meaning, so the field stays comparable
      # across the two paths
      pid_log.f = float(self.pid.f if self.nn is None else ff)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
