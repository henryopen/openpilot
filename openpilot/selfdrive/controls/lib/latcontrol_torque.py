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
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

# The assumption above holds at speed and falls apart below it: lateral acceleration is
# curvature times speed squared, so at 15 km/h a junction turn asks for a couple of m/s^2
# while the steering rack still has to be forced round against a stationary tyre. Measured
# on this car, the same torque produces 0.08 of the lateral acceleration the model predicts
# at 10-20 km/h and 2.8 of it above 70. The error is what shrinks with speed, so scale the
# error back up rather than inventing a second feedforward.
#
# The curve first taken here on 09-06 was StarPilot's, [12, 10.5, 8, 5]. That was the wrong
# half of a pair: StarPilot runs it against KI = 0.35, and this car runs comma's KI = 0.15,
# so the low gain arrived without the integrator that pays for it. What openpilot itself
# carried, and what FrogPilot and CarrotPilot still carry, is 20% higher below 20 m/s.
# Replayed over this car's 09-15 drives (100.0% reproduction of the logged error and P):
# error goes up 1.185x at 3-7 km/h decaying to 1.099x by 60, which lifts median torque there
# from 54.3 to 63.6 counts - across the 60 counts this rack needs before the wheel moves at
# all, so the share of frames that clear it goes 46.4% -> 52.3%. The cost is saturation
# +1.35pt and frame-to-frame movement +14.5%, against +12.9pt and +84% for the factor-table
# plus lower-KP attempt that was rejected on 09-15 for exactly those two numbers.
LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]
LOW_SPEED_MIN = 1.0  # keeps the divide below sane at a standstill

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

# Upstream freezes the integrator below 5 m/s. That guard is for cars whose rack will not
# move at low speed; this one is a full-time lateral car with minSteerSpeed = 0, so all it
# does here is switch the integrator off for the whole junction speed range. Measured over
# the 2026-09-06 drive (294k active frames): in a turn held steady at 10-20 km/h the
# integrator sat at |I| p50 0.1032 with p90 0.1034 -- frozen, not small -- and the car
# stayed 11% short of the requested curvature no matter how long the turn was held, while
# above 45 km/h the same shortfall decayed to 4%. Take the threshold from the car, the way
# StarPilot does, and keep a floor so it still resets at a standstill.
MIN_LATERAL_CONTROL_SPEED = 0.3  # m/s

# Planned jerk comes out of a difference between buffer entries, so it carries the high
# frequency of the request straight through. It is worth clipping before the friction term
# sees it, but it is NOT worth leading the setpoint with: replaying this car's own drives,
# setpoint += jerk * lat_delay doubled the frame-to-frame movement of the feedforward
# (p99 10.5 -> 20.8 counts) to buy 1 count of median feedforward. StarPilot carries that
# lead, but with a small-signal deadzone alongside it that there is nothing here to size.
MAX_LAT_JERK = 2.5  # m/s^3

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
    self.integrator_reset_speed = max(CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED)
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
    cloudlog.info(f"lateral feedforward: {'neural' if self.nn else 'linear'}")
    self.straight_p = not os.path.isfile(STRAIGHT_P_OFF_FLAG)
    cloudlog.info(f"straight-road P scale: {'on' if self.straight_p else 'off'}")

  def update_torque_parameters(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def _nn_feedforward(self, lateral_accel, v_ego, lateral_jerk):
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

    # correcting in lateral acceleration space understates how far off the car is at low
    # speed, where the same miss is worth far less acceleration; scale it back to what the
    # steering rack actually has to do
    low_speed_factor = (np.interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y) / max(CS.vEgo, LOW_SPEED_MIN)) ** 2
    current_kp = np.interp(CS.vEgo, INTERP_SPEEDS, KP_INTERP)
    error = (setpoint - measurement) * (1 + low_speed_factor / max(current_kp, 1e-3))

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

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < self.integrator_reset_speed
      p_scale = 1.0
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
        ff_torque = self._nn_feedforward(ff, CS.vEgo, desired_lateral_jerk)
        feedback_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=0.0,
                                            freeze_integrator=freeze_integrator, p_scale=p_scale)
        output_torque = ff_torque + self.torque_from_lateral_accel(feedback_lataccel, self.torque_params)
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
