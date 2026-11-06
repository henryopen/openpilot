import os

import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# How quickly the ESC may build braking - SCC14 JerkLowerLimit (2026-10-17). The driver: braking still feels
# abrupt, not like the stock ACC. The stock ACC on this car (routes 00000001 / 00000006, its own SCC14 off the
# bus) sends JerkLowerLimit 1.2 (0.5 at times) and JerkUpperLimit 1.0; openpilot sent 5.0. Matched against
# it (following at 15-28 km/h, closing at 2-6 km/h) the request 0.5 s into braking was about the same,
# 0.61 stock against 0.70, but the car was at -0.22 against -0.35: it is the car building the braking
# faster, not the request. That end cannot be simulated, so this is tried on the road. Below the urgent
# line the planner already deepens braking by at most BRAKE_ONSET_J (1.5 m/s^3), so 1.2 here is the stock
# value the car was tuned for; anything urgent gets 5.0 at once and for LJ_HOLD after: a request under
# LJ_URGENT_ACCEL, or braking deepening faster than LJ_URGENT_RATE (the planner skips its build-up for a lead
# reached within 3 s, nearer than 0.7 of the follow distance, a committed stop or an FCW, and those come in
# faster than that). Only the braking side is watched: the step from accelerating down to zero is not urgency.
# Replayed over 10-14/10-16's commands (122 min): 51% of the time braking is at 1.2, 28% of brakings never
# leave it; all 138 harder than -1.5 switch to 5.0, a median 0.38 s before the request reaches -1.0 (at 2.5
# m/s^3 63% / 46%, but the switch only as the request reached -1.0).
# Custin only - the stock numbers are this car's. /data/stock_jerk_off brings back 5.0 throughout.
LJ_COMFORT = 1.2          # m/s^3, stock
LJ_URGENT = 5.0           # m/s^3, what was always sent
LJ_URGENT_ACCEL = -1.0    # m/s^2
LJ_URGENT_RATE = 2.0      # m/s^3, braking deepening faster than this over LJ_WINDOW
LJ_WINDOW = 5             # SCC frames (50 Hz) -> 0.1 s
LJ_HOLD = 1.5             # s
STOCK_JERK_OFF_FLAG = '/data/stock_jerk_off'

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

# On some HKG CAN and CAN FD non-CANFD_ALT_BUTTONS, the cancel button (CF_Clu_CruiseSwState / CRUISE_BUTTONS = 4) is
# a pause/resume toggle, not a dedicated cancel. Firing it mid-brake inadvertently can cause a re-enable attempt
# and triggers the "SCC Conditions Not Met" alert. Delaying the button send lets factory SCC disengage
# naturally on brake press. We send ~100 ms later if it fails to do so, or if we want to cancel for another reason.
CANCEL_BUTTON_DELAY_FRAMES = 10

# The driver torque limit below only pulls us back when the driver is pushing the *other* way.
# Push the same way and the allowance opens up past STEER_MAX, so nothing is holding the two
# apart and the MDPS drops out with ToiUnavail - measured on the Custin, twice in 7.2 hours,
# both at low speed with a big angle and our torque pinned at 384 while the driver helped.
# Backing off while the driver is already turning costs nothing (they are supplying the
# torque) and takes the time spent above a 500 count sum from 19.3 s to 0.3 s. Hands off the
# wheel this changes nothing at all.
DRIVER_ASSIST_YIELD = 1.0
DRIVER_ASSIST_MIN_TORQUE = 120
# STEER_MAX went above 384 with a lower allowance (409 / 188 since 10-16, see values.py). The yield
# above was "384 - (|driver| - 200)", a sum of ours and the driver's of at most 584, and that sum
# is what the MDPS cares about. Held at 584 here rather than letting it rise with the ceiling:
# same as before whenever the driver is turning with us, and the full ceiling only hands off.
DRIVER_ASSIST_SUM_MAX = 584
# And against a driver the old 384 stays the ceiling: the driver limit alone would let a light
# opposing hand (under the allowance) meet the full ceiling, more than before - the extra is for
# the wheel when nobody is on it. 50, because a hand-off wheel being driven through a corner
# reads up to about +-40 of its own inertia (10-08..10-14 turn-ins), which must not count.
DRIVER_AGAINST_MIN = 50

# Handing the wheel over rather than being trimmed down to a share of it. Measured on the
# 09-20 drive: hands off, the controller gets 94% of what it asks for and the wheel unwinds
# at 100 deg/s; with a hand on it, 67% and 80 deg/s, and the driver limit is what takes the
# difference on 59.4% of those frames. The driver's torque does not separate intent from a
# resting hand - pushing the wheel reads a median 141 and resisting it 100, and no threshold
# splits them (at 200: 33% of pushes, 19% of rests) - so this does not try to read intent.
# It draws a line instead: under STEER_DRIVER_ALLOWANCE we take the wheel and the limit no
# longer trims us, over it we let go of it completely. What is gone is the middle, where we
# held a third of the wheel and the driver held the rest and neither got what they wanted.
# Held for half a second and released at a lower torque so a wobble across the line does not
# chatter; the rate limit takes the torque to zero and back over ~0.15 s either way, so the
# handover is not felt as a step.
# 300 rather than the allowance itself. A hand simply resting on the wheel reaches 227 at
# the 90th percentile and 305 at the 99th, so a line at the allowance (200) would be crossed
# by the resting hand alone: replayed over the 09-20 drive it hands the wheel over for 22.9%
# of the drive and 60.3% of every unwind, which is not assistance. Scanned against that:
#
#      threshold   handed over   of each unwind   of what we asked for, when held
#         200         22.9%          60.3%                 100%
#         250         10.9%          19.0%                  97%
#         300          3.7%          10.4%                  99%
#         400          0.5%           1.9%                  98%
#
# 300 sits just past where a resting hand reaches and inside where a deliberate push lives
# (90th percentile 285), so it answers effort rather than contact, and the 99% says the band
# between 200 and 300 costs almost nothing.
# 09-24: raised to 400 against our torque (d15e4cd2, 279d6c3a) on the reading that the driver
# torque against ours at turn-in was a resting hand the wheel was turning through. It was not.
# Of the 36 times the line was crossed that drive, the wheel was moving the driver's way in 25
# and held still in 7, and in 32 of the 33 against us the controller was pushing exactly where
# its own error said - the driver was correcting the path openpilot had chosen (too tight at
# 93 and 97 degrees, unwinding early at 252). Handing the wheel over there is the point of
# this line, so it is back at 300 both ways. Sign note: this fork returns -output_torque, so
# torqueState's desired/actual lateral accel carry the opposite sign to actuators.torque.
DRIVER_HANDOVER_TORQUE = 300
DRIVER_HANDOVER_FRAMES = 50      # 0.5 s at 100 Hz
DRIVER_HANDOVER_RELEASE = 250    # hysteresis, so a wobble across the line does not chatter

# A faster limit for unwinding out of a turn was carried here from 2026-09-06 to 09-07 and
# is gone because its premise was wrong. It read "unwinding means building torque the other
# way", so it applied when the new torque opposed the last one. Coming out of a turn does
# not do that: the torque falls from large to small with the same sign, which is governed by
# STEER_DELTA_DOWN (7) and never touches STEER_DELTA_UP at all. Over the drive that carried
# it, all 269 sign reversals while engaged had a previous torque under 20 counts out of 384
# - a threshold of 20 leaves exactly none - at a median 49.6 km/h and 4.4 degrees of wheel.
# They are not turns being unwound, they are the torque crossing zero while running straight,
# which is what it does all the time. So the rule never once did what it was written for,
# and meanwhile it handed the fastest ramp to the most common state on the road:
# steerFaultTemporary went 0.10% -> 0.58% of frames, latActive 55% -> 7%, and between 30 and
# 60 km/h lateral was available 10.7% of the time against 96.8% the day before.
#
# Anything aimed at how the wheel returns has to act on STEER_DELTA_DOWN or on the torque
# request itself. Rate limits keyed on the sign of the torque will fire on straight roads.


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    # only this car has the measurements behind the two changes below
    self.tuned_lateral = CP.carFingerprint == CAR.HYUNDAI_CUSTIN_1ST_GEN
    self.handover_frames = 0
    self.stock_jerk = CP.carFingerprint == CAR.HYUNDAI_CUSTIN_1ST_GEN
    self.brake_hist = []
    self.jerk_urgent_frames = 0
    self.jerk_flag_off = False
    self.last_button_frame = 0
    self.cancel_counter = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_TORQUE_SCALE))
    limits = self.params
    if self.tuned_lateral:
      driver_torque = CS.out.steeringTorque
      # Past the allowance, let go of the wheel instead of keeping hold of whatever the
      # driver limit leaves us. Below it we now get everything we ask for, so this is where
      # the two meet: there is no band left where both of us are pulling on the same wheel.
      if abs(driver_torque) > DRIVER_HANDOVER_TORQUE:
        self.handover_frames = DRIVER_HANDOVER_FRAMES
      elif self.handover_frames > 0 and abs(driver_torque) < DRIVER_HANDOVER_RELEASE:
        self.handover_frames -= 1
      if self.handover_frames > 0:
        new_torque = 0
      elif new_torque * driver_torque > 0:
        cap = max(min(self.params.STEER_MAX, DRIVER_ASSIST_SUM_MAX - DRIVER_ASSIST_YIELD * abs(driver_torque)), DRIVER_ASSIST_MIN_TORQUE)
        new_torque = int(round(np.clip(new_torque, -cap, cap)))
      elif new_torque * driver_torque < 0 and abs(driver_torque) > DRIVER_AGAINST_MIN:
        cap = self.params.STEER_TORQUE_SCALE
        new_torque = int(round(np.clip(new_torque, -cap, cap)))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, limits)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # Delay the cancel button send so the brake can disengage factory SCC first.
    # Reset whenever openpilot is no longer requesting cancel.
    self.cancel_counter = self.cancel_counter + 1 if CC.cruiseControl.cancel else 0

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      # Hold torque with induced temporary fault when cutting the actuation bit
      # FIXME: we don't use this with CAN FD?
      torque_fault = CC.latActive and not apply_steer_req

      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_TORQUE_SCALE
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def lower_jerk(self, accel):
    """SCC14 JerkLowerLimit for this SCC frame (50 Hz) - see LJ_* above."""
    if self.frame % 100 == 0:
      self.jerk_flag_off = os.path.exists(STOCK_JERK_OFF_FLAG)
    h = self.brake_hist
    h.append(min(accel, 0.))
    if len(h) > LJ_WINDOW + 1:
      h.pop(0)
    rate = (h[0] - h[-1]) / ((len(h) - 1) * 2 * DT_CTRL) if len(h) > 1 else 0.
    if accel < LJ_URGENT_ACCEL or rate > LJ_URGENT_RATE:
      self.jerk_urgent_frames = int(LJ_HOLD / (2 * DT_CTRL))
    elif self.jerk_urgent_frames > 0:
      self.jerk_urgent_frames -= 1
    if not self.stock_jerk or self.jerk_flag_off or self.jerk_urgent_frames > 0:
      return LJ_URGENT
    return LJ_COMFORT

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      # The brake hands the longitudinal over while moving (selfdrived holds it through the
      # release) - and while it does, the ACC has to read as not acting, the way the stock ACC
      # does. cruiseControl.override is set for the brake as well as the accelerator, so without
      # this SCC12 said mode 2, "the driver is on the accelerator", and on 09-23 the car kept
      # pulling with the brake down: 249 of 11977 braking frames in mode 2 were accelerating
      # above +0.3 m/s^2 with aReqRaw at 0, against 2 of 24122 in mode 0. Ported from the
      # sunnypilot version the driver drove on 08-25 (opendbc fe6cba27); the accelerator is
      # untouched and stays in override mode.
      brake_override = CC.enabled and not CC.longActive and not CS.out.gasPressed
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP,
                                                      brake_override, lower_jerk=self.lower_jerk(accel)))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          # Here we send ACC message to cancel, not buttons. Don't delay
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          elif self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
