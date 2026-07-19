"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math

from openpilot.common.params import Params

# Planner-replan jitter shows up as small nonzero desired_lateral_jerk even on a straight
# road with a near-zero lateral accel error. Fed into get_friction() that noise still
# clears the (much smaller, sensor-deadzone-sized) lateral_accel_deadzone and comes out
# as a real friction-compensation torque command, i.e. chatter with no steering need
# behind it. Deadbanding the jerk itself (not the combined error) kills that noise floor
# while leaving real jerk (an actual curve entry or lane change, an order of magnitude
# larger) essentially unaffected.
FRICTION_JERK_DEADZONE = 0.3  # m/s^3


class LatControlTorqueExtOverride:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.enforce_torque_control_toggle = self.params.get_bool("EnforceTorqueControl")  # only during init
    self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
    self.friction_jerk_deadzone_enabled = self.params.get_bool("FrictionJerkDeadzoneEnabled")
    self.frame = -1

  def get_friction_jerk(self, desired_lateral_jerk: float) -> float:
    if not self.friction_jerk_deadzone_enabled:
      return desired_lateral_jerk

    return math.copysign(max(abs(desired_lateral_jerk) - FRICTION_JERK_DEADZONE, 0.0), desired_lateral_jerk)

  def update_override_torque_params(self, torque_params) -> bool:
    if not self.enforce_torque_control_toggle:
      return False

    self.frame += 1
    if self.frame % 300 == 0:
      self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")

      if not self.torque_override_enabled:
        return False

      torque_params.latAccelFactor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
      torque_params.friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))
      return True

    return False
