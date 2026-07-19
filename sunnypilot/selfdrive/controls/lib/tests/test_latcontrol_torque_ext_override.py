"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import (
  FRICTION_JERK_DEADZONE, LatControlTorqueExtOverride,
)


class TestLatControlTorqueExtOverrideFrictionJerkDeadzone:

  def setup_method(self):
    self.override = LatControlTorqueExtOverride(None)

  def test_disabled_by_default(self):
    assert self.override.friction_jerk_deadzone_enabled is False

  def test_disabled_is_passthrough(self):
    self.override.friction_jerk_deadzone_enabled = False

    for jerk in (0.0, 0.05, -0.05, FRICTION_JERK_DEADZONE, -FRICTION_JERK_DEADZONE, 2.0, -2.0):
      assert self.override.get_friction_jerk(jerk) == jerk

  def test_enabled_kills_jerk_below_deadzone(self):
    self.override.friction_jerk_deadzone_enabled = True

    for jerk in (0.0, 0.05, -0.05, 0.29, -0.29):
      assert self.override.get_friction_jerk(jerk) == 0.0

  def test_enabled_boundary_at_deadzone(self):
    self.override.friction_jerk_deadzone_enabled = True

    assert self.override.get_friction_jerk(FRICTION_JERK_DEADZONE) == 0.0
    assert self.override.get_friction_jerk(-FRICTION_JERK_DEADZONE) == 0.0

  def test_enabled_shifts_jerk_above_deadzone_preserving_sign(self):
    self.override.friction_jerk_deadzone_enabled = True

    assert self.override.get_friction_jerk(2.0) == 2.0 - FRICTION_JERK_DEADZONE
    assert self.override.get_friction_jerk(-2.0) == -(2.0 - FRICTION_JERK_DEADZONE)
