import math
import unittest

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE
from openpilot.selfdrive.controls.lib.standstill_hold import (ARM_SPEED, GAP_BRAKE_MAX, GAP_BRAKE_MIN, GAP_LAUNCH_LEAD_V,
                                                              GAP_LAUNCH_MARGIN, GAP_SPEED, RELEASE_M, StandstillHold)

DT = 0.05


class Lead:
  def __init__(self, dRel, vLead=0., aLeadK=0., present=True, radar=True):
    self.dRel, self.vLead, self.aLeadK, self.present, self.radar = dRel, vLead, aLeadK, present, radar


class TestStandstillHold(unittest.TestCase):
  def _held_at(self, d):
    h = StandstillHold(DT)
    for _ in range(20):
      self.assertTrue(h.update(True, 0., Lead(d), False))
    return h

  def test_no_release_inside_stop_distance(self):
    # held 3.7 m behind a car that creeps 1.2 m: still inside the stop distance, stay held
    h = self._held_at(3.7)
    for _ in range(100):
      held = h.update(True, 0., Lead(3.7 + RELEASE_M + 0.2), False)
    self.assertTrue(held)

  def test_release_outside_stop_distance(self):
    h = self._held_at(4.5)
    d = STOP_DISTANCE + 1.0       # moved 1.5 m (the filtered range only approaches a 1.0 m move)
    for _ in range(100):
      held = h.update(True, 0., Lead(d), False)
    self.assertFalse(held)

  def test_inside_no_throttle_when_stopped(self):
    for vl in (0., 1.0):
      self.assertEqual(StandstillHold(DT).gap_limit(True, 0., Lead(STOP_DISTANCE - 1., vLead=vl), False), 0.)

  def test_inside_rolling_brakes(self):
    a = StandstillHold(DT).gap_limit(True, 1.0, Lead(3.5), False)
    self.assertLessEqual(a, -GAP_BRAKE_MIN)
    self.assertGreaterEqual(a, -GAP_BRAKE_MAX)

  def test_inside_lead_leaving_no_throttle_no_brake(self):
    self.assertEqual(StandstillHold(DT).gap_limit(True, 1.0, Lead(4.0, vLead=2.0), False), 0.)

  def test_held_rolling_brakes_towards_line(self):
    a = StandstillHold(DT).gap_limit(True, 1.5, Lead(STOP_DISTANCE + 1.5), True)
    self.assertLessEqual(a, -GAP_BRAKE_MIN)

  def test_steady_follow_untouched(self):
    # 8 km/h, 7 m behind a car doing the same speed: nothing to say
    v = 8 / 3.6
    self.assertTrue(math.isinf(StandstillHold(DT).gap_limit(True, v, Lead(7.0, vLead=v), False)))

  def test_lead_stopping_ahead_brakes_early(self):
    # 10/29 19:06:35: 8 km/h, 6.6 m, the lead at 4.9 km/h and braking
    a = StandstillHold(DT).gap_limit(True, 7.9 / 3.6, Lead(6.6, vLead=4.9 / 3.6, aLeadK=-1.2), False)
    self.assertLessEqual(a, -GAP_BRAKE_MIN)

  def test_fast_or_disabled_untouched(self):
    self.assertTrue(math.isinf(StandstillHold(DT).gap_limit(True, GAP_SPEED + 0.1, Lead(3.0), False)))
    self.assertTrue(math.isinf(StandstillHold(DT).gap_limit(False, 1.0, Lead(3.0), False)))
    self.assertTrue(math.isinf(StandstillHold(DT).gap_limit(True, 1.0, Lead(3.0, present=False), False)))

  def test_no_launch_behind_crawling_lead_near_line(self):
    # stopped, 6.0 m behind a car crawling at 4 km/h: wait
    self.assertEqual(StandstillHold(DT).gap_limit(True, 0., Lead(STOP_DISTANCE + 1.0, vLead=4 / 3.6), False), 0.)
    # ...but go once it is far enough, or actually driving off
    self.assertTrue(math.isinf(StandstillHold(DT).gap_limit(True, 0., Lead(STOP_DISTANCE + GAP_LAUNCH_MARGIN + 0.1, vLead=4 / 3.6), False)))
    self.assertTrue(math.isinf(StandstillHold(DT).gap_limit(True, 0., Lead(STOP_DISTANCE + 1.0, vLead=GAP_LAUNCH_LEAD_V + 0.5), False)))

  def test_ramp_after_waiting(self):
    h = StandstillHold(DT)
    self.assertEqual(h.gap_limit(True, 0., Lead(STOP_DISTANCE + 0.5, vLead=1.0), False), 0.)
    self.assertTrue(math.isinf(h.launch_cap(0.)))
    self.assertTrue(math.isinf(h.gap_limit(True, 0., Lead(STOP_DISTANCE + 2.0, vLead=1.0), False)))
    self.assertLess(h.launch_cap(0.), 0.5)       # let go: on the release ramp, not the MPC's whole gap

  def test_arm_speed_boundary(self):
    self.assertEqual(StandstillHold(DT).gap_limit(True, ARM_SPEED - 0.01, Lead(3.0), False), 0.)


if __name__ == '__main__':
  unittest.main()
