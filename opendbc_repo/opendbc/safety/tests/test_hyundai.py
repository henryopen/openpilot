#!/usr/bin/env python3
import random
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase


# 4 bit checkusm used in some hyundai messages
# lives outside the can packer because we never send this msg
def checksum(msg):
  addr, dat, bus = msg

  chksum = 0
  if addr == 0x386:
    for i, b in enumerate(dat):
      for j in range(8):
        # exclude checksum and counter bits
        if (i != 1 or j < 6) and (i != 3 or j < 6) and (i != 5 or j < 6) and (i != 7 or j < 6):
          bit = (b >> j) & 1
        else:
          bit = 0
        chksum += bit
    chksum = (chksum ^ 9) & 0xF
    ret = bytearray(dat)
    ret[5] |= (chksum & 0x3) << 6
    ret[7] |= (chksum & 0xc) << 4
  else:
    for i, b in enumerate(dat):
      if addr in [0x260, 0x421] and i == 7:
        b &= 0x0F if addr == 0x421 else 0xF0
      elif addr == 0x394 and i == 6:
        b &= 0xF0
      elif addr == 0x394 and i == 7:
        continue
      chksum += sum(divmod(b, 16))
    chksum = (16 - chksum) % 16
    ret = bytearray(dat)
    ret[6 if addr == 0x394 else 7] |= chksum << (4 if addr == 0x421 else 0)

  return addr, ret, bus


class TestHyundaiSafety(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485)}  # LKAS11
  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485]}

  MAX_RATE_UP = 7
  MAX_RATE_DOWN = 7
  MAX_TORQUE_LOOKUP = [0], [409]
  MAX_RT_DELTA = 262
  DRIVER_TORQUE_ALLOWANCE = 188
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  cnt_gas = 0
  cnt_speed = 0
  cnt_brake = 0
  cnt_cruise = 0
  cnt_button = 0

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, 0)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=0):
    values = {"CF_Clu_CruiseSwState": buttons, "CF_Clu_CruiseSwMain": main_button, "CF_Clu_AliveCnt1": self.cnt_button}
    self.__class__.cnt_button += 1
    return self.packer.make_can_msg_safety("CLU11", bus, values)

  def _user_gas_msg(self, gas):
    values = {"CF_Ems_AclAct": gas, "AliveCounter": self.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

  def _user_brake_msg(self, brake):
    values = {"DriverOverride": 2 if brake else random.choice((0, 1, 3)),
              "AliveCounterTCS": self.cnt_brake % 8}
    self.__class__.cnt_brake += 1
    return self.packer.make_can_msg_safety("TCS13", 0, values, fix_checksum=checksum)

  def _speed_msg(self, speed):
    # safety doesn't scale, so undo the scaling
    values = {"WHL_SPD_%s" % s: speed * 0.03125 for s in ["FL", "FR", "RL", "RR"]}
    values["WHL_SPD_AliveCounter_LSB"] = (self.cnt_speed % 16) & 0x3
    values["WHL_SPD_AliveCounter_MSB"] = (self.cnt_speed % 16) >> 2
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("WHL_SPD11", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": enable, "CR_VSM_Alive": self.cnt_cruise % 16}
    self.__class__.cnt_cruise += 1
    return self.packer.make_can_msg_safety("SCC12", self.SCC_BUS, values, fix_checksum=checksum)

  def _torque_driver_msg(self, torque):
    values = {"CR_Mdps_StrColTq": torque}
    return self.packer.make_can_msg_safety("MDPS12", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"CR_Lkas_StrToqReq": torque, "CF_Lkas_ActToi": steer_req}
    return self.packer.make_can_msg_safety("LKAS11", 0, values)


class TestHyundaiSafetyAltLimits(TestHyundaiSafety):
  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_RT_DELTA = 75
  MAX_TORQUE_LOOKUP = [0], [270]

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.ALT_LIMITS)
    self.safety.init_tests()


class TestHyundaiSafetyAltLimits2(TestHyundaiSafety):
  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_RT_DELTA = 75
  MAX_TORQUE_LOOKUP = [0], [170]

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.ALT_LIMITS_2)
    self.safety.init_tests()


class TestHyundaiSafetyCameraSCC(TestHyundaiSafety):
  BUTTONS_TX_BUS = 2  # tx on 2, rx on 0
  SCC_BUS = 2  # rx on 2

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()


class TestHyundaiSafetyFCEV(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.FCEV_GAS)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"ACCELERATOR_PEDAL": gas}
    return self.packer.make_can_msg_safety("FCEV_ACCELERATOR", 0, values)


class TestHyundaiLegacySafety(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiLegacy, 0)
    self.safety.init_tests()


class TestHyundaiLegacySafetyEV(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiLegacy, HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"Accel_Pedal_Pos": gas}
    return self.packer.make_can_msg_safety("E_EMS11", 0, values, fix_checksum=checksum)


class TestHyundaiLegacySafetyHEV(TestHyundaiSafety):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiLegacy, HyundaiSafetyFlags.HYBRID_GAS)
    self.safety.init_tests()

  def _user_gas_msg(self, gas):
    values = {"CR_Vcu_AccPedDep_Pos": gas}
    return self.packer.make_can_msg_safety("E_EMS11", 0, values, fix_checksum=checksum)


class TestHyundaiLongitudinalSafety(HyundaiLongitudinalBase, TestHyundaiSafety):
  TX_MSGS = [[0x340, 0], [0x4F1, 0], [0x485, 0], [0x420, 0], [0x421, 0], [0x50A, 0], [0x389, 0], [0x4A2, 0], [0x38D, 0], [0x483, 0], [0x7D0, 0]]

  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485, 0x421, 0x420, 0x50A, 0x389]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485, 0x421, 0x420, 0x50A, 0x389)}  # LKAS11, LFAHDA_MFC, SCC12, SCC11, SCC13, SCC14

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x421, 0)

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0, aeb_stop_req=False):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
      "AEB_CmdAct": int(aeb_req),
      "AEB_StopReq": int(aeb_stop_req),
      "CR_VSM_DecCmd": aeb_decel,
    }
    return self.packer.make_can_msg_safety("SCC12", self.SCC_BUS, values)

  def _fca11_msg(self, idx=0, vsm_aeb_req=False, fca_aeb_req=False, aeb_decel=0):
    values = {
      "CR_FCA_Alive": idx % 0xF,
      "FCA_Status": 2,
      "CR_VSM_DecCmd": aeb_decel,
      "CF_VSM_DecCmdAct": int(vsm_aeb_req),
      "FCA_CmdAct": int(fca_aeb_req),
    }
    return self.packer.make_can_msg_safety("FCA11", 0, values)

  def test_no_aeb_fca11(self):
    self.assertTrue(self._tx(self._fca11_msg()))
    self.assertFalse(self._tx(self._fca11_msg(vsm_aeb_req=True)))
    self.assertFalse(self._tx(self._fca11_msg(fca_aeb_req=True)))
    self.assertFalse(self._tx(self._fca11_msg(aeb_decel=1.0)))

  def test_no_aeb_scc12(self):
    self.assertTrue(self._tx(self._accel_msg(0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_req=True)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_decel=1.0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_stop_req=True)))


class TestHyundaiLongitudinalSafetyCameraSCC(HyundaiLongitudinalBase, TestHyundaiSafety):
  TX_MSGS = [[0x340, 0], [0x4F1, 2], [0x485, 0], [0x420, 0], [0x421, 0], [0x50A, 0], [0x389, 0], [0x4A2, 0]]

  FWD_BLACKLISTED_ADDRS = {2: [0x340, 0x485, 0x420, 0x421, 0x50A, 0x389]}
  RELAY_MALFUNCTION_ADDRS = {0: (0x340, 0x485, 0x421, 0x420, 0x50A, 0x389)}  # LKAS11, LFAHDA_MFC, SCC12, SCC11, SCC13, SCC14

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CAMERA_SCC)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0, aeb_stop_req=False):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
      "AEB_CmdAct": int(aeb_req),
      "AEB_StopReq": int(aeb_stop_req),
      "CR_VSM_DecCmd": aeb_decel,
    }
    return self.packer.make_can_msg_safety("SCC12", self.SCC_BUS, values)

  def test_no_aeb_scc12(self):
    self.assertTrue(self._tx(self._accel_msg(0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_req=True)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_decel=1.0)))
    self.assertFalse(self._tx(self._accel_msg(0, aeb_stop_req=True)))

  def test_tester_present_allowed(self):
    pass

  def test_disabled_ecu_alive(self):
    pass


class TestHyundaiSafetyFCEVLong(TestHyundaiLongitudinalSafety, TestHyundaiSafetyFCEV):
  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.FCEV_GAS | HyundaiSafetyFlags.LONG)
    self.safety.init_tests()


class TestHyundaiPedalHandover(unittest.TestCase):
  """ALT_EXP_PEDAL_HANDOVER: the brake hands control over while moving and ends it stopped,
  and the accelerator authorises from a standstill. Standalone rather than a subclass of the
  Hyundai suite: inheriting it would re-run 46 cases that know nothing about this flag, and
  the framework's cross-class TX check reads two classes with the same TX list as a mistake.
  """
  ALT_EXP_ALWAYS_ON_LATERAL = 32
  ALT_EXP_PEDAL_HANDOVER = 64
  ALT_EXP = ALT_EXP_PEDAL_HANDOVER

  cnt_gas = 0
  cnt_speed = 0
  cnt_brake = 0

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_can_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
    self.safety.init_tests()
    self.safety.set_alternative_experience(self.ALT_EXP)

  def tearDown(self):
    self.safety.set_alternative_experience(0)

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _speed_msg(self, speed):
    values = {"WHL_SPD_%s" % s: speed * 0.03125 for s in ["FL", "FR", "RL", "RR"]}
    values["WHL_SPD_AliveCounter_LSB"] = (self.__class__.cnt_speed % 16) & 0x3
    values["WHL_SPD_AliveCounter_MSB"] = (self.__class__.cnt_speed % 16) >> 2
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("WHL_SPD11", 0, values, fix_checksum=checksum)

  def _user_brake_msg(self, brake):
    values = {"DriverOverride": 2 if brake else 1,
              "AliveCounterTCS": self.__class__.cnt_brake % 8}
    self.__class__.cnt_brake += 1
    return self.packer.make_can_msg_safety("TCS13", 0, values, fix_checksum=checksum)

  def _user_gas_msg(self, gas):
    values = {"CF_Ems_AclAct": gas, "AliveCounter": self.__class__.cnt_gas % 4}
    self.__class__.cnt_gas += 1
    return self.packer.make_can_msg_safety("EMS16", 0, values, fix_checksum=checksum)

  def _at_speed(self, speed):
    for _ in range(10):
      self._rx(self._speed_msg(speed))

  def test_brake_while_moving_keeps_controls(self):
    self._at_speed(20)
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed(), "brake while moving must not end control")
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self.safety.get_controls_allowed(), "releasing it must leave control in place")

  def test_brake_at_standstill_ends_controls(self):
    self._at_speed(0)
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed(), "brake at a standstill is the way out")

  def test_gas_at_standstill_authorises(self):
    self._at_speed(0)
    self.safety.set_controls_allowed(False)
    self._rx(self._user_gas_msg(0))
    self._rx(self._user_gas_msg(1))
    self.assertTrue(self.safety.get_controls_allowed(), "a press at a standstill has to authorise")

  def test_gas_while_moving_does_not_authorise(self):
    self._at_speed(20)
    self.safety.set_controls_allowed(False)
    self._rx(self._user_gas_msg(0))
    self._rx(self._user_gas_msg(1))
    self.assertFalse(self.safety.get_controls_allowed(), "moving, the accelerator is an override")

  def test_flag_off_restores_stock_behaviour(self):
    self.safety.set_alternative_experience(0)
    self._at_speed(20)
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed(), "without the flag the brake still ends it")


class TestHyundaiPedalHandoverWithAlwaysOnLateral(TestHyundaiPedalHandover):
  """The flag never travels alone on the car: AlwaysOnLateral is on, so card.py sends 32|64.
  The two bits are read in different places - always-on lateral in lateral.h, the pedals in
  generic_rx_checks - so they have no reason to interact, but nothing was running the pair
  the car actually runs. This re-runs the whole set under it.
  """
  ALT_EXP = TestHyundaiPedalHandover.ALT_EXP_ALWAYS_ON_LATERAL | \
            TestHyundaiPedalHandover.ALT_EXP_PEDAL_HANDOVER


class TestHyundaiKeepAuthorisation(TestHyundaiPedalHandover):
  """ALT_EXP_KEEP_AUTHORISATION on top of what the car runs (32|64): once authorised, neither the
  brake at a standstill nor the middle button take it away; a new safety mode (ignition) does.
  The pedal-handover cases are inherited and re-run under it, except the stop that is now
  supposed to keep control."""
  ALT_EXP_KEEP_AUTHORISATION = 128
  ALT_EXP = TestHyundaiPedalHandover.ALT_EXP_ALWAYS_ON_LATERAL | TestHyundaiPedalHandover.ALT_EXP_PEDAL_HANDOVER | \
            ALT_EXP_KEEP_AUTHORISATION
  BTN_NONE, BTN_RESUME, BTN_SET, BTN_CANCEL = 0, 1, 2, 4
  cnt_button = 0

  def _button_msg(self, buttons):
    values = {"CF_Clu_CruiseSwState": buttons, "CF_Clu_CruiseSwMain": 0, "CF_Clu_AliveCnt1": self.__class__.cnt_button % 16}
    self.__class__.cnt_button += 1
    return self.packer.make_can_msg_safety("CLU11", 0, values)

  def _press(self, button):
    self._rx(self._button_msg(button))
    self._rx(self._button_msg(self.BTN_NONE))

  def test_brake_at_standstill_ends_controls(self):
    # inherited expectation does not hold under this flag - see test_brake_at_standstill_keeps_controls
    pass

  def test_brake_at_standstill_keeps_controls(self):
    self._at_speed(0)
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self.safety.get_controls_allowed(), "a stop with the brake keeps the drive's authorisation")

  def test_cancel_button_keeps_controls(self):
    self._at_speed(15)
    self.safety.set_controls_allowed(True)
    self._rx(self._button_msg(self.BTN_CANCEL))
    self.assertTrue(self.safety.get_controls_allowed(), "holding the middle button keeps it")
    self._rx(self._button_msg(self.BTN_NONE))
    self.assertTrue(self.safety.get_controls_allowed(), "and so does releasing it")

  def test_first_authorisation_still_needs_a_button(self):
    self._at_speed(15)
    self.safety.set_controls_allowed(False)
    self._rx(self._user_brake_msg(True))
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_controls_allowed(), "nothing but the driver gives the first one")
    self._press(self.BTN_RESUME)
    self.assertTrue(self.safety.get_controls_allowed(), "RES gives it")

  def test_middle_button_gives_the_first_authorisation(self):
    self._at_speed(15)
    self.safety.set_controls_allowed(False)
    self._press(self.BTN_CANCEL)
    self.assertTrue(self.safety.get_controls_allowed(), "a press while off still resumes")

  def test_new_safety_mode_ends_it(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundai, HyundaiSafetyFlags.LONG)
    self.assertFalse(self.safety.get_controls_allowed(), "the ignition cycle (a new safety mode) ends it")

  def test_without_the_flag_the_middle_button_ends_it(self):
    self.safety.set_alternative_experience(self.ALT_EXP_ALWAYS_ON_LATERAL | self.ALT_EXP_PEDAL_HANDOVER)
    self._at_speed(15)
    self.safety.set_controls_allowed(True)
    self._rx(self._button_msg(self.BTN_CANCEL))
    self.assertFalse(self.safety.get_controls_allowed(), "stock pause still ends it")


if __name__ == "__main__":
  unittest.main()
