"""Brake as a longitudinal override instead of a disengagement while the car is moving.

The panda safety keeps controls_allowed on a brake press above the brand's threshold
(see brake_release_resume in opendbc/safety) so openpilot can carry on once the pedal
is released. selfdrived and controlsd have to agree with it: selfdrived must not raise
the USER_DISABLE pedalPressed event, and controlsd must stop longitudinal actuation for
as long as the pedal is down. Below the threshold the brake still disengages.
"""
from opendbc.car import structs

# matches HYUNDAI_BRAKE_OVERRIDE_THRSLD in opendbc/safety/modes/hyundai.h
BRAKE_OVERRIDE_MIN_SPEED = 0.5  # m/s

# brands whose safety sets brake_release_resume
BRAKE_OVERRIDE_BRANDS = ('hyundai',)


def brake_is_override(CP: structs.CarParams, CS: structs.CarState) -> bool:
  return CP.brand in BRAKE_OVERRIDE_BRANDS and CS.brakePressed and CS.vEgo > BRAKE_OVERRIDE_MIN_SPEED
