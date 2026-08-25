"""The middle cruise button pauses and resumes longitudinal control, like the stock ACC.

On this platform the middle button is a pause/resume button, not a cancel one - opendbc
says so itself in hyundai/values.py: "CANCEL = 4  # on newer models, this is a pause/resume
button". openpilot maps it to buttonCancel, which is USER_DISABLE + NO_ENTRY, so pressing it
disengages, stops sending the set speed to the cluster (VSetDis is gated on CC.enabled) and
requires a SET/RES press to come back. None of that matches the car.

Here it stays a toggle: enabled is untouched and only longActive drops, so the set speed
stays on the cluster and lateral control (MADS) keeps running. Resuming matches the stock
button layout:

  - middle button again
  - RES  (returns to the previous set speed)
  - SET  (takes the current speed)

Pedals do not clear the pause. Brake and accelerator are overrides that resume when released
(see brake_override.py); a deliberate button press is not.

car_events raises cancel on both the rising and the falling edge, so only the press is
counted - otherwise one press toggles twice and nothing happens.
"""
from opendbc.car import structs

ButtonType = structs.CarState.ButtonEvent.Type

# Buttons that end the pause, mirroring the stock ACC
RESUME_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.resumeCruise)


class LongitudinalPause:
  def __init__(self):
    self.paused = False

  def update(self, CP: structs.CarParams, CS: structs.CarState, enabled: bool) -> bool:
    # Only meaningful when openpilot owns longitudinal. Without OP long the button has to
    # keep its stock path so the factory ACC handles it.
    if not CP.openpilotLongitudinalControl or not enabled:
      self.paused = False
      return False

    for b in CS.buttonEvents:
      if not b.pressed:
        continue
      if b.type == ButtonType.cancel:
        self.paused = not self.paused
      elif b.type in RESUME_BUTTONS:
        self.paused = False

    return self.paused
