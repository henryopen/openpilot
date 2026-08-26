import math
import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params


# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)  # round here to avoid rounding errors incrementing set speed

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}

# Speed limits from the offline map. Outside this the value is bad map data rather than a
# road we could be on. Compared as rounded km/h so a real 50 km/h limit cannot be lost to a
# floating point ulp at the boundary.
SPEED_LIMIT_MIN_KPH = 50
SPEED_LIMIT_MAX_KPH = 110
# A new limit this far below the current set speed is never adopted automatically: dropping
# that much on a highway is dangerous. The driver can still take it with the +/- buttons.
SPEED_LIMIT_MAX_AUTO_DROP_KPH = 30
SPEED_LIMIT_ASSIST = 3          # SpeedLimitMode: 0 off, 3 assist
SPEED_LIMIT_READ_INTERVAL = 50  # cruise runs at 100 Hz, mapd only updates a couple of times a second


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    self.params = Params()
    self.mem_params = Params("/dev/shm/params")
    self.speed_limit_frame = 0
    self.speed_limit_kph = 0.
    self.speed_limit_mode = 0
    self.speed_limit_offset = 0.

  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric):
    self.v_cruise_kph_last = self.v_cruise_kph

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self.update_button_timers(CS, enabled)
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = V_CRUISE_UNSET
          self.v_cruise_cluster_kph = V_CRUISE_UNSET
        elif CS.cruiseState.speed == -1:
          self.v_cruise_kph = -1
          self.v_cruise_cluster_kph = -1
    else:
      self.v_cruise_kph = V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_UNSET

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      self._update_speed_limit()
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    # a short press jumps by 10 and lands on a round number, a long press trims by 1
    v_cruise_delta_interval = 1 if long_press else 10
    v_cruise_delta = v_cruise_delta * v_cruise_delta_interval
    if v_cruise_delta_interval == 10 and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = np.clip(round(self.v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX)

  def _update_speed_limit(self):
    """Follow the posted limit from the offline map, once per change.

    The set speed only moves when the limit itself changes, so a button press still has the
    final say: whatever is dialled in stays until the road's limit actually changes.
    """
    self.speed_limit_frame += 1
    if self.speed_limit_frame % SPEED_LIMIT_READ_INTERVAL != 0:
      return

    self.speed_limit_mode = self.params.get("SpeedLimitMode", return_default=True)
    if self.speed_limit_mode != SPEED_LIMIT_ASSIST:
      return
    self.speed_limit_offset = self.params.get("SpeedLimitValueOffset", return_default=True)

    limit_kph = round((self.mem_params.get("MapSpeedLimit") or 0.) * CV.MS_TO_KPH)
    if not SPEED_LIMIT_MIN_KPH <= limit_kph <= SPEED_LIMIT_MAX_KPH:
      return

    if limit_kph == self.speed_limit_kph:
      return

    target = float(np.clip(limit_kph + self.speed_limit_offset, V_CRUISE_MIN, V_CRUISE_MAX))
    if self.v_cruise_kph - target > SPEED_LIMIT_MAX_AUTO_DROP_KPH:
      # Too big a drop to take on its own. Leave the limit unrecorded so it is reconsidered
      # once the driver has brought the set speed closer themselves.
      return

    self.speed_limit_kph = limit_kph
    self.v_cruise_kph = target

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    if self.CP.pcmCruise:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    # set picks the next ten at or above the current speed, every other button resumes
    # the speed that was set before. with nothing set yet there is nothing to resume to,
    # so those fall back to the same next ten.
    if any(b.type == ButtonType.decelCruise for b in CS.buttonEvents) or not self.v_cruise_initialized:
      # round first, m/s to km/h leaves 60 as 60.00000000000001 and that would jump a whole step
      next_ten = math.ceil(round(CS.vEgo * CV.MS_TO_KPH, 1) / 10.) * 10.
      self.v_cruise_kph = int(np.clip(next_ten, initial, V_CRUISE_MAX))
    else:
      self.v_cruise_kph = self.v_cruise_kph_last

    self.v_cruise_cluster_kph = self.v_cruise_kph
