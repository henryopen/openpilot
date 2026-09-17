"""Fuel remaining by subtraction, the way the driver did it on his VW.

The float gauge is not good enough to read continuously: it pins at the top for 14.7 km
after a fill (2026-09-16) and swings +/- 0.5 L while driving as the fuel moves around. So
it is used for two things only - noticing that a fill happened, and saying how much went
in - and the number on the display comes from arithmetic:

    used      = (odometer - odometer_at_fill) / km_per_litre
    remaining = litres_at_fill - used

Both inputs are the cluster's own, so this agrees with the dash rather than competing with
it. CF_Clu_Odometer is absolute, which matters: nothing has to be integrated, and a reboot
or a lost log does not lose the count.

Accuracy, measured over the tank between the 2026-09-02 and 2026-09-16 fills: 436.2 km on
47.763 L, and 436.2 / 9.3 km/L predicts 46.90 L - 1.8% optimistic. That bias is learned
away rather than hard-coded, see _learn().

State is a small JSON file rather than Params: it is display-only, and nothing in the
control path should be able to trip over it.
"""
import json
import os
import time

STATE_PATH = '/data/fuel_state.json'
TANK_L = 58.0

# A fill is a rise the float cannot produce by sloshing. Measured swing while driving is
# +/- 2 counts, which is 0.5 L, so 3 L is clear of it by a wide margin.
FILL_RISE_L = 3.0
# and it has to still be there after the car has settled - the float overshoots while fuel
# is going in
FILL_SETTLE_S = 20.0
GAUGE_FULL_FRAC = 0.985        # 199/200 counts: at the top the float saturates
ECONOMY_MIN, ECONOMY_MAX = 3.0, 30.0    # km/L outside this is the cluster saying "unknown"


class FuelTracker:
  def __init__(self, path: str = STATE_PATH):
    self.path = path
    self.s = self._load()
    self._gauge_hist: list[tuple[float, float]] = []

  # ---------- persistence ----------

  def _load(self) -> dict:
    try:
      with open(self.path) as f:
        s = json.load(f)
      if all(k in s for k in ('odo0', 'litres0')):
        return s
    except Exception:
      pass
    return {}

  def _save(self) -> None:
    try:
      tmp = self.path + '.tmp'
      with open(tmp, 'w') as f:
        json.dump(self.s, f)
      os.replace(tmp, self.path)
    except Exception:
      pass

  # ---------- the bias the cluster's economy carries ----------

  def _learn(self, gauge_litres: float) -> None:
    """A fill is the only time the truth is known, so correct the economy there.

    At the moment of a fill the float is reliable again - it is off the bottom and, if the
    tank was not filled to the brim, off the top as well. So compare what was predicted to
    be left against what is actually there, and keep a ratio to apply to km/L. Averaged
    over fills rather than replaced, because one bad float reading should not undo it.
    """
    pred = self.s.get('remaining')
    if pred is None or pred <= 0:
      return
    burned_pred = self.s['litres0'] - pred
    burned_real = self.s['litres0'] - gauge_litres
    if burned_pred < 5.0 or burned_real < 5.0:
      return                      # too little burned for the ratio to mean anything
    ratio = burned_real / burned_pred
    if not 0.7 < ratio < 1.4:
      return                      # implausible - a bad float reading, not a bias
    old = self.s.get('econ_corr', 1.0)
    n = self.s.get('econ_n', 0)
    self.s['econ_corr'] = (old * n + ratio) / (n + 1)
    self.s['econ_n'] = min(n + 1, 10)

  # ---------- main ----------

  def update(self, odometer: float, km_per_litre: float, gauge_frac: float,
             now: float | None = None) -> dict:
    """Returns {litres, percent, source, used, since_km} or {} if nothing can be said."""
    now = time.monotonic() if now is None else now
    if odometer is None or odometer <= 0:
      return {}
    gauge_litres = max(0.0, min(1.0, gauge_frac)) * TANK_L

    # --- has a fill happened? the float has to rise and then stay risen ---
    self._gauge_hist.append((now, gauge_litres))
    self._gauge_hist = [(t, v) for t, v in self._gauge_hist if now - t <= 120.0]
    if len(self._gauge_hist) > 5:
      old = min(v for t, v in self._gauge_hist if now - t >= FILL_SETTLE_S) \
          if any(now - t >= FILL_SETTLE_S for t, _ in self._gauge_hist) else None
      if old is not None and gauge_litres - old >= FILL_RISE_L:
        settled = [v for t, v in self._gauge_hist if now - t <= 5.0]
        if settled and max(settled) - min(settled) < 0.6:      # steady again
          self._learn(old)
          # at the top the float saturates, so take the tank; otherwise believe it
          self.s['litres0'] = TANK_L if gauge_frac >= GAUGE_FULL_FRAC else gauge_litres
          self.s['odo0'] = odometer
          self.s.pop('adopted', None)   # no longer a guess off the float
          self._gauge_hist = []
          self._save()

    # --- no basis yet: adopt the float, so there is something to show ---
    if 'odo0' not in self.s:
      self.s['litres0'] = gauge_litres
      self.s['odo0'] = odometer
      self.s['adopted'] = True
      self._save()

    # --- odometer went backwards, or a different car: start over ---
    if odometer < self.s['odo0'] - 0.5:
      self.s = {'litres0': gauge_litres, 'odo0': odometer, 'adopted': True}
      self._save()

    since = max(0.0, odometer - self.s['odo0'])
    econ = km_per_litre * self.s.get('econ_corr', 1.0)
    if not ECONOMY_MIN <= econ <= ECONOMY_MAX:
      # the cluster is not offering a usable figure (it reads 102.3 right after a fill),
      # so hold the last good answer rather than showing nonsense
      last = self.s.get('remaining')
      if last is None:
        return {}
      return {'litres': last, 'percent': 100.0 * last / TANK_L, 'source': 'held',
              'used': self.s['litres0'] - last, 'since_km': since}

    used = since / econ
    remaining = max(0.0, self.s['litres0'] - used)
    self.s['remaining'] = remaining
    # persist once a kilometre. The first version tested int(since) % 5, which is true for
    # every frame inside that kilometre - a few hundred writes each time.
    if odometer - self.s.get('saved_odo', -99) >= 1.0:
      self.s['saved_odo'] = odometer
      self._save()
    return {'litres': remaining, 'percent': 100.0 * remaining / TANK_L,
            'source': 'adopted' if self.s.get('adopted') else 'fill',
            'used': used, 'since_km': since,
            'econ': econ, 'corr': self.s.get('econ_corr', 1.0)}
