"""Neural feedforward for the torque controller, trained on this car's own logs.

torque_from_lateral_accel divides by a single latAccelFactor, which says the rack needs the
same torque per m/s^2 at 15 km/h as at 110. Measured over 2.17M frames of this car, the
steady-state factor runs 1.44 at 15 km/h, 2.78 at 38, 4.01 at 55 and 8.48 at 110 - a 5.9x
spread that one constant cannot cover. The constant sits near the middle, so low speed is
asked for far less torque than the wheel actually needs, and the proportional term has to
make up the rest. That is why KP is interpolated up to 250 down there, and why the output
oscillates once the error is small enough for the gain to swing it.

This replaces the division with a small network taking what the controller already has at
that point in the frame - speed, the lateral acceleration it is asking for, the planned
jerk, and the last 0.3 s of requests. Deliberately no future terms: those would have to come
from the planner, and filling them with the current value would feed the model inputs whose
distribution does not match what it was trained on.

The file format is Twilsonco's, the same one sunnypilot's NNLC and StarPilot's NNFF read, so
the model can be inspected with their tooling. The weights are ours - none of the 114 models
in those projects fits this car, their lowest low-speed factor is 1.03 against our 1.44.
"""
import json
import os
import numpy as np

MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nnff_models')

# The model is bounded on the region the logs cover (peak 304 counts against a 384 limit),
# but nothing structural stops a network from extrapolating, so the output is clipped.
OUTPUT_LIMIT = 1.0


def _sigmoid(x):
  return 1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))


class NNFeedforward:
  """Twilsonco-format feedforward net. Pure numpy, no torch on the device."""

  def __init__(self, path):
    with open(path, encoding='utf-8') as f:
      params = json.load(f)

    self.input_size = params['input_size']
    self.input_vars = params.get('input_vars', [])
    self.input_mean = np.array(params['input_mean'], dtype=np.float32).T
    self.input_std = np.array(params['input_std'], dtype=np.float32).T
    self.layers = []
    for i, layer in enumerate(params['layers']):
      W = np.array(layer[f'dense_{i + 1}_W'], dtype=np.float32).T
      b = np.array(layer[f'dense_{i + 1}_b'], dtype=np.float32).T
      act = layer['activation'].replace('σ', 'sigmoid')
      if act not in ('sigmoid', 'identity'):
        raise ValueError(f'unknown activation {act}')
      self.layers.append((W, b, act))

  def evaluate(self, inputs):
    x = (np.array(inputs, dtype=np.float32) - self.input_mean) / self.input_std
    for W, b, act in self.layers:
      x = x.dot(W) + b
      if act == 'sigmoid':
        x = _sigmoid(x)
    return float(np.clip(x[0, 0], -OUTPUT_LIMIT, OUTPUT_LIMIT))


def load_model(car_fingerprint):
  """Return the net for this car, or None if there is no model for it."""
  path = os.path.join(MODEL_DIR, f'{car_fingerprint}.json')
  if not os.path.isfile(path):
    return None
  return NNFeedforward(path)
