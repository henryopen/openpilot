#!/usr/bin/env python3
"""On C3, once per model: compile c3lights' finder for the Adreno 630 and save the jitted program, the way openpilot's
tinygrad_repo/examples/openpilot/compile3.py does for the driving model. c3lights then loads it in ~4 s instead of
recompiling for ~6-8 min on every start (and runs it on the GPU: ~0.46 s a frame, ~0.1 CPU cores).
  python compile_lights_gpu.py [models/c3light_ft1n_704x1248_raw.onnx] [models/c3light_ft1n_704x1248_raw_qcom.pkl]
fp32 on purpose (IMAGE=2, no FLOAT16): fp16 was 7% faster on the Custin set but not identical to the CPU.
"""
import os
import pickle
import sys
import time

import numpy as np

os.environ.setdefault('QCOM', '1')
os.environ.setdefault('IMAGE', '2')
os.environ.setdefault('NOLOCALS', '1')
os.environ.setdefault('JIT_BATCH_SIZE', '0')
sys.path.insert(0, '/data/openpilot/tinygrad_repo')
from tinygrad import Tensor, TinyJit, Device
from tinygrad.frontend.onnx import OnnxRunner

HERE = os.path.dirname(os.path.abspath(__file__))
onnx_file = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'models/c3light_ft1n_704x1248_raw.onnx')
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, 'models/c3light_ft1n_704x1248_raw_qcom.pkl')
t0 = time.perf_counter()
run_onnx = OnnxRunner(onnx_file)
name, spec = next(iter(run_onnx.graph_inputs.items()))
print('input', name, spec.shape, 'device', Device.DEFAULT, flush=True)
jit = TinyJit(lambda **kw: next(iter(run_onnx({k: v.to(Device.DEFAULT) for k, v in kw.items()}).values())).cast('float32'),
              prune=True)
rng = np.random.default_rng(0)
buf = rng.random(spec.shape, dtype=np.float32)
x = Tensor(buf, device='NPY').realize()       # wraps buf: writing into buf changes the next run's input
last = None
for i in range(3):
  if i < 2:
    buf[:] = rng.random(spec.shape, dtype=np.float32)
  last = jit(**{name: x}).numpy()
  print(f'run {i} {time.perf_counter() - t0:.0f} s', flush=True)
np.testing.assert_equal(jit(**{name: x}).numpy(), last)
tmp = out + '.tmp'
with open(tmp, 'wb') as f:
  pickle.dump(jit, f)
os.replace(tmp, out)
print(f'saved {out} {os.path.getsize(out) / 1e6:.1f} MB, kernels {len(jit.captured.jit_cache)}, total {time.perf_counter() - t0:.0f} s')
