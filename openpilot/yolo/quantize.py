#!/usr/bin/env python3
"""quantize: build an INT8 yolo11n for the device, calibrated on this car's own footage.

FP32 costs ~570 ms per frame on the comma four's CPU, which caps the HUD at well under 1 Hz
and takes half a core to do it. Static INT8 quantisation is the lever that does not involve
taking the GPU away from modeld.

Run on a workstation (needs onnxruntime's quantisation tools), then copy the result to the
device. Not run on the device itself: it needs the full onnxruntime toolchain and several
minutes of CPU that the car does not need to spend.

  ./quantize.py --model yolo11n.onnx --calib calib.npz --out yolo11n_int8.onnx
"""
import argparse
from pathlib import Path

import numpy as np
from onnxruntime.quantization import (CalibrationDataReader, CalibrationMethod, QuantFormat,
                                      QuantType, quantize_static)
from onnxruntime.quantization.shape_inference import quant_pre_process


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--model', default='yolo11n.onnx')
  ap.add_argument('--calib', default='calib.npz')
  ap.add_argument('--out', default='yolo11n_int8.onnx')
  ap.add_argument('--limit', type=int, default=0)
  args = ap.parse_args()

  frames = np.load(args.calib)['frames']
  if args.limit:
    frames = frames[:args.limit]
  print(f'calibration frames: {frames.shape}')

  import onnxruntime as ort
  name = ort.InferenceSession(args.model, providers=['CPUExecutionProvider']).get_inputs()[0].name
  h_in, w_in = 384, 672

  def to_input(rgb):
    pad = np.zeros((h_in, w_in, 3), np.uint8)
    top = max(0, (h_in - rgb.shape[0]) // 2)
    pad[top:top + rgb.shape[0], :rgb.shape[1]] = rgb
    return np.ascontiguousarray(pad.transpose(2, 0, 1)[None].astype(np.float32) / 255.0)

  class Reader(CalibrationDataReader):
    def __init__(self):
      self.it = iter([{name: to_input(f)} for f in frames])

    def get_next(self):
      return next(self.it, None)

  prepped = Path(args.out).with_suffix('.prep.onnx')
  quant_pre_process(args.model, str(prepped), skip_symbolic_shape=False)
  print(f'pre-processed -> {prepped}')

  quantize_static(
    str(prepped), args.out, Reader(),
    quant_format=QuantFormat.QDQ,
    activation_type=QuantType.QUInt8,   # ARM's NEON kernels are happiest with unsigned activations
    weight_type=QuantType.QInt8,
    per_channel=True,
    calibrate_method=CalibrationMethod.MinMax,
  )
  a, b = Path(args.model).stat().st_size, Path(args.out).stat().st_size
  print(f'{args.model} {a / 1e6:.1f} MB -> {args.out} {b / 1e6:.1f} MB')


if __name__ == '__main__':
  main()
