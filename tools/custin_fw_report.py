#!/usr/bin/env python3
"""Print the ECU firmware versions this car answered with, so a Custin that is not recognised can be added.

On the comma four, after the car has been started (ignition on) once with this software:
  cd /data/openpilot && python3 tools/custin_fw_report.py
Paste everything it prints into a GitHub issue. It prints no VIN and no location.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from opendbc.car.structs import car  # noqa: E402
from openpilot.common.params import Params  # noqa: E402


def report(cp) -> list[str]:
  fws = sorted((f for f in cp.carFw if not f.logging), key=lambda f: (str(f.ecu), f.address, f.subAddress, f.fwVersion))
  out = [f"platform: {cp.carFingerprint}" + ("  (not recognised)" if cp.carFingerprint == "MOCK" else ""),
         f"fuzzy: {cp.fuzzyFingerprint}",
         f"ecus: {len(fws)}",
         ""]
  for f in fws:
    sub = None if f.subAddress == 0 else f.subAddress
    out.append(f"  (Ecu.{f.ecu}, {hex(f.address)}, {sub}): [{bytes(f.fwVersion)!r}],  # {f.brand}")
  return out


def main() -> int:
  raw = Params().get("CarParamsPersistent")
  if not raw:
    print("No car parameters yet: start the car (ignition on) once with this software, then run this again.")
    return 1
  with car.CarParams.from_bytes(raw) as cp:
    print("\n".join(report(cp)))
  return 0


if __name__ == "__main__":
  sys.exit(main())
