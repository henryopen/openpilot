#!/usr/bin/env python3
"""Watch the body-related CAN frames live and print which bits move.

Reverse engineering the light and wiper controls needs the one thing a log cannot give:
you operating the switch while something watches. Run this with the ignition on, then move
the light stalk, the wiper stalk, or walk the cluster's settings menu. Every bit that
changes is printed with its address, bit number, old and new value, and the DBC name when
there is one.

  ssh comma@<car> 'source /usr/local/venv/bin/activate && cd /data/openpilot && \
    PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/watch_body_can.py'

What is already known about this car (2026-09-13, off the 9/9 logs):
  0x541 CGW1     bit31 HeadLampLow, bit24 WiperIntSw, bit21-23 WiperIntT, bit38-39 LightSwState=3
                 bit27 WiperAutoSw=0, bit28-30 RainSnsState=0, bit56 RainSnsOption=0 (no sensor)
  0x553 CGW2     bit16 AutoLightValue - moves with the headlamps
  0x520 CGW3     CR_Photosensor_LH/RH - the whole frame is zero on this car
  0x410 CGW_USM1 bit35-37 AutoLightRValue = 0, the auto-light sensitivity setting
  0x50C CLU13    bit59 CF_Clu_AltLStatus = 1 constantly - the cluster telling the BCM that
                 auto light is on. This is a cluster -> BCM path, not a BCM broadcast.
  0x340 LKAS11   bit14/29/34 the HBA fields, which openpilot already forwards untouched

So: if walking the settings menu moves a bit in 0x410, that setting is reachable on CAN.
If moving the wiper stalk moves something outside 0x541, that is a second path worth having.
"""
import argparse
import sys
from collections import defaultdict

sys.path.insert(0, '/data/openpilot')

import openpilot.cereal.messaging as messaging

WATCH = {
  0x410: 'CGW_USM1', 0x520: 'CGW3', 0x541: 'CGW1', 0x553: 'CGW2', 0x559: 'CGW4',
  0x50C: 'CLU13', 0x52A: 'CLU15', 0x5B0: 'CLU12', 0x340: 'LKAS11', 0x07F: 'CGW5',
  0x520 + 0: 'CGW3', 0x522: 'GW_IPM_PE_1', 0x391: 'BCM_PO_11',
}

NAMES = {
  (0x541, 21): 'WiperIntT', (0x541, 24): 'WiperIntSw', (0x541, 25): 'WiperLowSw',
  (0x541, 26): 'WiperHighSw', (0x541, 27): 'WiperAutoSw', (0x541, 28): 'RainSnsState',
  (0x541, 31): 'HeadLampLow', (0x541, 32): 'HeadLampHigh', (0x541, 37): 'ALightStat',
  (0x541, 38): 'LightSwState', (0x541, 47): 'WiperMistSw', (0x541, 51): 'PassingSW',
  (0x541, 53): 'HLpHighSw', (0x541, 56): 'RainSnsOption',
  (0x553, 16): 'AutoLightValue', (0x553, 32): 'WiperParkPosition', (0x553, 54): 'AutoLightOption',
  (0x410, 35): 'AutoLightRValue', (0x410, 38): 'RearWiperRValue', (0x410, 16): 'WlightRValue',
  (0x50C, 59): 'CF_Clu_AltLStatus',
  (0x340, 14): 'HbaLamp', (0x340, 29): 'HbaSysState', (0x340, 34): 'HbaOpt',
}


def name_for(addr, bit):
  """Nearest named signal at or below this bit, so multi-bit fields still report."""
  best = None
  for (a, b), n in NAMES.items():
    if a == addr and b <= bit and (best is None or b > best[0]):
      best = (b, n)
  if best is None:
    return ''
  return '  %s%s' % (best[1], '' if best[0] == bit else '+%d' % (bit - best[0]))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--bus', type=int, default=0)
  ap.add_argument('--known-only', action='store_true',
                  help='only the named body messages; default is everything, because the DBC '
                       'describes 65%% of the live bits on this bus and the rest is where an '
                       'undocumented command would be')
  args = ap.parse_args()
  args.all = not args.known_only

  sm = messaging.SubMaster(['can'])
  last = {}
  counts = defaultdict(int)
  print('watching bus %d. move the stalks or walk the settings menu.' % args.bus)
  print('%-10s %-14s %-6s %-9s %s' % ('addr', 'name', 'bit', 'change', 'signal'))
  while True:
    sm.update(1000)
    if not sm.updated['can']:
      continue
    for msg in sm['can']:
      if msg.src != args.bus:
        continue
      if not args.all and msg.address not in WATCH:
        continue
      val = int.from_bytes(msg.dat, 'little')
      prev = last.get(msg.address)
      last[msg.address] = val
      if prev is None or prev == val:
        continue
      diff = prev ^ val
      for bit in range(len(msg.dat) * 8):
        if not (diff >> bit) & 1:
          continue
        counts[(msg.address, bit)] += 1
        # a counter or checksum flips constantly; say so rather than spamming
        tag = '  (flips a lot, probably a counter)' if counts[(msg.address, bit)] > 50 else ''
        if msg.address not in WATCH:
          tag += '   [address not in the DBC]'
        print('0x%03X     %-14s %-6d %d -> %d   %s%s' %
              (msg.address, WATCH.get(msg.address, ''), bit,
               (prev >> bit) & 1, (val >> bit) & 1, name_for(msg.address, bit), tag))


if __name__ == '__main__':
  try:
    main()
  except KeyboardInterrupt:
    pass
