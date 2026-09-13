#!/usr/bin/env python3
"""Which ECUs answer diagnostics here, and does the body one look like it takes IO control?

The wipers and the low beams have no command anywhere in the community DBC. The only
lighting command in it is CF_VSM_HBACmd, which rides in SCC12 and FCA11 and is about the
high beam. But that DBC is what the community has reversed, not what the car implements,
and there is a second way to make an ECU move something: UDS IO control by identifier
(service 0x2F), which is what a workshop tool uses to run an actuator on the bench.

This script only ASKS. TesterPresent to see who is home, then read-only identifiers. It
writes nothing and moves nothing. Single-frame UDS by hand rather than the isotp helper,
because every request here fits in one frame and the helper needs openpilot's plumbing.

  sudo systemctl stop comma      # openpilot must not be driving the panda
  cd /data/openpilot && PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/probe_ecus.py
  sudo systemctl start comma     # put it back

Ignition has to be on.
"""
import argparse
import sys
import time

sys.path.insert(0, '/data/openpilot')

from panda import Panda
from opendbc.car.structs import CarParams

# What openpilot already talks to, plus the body side it has never had a reason to ask.
KNOWN = {
  0x7d0: 'fwdRadar', 0x7d4: 'eps', 0x7c4: 'fwdCamera', 0x7e0: 'engine', 0x7e1: 'transmission',
  0x7b3: 'hvac', 0x7b1: 'parkingAdas', 0x730: 'adas', 0x7c6: 'cluster', 0x7b7: 'cornerRadar',
  0x7a0: 'body?', 0x770: 'body/IPM?', 0x760: 'gateway?', 0x7a5: 'smartKey?', 0x7d1: 'abs?',
}
RX_OFFSET = 0x08


def single_frame(payload: bytes) -> bytes:
  """UDS single frame: length nibble then the service bytes, padded to 8."""
  assert len(payload) <= 7
  return bytes([len(payload)]) + payload + b'\x00' * (7 - len(payload))


def ask(p, bus, addrs, payload, wait=0.35):
  """Send one request to each address, collect whatever comes back."""
  p.can_clear(0xFFFF)
  frame = single_frame(payload)
  for a in addrs:
    try:
      p.can_send(a, frame, bus)
    except Exception:
      pass
  time.sleep(wait)
  out = {}
  for addr, dat, src in p.can_recv():
    if src != bus:
      continue
    tx = addr - RX_OFFSET
    if tx in addrs:
      out.setdefault(tx, bytes(dat))
  return out


def describe(dat: bytes) -> str:
  if len(dat) < 2:
    return dat.hex()
  sid = dat[1]
  if sid == 0x7f:
    nrc = dat[3] if len(dat) > 3 else 0
    meaning = {0x11: 'service not supported', 0x12: 'subfunction not supported',
               0x22: 'conditions not correct', 0x31: 'request out of range',
               0x33: 'security access denied', 0x7f: 'not in this session'}.get(nrc, '')
    return 'refused (NRC %02X %s)' % (nrc, meaning)
  return dat.hex()


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--bus', type=int, default=0)
  ap.add_argument('--scan', action='store_true', help='sweep 0x700-0x7ff rather than the known list')
  ap.add_argument('--obd', action='store_true', help='switch the OBD port multiplexing on')
  args = ap.parse_args()

  p = Panda()
  p.reset()
  print('panda connected, safety was %s' % p.health()['safety_mode'])
  # elm327 is the diagnostics-only mode: isotp out, nothing else.
  p.set_safety_mode(CarParams.SafetyModel.elm327, 1)
  if args.obd:
    p.set_obd(True)
  time.sleep(0.2)

  addrs = list(range(0x700, 0x800)) if args.scan else sorted(KNOWN)
  print('asking %d addresses on bus %d who is there...\n' % (len(addrs), args.bus))

  alive = {}
  for chunk in [addrs[i:i + 16] for i in range(0, len(addrs), 16)]:
    alive.update(ask(p, args.bus, chunk, b'\x3e\x00'))

  if not alive:
    print('nobody answered.')
    print('  - is the ignition on?')
    print('  - is openpilot stopped?  sudo systemctl stop comma')
    print('  - try --obd, and try --bus 1')
    p.set_safety_mode(CarParams.SafetyModel.noOutput)
    return

  print('%-8s %-13s %s' % ('addr', 'guess', 'TesterPresent'))
  for a in sorted(alive):
    print('0x%03X    %-13s %s' % (a, KNOWN.get(a, ''), describe(alive[a])))

  found = sorted(alive)
  print('\nidentifiers (read only):')
  for did, what in ((0xF190, 'VIN'), (0xF18C, 'serial'), (0xF186, 'session')):
    res = ask(p, args.bus, found, b'\x22' + did.to_bytes(2, 'big'))
    for a in sorted(res):
      dat = res[a]
      txt = describe(dat)
      if dat[1] == 0x62:
        try:
          txt = dat[4:].decode('ascii', 'replace').strip('\x00 \xff') or dat.hex()
        except Exception:
          txt = dat.hex()
      print('  0x%03X %-8s %s' % (a, what, txt))

  # Does anyone even admit to service 0x2F? Asking with an obviously invalid identifier is
  # the polite way to find out: a supporting ECU refuses with "request out of range",
  # one without the service refuses with "service not supported".
  print('\nIO control (0x2F) probe with a deliberately invalid identifier:')
  res = ask(p, args.bus, found, b'\x2f\xff\xff\x00')
  for a in sorted(res):
    d = describe(res[a])
    hint = ''
    if 'out of range' in d or 'conditions' in d:
      hint = '   <- supports 0x2F, the identifier was just wrong'
    elif 'service not supported' in d:
      hint = '   <- no IO control here'
    print('  0x%03X  %s%s' % (a, d, hint))

  p.set_safety_mode(CarParams.SafetyModel.noOutput)
  print('\nput back to noOutput. Nothing was written. '
        'Remember: sudo systemctl start comma')


if __name__ == '__main__':
  main()
