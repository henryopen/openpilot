#!/usr/bin/env python3
"""Can anything on this car be commanded over CAN at all? Ask, in one pass, read only.

The question before "which address is the wiper" is "is there any write path". Broadcast
frames are not one: CGW1 is the BCM telling everyone where the stalks are, and overwriting
it cannot move a stalk that is hard wired into the BCM. The way a factory line moves an
actuator is diagnostics - UDS - and openpilot already proves that channel is open, because
it queries firmware versions over it every boot.

This walks the whole thing and writes nothing:

  1. who answers TesterPresent on 0x700-0x7ff          - finds the body ECU's address
  2. what they say their identity is (0x22, read only) - confirms a real UDS stack
  3. do they admit to IO control 0x2F                  - asked with an invalid identifier,
       so a supporting ECU answers "request out of range" and a non-supporting one
       answers "service not supported". Nothing is actuated either way.
  4. do they admit to routineControl 0x31              - same trick. HKG actuator tests
       are often routines rather than IO control.
  5. will the body ECU open an extended session 0x10 03 - the session actuator tests need

If step 3 or 4 comes back "out of range" from a body ECU, there IS a write path and the
remaining work is finding the identifier. If everything says "service not supported",
CAN is a dead end for this and the answer is a relay.

  sudo systemctl stop comma
  cd /data/openpilot && PYTHONPATH=/data/openpilot python openpilot/yolo/analysis/probe_ecus.py
  sudo systemctl start comma

IGNITION MUST BE ON. With the car asleep the bus is silent and nothing answers - that is
not a result, it is a missing precondition. The script checks and tells you.
"""
import argparse
import sys
import time

sys.path.insert(0, '/data/openpilot')

from panda import Panda
from opendbc.car.structs import CarParams

KNOWN = {
  0x7d0: 'fwdRadar', 0x7d4: 'eps', 0x7c4: 'fwdCamera', 0x7e0: 'engine', 0x7e1: 'transmission',
  0x7b3: 'hvac', 0x7b1: 'parkingAdas', 0x730: 'adas', 0x7c6: 'cluster', 0x7b7: 'cornerRadar',
  0x7a0: 'body?', 0x770: 'body/IPM?', 0x760: 'gateway?', 0x7a5: 'smartKey?', 0x7d1: 'abs?',
}
RX_OFFSET = 0x08
NRC = {0x10: 'general reject', 0x11: 'service not supported', 0x12: 'subfunction not supported',
       0x13: 'wrong length', 0x22: 'conditions not correct', 0x31: 'request out of range',
       0x33: 'security access denied', 0x7e: 'service not supported in session',
       0x7f: 'service not supported in session'}


def frame(payload: bytes) -> bytes:
  assert len(payload) <= 7
  return bytes([len(payload)]) + payload + b'\x00' * (7 - len(payload))


def ask(p, bus, addrs, payload, wait=0.4):
  p.can_clear(0xFFFF)
  f = frame(payload)
  for a in addrs:
    try:
      p.can_send(a, f, bus)
    except Exception:
      pass
  time.sleep(wait)
  out = {}
  for addr, dat, src in p.can_recv():
    if src != bus:
      continue
    tx = addr - RX_OFFSET
    if tx in addrs and tx not in out:
      out[tx] = bytes(dat)
  return out


def verdict(dat: bytes):
  """(is_negative, nrc, text)"""
  if len(dat) < 3:
    return None, 0, dat.hex()
  if dat[1] == 0x7f:
    nrc = dat[3] if len(dat) > 3 else 0
    return True, nrc, 'refused: %02X %s' % (nrc, NRC.get(nrc, ''))
  return False, 0, 'positive: ' + dat.hex()


def bus_alive(p, bus, seconds=2.0):
  p.can_clear(0xFFFF)
  t0 = time.monotonic()
  n = 0
  while time.monotonic() - t0 < seconds:
    n += sum(1 for _, _, src in p.can_recv() if src == bus)
    time.sleep(0.05)
  return n


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--bus', type=int, default=0)
  ap.add_argument('--obd', action='store_true', help='switch OBD port multiplexing on')
  ap.add_argument('--quick', action='store_true', help='only the known addresses, not the full sweep')
  args = ap.parse_args()

  p = Panda()
  p.reset()
  print('panda ok, safety was %s' % p.health()['safety_mode'])
  p.set_safety_mode(CarParams.SafetyModel.elm327, 1)
  if args.obd:
    p.set_obd(True)
  time.sleep(0.3)

  n = bus_alive(p, args.bus)
  print('bus %d traffic: %d frames in 2 s' % (args.bus, n))
  if n == 0:
    print('\nThe bus is silent - the car is asleep. Turn the ignition on and run this again.')
    print('Nothing below would mean anything with the bus down.')
    p.set_safety_mode(CarParams.SafetyModel.noOutput)
    return

  addrs = sorted(KNOWN) if args.quick else list(range(0x700, 0x800))
  print('\n[1] TesterPresent to %d addresses...' % len(addrs))
  alive = {}
  for i in range(0, len(addrs), 16):
    alive.update(ask(p, args.bus, addrs[i:i + 16], b'\x3e\x00'))
  if not alive:
    print('    nobody answered. Try --obd, or --bus 1.')
    p.set_safety_mode(CarParams.SafetyModel.noOutput)
    return
  found = sorted(alive)
  for a in found:
    print('    0x%03X  %-13s %s' % (a, KNOWN.get(a, ''), verdict(alive[a])[2]))

  print('\n[2] identity (read only)')
  for did, what in ((0xF190, 'VIN'), (0xF18C, 'serial'), (0xF187, 'partNo')):
    res = ask(p, args.bus, found, b'\x22' + did.to_bytes(2, 'big'))
    for a in sorted(res):
      neg, _, txt = verdict(res[a])
      if not neg:
        try:
          txt = res[a][4:].decode('ascii', 'replace').strip('\x00 \xff') or txt
        except Exception:
          pass
      print('    0x%03X %-7s %s' % (a, what, txt))

  print('\n[3] IO control 0x2F, asked with a deliberately invalid identifier')
  print('    "request out of range"  = SUPPORTS it, we just named the wrong thing  <-- what we want')
  print('    "service not supported" = no write path here')
  res = ask(p, args.bus, found, b'\x2f\xff\xff\x00')
  supports_2f = []
  for a in sorted(res):
    neg, nrc, txt = verdict(res[a])
    star = ''
    if neg and nrc in (0x31, 0x22, 0x33, 0x13):
      star = '   <== SUPPORTS 0x2F'
      supports_2f.append(a)
    elif not neg:
      star = '   <== answered positively?!'
      supports_2f.append(a)
    print('    0x%03X  %-45s%s' % (a, txt, star))

  print('\n[4] routineControl 0x31, same trick')
  res = ask(p, args.bus, found, b'\x31\x01\xff\xff')
  supports_31 = []
  for a in sorted(res):
    neg, nrc, txt = verdict(res[a])
    star = ''
    if neg and nrc in (0x31, 0x22, 0x33, 0x13):
      star = '   <== SUPPORTS 0x31'
      supports_31.append(a)
    print('    0x%03X  %-45s%s' % (a, txt, star))

  print('\n[5] extended diagnostic session 0x10 03 (the session actuator tests need)')
  res = ask(p, args.bus, found, b'\x10\x03')
  opens = []
  for a in sorted(res):
    neg, _, txt = verdict(res[a])
    if not neg:
      opens.append(a)
    print('    0x%03X  %s%s' % (a, txt, '   <== session opened' if not neg else ''))
  # be polite: drop everyone back to the default session
  ask(p, args.bus, found, b'\x10\x01', wait=0.2)

  print('\n--- so ---')
  if supports_2f or supports_31:
    print('There IS a diagnostic write path on this car.')
    print('  0x2F IO control : %s' % (', '.join('0x%03X' % a for a in supports_2f) or 'none'))
    print('  0x31 routines   : %s' % (', '.join('0x%03X' % a for a in supports_31) or 'none'))
    print('  extended session: %s' % (', '.join('0x%03X' % a for a in opens) or 'none'))
    print('Next is finding the identifier for the wipers, which is a search, not a question')
    print('of whether it is possible.')
  else:
    print('No ECU admitted to 0x2F or 0x31. If that holds with the ignition on and after')
    print('trying --obd, CAN is a dead end for driving the wipers and the answer is a relay.')

  p.set_safety_mode(CarParams.SafetyModel.noOutput)
  print('\npanda back to noOutput. Nothing was written. Remember: sudo systemctl start comma')


if __name__ == '__main__':
  main()
