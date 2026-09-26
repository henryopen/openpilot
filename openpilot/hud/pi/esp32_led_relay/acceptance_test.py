"""Acceptance test for the relay, run from the machine the board is plugged into.

  python acceptance_test.py COM5          (or /dev/ttyACM0)

Only info, getVerificationCode and userLogin are called - never anything that changes the card's
settings. All three must come back 200; the last line is the info round trip, for reference.
"""
import hashlib
import json
import struct
import sys
import time

import serial

s = serial.Serial()
s.port, s.baudrate, s.timeout = sys.argv[1], 921600, 0.2
s.dtr, s.rts = False, False   # these drive the board's reset/boot pins
s.open()


def req(method, path, body=b"", timeout=12):
  p = path.encode()
  s.write(b"LRQ1" + method.encode() + struct.pack(">HI", len(p), len(body)) + p + body)
  deadline, win = time.monotonic() + timeout, b""
  while win != b"LRS1":
    c = s.read(1)
    if c:
      win = (win + c)[-4:]
    elif time.monotonic() > deadline:
      raise TimeoutError("no reply")
  code, n = struct.unpack(">hI", s.read(6))
  data = b""
  while len(data) < n:
    data += s.read(n - len(data))
  return code, data


def fn(name, inp):
  return json.dumps({"protocol": {"name": "YQ-COM2", "version": "1.0",
                                  "remotefunction": {"name": name, "input": inp}}}).encode()


t0 = time.monotonic()
while not json.loads(req("I", "/")[1]).get("wifi") and time.monotonic() - t0 < 15:
  time.sleep(0.2)   # opening the port may reset the board; it rejoins the AP in ~2 s
print("1 info:", req("I", "/"))
code, d = req("P", "/", fn("getVerificationCode", {"username": "guest"}))
print("2 verification code:", code, d[:200])
v = json.loads(d)["remotefunction"]["output"]["verificationcode"]
pw = hashlib.sha1((v + hashlib.sha1(b"guest").hexdigest()).encode()).hexdigest()
code, d = req("P", "/", fn("userLogin", {"username": "guest", "verificationcode": v, "password": pw}))
print("3 login:", code, d[:200])
t0 = time.monotonic()
for _ in range(20):
  req("I", "/")
print(f"4 20 info round trips: {(time.monotonic() - t0) * 1000:.0f} ms")
