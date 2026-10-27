#!/usr/bin/env python3
"""C3's voice commands, engaged or not (the driver, 10/23: "C3語音應該不管有沒有engaged都要可以用").

The same six commands the PiBar's LD3320 had (10/15; its speech_agent.py removed 10/26): 小白 + 加速 / 減速
(MAX +-10), 綠燈 (go), 謝謝 / 抱歉 / 臨時 (rear-window captions); 10/26 the other five keypad captions (禮讓行人 /
救護車 / 事故 / 靠邊 / 不熟) and 取消 (off). The LD3320 failed in the car (10/16-17: 1 command out of 55,
road noise) and was switched off. 10/18 on C3 instead: its own mic (arecord hw:0,0, 16 kHz mono) and sherpa-onnx's
Chinese keyword spotter (zipformer wenetspeech 3.3M, fp32, epoch-12-avg-2), always listening, 1 thread ~11% of a
core. Scripted check in the office, the driver reading prompts off C3's screen: 22/24 heard, none heard as another
word, nothing from 30 s of chat. Until 10/23 display only; the first drive heard 小白減速 / 小白加速 and did nothing.

From 10/23 they act, the same way the PiBar keypad does:
  加速 / 減速  max_up / max_down (MAX +-10, like the left knob: engaged or not, never an engage), POSTed with the
               keypad token to C4's HUD server over the USB link (http://10.44.0.2:8902/keypad), which hands it to
               card / plannerd (selfdrive/keypad.py). Once, 0.5 s timeout, no retry - like the keypad.
  開啟 / 關閉  main_on / main_off; 接手 / 放手  engage / disengage (10/27) - keypad keys 1-4, the same way.
  captions (謝謝 ...)  rear-window captions: nothing sent from here; the PiBar's led_agent sees the hit in C4's
               HUD stream (c3.voice) and presses the caption key itself.
  綠燈         go (moves a stopped car): only with the flag file GO_FLAG; without it display only. A false hit at
               a red light would move the car, and the road-noise false-hit rate has not been measured.
Every hit is still kept as a clip.

State for c3calib -> C4 -> HUD: /dev/shm/c3voice.json, every second (ok = the mic is delivering), n counts hits.
Hits: /data/c3link/voice.log, and the 4 s of audio up to each hit in /data/c3link/voice_clips/ (newest 300 kept),
so false hits in the car can be listened to and the keywords tuned.
"""
import collections
import datetime
import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'pylib'))          # sherpa_onnx, pip --target'ed (C3's python 3.11)
import numpy as np
import sherpa_onnx

SR = 16000
CHUNK = SR // 10                                          # 0.1 s
MODEL = os.path.join(HERE, 'models/kws')
KEYWORDS = os.path.join(HERE, 'c3voice_keywords.txt')
STATE = '/dev/shm/c3voice.json'
LOG = os.path.join(HERE, 'voice.log')
CLIPS = os.path.join(HERE, 'voice_clips')
CLIP_S, KEEP_CLIPS = 4., 300
C4_KEYPAD = 'http://10.44.0.2:8902/keypad'                # C4 over the USB link (usb_link.sh)
TOKEN_FILE = os.path.join(HERE, 'keypad_token')          # a copy of C4's /data/keypad_token
GO_FLAG = os.path.join(HERE, 'VOICE_GO')                 # present = 小白綠燈 really sends go
SEND_TIMEOUT = 0.5
# 10/27, the driver: "C3的語音蠻穩的" - MAIN and engage / disengage too, the same commands as keypad keys 1-4
# (C4 decides as it does for the keypad: engage only where the panda still allows it). Offline on C3 (SAPI zh-TW at
# three speeds + the 21 clips from the car): the four new words 12/12, none heard as another, and "我先接手一下 /
# 放手讓它開 / 開啟冷氣 / 關閉車窗" hit nothing.
CAR = {'小白加速': 'max_up', '小白減速': 'max_down', '小白綠燈': 'go',
       '小白開啟': 'main_on', '小白關閉': 'main_off', '小白接手': 'engage', '小白放手': 'disengage'}
# acted on by the PiBar's led_agent. 10/26: all eight keypad captions (the touch screen's jobs went to the keypad's
# right knob, the captions to the voice; the keypad keys stay as the backup), and 取消 turns off whichever is showing.
LED = ('小白謝謝', '小白抱歉', '小白臨時', '小白禮讓行人', '小白救護車', '小白事故', '小白靠邊', '小白不熟')
LED_OFF = '小白取消'


def write_json(path, obj):
  tmp = path + '.tmp'
  with open(tmp, 'w', encoding='utf-8') as f:
    json.dump(obj, f, ensure_ascii=False)
  os.replace(tmp, path)


def send(cmd):
  """One command to C4's keypad endpoint; 'sent' or why not."""
  try:
    with open(TOKEN_FILE) as f:
      token = f.read().strip()
  except OSError:
    return 'no token'
  body = json.dumps({'cmd': cmd, 'token': token}).encode()
  req = urllib.request.Request(C4_KEYPAD, data=body, method='POST', headers={'Content-Type': 'application/json'})
  try:
    with urllib.request.urlopen(req, timeout=SEND_TIMEOUT) as r:
      return 'sent' if r.status == 200 else f'http {r.status}'
  except urllib.error.HTTPError as e:
    return f'http {e.code}'
  except Exception as e:  # no USB link, C4 not up: the driver sees it on the HUD
    return f'no link ({type(e).__name__})'


def act(tag):
  if tag in CAR:
    if CAR[tag] == 'go' and not os.path.exists(GO_FLAG):
      return 'display only'
    return send(CAR[tag])
  if tag == LED_OFF:
    return 'led off'
  return 'led' if tag in LED else ''


def make_spotter():
  ep = 'epoch-12-avg-2-chunk-16-left-64'
  return sherpa_onnx.KeywordSpotter(
    tokens=f'{MODEL}/tokens.txt', encoder=f'{MODEL}/encoder-{ep}.onnx', decoder=f'{MODEL}/decoder-{ep}.onnx',
    joiner=f'{MODEL}/joiner-{ep}.onnx', keywords_file=KEYWORDS, num_threads=1, provider='cpu',
    keywords_score=1.0, keywords_threshold=0.25)


def save_clip(ring, tag):
  os.makedirs(CLIPS, exist_ok=True)
  name = datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + f'_{tag}.wav'
  with wave.open(os.path.join(CLIPS, name), 'wb') as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SR)
    w.writeframes(b''.join(ring))
  old = sorted(os.listdir(CLIPS))
  for f in old[:max(0, len(old) - KEEP_CLIPS)]:
    os.remove(os.path.join(CLIPS, f))
  return name


def main():
  kws = make_spotter()
  stream = kws.create_stream()
  ring = collections.deque(maxlen=int(CLIP_S * 10))
  n, last, last_t, res = 0, '', 0., ''
  rec, last_state, last_audio = None, 0., 0.
  while True:
    now = time.monotonic()
    if rec is None or rec.poll() is not None:
      rec = subprocess.Popen(['arecord', '-q', '-D', 'hw:0,0', '-f', 'S16_LE', '-r', str(SR), '-c', '1', '-t', 'raw'],
                             stdout=subprocess.PIPE)
    raw = rec.stdout.read(CHUNK * 2) if rec.stdout else b''
    if len(raw) == CHUNK * 2:
      last_audio = now
      ring.append(raw)
      a = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768
      stream.accept_waveform(SR, a)
      while kws.is_ready(stream):
        kws.decode_stream(stream)
        tag = kws.get_result(stream)
        if tag:
          kws.reset_stream(stream)
          res = act(tag)
          n, last, last_t = n + 1, tag, time.monotonic()
          try:
            clip = save_clip(ring, tag)
          except OSError as e:
            clip = f'clip failed: {e}'
          with open(LOG, 'a', encoding='utf-8') as f:
            f.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tag} {clip} {res}\n")
    else:
      time.sleep(0.5)                                     # arecord died; started again next round
      if rec.poll() is None:
        rec.kill()
    if now - last_state >= 1.:
      last_state = now
      write_json(STATE, {'ok': now - last_audio < 2., 'n': n, 'last': last, 'last_t': last_t, 'res': res, 't': now})


if __name__ == '__main__':
  main()
