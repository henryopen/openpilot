#!/usr/bin/env python3
"""C3's voice commands - DISPLAY ONLY for now: what it hears goes to C4's HUD, nothing is sent to the car.

The same six commands the PiBar's LD3320 had (hud/pi/speech_agent.py, 10/15): 小白 + 加速 / 減速 (MAX +-10),
綠燈 (go), 謝謝 / 抱歉 / 臨時 (rear-window captions). The LD3320 failed in the car (10/16-17: 1 command out of 55,
road noise) and was switched off. 10/18 on C3 instead: its own mic (arecord hw:0,0, 16 kHz mono) and sherpa-onnx's
Chinese keyword spotter (zipformer wenetspeech 3.3M, fp32, epoch-12-avg-2), always listening, 1 thread ~11% of a
core. Scripted check in the office, the driver reading prompts off C3's screen: 22/24 heard, none heard as another
word, nothing from 30 s of chat. Not yet tried in the car - so: display only, and every hit is kept as a clip.

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


def write_json(path, obj):
  tmp = path + '.tmp'
  with open(tmp, 'w', encoding='utf-8') as f:
    json.dump(obj, f, ensure_ascii=False)
  os.replace(tmp, path)


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
  n, last, last_t = 0, '', 0.
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
          n, last, last_t = n + 1, tag, time.monotonic()
          try:
            clip = save_clip(ring, tag)
          except OSError as e:
            clip = f'clip failed: {e}'
          with open(LOG, 'a', encoding='utf-8') as f:
            f.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} {tag} {clip}\n")
    else:
      time.sleep(0.5)                                     # arecord died; started again next round
      if rec.poll() is None:
        rec.kill()
    if now - last_state >= 1.:
      last_state = now
      write_json(STATE, {'ok': now - last_audio < 2., 'n': n, 'last': last, 'last_t': last_t, 't': now})


if __name__ == '__main__':
  main()
