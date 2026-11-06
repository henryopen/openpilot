#!/usr/bin/env python3
"""accel_compare: the acceleration limits we run now, beside open251021's eco profile.

Both sides are more than one curve. The base limit is a function of current speed, and each
fork then multiplies it by things that depend on the set speed and the gap still to close,
so comparing only the base curves would be misleading. Everything here is printed in km/h
because that is how the car is driven.

  ./accel_compare.py
"""
import numpy as np

CV = 3.6

# ---- what runs on the car now (openpilot master + this fork's changes) -------------------
A_CRUISE_MAX_VALS = [1.2, 1.1, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -1.2
J_CRUISE_COMFORT = 0.16          # shapes the command as sqrt(2*j*error) near the set speed
V_CRUISE_DEADZONE = 0.25         # added today
_LOW_SET_SPEED_BP = [0., 7.5, 15.]   # added today: scale by how low the set speed is

# ---- open251021 (FrogPilot) -------------------------------------------------------------
ACCEL_MIN = -3.5                 # opendbc value both forks import
A_CRUISE_MAX_BP_CUSTOM = [0.0, 5., 10., 15., 20., 25., 40.]
A_CRUISE_MAX_VALS_ECO = [1.6, 1.0, 0.5, 0.5, 0.5, 0.3, 0.2]
A_CRUISE_MAX_VALS_SPORT = [3.0, 2.5, 2.0, 1.5, 1.0, 0.8, 0.6]
CITY_SPEED_LIMIT = 15            # m/s


def now_base(v_ego):
  return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))


def now_set_speed_scale(max_accel, v_cruise):
  return float(np.interp(v_cruise, _LOW_SET_SPEED_BP, [max_accel / 4, max_accel / 2, max_accel]))


def eco_base(v_ego):
  return float(np.interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_ECO))


def sport_base(v_ego):
  return float(np.interp(v_ego, A_CRUISE_MAX_BP_CUSTOM, A_CRUISE_MAX_VALS_SPORT))


def eco_low_speeds(max_accel, v_cruise):
  return float(np.interp(v_cruise, [0., CITY_SPEED_LIMIT / 2, CITY_SPEED_LIMIT],
                         [max_accel / 4, max_accel / 2, max_accel]))


def eco_ramp_off(max_accel, v_cruise, v_ego):
  return float(np.interp(v_cruise - v_ego, [0., 1., 5.], [0., 0.5, max_accel]))


def comfort_cmd(v_cruise, v_ego, deadzone):
  """What the law actually asks for, before any ceiling is applied."""
  err = v_cruise - v_ego
  if deadzone:
    err = 0.0 if abs(err) <= deadzone else err - np.sign(err) * deadzone
  return float(min(abs(err), np.sqrt(2. * J_CRUISE_COMFORT * abs(err))))


def main():
  print('=' * 78)
  print('1. 基礎加速上限（只看目前車速）')
  print('=' * 78)
  print(f'{"車速":>8} {"現在":>10} {"ECO":>10} {"SPORT":>10}   ECO vs 現在')
  for kph in (0, 10, 18, 27, 36, 54, 72, 90, 108):
    v = kph / CV
    a, e, s = now_base(v), eco_base(v), sport_base(v)
    d = '較猛' if e > a + 0.02 else ('較緩' if e < a - 0.02 else '相同')
    print(f'{kph:6d}km/h {a:10.2f} {e:10.2f} {s:10.2f}   {d} ({e - a:+.2f})')

  print()
  print('=' * 78)
  print('2. 起步上限，依「設定速度」縮放（兩邊都有，公式相同）')
  print('=' * 78)
  print(f'{"設定速度":>10} {"現在":>10} {"ECO":>10}')
  for kph in (30, 40, 50, 54, 60, 80, 100):
    vc = kph / CV
    a = now_set_speed_scale(now_base(0.0), vc)
    e = eco_low_speeds(eco_base(0.0), vc)
    print(f'{kph:8d}km/h {a:10.2f} {e:10.2f}')
  print('  註：ECO 這層要 human_acceleration 開著才生效；我們是無條件生效')

  print()
  print('=' * 78)
  print('3. 接近設定速度時怎麼收斂（這是實際會不會衝過頭的關鍵）')
  print('=' * 78)
  print(f'{"還差":>8} {"現在(deadzone+sqrt塑形)":>24} {"ECO(ramp_off 上限)":>22}')
  for kph in (0.5, 0.9, 1.5, 3.0, 5.0, 10.0, 20.0):
    gap = kph / CV
    now = comfort_cmd(gap, 0.0, V_CRUISE_DEADZONE)
    eco_cap = eco_ramp_off(eco_base(50 / CV), 50 / CV, 50 / CV - gap)
    print(f'{kph:6.1f}km/h {now:24.3f} {eco_cap:22.3f}')
  print('  註：兩邊機制不同。我們是「把小誤差直接忽略」，')
  print('      ECO 是「按剩餘差距把上限壓下來」，差距 0 時上限為 0。')

  print()
  print('=' * 78)
  print('4. 減速側')
  print('=' * 78)
  print(f'  巡航減速下限   現在 {A_CRUISE_MIN:.2f}      ECO {ACCEL_MIN:.2f}  (SPORT {ACCEL_MIN * 2:.2f})')
  print(f'  ECO 允許的減速是我們的 {ACCEL_MIN / A_CRUISE_MIN:.1f} 倍')

  print()
  print('=' * 78)
  print('5. 實際情境：從停止起步，設定 40 km/h，無前車')
  print('=' * 78)
  print(f'{"車速":>8} {"現在上限":>10} {"現在指令":>10} {"ECO上限":>10} {"ECO指令":>10}')
  vc = 40 / CV
  for kph in (0, 5, 10, 20, 30, 36, 39, 40):
    v = kph / CV
    a_cap = now_set_speed_scale(now_base(v), vc)
    a_cmd = min(comfort_cmd(vc, v, V_CRUISE_DEADZONE), a_cap)
    e_cap = eco_low_speeds(eco_base(v), vc)
    e_cap = min(e_cap, eco_ramp_off(e_cap, vc, v))
    e_cmd = min(comfort_cmd(vc, v, 0.0), e_cap)
    print(f'{kph:6d}km/h {a_cap:10.2f} {a_cmd:10.2f} {e_cap:10.2f} {e_cmd:10.2f}')


if __name__ == '__main__':
  main()
