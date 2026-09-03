"""Feed real capnp readers to StopForLights, the way the car will, before the car sees it.

The device script does this on the car with LogReader; this is the same test run here,
where the recordings already are. Only the two device-only imports are stubbed - Params
and the constants - so the module itself, and every field access in it, is the real one.
Nothing is converted to a list: that conversion is what hid the capnp slice bug on 08-27.
"""
import importlib.util
import sys
import types

import capnp
import zstandard

# --- stub only what the device provides, nothing of the module's own logic -------------
CV = types.SimpleNamespace(KPH_TO_MS=1 / 3.6, MS_TO_KPH=3.6)
mod_const = types.ModuleType('openpilot.common.constants')
mod_const.CV = CV


class Params:
  def get_bool(self, k):
    return True


mod_params = types.ModuleType('openpilot.common.params')
mod_params.Params = Params
mod_rt = types.ModuleType('openpilot.common.realtime')
mod_rt.DT_MDL = 0.05
for name, m in [('openpilot', types.ModuleType('openpilot')),
                ('openpilot.common', types.ModuleType('openpilot.common')),
                ('openpilot.common.constants', mod_const),
                ('openpilot.common.params', mod_params),
                ('openpilot.common.realtime', mod_rt)]:
  sys.modules[name] = m

SRC = r'E:/Documents/GitHub/openpilot-master/openpilot/selfdrive/controls/lib/stop_for_lights.py'
spec = importlib.util.spec_from_file_location('stop_for_lights', SRC)
sfl_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sfl_mod)

capnp.remove_import_hook()
import os
# The car's own schema. The 2025-11 viewer copy loads too, but it predates selfdriveState
# and still calls leadOne.present "status", so every frame here raised and the check passed
# on nothing at all.
SCHEMA = r'F:/c4sunny/schema_hcop'
os.chdir(SCHEMA)
log = capnp.load('log.capnp', imports=[SCHEMA])

SEGS = []
for base in [r'F:/c4sunny/rlog20260830F/day', r'F:/c4sunny/rlog20260829F/night']:
  for d in sorted(os.listdir(base), key=lambda s: int(s.rsplit('--', 1)[1]))[:12]:
    p = os.path.join(base, d, 'rlog.zst')
    if os.path.exists(p):
      SEGS.append(p)

sfl = sfl_mod.StopForLights()
n = arm = act = 0
v_ego = v_cruise = 0.0
gas = False
lead = None
errs = []
for p in SEGS:
  data = zstandard.ZstdDecompressor().stream_reader(open(p, 'rb')).read()
  for e in log.Event.read_multiple_bytes(data):
    try:
      w = e.which()
    except Exception:
      continue
    if w == 'carState':
      cs = e.carState
      v_ego, gas = float(cs.vEgo), bool(cs.gasPressed)
      v_cruise = 40 / 3.6
    elif w == 'radarState':
      try:
        lead = e.radarState.leadOne
      except Exception:
        lead = None
    elif w == 'modelV2':
      n += 1
      try:
        sfl.update(e.modelV2, v_ego, v_cruise, gas, lead)   # real capnp, zero conversion
        if sfl.armed:
          arm += 1
        if sfl.is_active:
          act += 1
          sfl.obstacle_x(6.0)   # the junction now reaches the MPC as an obstacle, not a curve
      except Exception as ex:
        errs.append(f'{type(ex).__name__}: {ex}')
        if len(errs) > 3:
          break
  if errs:
    break

print(f'{len(SEGS)} 段、{n} 幀 modelV2，真 capnp 零轉換')
if errs:
  print(f'✗ 例外 {len(errs)} 次：')
  for x in errs[:4]:
    print('   ', x)
else:
  print('✓ 沒有例外')
print(f'  武裝 {arm} 幀（{arm/max(n,1)*100:.1f}%）  承諾 {act} 幀（{act/max(n,1)*100:.1f}%）')
