"""抽出 dash.html 裡的 inline <script>，交給 node --check 驗語法。
改了 HUD 的 JS 一定要跑這個：語法錯了整個畫面會空白，而那是在車上才發現。"""
import re
import subprocess
import sys
import tempfile
from pathlib import Path

src = Path(r'E:/Documents/GitHub/openpilot-master/openpilot/hud/dash.html').read_text(encoding='utf-8')
blocks = re.findall(r'<script\b([^>]*)>(.*?)</script>', src, re.S | re.I)
print(f'找到 {len(blocks)} 個 script 區塊')

bad = 0
for i, (attrs, body) in enumerate(blocks):
  if 'src=' in attrs.lower():
    print(f'  [{i}] 外部 script，跳過')
    continue
  if not body.strip():
    continue
  with tempfile.NamedTemporaryFile('w', suffix='.js', delete=False, encoding='utf-8') as f:
    f.write(body)
    tmp = f.name
  r = subprocess.run(['node', '--check', tmp], capture_output=True, text=True)
  if r.returncode == 0:
    print(f'  [{i}] {len(body):7d} bytes  OK')
  else:
    bad += 1
    print(f'  [{i}] {len(body):7d} bytes  語法錯誤：')
    print('     ' + (r.stderr or r.stdout).strip().replace('\n', '\n     ')[:600])
  Path(tmp).unlink(missing_ok=True)

print('全部通過' if not bad else f'{bad} 個區塊有語法錯誤')
sys.exit(1 if bad else 0)
