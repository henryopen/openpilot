#!/usr/bin/env python3
"""sunnypilot local settings web UI (standalone, stdlib only).

Lives in /data/sp_webui/ (outside /data/openpilot) so openpilot updates and
branch switches never touch it. Renders the device's own settings_ui.json.
LAN-only by trust: binds 0.0.0.0, no auth (user's decision, home/office LAN).
"""
import json
import os
import re
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = int(os.environ.get("SPWEB_PORT", "8899"))
PARAMS_DIR = os.environ.get("SPWEB_PARAMS", "/data/params/d")
MEM_PARAMS_DIR = os.environ.get("SPWEB_MEM_PARAMS", "/dev/shm/params/d")
CACHE_DIR = os.environ.get("SPWEB_CACHE", os.path.dirname(os.path.abspath(__file__)))
SETTINGS_UI_CANDIDATES = [
  os.environ.get("SPWEB_SETTINGS", ""),
  "/data/openpilot/sunnypilot/sunnylink/settings_ui.json",           # old layout (release-mici)
  "/data/openpilot/openpilot/sunnypilot/sunnylink/settings_ui.json",  # new layout (master)
]
NATIONS_URL = "https://raw.githubusercontent.com/pfeiferj/openpilot-mapd/main/nation_bounding_boxes.json"
NATIONS_CACHE = os.path.join(CACHE_DIR, "nations_cache.json")

# capability context for THIS device (Hyundai Custin, stock long, ICBM on)
CAPABILITIES = {
  "has_longitudinal_control": False,
  "has_icbm": True,
  "icbm_available": True,
  "brand": "hyundai",
  "is_sp_release": True,
}

OSM_KEYS = {"OsmLocal", "OsmLocationName", "OsmLocationTitle", "OsmStateName",
            "OsmStateTitle", "OsmDbUpdatesCheck", "OsmDownloadedDate"}
EXTRA_KEYS = {"IsMetric", "DisableUpdates"}  # writable besides settings_ui items


# ---------- params ----------

def read_param(key, mem=False):
  path = os.path.join(MEM_PARAMS_DIR if mem else PARAMS_DIR, key)
  try:
    with open(path, "rb") as f:
      return f.read().decode("utf-8", errors="replace")
  except OSError:
    return None


def write_param(key, value):
  os.makedirs(PARAMS_DIR, exist_ok=True)
  path = os.path.join(PARAMS_DIR, key)
  tmp = os.path.join(PARAMS_DIR, f".tmp_spweb_{key}")
  with open(tmp, "wb") as f:
    f.write(str(value).encode())
    f.flush()
    os.fsync(f.fileno())
  os.rename(tmp, path)
  try:  # fsync the dir so the rename survives power loss (not supported on Windows dev box)
    dfd = os.open(PARAMS_DIR, os.O_RDONLY)
    os.fsync(dfd)
    os.close(dfd)
  except OSError:
    pass


def remove_param(key):
  try:
    os.remove(os.path.join(PARAMS_DIR, key))
  except OSError:
    pass


# ---------- settings_ui.json ----------

def load_settings_ui():
  for p in SETTINGS_UI_CANDIDATES:
    if p and os.path.isfile(p):
      with open(p, encoding="utf-8") as f:
        return json.load(f)
  return {"panels": []}


def iter_items(ui):
  for p in ui.get("panels", []):
    for s in p.get("sections", []):
      yield from s.get("items", [])
      for sp in s.get("sub_panels", []):
        yield from sp.get("items", [])


def allowed_keys(ui):
  keys = {it["key"] for it in iter_items(ui) if it.get("key")}
  return keys | OSM_KEYS | EXTRA_KEYS


def offroad_only_keys(ui):
  keys = set()
  for it in iter_items(ui):
    for c in it.get("enablement", []) + it.get("visibility", []):
      if c.get("type") == "offroad_only":
        keys.add(it.get("key"))
  return keys


# ---------- condition evaluator ----------

def as_bool(v):
  return v in ("1", 1, True, "true", "True")


def eval_cond(c, onroad):
  t = c.get("type")
  if t == "offroad_only":
    return not onroad
  if t == "not_engaged":
    return not onroad  # good enough: offroad implies not engaged
  if t == "capability":
    val = CAPABILITIES.get(c.get("field"))
    if val is None:
      return True  # unknown capability: don't hide
    return val == c.get("equals")
  if t == "param":
    raw = read_param(c.get("key"))
    want = c.get("equals")
    if isinstance(want, bool):
      return as_bool(raw) == want
    return (raw or "") == str(want)
  if t == "param_compare":
    try:
      cur = float(read_param(c.get("key")) or 0)
      ref = float(c.get("value", 0))
    except ValueError:
      return False
    op = c.get("op", "==")
    return {">": cur > ref, "<": cur < ref, ">=": cur >= ref,
            "<=": cur <= ref, "==": cur == ref, "!=": cur != ref}.get(op, False)
  if t == "not":
    return not eval_cond(c.get("condition", {}), onroad)
  if t == "any":
    return any(eval_cond(x, onroad) for x in c.get("conditions", []))
  if t == "all":
    return all(eval_cond(x, onroad) for x in c.get("conditions", []))
  return True


def eval_conds(conds, onroad):
  return all(eval_cond(c, onroad) for c in (conds or []))


# ---------- model ----------

def build_model():
  ui = load_settings_ui()
  onroad = as_bool(read_param("IsOnroad"))
  panels = []
  for p in ui.get("panels", []):
    sections = []
    for s in p.get("sections", []):
      def conv(items):
        out = []
        for it in items:
          key = it.get("key")
          out.append({
            "key": key,
            "title": it.get("title", key),
            "description": it.get("description", ""),
            "widget": it.get("widget", "toggle"),
            "options": [
              {"label": o.get("label"), "value": o.get("value"),
               "enabled": eval_conds(o.get("enablement"), onroad)}
              for o in it.get("options", [])
            ],
            "min": it.get("min"), "max": it.get("max"), "step": it.get("step"),
            "unit": it.get("unit"),
            "value": read_param(key),
            "visible": eval_conds(it.get("visibility"), onroad),
            "enabled": eval_conds(it.get("enablement"), onroad),
          })
        return out

      sub_panels = []
      for sp in s.get("sub_panels", []):
        sub_panels.append({
          "label": sp.get("label", ""),
          "visible": eval_conds([sp.get("trigger_condition")] if sp.get("trigger_condition") else [], onroad),
          "items": conv(sp.get("items", [])),
        })
      sections.append({
        "title": s.get("title", ""),
        "description": s.get("description", ""),
        "visible": eval_conds(s.get("enablement"), onroad),
        "items": conv(s.get("items", [])),
        "sub_panels": sub_panels,
      })
    panels.append({"id": p.get("id"), "label": p.get("label"), "sections": sections})

  progress = None
  try:
    progress = json.loads(read_param("OSMDownloadProgress", mem=True) or "null")
  except json.JSONDecodeError:
    pass
  downloading = bool(read_param("OSMDownloadLocations", mem=True))

  return {
    "panels": panels,
    "status": {
      "onroad": onroad,
      "branch": read_param("GitBranch"),
      "version": read_param("Version"),
      "is_metric": as_bool(read_param("IsMetric")),
    },
    "osm": {
      "location_title": read_param("OsmLocationTitle"),
      "location_name": read_param("OsmLocationName"),
      "downloaded_date": read_param("OsmDownloadedDate"),
      "pending": as_bool(read_param("OsmDbUpdatesCheck")),
      "downloading": downloading,
      "progress": progress,
      "mapd_version": read_param("MapdVersion"),
    },
  }


# ---------- OSM nations ----------

_nations_lock = threading.Lock()


def get_nations():
  with _nations_lock:
    try:
      req = urllib.request.Request(NATIONS_URL, headers={"User-Agent": "sp-webui"})
      with urllib.request.urlopen(req, timeout=10) as r:
        data = json.loads(r.read().decode())
      with open(NATIONS_CACHE, "w", encoding="utf-8") as f:
        json.dump(data, f)
      return data
    except Exception:
      try:
        with open(NATIONS_CACHE, encoding="utf-8") as f:
          return json.load(f)
      except Exception:
        return {}


def start_osm_download(ref, title):
  # mirror selfdrive/ui/sunnypilot/layouts/settings/osm.py behavior
  write_param("OsmLocal", "1")
  remove_param("OsmStateName")
  remove_param("OsmStateTitle")
  write_param("OsmLocationName", ref)
  write_param("OsmLocationTitle", title)
  write_param("OsmDbUpdatesCheck", "1")


# ---------- HTTP ----------

class Handler(BaseHTTPRequestHandler):
  server_version = "sp-webui/1.0"

  def log_message(self, format, *args):  # noqa: A002
    pass

  def _json(self, obj, code=200):
    body = json.dumps(obj).encode()
    self.send_response(code)
    self.send_header("Content-Type", "application/json; charset=utf-8")
    self.send_header("Content-Length", str(len(body)))
    self.end_headers()
    self.wfile.write(body)

  def _read_body(self):
    n = int(self.headers.get("Content-Length", 0))
    return json.loads(self.rfile.read(n).decode() or "{}")

  def do_GET(self):
    if self.path == "/" or self.path.startswith("/index"):
      body = PAGE.encode()
      self.send_response(200)
      self.send_header("Content-Type", "text/html; charset=utf-8")
      self.send_header("Content-Length", str(len(body)))
      self.end_headers()
      self.wfile.write(body)
    elif self.path == "/api/model":
      self._json(build_model())
    elif self.path == "/api/osm/nations":
      self._json(get_nations())
    else:
      self._json({"error": "not found"}, 404)

  def do_POST(self):
    try:
      if self.path == "/api/set":
        req = self._read_body()
        key, value = req.get("key"), req.get("value")
        ui = load_settings_ui()
        if key not in allowed_keys(ui):
          return self._json({"ok": False, "error": f"key not allowed: {key}"}, 400)
        if as_bool(read_param("IsOnroad")) and key in offroad_only_keys(ui):
          return self._json({"ok": False, "error": "offroad only: stop the car first"}, 409)
        if isinstance(value, bool):
          value = "1" if value else "0"
        if not re.fullmatch(r"[\w .,:+-]*", str(value)):
          return self._json({"ok": False, "error": "bad value"}, 400)
        write_param(key, value)
        return self._json({"ok": True, "key": key, "value": read_param(key)})
      if self.path == "/api/osm/download":
        req = self._read_body()
        ref, title = req.get("ref", ""), req.get("title", "")
        if not re.fullmatch(r"[A-Za-z-]{2,20}", ref):
          return self._json({"ok": False, "error": "bad ref"}, 400)
        start_osm_download(ref, title or ref)
        return self._json({"ok": True})
      return self._json({"error": "not found"}, 404)
    except Exception as e:  # keep server alive
      return self._json({"ok": False, "error": str(e)}, 500)


PAGE = r"""<!doctype html>
<html lang="zh-Hant"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>C4 settings</title>
<style>
:root{--bg:#101418;--card:#1a2027;--fg:#e8edf2;--dim:#8b98a5;--acc:#33b864;--warn:#e0a030;--line:#2a323c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:16px/1.45 system-ui,"Noto Sans TC",sans-serif}
header{padding:14px 16px 8px}h1{font-size:20px;margin:0}#status{color:var(--dim);font-size:13px;margin-top:2px}
#tabs{display:flex;gap:6px;overflow-x:auto;padding:10px 12px;position:sticky;top:0;background:var(--bg);z-index:5}
#tabs button{flex:0 0 auto;border:1px solid var(--line);background:var(--card);color:var(--fg);padding:8px 14px;border-radius:20px;font-size:15px}
#tabs button.on{background:var(--acc);border-color:var(--acc);color:#08130b;font-weight:600}
main{padding:0 12px 60px;max-width:760px;margin:0 auto}
.sec{margin:14px 0 6px;color:var(--dim);font-size:13px;text-transform:uppercase;letter-spacing:.06em}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;overflow:hidden;margin-bottom:10px}
.item{display:flex;align-items:center;gap:12px;padding:12px 14px;border-top:1px solid var(--line)}
.item:first-child{border-top:0}.item.off{opacity:.42;pointer-events:none}
.grow{flex:1;min-width:0}.t{font-size:15px}.d{color:var(--dim);font-size:12.5px;margin-top:2px;white-space:pre-wrap}
.sw{position:relative;width:50px;height:28px;flex:0 0 auto;border-radius:14px;background:#39434e;transition:.15s;cursor:pointer}
.sw.on{background:var(--acc)}.sw i{position:absolute;top:3px;left:3px;width:22px;height:22px;border-radius:50%;background:#fff;transition:.15s}
.sw.on i{left:25px}
.seg{display:flex;flex-wrap:wrap;gap:6px}.seg button{border:1px solid var(--line);background:#232b34;color:var(--fg);padding:6px 12px;border-radius:8px;font-size:13.5px}
.seg button.on{background:var(--acc);border-color:var(--acc);color:#08130b;font-weight:600}
.seg button:disabled{opacity:.35}
.num{display:flex;align-items:center;gap:8px}.num button{width:38px;height:34px;border:1px solid var(--line);background:#232b34;color:var(--fg);border-radius:8px;font-size:18px}
.num span{min-width:56px;text-align:center;font-size:16px}
.badge{font-size:12px;color:var(--warn)}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:#000c;padding:10px 18px;border-radius:10px;font-size:14px;opacity:0;transition:.25s;pointer-events:none}
#toast.show{opacity:1}
select{background:#232b34;color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:8px;font-size:15px;max-width:100%}
.btn{background:var(--acc);border:0;color:#08130b;font-weight:600;padding:10px 16px;border-radius:10px;font-size:15px}
.btn:disabled{opacity:.4}
.bar{height:8px;background:#232b34;border-radius:4px;overflow:hidden}.bar i{display:block;height:100%;background:var(--acc);width:0}
</style></head><body>
<header><h1>C4 settings</h1><div id="status">連線中…</div></header>
<nav id="tabs"></nav>
<main id="main"></main>
<div id="toast"></div>
<script>
let MODEL=null,TAB=localStorage.getItem('tab')||'',NATIONS=null;
const $=s=>document.querySelector(s);
function toast(m){const t=$('#toast');t.textContent=m;t.classList.add('show');clearTimeout(t._h);t._h=setTimeout(()=>t.classList.remove('show'),2200)}
async function api(p,body){const r=await fetch(p,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{});return r.json()}
async function setParam(key,value){const r=await api('/api/set',{key,value});if(!r.ok){toast('寫入失敗: '+(r.error||''));}else{toast(key+' = '+r.value)}await refresh(false)}
function boolVal(v){return v==='1'}
function itemRow(it){
  if(!it.visible)return '';
  const off=it.enabled?'':' off';
  let ctrl='';
  if(it.widget==='toggle'){
    ctrl=`<div class="sw ${boolVal(it.value)?'on':''}" onclick="setParam('${it.key}',${boolVal(it.value)?'0':'1'})"><i></i></div>`;
  }else if(it.widget==='multiple_button'){
    ctrl='<div class="seg">'+it.options.map(o=>
      `<button ${o.enabled?'':'disabled'} class="${String(o.value)===String(it.value??'')?'on':''}" onclick="setParam('${it.key}','${o.value}')">${o.label}</button>`).join('')+'</div>';
  }else if(it.widget==='option'){
    const v=parseFloat(it.value??it.min??0)||0;
    const unit=it.unit?(MODEL.status.is_metric?it.unit.metric:it.unit.imperial):'';
    ctrl=`<div class="num"><button onclick="setParam('${it.key}','${Math.max(it.min??-999,v-(it.step||1))}')">−</button><span>${isNaN(v)?'—':v} ${unit||''}</span><button onclick="setParam('${it.key}','${Math.min(it.max??999,v+(it.step||1))}')">+</button></div>`;
  }else{
    ctrl=`<span class="badge">${it.value??'—'}</span>`;
  }
  const seg=it.widget==='multiple_button';
  return `<div class="item${off}"><div class="grow"><div class="t">${it.title}</div>${it.description?`<div class="d">${it.description}</div>`:''}${seg?'<div style="margin-top:8px">'+ctrl+'</div>':''}</div>${seg?'':ctrl}</div>`;
}
function osmSection(){
  const o=MODEL.osm;let dl='';
  if(o.downloading||o.pending){
    const p=o.progress||{},tot=p.total_files||0,done=p.downloaded_files||0;
    const pct=tot?Math.round(done/tot*100):0;
    dl=`<div class="item"><div class="grow"><div class="t">下載中… ${done}/${tot} (${pct}%)</div><div class="bar" style="margin-top:6px"><i style="width:${pct}%"></i></div></div></div>`;
  }
  const cur=o.location_title?`目前圖資：${o.location_title}${o.downloaded_date?'（'+new Date(parseFloat(o.downloaded_date)*1000).toLocaleDateString()+'）':''}`:'尚未下載任何圖資';
  return `<div class="sec">OSM 圖資（Speed Limit / SCC-Map 需要）</div><div class="card">
    <div class="item"><div class="grow"><div class="t">${cur}</div><div class="d">mapd ${o.mapd_version||''}</div></div></div>
    <div class="item"><div class="grow"><select id="nation"><option>載入國家清單…</option></select></div>
    <button class="btn" onclick="osmGo()">下載</button></div>${dl}</div>`;
}
async function osmGo(){
  const sel=$('#nation');const ref=sel.value;if(!ref||!NATIONS)return;
  const title=NATIONS[ref]?NATIONS[ref].full_name:ref;
  if(!confirm(`下載 ${title} 圖資？（需要 WiFi，數百 MB）`))return;
  const r=await api('/api/osm/download',{ref,title});
  toast(r.ok?'已開始下載':'失敗: '+(r.error||''));refresh(false);
}
async function loadNations(){
  if(NATIONS)return;NATIONS=await api('/api/osm/nations');
  const sel=$('#nation');if(!sel)return;
  const cur=MODEL.osm.location_name||'TW';
  sel.innerHTML=Object.entries(NATIONS).sort((a,b)=>a[1].full_name.localeCompare(b[1].full_name))
    .map(([k,v])=>`<option value="${k}" ${k===cur?'selected':''}>${v.full_name}</option>`).join('');
}
function render(){
  const tabs=$('#tabs');
  tabs.innerHTML=MODEL.panels.map(p=>`<button class="${p.id===TAB?'on':''}" onclick="TAB='${p.id}';localStorage.setItem('tab',TAB);render()">${p.label}</button>`).join('');
  if(!MODEL.panels.find(p=>p.id===TAB))TAB=MODEL.panels[0]?.id;
  const panel=MODEL.panels.find(p=>p.id===TAB);if(!panel)return;
  let html='';
  for(const s of panel.sections){
    if(!s.visible)continue;
    const rows=s.items.map(itemRow).join('')+s.sub_panels.filter(sp=>sp.visible).map(sp=>sp.items.map(itemRow).join('')).join('');
    if(!rows)continue;
    html+=(s.title?`<div class="sec">${s.title}</div>`:'')+`<div class="card">${rows}</div>`;
  }
  if(panel.id==='cruise')html+=osmSection();
  $('#main').innerHTML=html;
  const st=MODEL.status;
  $('#status').textContent=`${st.branch||''} ${st.version||''} · ${st.onroad?'⚠️ 行車中（多數設定鎖定）':'✅ Offroad 可設定'}`;
  if(panel.id==='cruise')loadNations();
}
async function refresh(first=true){
  try{MODEL=await api('/api/model');if(!TAB)TAB=MODEL.panels[0]?.id;render()}
  catch(e){$('#status').textContent='連不上裝置…'}
}
refresh();setInterval(()=>refresh(false),4000);
</script></body></html>
"""


def main():
  srv = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
  print(f"sp-webui listening on :{PORT}, params={PARAMS_DIR}")
  srv.serve_forever()


if __name__ == "__main__":
  main()
