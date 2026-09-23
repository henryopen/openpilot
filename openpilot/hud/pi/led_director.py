"""後窗 LED 要顯示哪個畫面 —— 純邏輯，不碰網路也不畫圖。

輸入是車機 hud/server.py /stream 的一格 JSON（dict），輸出是 Screen(key, sev, params)。
同一份給 PiBar 上的 led_agent.py 用，也給離線重放（rlog → 同樣形狀的 dict）驗證用。

畫面衝突的處理（2026-09-23 跟使用者定的）：
  1. 本來就是同一件事的合併成一個畫面：
     - 減速的原因（前車／紅燈／彎道）是同一個減速畫面的內容，不是各自插播
     - 巡航和跟車是同一個畫面：前車每分鐘會掉失 2.6-2.9 次（9/8 實測），分開的話每 20 秒跳一次。
       前車連續出現 LEAD_ON 秒才顯示距離，連續消失 LEAD_OFF 秒才收起來
  2. 會被下一個畫面馬上蓋掉的，拉長：
     - 起步一直顯示到 LAUNCH_DONE（不然零點幾秒就被加速蓋掉）
     - 等紅燈／停等在停住那一刻就決定，停等期間不改判
     - 前車急煞至少 ALERT_HOLD 秒
  3. 切換規則：往嚴重的方向立刻切；往輕微的方向，目前畫面要播滿 MIN_SHOW 秒、
     而且新畫面要連續被選中 DEBOUNCE 秒才切。

OP 沒接手（selfdriveState.enabled 為 false）或車機待機：只顯示日期時間 —— 那時候規劃器的
aTarget／reason 不是車子實際在做的事，照著顯示會亂講話。
"""
from dataclasses import dataclass, field

KPH = 1 / 3.6

MIN_SHOW = 2.0          # 秒：一個畫面至少播這麼久才能被「較輕微」的畫面取代
DEBOUNCE = 1.0          # 秒：較輕微的新畫面要連續被選中這麼久
ENTER = 1.0             # 秒：較嚴重的新畫面也要連續被選中這麼久才上屏（URGENT 除外）。
                        # 9/18-9/20 重放：沒有這條時「準備減速」77 次裡 50 次不到 1.5 秒就被停車蓋掉
URGENT = {"hard_brake"}
ALERT_HOLD = 2.5        # 秒：前車急煞至少顯示這麼久
LEAD_ON, LEAD_OFF = 3.0, 5.0
LAUNCH_DONE = 15 * KPH  # 起步畫面維持到這個速度
LAUNCH_MAX = 8.0        # 秒：起步畫面最長（沒加速上去就算了）
STOPPED_V = 0.3         # m/s 以下當作停住
FOLLOW_STOP_GAP = 15.0  # m：停住時前車在這距離內 = 跟車停等，否則看停車前的原因

# 縱向門檻（m/s²），進入／離開分開做遲滯
DECEL_IN, DECEL_OUT = -0.5, -0.3
COAST_IN, COAST_OUT = -0.2, -0.1
ACCEL_IN, ACCEL_OUT = 0.5, 0.3
HARD_BRAKE = -2.5       # aTarget 到這裡且原因是前車 → 前車急煞

LIMIT_AHEAD_MAX = 500.0  # m：速限預告只在這距離內顯示

# 自然接續：前一個畫面講的事已經發生了，下一個直接接上，不等 MIN_SHOW
# （例如減速→停住：不然會卡在「停車 1m」兩秒）
FLOW = {("stop", "stopped"), ("decel", "stopped"), ("coast", "stopped"), ("hard_brake", "stopped"),
        ("stopped", "launch"), ("launch", "accel"), ("launch", "cruise")}

SEV = {"clock": -1, "cruise": 0, "accel": 1, "launch": 1, "limit_ahead": 1,
       "stopped": 1, "coast": 2, "decel": 3, "lane": 4, "turn": 4,
       "stop": 5, "hard_brake": 6}


@dataclass
class Screen:
  key: str
  params: dict = field(default_factory=dict)

  @property
  def sev(self):
    return SEV[self.key]


def _g(d, *path, default=None):
  for p in path:
    if not isinstance(d, dict) or p not in d:
      return default
    d = d[p]
  return d


class Director:
  def __init__(self):
    self.cur = Screen("clock")
    self.cur_since = 0.0
    self.pending_key = None
    self.pending_since = 0.0
    # 遲滯與記憶
    self.decel = self.coast = self.accel = False
    self.lead_shown = False
    self.lead_seen_since = None
    self.lead_lost_since = None
    self.last_d_rel = None
    self.stopped_since = None
    self.stop_kind = None          # 停住那一刻決定：'light' / 'follow'
    self.last_decel_reason = ""
    self.launch_since = None
    self.alert_until = 0.0

  # ------------------------------------------------------------ 候選
  def _candidate(self, d, now):
    sd = _g(d, "selfdriveState", default={})
    if d.get("standby", True) or not sd.get("enabled", False) or "carState" not in d:
      self._reset_drive()
      return Screen("clock")

    cs, ctl, rs = d["carState"], d.get("control", {}), d.get("radarState", {})
    v = float(cs.get("vEgo", 0.0))
    a = float(ctl.get("aTarget", 0.0))
    reason = ctl.get("reason", "") or ""
    gas, brake = cs.get("gasPressed", False), cs.get("brakePressed", False)

    # --- 遲滯旗標
    self.decel = a < DECEL_IN or (self.decel and a < DECEL_OUT)
    self.coast = (not self.decel) and (a < COAST_IN or (self.coast and a < COAST_OUT))
    self.accel = gas or a > ACCEL_IN or (self.accel and a > ACCEL_OUT)
    if self.decel:
      self.last_decel_reason = reason

    # --- 前車：出現 LEAD_ON 秒才顯示、連續消失 LEAD_OFF 秒才收。
    # 短暫掉失不讓「出現」計時歸零 —— 實車每分鐘掉 2.6-2.9 次，每次一歸零就永遠累積不到 LEAD_ON
    # （模擬每 3 秒掉 0.6 秒時，跟車畫面整段都沒出現過）。掉失那一下沿用最後看到的距離。
    present = bool(rs.get("leadStatus", False))
    if present:
      self.lead_lost_since = None
      self.lead_seen_since = self.lead_seen_since or now
      self.last_d_rel = float(rs.get("dRel", 0.0))
      if now - self.lead_seen_since >= LEAD_ON:
        self.lead_shown = True
    else:
      self.lead_lost_since = self.lead_lost_since or now
      if now - self.lead_lost_since >= LEAD_OFF:
        self.lead_shown = False
        self.lead_seen_since = None
    d_rel = float(rs.get("dRel", 0.0)) if present else None
    shown_d_rel = d_rel if present else (self.last_d_rel if self.lead_shown else None)

    # --- 停住／起步
    stopped = v < STOPPED_V and (cs.get("standstill", False) or v < 0.1)
    if stopped:
      if self.stopped_since is None:
        self.stopped_since = now
        close = present and d_rel is not None and d_rel < FOLLOW_STOP_GAP
        self.stop_kind = "follow" if close or self.last_decel_reason != "stoplight" else "light"
      self.launch_since = None
    else:
      if self.stopped_since is not None:            # 剛從停住開始動
        self.launch_since = now
      self.stopped_since = None
    launching = (self.launch_since is not None and v < LAUNCH_DONE
                 and now - self.launch_since < LAUNCH_MAX)
    if not launching:
      self.launch_since = None

    # --- 依嚴重度從高到低
    if now < self.alert_until or (a <= HARD_BRAKE and reason.startswith(("lead", "weaklead")) and not stopped):
      if now >= self.alert_until:
        self.alert_until = now + ALERT_HOLD
      return Screen("hard_brake")
    # 停車預告用模型速度曲線停下來的位置（modelV2.stopAhead）。9/18-9/20 兩趟 37 段重放：
    # 觸發 21 次、20 秒內真的停住 16 次（76%）、中位提早 5.3 秒。
    # ⛔ 不要用 control.stopDistance：規劃沒停下來時它回的是整段 10 秒的行駛距離，不是 0（同樣資料 29%）；
    # ⛔ control.stop（shouldStop）行進中一次都沒亮過。
    stop_ahead = float(_g(d, "modelV2", "stopAhead", default=0.0) or 0.0)
    if not stopped and stop_ahead > 0 and v > 1.0:
      return Screen("stop", {"dist": stop_ahead})

    intent = d.get("intent", {})
    if intent.get("laneChangeState") == "laneChangeStarting":
      return Screen("lane", {"dir": "left" if intent.get("laneChangeDirection") == "left" else "right"})
    if intent.get("turn") in ("left", "right"):
      return Screen("turn", {"dir": intent["turn"]})

    if self.decel and not brake:
      return Screen("decel", {"reason": reason})
    if self.coast and not brake and not stopped:
      return Screen("coast")
    if stopped:
      waited = now - self.stopped_since
      if self.stop_kind == "light":
        return Screen("stopped", {"kind": "light", "secs": int(waited)})
      return Screen("stopped", {"kind": "follow", "dRel": d_rel})
    if launching:
      return Screen("launch")

    lm = d.get("liveMapDataSP", {})
    ahead, dist = float(lm.get("speedLimitAhead", 0.0)), float(lm.get("speedLimitAheadDistance", 0.0))
    cur_limit = float(lm.get("speedLimit", 0.0))
    if lm.get("speedLimitAheadValid") and 0 < dist < LIMIT_AHEAD_MAX and abs(ahead - cur_limit) > 1.0:
      return Screen("limit_ahead", {"limit": round(ahead / KPH), "dist": dist})
    if self.accel:
      return Screen("accel")
    kph = round(float(cs.get("vEgoCluster", v)) / KPH)
    if self.lead_shown and shown_d_rel is not None:
      return Screen("cruise", {"lead": True, "dRel": shown_d_rel, "kph": kph})
    return Screen("cruise", {"lead": False, "kph": kph})

  def _reset_drive(self):
    self.decel = self.coast = self.accel = False
    self.lead_shown = False
    self.lead_seen_since = self.lead_lost_since = None
    self.stopped_since = self.launch_since = None
    self.stop_kind = None
    self.alert_until = 0.0

  # ------------------------------------------------------------ 仲裁
  def update(self, d, now):
    """回傳 (Screen, 這個畫面已經顯示幾秒)。now 用單調時鐘（離線重放傳 log 的時間）。"""
    want = self._candidate(d, now)
    if want.key == self.cur.key:
      self.cur = want                                # 同一個畫面：只更新內容（距離、秒數）
      self.pending_key = None
      return self.cur, now - self.cur_since
    if self.pending_key != want.key:                 # 新畫面要「連續」被選中才算數
      self.pending_key, self.pending_since = want.key, now
    held = now - self.pending_since
    if want.key in URGENT or (self.cur.key, want.key) in FLOW:
      self._switch(want, now)
    elif want.sev > self.cur.sev:
      if held >= ENTER:
        self._switch(want, now)
    elif now - self.cur_since >= MIN_SHOW and held >= DEBOUNCE:
      self._switch(want, now)
    return self.cur, now - self.cur_since

  def _switch(self, s, now):
    self.cur, self.cur_since = s, now
    self.pending_key = None
