"""後窗 LED 要顯示哪個畫面、講什麼字 —— 純邏輯，不碰網路也不畫圖。

輸入是車機 hud/server.py /stream 的一格 JSON（dict），輸出是 Screen(key, params)；
Screen.pages 是這個畫面要輪替的幾頁字（每頁 PAGE_SECS 秒），led_render 照著畫。
同一份給 PiBar 上的 led_agent.py 用，也給離線重放（rlog → 同樣形狀的 dict）驗證用。

情境照使用者 2026-09-24 的「OP 車後 LED 顯示情境表」：
  先分「前車在不在跟車距離內」（跟 HUD「淨空」同一把尺：planner 的 followDistance × 1.2），
  有前車講前車（穩定跟車／前車減速／跟上前車／前車急煞／前車停止／前車起步），
  沒前車講自己（省油加速／平穩行駛／前方速限／前方停車／停止中／調低 MAX 減速／前方有狀況），
  彎道、變換車道、路口轉彎不分有沒有前車。
  字：重要的大字、補充的小字；一頁兩秒，後車才讀得完兩行中文。
  跟車距離附近有緩衝區（使用者表上的 DEADZONE）：進入／離開分開做遲滯，不會在門檻上來回跳。
  ⛔ 不顯示自己的時速、也不顯示目標時速：9/24 使用者「免得別人說我們超速」—— MAX 跟著速限走、
  實務上會設到速限 +10。前車時速照顯示（使用者：那是前車的事，我們車速不一定等於前車）。

畫面衝突的處理（2026-09-23 定的，沿用）：
  1. 會被下一個畫面馬上蓋掉的，拉長：停等在停住那一刻就決定種類；前車急煞至少 ALERT_HOLD 秒；
     停住要開到 MOVING_V 才算離開（停等時往前挪不算走）。
  2. 往嚴重的方向要連續被選中 ENTER 秒才切（URGENT 除外）；往輕微的方向，目前畫面要播滿
     一輪（至少 MIN_SHOW 秒，多頁的播到 MAX_SHOW 秒）、新畫面也要連續被選中 DEBOUNCE 秒才切。
  3. 前一個畫面講的事已經發生了，下一個直接接上（FLOW），不等。

OP 沒接手（selfdriveState.enabled 為 false）或車機待機：只顯示日期時間 —— 那時候規劃器的
aTarget／reason 不是車子實際在做的事，照著顯示會亂講話。
YOLO 沒上、OP 分不出紅燈：「前方停車」「停止中」只講車要停，FORBIDDEN 講燈號。
"""
from dataclasses import dataclass, field

KPH = 1 / 3.6

PAGE_SECS = 2.0         # 秒：輪替的每一頁
MIN_SHOW = 2.0          # 秒：一個畫面至少播這麼久才能被「較輕微」的畫面取代
MAX_SHOW = 4.0          # 秒：多頁的畫面播完一輪（上限這麼久）才讓位
DEBOUNCE = 1.0          # 秒：較輕微的新畫面要連續被選中這麼久
ENTER = 1.0             # 秒：較嚴重的新畫面也要連續被選中這麼久才上屏（URGENT 除外）。
                        # 9/18-9/20 重放：沒有這條時「準備減速」77 次裡 50 次不到 1.5 秒就被停車蓋掉
URGENT = {"hard_brake"}
ALERT_HOLD = 2.5        # 秒：前車急煞至少顯示這麼久
LIMIT_CHANGED_HOLD = 5.0  # 秒：速限變更（使用者表上定 5 秒）
LEAD_ON, LEAD_OFF = 3.0, 5.0
STOPPED_V = 0.3         # m/s 以下當作停住
MOVING_V = 1.0          # m/s 以上才算離開停住
FOLLOW_STOP_GAP = 15.0  # m：停住時前車在這距離內 = 跟車停等，否則是停止中
LEAD_LAUNCH_V = 1.0     # m/s：跟車停等中前車開到這速度 = 前車起步
LEAD_LAUNCH_GAP = 2.0   # m：或前車比停住時拉開這麼多

# 縱向門檻（m/s²），進入／離開分開做遲滯
DECEL_IN, DECEL_OUT = -0.5, -0.3
COAST_IN, COAST_OUT = -0.2, -0.1
ACCEL_IN, ACCEL_OUT = 0.3, 0.15
HARD_BRAKE = -2.5       # aTarget 到這裡且原因是前車 → 前車急煞

# 跟車距離（比的是 dRel / followDistance）
ZONE_IN, ZONE_OUT = 1.2, 1.35        # 前車進到這倍數內 = 「有前車」；離開要超過 ZONE_OUT
CATCH_UP_MAX = 2.5                   # 跟上前車：前車拉開到這倍數內、我們在加速
# 9/24 重放三趟（118 分鐘）：門檻原本是 0.8／前車慢 1.5 m/s／前車快 0.5 m/s「或」在加速，
# 穩定跟車 ⇄ 跟上前車 216 次、⇄ 前車減速 185 次，每次只停 3.5 秒 —— 屏跟著前車的小加減速一起跳。
# 緩衝區拉寬：要明顯靠近／明顯被拉開、而且我們真的在加減速才換畫面。
CLOSE_IN, CLOSE_OUT = 0.7, 0.8       # 距離低於跟車距離
FAR_IN, FAR_OUT = 1.1, 1.05          # 距離高於跟車距離
CLOSING_IN, CLOSING_OUT = -2.0, -0.8  # m/s：前車比我們慢這麼多 = 前車在減速／我們在靠近
PULLING_AWAY = 1.0                   # m/s：前車比我們快這麼多、我們又在加速 = 跟上前車

AT_MAX = 3              # km/h：儀表速度離 MAX 這麼近就算「已達 MAX」
OVER_MAX = 2            # km/h：高於 MAX 這麼多、又在減速 = 調低 MAX 在減速

# 儀表速度換算（9/10、22915 樣本）：後車看自己的儀表，講前車時速也用儀表的尺
DASH_K, DASH_B = 1.0272, 4.00
LIMIT_AHEAD_MAX = 500.0  # m：前方速限只在這距離內預告
CURVE_SIDE_M = 0.3      # m：模型路線在 30-60 m 處偏這麼多才講左右

# 自然接續：前一個畫面講的事已經發生了，下一個直接接上
FLOW = {("stop", "stopped"), ("junction", "stopped"), ("junction", "stopped_follow"),
        ("lower_max", "stopped"), ("slow_ahead", "stopped"), ("decel", "stopped"),
        ("hard_brake", "stopped_follow"), ("lead_stop", "stopped_follow"), ("lead_slow", "stopped_follow"),
        ("stopped_follow", "lead_launch"), ("stopped", "accel"), ("stopped_follow", "catch_up"), ("stopped_follow", "accel")}
# 前車起步不在裡面：我們幾乎馬上跟著走，直接接續的話只閃 1 秒（9/24 重放 27 次裡 19 次不到 1.5 秒）

SEV = {"clock": -1,
       "cruise": 0, "follow": 0,
       "accel": 1, "catch_up": 1, "stopped": 1, "stopped_follow": 1,
       "limit_changed": 2, "lead_launch": 2, "slow_ahead": 2,
       "decel": 3, "lower_max": 3, "curve": 3, "lead_slow": 3,
       "lane": 4, "turn": 4, "junction": 4,
       "stop": 5, "lead_stop": 5,
       "hard_brake": 6}


@dataclass
class Page:
  """一頁：左邊圖示、上面大字、下面小字（小字空的就只有一行大字）。"""
  icon: str
  big: str
  small: str = ""
  color: tuple = (255, 255, 255)
  small_color: tuple | None = None
  arg: object = None          # 圖示參數（速限牌的數字）
  alert: bool = False         # 整面紅底閃爍


WHITE, GREEN, YELLOW, RED, CYAN = (255, 255, 255), (0, 235, 60), (255, 190, 0), (255, 20, 20), (0, 200, 255)
AMBER = (255, 150, 0)       # 方向燈色：準備左/右轉
GRAY_TXT = (200, 200, 200)


@dataclass
class Screen:
  key: str
  params: dict = field(default_factory=dict)

  @property
  def sev(self):
    return SEV[self.key]

  @property
  def pages(self):
    return pages(self)

  def page(self, shown):
    """顯示了 shown 秒時輪到哪一頁。"""
    ps = self.pages
    return ps[int(shown // PAGE_SECS) % len(ps)]


def _ahead_page(p):
  a = p.get("ahead")
  if not a:
    return []
  return [Page("limit", "前方速限", f"{round(a['dist'])} M 後變 {a['limit']}", WHITE, arg=a["limit"])]


def pages(s):
  k, p = s.key, s.params
  if k == "clock":
    return [Page("clock", "")]
  if k == "hard_brake":
    return [Page("none", "前車急煞", "請注意", WHITE, alert=True)]
  if k == "lead_stop":
    return [Page("car_red", "前車停止", "請注意 減速中", RED)]
  if k == "stop":
    return [Page("octagon", "前方停車", "減速中", RED),
            Page("octagon", f"還有 {max(0, round(p.get('dist', 0)))} M", "準備停車", RED)]
  if k in ("lane", "turn"):
    left = p.get("dir") == "left"
    side = "左" if left else "右"
    if k == "lane":
      return [Page("lane_l" if left else "lane_r", f"向{side}切換", "變換車道", CYAN)]
    return [Page("turn_l" if left else "turn_r", f"準備{side}轉", "請注意", AMBER)]
  if k == "curve":
    side = {"left": "左", "right": "右"}.get(p.get("dir", ""), "")
    icon = "curve_l" if p.get("dir") == "left" else "curve_r"
    if p.get("hold"):
      return [Page(icon, "彎道中", "定速通過", YELLOW)]
    return [Page(icon, f"前方{side}彎" if side else "前方彎道", "減速中", YELLOW)]
  if k == "junction":
    return [Page("chev_down", "前方路口", "減速中", YELLOW)]
  if k == "lead_slow":
    return [Page("chev_down", "前車減速", "請注意", YELLOW)]
  if k == "lower_max":
    return [Page("chev_down", "減速中", "調整車速", YELLOW)]
  if k == "slow_ahead":
    return [Page("chev_down", "前方有狀況", "減速觀察中", YELLOW)]
  if k == "decel":
    return [Page("chev_down", "減速中", "", YELLOW)]
  if k == "limit_changed":
    return [Page("limit", "速限變更", f"目前道路 {p.get('limit', 0)}", WHITE, arg=p.get("limit", 0))]
  if k == "lead_launch":
    return [Page("car", "前車起步", "即將跟上", GREEN)]
  if k == "catch_up":
    return [Page("chev_up", "跟上前車", f"前車時速 {p.get('lead_kph', 0)}", GREEN)]
  if k == "stopped_follow":
    d = p.get("dRel")
    return [Page("car_gray", "前車停止", f"保持距離 {round(d)} M" if d else "保持安全距離", WHITE)]
  if k == "stopped":
    return [Page("octagon", "停止中", f"請稍候 {p.get('secs', 0)} 秒", RED)]
  if k == "follow":
    return [Page("car", f"跟車 {round(p.get('dRel', 0))}M", f"前車時速 {p.get('lead_kph', 0)}", WHITE)]
  if k == "accel":
    return [Page("chev_up", "省油加速", "請稍候", GREEN)] + _ahead_page(p)
  if k == "cruise":
    return [Page("road", "平穩行駛", "請保持車距", WHITE)] + _ahead_page(p)
  raise KeyError(k)


def dash_kph(v):
  """真實車速（m/s）→ 儀表上會看到的 km/h。"""
  return round(DASH_K * v / KPH + DASH_B) if v > 0.3 else 0


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
    self._reset_drive()
    self.last_limit = None

  # ------------------------------------------------------------ 候選
  def _candidate(self, d, now):
    sd = _g(d, "selfdriveState", default={})
    if d.get("standby", True) or not sd.get("enabled", False) or "carState" not in d:
      self._reset_drive()
      return Screen("clock")

    cs, ctl, rs = d["carState"], d.get("control", {}), d.get("radarState", {})
    v = float(cs.get("vEgo", 0.0))
    kph = round(float(cs.get("vEgoCluster", v)) / KPH)
    cruise = float(cs.get("cruiseSpeed", 0.0) or 0.0)
    max_kph = round(cruise / KPH) if cruise > 0 else 0
    a = float(ctl.get("aTarget", 0.0))
    reason = ctl.get("reason", "") or ""
    gas, brake = cs.get("gasPressed", False), cs.get("brakePressed", False)
    follow = float(ctl.get("followDistance", 0.0) or 0.0)

    # --- 遲滯旗標
    self.decel = a < DECEL_IN or (self.decel and a < DECEL_OUT)
    self.coast = (not self.decel) and (a < COAST_IN or (self.coast and a < COAST_OUT))
    self.accel = gas or a > ACCEL_IN or (self.accel and a > ACCEL_OUT)

    # --- 前車：出現 LEAD_ON 秒才顯示、連續消失 LEAD_OFF 秒才收（實車每分鐘掉 2.6-2.9 次）。
    # 短暫掉失不讓「出現」計時歸零，掉失那一下沿用最後看到的距離與速度。
    present = bool(rs.get("leadStatus", False))
    if present:
      self.lead_lost_since = None
      self.lead_seen_since = self.lead_seen_since or now
      self.last_lead = (float(rs.get("dRel", 0.0)), float(rs.get("vLead", 0.0)), float(rs.get("vRel", 0.0)))
      if now - self.lead_seen_since >= LEAD_ON:
        self.lead_shown = True
    else:
      self.lead_lost_since = self.lead_lost_since or now
      if now - self.lead_lost_since >= LEAD_OFF:
        self.lead_shown = False
        self.lead_seen_since = None
    lead = self.last_lead if (present or self.lead_shown) else None
    d_rel, v_lead, v_rel = lead if lead else (None, 0.0, 0.0)
    ratio = d_rel / follow if (d_rel is not None and follow > 1.0) else None
    r = ratio if ratio is not None else 99.0     # 沒前車／沒跟車距離時當作很遠

    # 「有前車」= 前車在跟車距離內（使用者表上第一欄），進出分開做遲滯
    if self.lead_shown and ratio is not None:
      self.in_zone = r < ZONE_IN or (self.in_zone and r < ZONE_OUT)
    else:
      self.in_zone = False
    if self.in_zone:
      self.close = r < CLOSE_IN or v_rel < CLOSING_IN or \
        (self.close and (r < CLOSE_OUT or v_rel < CLOSING_OUT))
    else:
      self.close = False

    # --- 停住／起步
    if self.stopped_since is None:
      stopped = v < STOPPED_V and (cs.get("standstill", False) or v < 0.1)
    else:
      stopped = v < MOVING_V
    if stopped:
      if self.stopped_since is None:
        self.stopped_since = now
        near = present and d_rel is not None and d_rel < FOLLOW_STOP_GAP
        self.stop_kind = "follow" if near else "alone"
        self.stop_d_rel = d_rel if near else None
    else:
      self.stopped_since = None
    stop_ahead = float(_g(d, "modelV2", "stopAhead", default=0.0) or 0.0)

    # --- 速限
    lm = d.get("liveMapDataSP", {})
    limit = float(lm.get("speedLimit", 0.0) or 0.0)
    limit_kph = round(limit / KPH) if lm.get("speedLimitValid") and limit > 0 else 0
    if limit_kph:
      if self.last_limit and abs(limit_kph - self.last_limit) >= 5:
        self.limit_changed_until = now + LIMIT_CHANGED_HOLD
        self.limit_changed_to = limit_kph
      self.last_limit = limit_kph
    ahead = None
    a_lim, a_dist = float(lm.get("speedLimitAhead", 0.0) or 0.0), float(lm.get("speedLimitAheadDistance", 0.0) or 0.0)
    if lm.get("speedLimitAheadValid") and 0 < a_dist < LIMIT_AHEAD_MAX and abs(a_lim - limit) > 1.0:
      ahead = {"limit": round(a_lim / KPH), "dist": a_dist}

    # ============ 依嚴重度從高到低
    if now < self.alert_until or (a <= HARD_BRAKE and reason.startswith(("lead", "weaklead")) and not stopped):
      if now >= self.alert_until:
        self.alert_until = now + ALERT_HOLD
      return Screen("hard_brake")

    if not stopped:
      # 前車停了、我們還在走：前車在跟車距離內（跟車距離本身會隨接近速度拉長）且幾乎靜止。
      # 「還在走」就是外面這個 not stopped（有 STOPPED_V／MOVING_V 遲滯），不另設車速門檻 ——
      # 原本多一條 v > 2.0，慢慢靠近停住的前車時一掉到 7.2 km/h 以下就變「前車減速」：9/23-9/25 重放
      # 前車幾乎停住（前後 2 秒 vLead 中位 < 0.5）時 1035 格「前車減速」，94% 是這條（前車資料 100% 雷達）。
      if lead and r < ZONE_IN and v_lead < 1.0:
        return Screen("lead_stop", {"dRel": d_rel})
      # 路口減速：HUD 講「路口煞停」的同一個條件（plan reason = stoplight，路口交接／模型在決定減速），
      # 而且真的在減速。9/23-9/25 重放 81 次、減速中那 1762 格 LED 講的是「減速中」54%、「遵守速限中」16%、
      # 「省油加速」8%，從沒講到路口。打方向燈時留給下面的「準備左/右轉」（使用者 9/24 的表）。
      intent0 = d.get("intent", {})
      signalling = intent0.get("turn") in ("left", "right") or intent0.get("laneChangeState") == "laneChangeStarting"
      if reason == "stoplight" and (self.decel or self.coast) and not brake and not signalling:
        return Screen("junction")
      # 前方需停止（沒前車）：模型速度曲線停下來的位置。9/18-9/20 兩趟 37 段重放：
      # 觸發 21 次、20 秒內真的停住 16 次（76%）、中位提早 5.3 秒。
      # ⛔ 不要用 control.stopDistance（規劃沒停時回整段行駛距離）；control.stop 行進中一次都沒亮過。
      if not self.in_zone and stop_ahead > 0 and v > 1.0:
        return Screen("stop", {"dist": stop_ahead})

    intent = d.get("intent", {})
    if intent.get("laneChangeState") == "laneChangeStarting":
      return Screen("lane", {"dir": "left" if intent.get("laneChangeDirection") == "left" else "right"})
    if intent.get("turn") in ("left", "right"):
      return Screen("turn", {"dir": intent["turn"]})

    if stopped:
      waited = int(now - self.stopped_since)
      if self.stop_kind == "follow":
        moved = present and self.stop_d_rel is not None and d_rel is not None and d_rel - self.stop_d_rel > LEAD_LAUNCH_GAP
        if present and (v_lead > LEAD_LAUNCH_V or moved):
          return Screen("lead_launch")
        return Screen("stopped_follow", {"dRel": d_rel if present else None})
      return Screen("stopped", {"secs": waited})

    # 彎道（通用）：減速中講「前方左/右彎」，彎中不加不減講「彎道中」
    if reason == "curve" and not brake:
      if self.decel or self.coast:
        return Screen("curve", {"dir": self._curve_dir(d)})
      if not self.accel:
        return Screen("curve", {"dir": self._curve_dir(d), "hold": True})

    if self.in_zone:
      if self.close or (self.decel and reason.startswith("lead") and not brake):
        why = "decel" if not self.close else ("near" if r < CLOSE_OUT else "closing")
        return Screen("lead_slow", {"dRel": d_rel, "why": why})
      self.far = r > FAR_IN or (self.far and r > FAR_OUT)
      if self.far and self.accel and v_rel > PULLING_AWAY:
        return Screen("catch_up", {"lead_kph": dash_kph(v_lead)})
      return Screen("follow", {"dRel": d_rel, "lead_kph": dash_kph(v_lead)})
    self.far = False
    # 前車拉開到跟車距離外、我們在追
    if self.lead_shown and r < CATCH_UP_MAX and self.accel and v_rel >= 0:
      return Screen("catch_up", {"lead_kph": dash_kph(v_lead)})

    # ---- 沒前車（在跟車距離內）
    slowing = (self.decel or self.coast) and not brake
    if slowing and present and reason.startswith(("lead", "weaklead")):
      return Screen("slow_ahead")
    if slowing and max_kph and kph > max_kph + OVER_MAX:
      return Screen("lower_max")
    if self.decel and not brake:
      return Screen("decel")
    if now < self.limit_changed_until:
      return Screen("limit_changed", {"limit": self.limit_changed_to})
    if self.accel and max_kph and kph < max_kph - AT_MAX:
      return Screen("accel", {"ahead": ahead})
    return Screen("cruise", {"ahead": ahead})

  @staticmethod
  def _curve_dir(d):
    """模型路線在 30-60 m 處往哪邊偏。openpilot 座標 y 正 = 右（9/24 實測：9/23 整趟方向盤右打 >30 度的
    1142 格裡 1110 格 y 為正、左打的 448 格裡 365 格為負）—— 一開始寫反，LED 左彎講成右彎。"""
    path = _g(d, "modelV2", "path", default=None)
    if not path or len(path) < 7:
      return ""
    y = sum(path[4:7]) / 3
    if abs(y) < CURVE_SIDE_M:
      return ""
    return "right" if y > 0 else "left"

  def _reset_drive(self):
    self.decel = self.coast = self.accel = False
    self.lead_shown = self.in_zone = self.close = self.far = False
    self.lead_seen_since = self.lead_lost_since = None
    self.last_lead = None
    self.stopped_since = None
    self.stop_kind = None
    self.stop_d_rel = None
    self.alert_until = 0.0
    self.limit_changed_until = 0.0
    self.limit_changed_to = 0

  # ------------------------------------------------------------ 仲裁
  def update(self, d, now):
    """回傳 (Screen, 這個畫面已經顯示幾秒)。now 用單調時鐘（離線重放傳 log 的時間）。"""
    want = self._candidate(d, now)
    if want.key == self.cur.key:
      self.cur = want                                # 同一個畫面：只更新內容（距離、秒數、時速）
      self.pending_key = None
      return self.cur, now - self.cur_since
    if self.pending_key != want.key:                 # 新畫面要「連續」被選中才算數
      self.pending_key, self.pending_since = want.key, now
    held = now - self.pending_since
    shown = now - self.cur_since
    min_show = min(MAX_SHOW, max(MIN_SHOW, PAGE_SECS * len(self.cur.pages)))
    if want.key in URGENT or (self.cur.key, want.key) in FLOW:
      self._switch(want, now)
    elif want.sev > self.cur.sev:
      if held >= ENTER:
        self._switch(want, now)
    elif shown >= min_show and held >= DEBOUNCE:
      self._switch(want, now)
    return self.cur, now - self.cur_since

  def _switch(self, s, now):
    self.cur, self.cur_since = s, now
    self.pending_key = None
