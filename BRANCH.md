# hcop 分支盤點

**這份是「這個分支相對於上游改了什麼、合上游前要看什麼」的單一入口。**
盤點日期 2026-09-07。要合上游、或想知道某個行為是誰改的，先讀這份。

- 本機編輯點：`E:\Documents\GitHub\openpilot-master`，branch `hcop`
- remote：`mine` = `henryopen/openpilot`（我們的），`origin` = `commaai/openpilot`（上游）
- 部署流程見 memory 的 `c4-deploy-workflow`；**車機上 FORBIDDEN 手改檔案**

---

## 1. 規模

| | 主 repo | opendbc |
|---|---|---|
| 分支點 | `084747c75` | — |
| 我們的 commit | **161** | 19 |
| 改動 | 109 檔　**+14509 −73** | 12 檔　+401 −85 |
| 上游現在（2026-09-07） | **`c51e3e5a7`** cinque terre model | `3e92d112` VW MEB harness |
| 上游領先 | 58 commit、207 檔、+13630 −9362 | — |

**161 個 commit 總共只刪了 73 行。** 幾乎純新增、極少動 stock code —— 這是下面「零衝突」的真正原因，
也是這個分支要維持的紀律：**新功能寫成獨立模組，stock 檔只留掛載點。**

## 2. 哪些會開車，哪些不會

| 類別 | 規模 | 會開車 |
|---|---|---|
| `openpilot/yolo/analysis/` | 41 檔 +6178 | ❌ 離線分析腳本，不上車 |
| **`selfdrive/controls/`** | **14 檔 +2205 −32** | ✅ **核心** |
| `openpilot/hud/`（web + Pi） | ~10 檔 +2600 | ❌ 外接顯示器 |
| `openpilot/yolo/`（yolod 服務） | ~15 檔 +1800 | ❌ 獨立服務，不進控制迴路 |
| **`selfdrive/car/`** | **3 檔 +99 −12** | ✅ 車控 |

### 新增的獨立模組（不動 stock）

| 檔案 | 行數 | 做什麼 |
|---|---|---|
| `controls/lib/stop_for_lights.py` | +266 | 紅綠燈停車（**目前整條停用**，band 模型不再載入） |
| `controls/lib/curve_speed.py` | +171 | 彎道限速 |
| `controls/lib/junction_handoff.py` | +138 | 路口把縱向交給模型 + 減速地板 |
| `controls/lib/auto_lane_change.py` | +110 | 自動變道計時 |
| `controls/lib/relc.py` | +96 | 路緣偵測擋變道 |
| `selfdrive/mapd/` | — | 離線 OSM 速限（讀取層已上機，控制層未做） |
| `controls/LONGITUDINAL.md` | +587 | 縱向的單一真相來源 |
| `controls/RADAR.md` | +371 | 雷達的單一真相來源 |

### 改到的 stock 檔（共 12 個，只刪 49 行）

| 檔案 | 改動 | 掛了什麼 |
|---|---|---|
| `controls/lib/longitudinal_planner.py` | +200 −7 | 彎道、速限、weak lead、junction、停等不拱、eco 曲線 |
| `controls/radard.py` | +145 −8 | lead 匹配三道閘、LeadHold、停等 hold |
| `car/cruise.py` | +69 −5 | 按鍵行為、速限、set speed 語意 |
| `controls/lib/latcontrol_torque.py` | +44 −4 | 低速誤差放大、jerk 限幅、回正凍結積分 |
| `controls/lib/desire_helper.py` | +42 −6 | 變道條件、路緣、turn desire |
| `controls/lib/longitudinal_mpc_lib/long_mpc.py` | +27 −5 | `T_FOLLOW`、jerk factor |
| `car/card.py` | +18 −1 | — |
| `car/car_events.py` | +12 −6 | — |
| `controls/controlsd.py` | +7 −1 | — |
| `modeld/modeld.py` | +4 −1 | RELC 掛載（3 行） |
| `controls/plannerd.py` | +1 −1 | — |
| `selfdrived/selfdrived.py` | +0 −5 | 拿掉 resumeBlocked |

---

## 3. 合上游的狀態（2026-09-07 實測）

**`git merge-tree --write-tree origin/master hcop` → exit=0，零衝突。**

兩邊都改到的檔案只有 5 個，且行號全部不重疊：

| 檔案 | 我們改的行 | 上游改的行 | 重疊 |
|---|---|---|---|
| `cereal/log.capnp` | 2630（保留槽改名，`@107/@108` ID 沒動） | 725、1004 | ❌ |
| `common/params_keys.h` | 8–70 | 92、127 | ❌ |
| `modeld/modeld.py` | 24、305、418 | 413、427 | ❌（差 1 行） |
| `selfdrived/selfdrived.py` | 205–210 | 159–175 | ❌ |
| `pyproject.toml` | 112 | 21 | ❌ |

opendbc 兩邊的改動檔案**交集為空**。

### ⚠️ 但 git 說能合 ≠ 合了能跑

**真正的風險是上游換了駕駛模型（cinque terre），而 git 不會為此報任何衝突。**

我們這些調校**全部是照舊模型的輸出行為量出來的**：

| 依賴模型行為的東西 | 依賴什麼 |
|---|---|
| `junction_handoff` 的 arm 條件 | 「模型自己的規劃塌陷成停止」 |
| `junction_handoff.a_floor` | 量到「模型只煞到需求的三分之二」 |
| `curve_speed` 的門檻 | 模型看得到的彎道曲率 |
| `stop_for_lights` | 模型的 `shouldStop`（已停用） |
| turn desire 的 5–40 km/h 門檻 | 模型對轉彎意圖的反應 |

**換模型後這些數字可能整組失效，症狀是行為變差而不是報錯。**

上游同批還把 `UsbGpu*` 全面改名成 `Chestnut*`（params、`modelV2.big`、`selfdrived`）。
我們沒有自己引用 `usbgpu`，那些出現全是 stock code，合併時會一起改掉，**語意風險低**。

### 建議的合併順序

1. 先實車驗完手上未驗證的改動（見 memory `MEMORY.md` 的上路清單）
2. 開一個分支合上游，**不要在 `hcop` 上直接合**
3. 在那個分支上重新量模型行為：junction handoff 觸發率、減速地板是否還適用、turn desire 命中率
4. 兩者混在一起就分不清行為變化是誰造成的 —— 這是不建議現在合的唯一理由

---

## 4. 已知的內部設計缺口

盤點時實測出來的，都**不是 bug，是要知道的邊界**：

### 4.1 `radard` 的 7% 窗會繞過後加的閘門

```python
same_object = 速度<1.5 and 橫向<2.0
same_object = same_object and 距離分歧 < MAX_SAME_OBJECT_DIST_GAP     # 50 m
same_object = same_object and 時距 > MIN_SAME_OBJECT_HEADWAY          # 0.6 s
dist_sane = abs(dRel - vision) < max(vision*0.07, 2.0) or same_object  # ← 這個 or
```

`or same_object` 讓 7% 窗成為繞道。實測（9/6，58830 個採用雷達的幀）：

| 閘門 | 想擋 | 被 7% 窗放行 | 真正擋到 |
|---|---|---|---|
| 時距 0.6 s | 311 | **7（2.3%）** | 304 |
| 距離分歧 50 m | 472 | **0** | 472 |

距離分歧不可能被繞過（要 `vision > 714 m` 才成立）。時距那 7 幀是車速 16.2 km/h、
雷達 2.48 m、視覺 3.57 m、分歧僅 1.09 m —— **兩個感測器一致說有近車，繞過是對的**。
**結論：不用修。** 但之後再往 `same_object` 加閘門時，要記得它不是最後一道。

### 4.2 縱向有 6 層加速度壓制，全部是 `min()`

`longitudinal_planner.py` 依序：

| 行 | 做什麼 |
|---|---|
| 254 | `a_cruise = get_cruise_accel(...)` 巡航基礎 |
| 263 | `min(a_cruise, curve_speed.a_target)` 彎道 |
| 292 | `min(a_cruise, 0.)` weak lead |
| 301 | `min(candidates)` mpc / cruise / e2e 三選最保守 |
| 308 | `output_a_target = junction.a_floor` 路口減速地板（**覆寫，不是 min**） |
| 329 | `min(output_a_target, 0.0)` 停等不拱 |

全部只會更保守，**不會互相打架導致亂加速**，但會互相遮蔽 —— `plan_reason` 只報一個來源，
所以歸因時不能只看它。308 是唯一的覆寫，且在 329 之前，順序正確。

### 4.3 兩個參數上的已知落差

- **`STOP_DISTANCE = 6.0` 但實際停在 3.6–4.9 m**（40 次停車實測），系統性短 1.5–2 m，原因未解。
  要「實際停 5 m」得把參數往**上**調不是往下。
- **`T_FOLLOW` relaxed 1.5 / standard 1.45**，兩檔差 0.05 秒實質相同（`get_jerk_factor` 對兩者都回 1.0）。

---

## 5. 161 個 commit 分類

完整清單：`git log --reverse --oneline origin/master..hcop`。以下按主題分組。

### 5.1 車輛基礎與按鍵（Custin C4 支援）

```
87d262305 hcop: my Custin C4 defaults and my own opendbc fork
bf9e3e0a2 hcop: pull in the Custin fingerprints
673eb0e6c opendbc: pull in the Custin torque tune
3d04ccc1a tools: a CAN capture helper for finding button bits, and the Custin radar
c4e5fb817 opendbc: turn the Custin's radar on
28a206dd2 opendbc: take the Custin radar flag back off
78bd71d09 opendbc: main turns off again with openpilot longitudinal
4c1812598 opendbc: pull in the cancel button fix
89662e6c9 set speed: a short press jumps by 10, a long press trims by 1
621404b06 set speed: set picks the next ten, the other buttons resume
bd0bfe138 long: the middle button toggles openpilot, and resume can start a drive
72076a3ad long: set speed is what the dash reads, not true wheel speed
beb96da82 speeds these work above, in the units this car is driven in
e04f4501d params: experimental mode off by default
cf24c1aa3 monitoring: let a param turn driver monitoring off
5a9164a56 params: stop re-reading settings at the frame rate
d1abeb3c7 long: Params has no put_nonblocking on master
450ce6f54 drop the rlog scratch scripts that slipped into the tree
```

### 5.2 橫向控制

```
2fca6586b lateral: always on lateral behind a param
acdc89e0a lane change: the blinker is enough once the blind spot has been clear a second
d71c05ead lane change: a road edge that close is no better than a car in the blind spot
d5c940da8 lat: tell the model we are turning at a junction, not changing lanes
e8f4ce746 controls: move the turn/lane-change line to 40 km/h
c1194eac5 lagd: let the estimator run on the roads this car is actually driven on
8b090e61c lateral: scale the error back up at low speed, where acceleration hides it
```

### 5.3 縱向 — 巡航與加速曲線

```
beaea377f taper the cruise accel as the set speed is approached
3f10ec7d0 long: ease off the launch when the set speed is low
856d9536b long: stop chasing the last km/h of the set speed
c55924266 long: take open251021's eco acceleration curve from 18 km/h up
055bfb596 long: taper the cruise ceiling through 36 to 72 km/h instead of holding 0.5
06ee20925 longitudinal: relaxed follows at 1.5 s
```

### 5.4 縱向 — 彎道限速

```
9965cc761 long: ease off for a curve the model can see coming
dabebc500 long: restore the speed floor the curve limiter lost in porting
97bccb507 long: scale the curve limiter's thresholds by speed
77f7aa7c1 long: separate the curve limiter's entry threshold from its floor
```

### 5.5 縱向 — 速限（OSM / mapd）

```
bf181fff9 mapd: offline OSM speed limits
c2c182a7c cruise: follow the posted speed limit
d7e3ea091 cruise: the speed limit rules this car already ran on
4467286ca params: default the speed limit to assist at plus ten
12fae3e17 yolo: feed mapd a real GPS track, to see whether it can name the next limit
5388d2505 yolo: tally which roads actually carry a speed limit
9121dd7b9 long: the speed limit chain works; what is missing is map data, and a reason on screen
```

### 5.6 縱向 — 紅綠燈停車 → 路口交接

前半是 `stop_for_lights`（**現已整條停用**），後半改走 junction handoff。

```
e5764c338 long: brake for the light the model can already see, without experimental mode
5009d37f3 long: hold the stop, and let the model release it
cbd36f99d long: the go signal flickers at a halt, so wait for it to mean it
a36192eac long: capnp lists index, they do not slice
43b10e49e stop_for_lights: brake by what stopping there needs, and start seeing it sooner
6db428557 stop_for_lights: let go when the car in front does, and do not sit on the pull away
998ce248f stop_for_lights: stand down when there is a car to follow
229deeb16 stop_for_lights: act on what the model asks for, not on a flag it never sets
64e494f4e stop_for_lights: answer a hint with a hint, and only commit once the stop holds still
70c0044e7 stop_for_lights: drop the lead release, which no longer holds and could not be trusted
ca1a1ffb5 stop_for_lights: read the lead's presence by the name this tree uses
9730aadb0 long: stop for a junction the way we stop behind a car
5009be259 long: arm on the plan coming to rest, and ask for the point only when it matters
0b75a2d5a long: believe the plan over half a second, and only the car at the line
fc55382a5 long: arming on a stopped target is right in principle and has no signal to fire on
7fa892f23 long: why the light detector cannot be wired in, and the cheap way out
8481b569f long: retrain the light detector on the band, and say what to judge it by
3c7447f34 long: the band model sees three times the red lights and still cannot brake for them
cb9538cfc long: let the model take the junction, and stop hiding the turn desire
f867390d2 junction: let a stopped lead lift the veto, not arm the handoff by itself
c1b7f8525 junction: let a lead veto the handoff, with nothing to argue about
6efbcf8a1 junction: arm on the plan stopping, not the path ending, and hold it to the distance
```

### 5.7 雷達與 lead 選擇

```
aad53f3ca radard: tighten how far a radar track may sit from the vision lead
16a7f8b17 opendbc: pick up the corrected radar azimuth scale
98e17b3f4 opendbc: read the Custin radar's lateral field as metres
6da0f75b6 opendbc: take the radar's relative speed from the track, not the range
66b08b292 publish every radar track, so the display stops decoding its own
4093f3cff radard: keep the radar when vision only disagrees about how far away it is
34985ad0c radard: hold the radar track through a blink in the vision match
eade804b1 radard: prefer the track we were already following, the way StarPilot does
142e15892 yolo: read the radar's own CAN, and price the lead gate
a43000afe long: pick up the radar keeping the ACC's target at close range
41d40270e yolo: audit the whole radar decode, not one gate at a time
0881c1270 radar: what the ten slots actually contain, and whether we are wasting it
f5c255278 radar: measure the far-range slots at the match, not by inference
d2656734f radar: re-verify every field we claim, and find the nine slots are not tracks
112e65607 radar: say when each field was settled, so 6.4 stops inviting a re-run
2a4775798 radar: finish reading the messages we never decoded, and stop there
4cb9c4cea long: keep the lead through a blink, and do not speed up at a lead we half see
dc21a06fa radard: a car and its match have to agree on where they are, not just how fast
59cfce5af radard: a range this short at this speed is not a car
f79295b9c longitudinal: stopped behind a stopped car, stay stopped
```

### 5.8 HUD（Pi 外接顯示器 + web）

```
1ef08bf00 hud: the external display's data stream, on this branch's own data
d30747c9d hud: the lead panel, on this branch's field names
8daf78183 hud: show the cars beside us, not just the one we follow
617501514 hud: the speed limit signs the page was already drawing
0cb1cc0a8 hud: the road edge you cannot change lanes into
da33b5f5e hud: the stop the model is already planning for
0d65843aa hud: read the road edge block off the controller that decides it
e331037db hud: the standby screen says where this Pi is
077f856b3 hud: the radar's other targets, and a display that holds still
c9399c4e0 hud: a plan view for the radar targets
369c39035 hud: the radar's lateral field is a distance, not an angle
52443582c hud: put the radar's azimuth back, it was right all along
c35bf6c99 hud: one decode of the radar, not two
69bc65cce hud: a reconnect button on the standby screen
600d44f8a hud: say which AP the standby screen is on
02a93c44c hud: switch APs from the standby screen
b4ffb8931 hud: read the set speed from vCruise
7d76d8977 hud: walk between the live view and the standby screen
b827c6f9c hud: say why the car is braking, not which slot did it
dddb826cf hud: the cluster says ON before openpilot is actually on
cda2b6310 hud: playback goes through the same smoothing as live, and says when it was
d36df101e hud: hold a side car in its lane as it closes in
115592431 hud: the stop sign shows up while there is still road to stop in
4fb63f6b0 hud: walk the next-limit distance down between mapd updates
f894cb21d long: publish the plan reason as a message, not a param
8091a71c3 hud: match the reason enum by value
8366b1f3f hud: bring the display that is actually running into the tree
0d2db7d83 hud: add the Pi's own scripts, and say which half runs where
fb4e72535 hud: replay a segment so the display can be checked without driving
3e4a8c688 hud: say why there is no speed limit, who moved the set speed, and how far the stop is
233235ef3 hud: start the car's hotspot by itself when there is no known wifi
d0729d24e hud: let the Pi retry the wifi itself instead of waiting out NetworkManager
19b5154db hud: show the set speed cruise is really working to, not just the one that was set
1f2c243f1 hud: put the next speed limit beside the current one, not under it
46a55c9e3 hud: say when the model has the junction, not only when it wins
31dc6ec32 hud: give the handoff its own banner, and make the speed limits readable
aa373e5d0 hud: use openpilot's own experimental icon instead of a line of text
aec5e3ba5 hud: draw what kind of vehicle it is, and the markings beside it
9e71010a4 hud: use drawn vehicle shapes instead of stacked rectangles
065db6e20 hud: draw the people, not a trapezoid with a head on it
0c2306a5b hud: render the traffic instead of drawing it
a43fc7987 hud: show the corner's target on the set speed, which was reading MAX always
75d5db77f hud: stop one vehicle from becoming several on the overlay
a57601de5 hud: face the lorry the way it is seen, square on the back
a6f414cab hud: ask the detector for its results four times a second, not once every 700 ms
eda43d793 hud: let the radar say where, and the detector only say what
```

### 5.9 YOLO 偵測服務（車機上獨立跑）

```
8aef877d4 yolo: bring the detector's source into the tree
91794f848 yolo: run the car's own models, one for the road and one for the lights
f8b8d26f2 yolo: show a light when it is seen, not when it has been seen twice
c6fecf24d yolo: stand it down - it earns nothing the car acts on
9c7e5c642 yolo: put the lane markings on the road, where modelV2 has only geometry
b12e9ff0b yolo: take cv2 off the device, which does not have it
09cd2a53a yolo: read the markings every third road frame, not every one
c4ba82524 yolo: stop reading traffic lights, and size the road by what things really are
3c6c6529e yolo: write down what a traffic light is worth on each camera
bbd08fe4d yolo: give the road model a third thread, which is where it is fastest
```

### 5.10 文件與離線分析（不上車）

```
1537ae52d long: write down what the longitudinal chain actually does, and the tools that read it
8b3593b72 radar: write down everything measured about this radar, so it stops being re-derived
f38955641 long: write down what the junction stop is measured to do now
09ea09d64 long: record what moving the distance test to the commitment measured
0d85e350f long: bring the plan up to date, including the rule the car disproved
15478b481 long: bring the plan's progress up to date
e16845b07 long: settle what the radar can still do for longitudinal, and write the plan
f7a4a5584 docs: what the radar was really saying, and why the commitment never came
bea456adc lint: leave the offline analysis scripts out of the car's rules
e4ea71779 hud: set comprehension, per ruff C401
8dee509b1 yolo: fix the f-strings my heredoc broke
f7a7f05bf yolo: fix the replay harness after the preferred-track change
aeb762f85 yolo: let the lead replay run on the car without being edited there
fdc624ce4 yolo: give the slot-swap check a baseline to compare against
ed49496e0 yolo: measure the stop law, not the trigger
8acaa6e30 yolo: count what the two gates cost when they are wrong
fa4c53685 yolo: list the moments the driver had to step in
01d63dbcf yolo: score the junction stop on junction stops, and in seconds
05586223b yolo: simulate a collision warning on the drives we have, before writing one
599c47cba yolo: measure whether a tracker would help following, and what actually breaks it
29ce788b0 yolo: exercise the new lead hold and cruise cap against the car's own capnp
```

---

## 6. 盤點怎麼重做

```bash
cd /e/Documents/GitHub/openpilot-master
git fetch origin master --depth=200

# 規模
git log --oneline origin/master..hcop | wc -l
git diff --stat origin/master...hcop | tail -1

# 兩邊都改到的檔案 = 潛在衝突點
comm -12 <(git diff --name-only $(git merge-base origin/master hcop)..origin/master | sort) \
         <(git diff --name-only origin/master...hcop | sort)

# 試合併，不動工作區
git merge-tree --write-tree --messages origin/master hcop
```

**`merge-tree` 乾淨只代表文字層面能合。** 上游動到模型、感測器語意或參數命名時，
真正該做的是重新量行為 —— 見第 3 節。
