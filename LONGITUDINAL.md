# 縱向控制鏈路盤點

**這份是「誰在決定油門煞車、他們怎麼互相干擾」的單一入口。**
盤點日期 2026-09-14，對應 HEAD `19cbfc1cc`。

要調縱向、要解釋某個行為、要加新機制之前 **先讀這份**，不要只查一條路徑就動手。
起因：2026-09-14 連續三次只查單一路徑就提結論，每次都漏掉別的機制在同時作用。

行號都對應本 commit，改檔後要更新。

---

## 1. 鏈路全圖

```
                      ┌──────────────── 三個候選者 ────────────────┐
 radarState ────────► │ MPC (acados)        → lead0 / lead1 / e2e │
 modelV2   ────────►  │ cruise (定速律)      → cruise             │
 v_cruise  ────────►  │ modelV2.action      → e2e（有條件參賽）    │
                      └───────────────────────────────────────────┘
                                        │
          修飾器（在候選算出前/後改它）  │
          curve_speed ──► a_cruise      │
          weak_lead   ──► a_cruise      │
          stop_for_lights ─► 餵 MPC 一個假障礙物 + a_min
                                        ▼
                            output = min(候選們)      planner:411
                                        │
          覆寫（min 之後，直接改結果）   │
          force stop 上限  max(out, −1.5)             planner:421-422
          junction.a_floor 下限（可到 −2.5）           planner:428-430
          standstill creep  min(out, 0)               planner:446-450
                                        ▼
                    clip(out, −3.5, +2.0)             planner:452
                                        ▼
                       longcontrol：PID or stopping   longcontrol.py
                                        ▼
                                     車輪
```

---

## 2. 三個候選者

| 候選 | 算在哪 | 它的目標 | 誰限制它 |
|---|---|---|---|
| **MPC**（lead0/lead1/e2e） | `long_mpc.py` acados 求解 | 與障礙物保持 `v²/2CB + t_follow·v + STOP_DISTANCE` | 只有成本（軟的）＋ `ACCEL_MIN/MAX` |
| **cruise** | `planner:210-240` `get_cruise_accel` | 追上設定速度 | `A_CRUISE_MAX` 曲線、橫向加速度預算、**jerk rate limit** |
| **e2e** | `modelV2.action.desiredAcceleration` | 模型自己說要多少 | 無（只在實驗模式或 junction 武裝時參賽，`planner:408`） |

**實測誰贏**（9/14 傍晚 20 分鐘市區，10997 幀，engaged/行進/未踩踏板）：
`lead0` 63.7%、`cruise` 32.2%、`e2e` 2.4%、`lead1` 1.7%。

### 2.1 MPC 的成本函數（`long_mpc.py:227-233`）

```python
costs = [((x_obstacle - x_ego) - desired_dist) / (v_ego + 10.),   # w = 3
         x_ego, v_ego, a_ego,                                      # w = 0, 0, 0
         a_ego - a_prev,                                           # w = 200
         j_ego]                                                    # w = 5
```

**距離誤差被 `/(v_ego + 10)` 壓縮。** 停車時（v≈0）：

- 車距少 1 m → `3·(1/10)² = 0.03` 分
- 加速度比上一幀多變 0.1 → `200·0.1² = 2.0` 分

→ **MPC 願意用 8.2 m 車距換「加速度不要多變 0.1 m/s²」**（`3(d/10)² = 200(0.1)²`）。
這是「停等太近」的根本原因，也是 `STOP_DISTANCE` 調了沒用的原因。

危險區約束（`long_mpc.py:241-245`，slack，`DANGER_ZONE_COST = 100`）：
實際間距 ≥ `0.75 × desired_dist`，同樣被 `/(v+10)` 除過 →
停在 4 m（侵入 1.25 m）只罰 1.56 分 ＝ 只值 0.088 的加速度變化。**安全底線也是可買賣的。**

### 2.2 cruise 的律（`planner:210-240`）

```python
comfort_accel = min(|Δv|, sqrt(2·0.16·|Δv|))          # 速度誤差的函數
target = clip(target, A_CRUISE_MIN, max_accel)         # max_accel = A_CRUISE_MAX 曲線
target = clip(target, prev ± j_cruise·dt)              # :237  ← 唯一的 rate limit
```

`J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]`（0/10/25/40 m/s）。
⚠ **這是全鏈路唯一對「加速度變化率」的硬限制**，而且只管 cruise 這一條。

---

## 3. 五個修飾器

| # | 誰 | 在哪 | 怎麼介入 | 觸發條件 |
|---|---|---|---|---|
| 1 | `curve_speed` | `planner:370-373` | `a_cruise = min(a_cruise, curve.a_target)` | 預測橫向加速度 > 1.3，速度 > 20 km/h |
| 2 | `weak_lead` | `planner:394-402` | `a_cruise = min(a_cruise, 0)` | 無雷達前車、模型前車機率 0.2–0.5、45–100 m |
| 3 | `stop_for_lights` | `planner:329-334` | 給 MPC 一個站著的假車 `stop_x`，並把 `a_min` 收到 **−1.5** | 模型的速度計畫掉到 2 m/s 以下且持續同意 |
| 4 | 橫向加速度預算 | `planner:214-217` | `max_accel = min(max_accel, √(a_total_max² − a_y²))` | 一直在算，轉彎時才咬 |
| 5 | `allow_throttle` | `planner:220-223` | 把 `max_accel` 收到滑行值 | 模型預測駕駛要踩油門的機率 < 0.4 |

⚠ 1、2、4、5 **只改 `a_cruise`**。MPC 與 e2e 完全不受它們限制 ——
所以「放寬加速天花板」只在 cruise 贏的那 32% 幀有效。

---

## 4. 仲裁（`planner:406-411`）

```python
candidates = [(MPC 的 a, mpc.source), (a_cruise, cruise)]
if 實驗模式 or junction.active: candidates.append((e2e 的 a, e2e))
output_a_target = min(candidates)          # 取最保守
```

**`min()` 沒有任何平滑。** 兩條特性完全不同的曲線（cruise 被 rate limit 焊死、
MPC 隨雷達雜訊浮動）在這裡硬接，接縫處沒有處理。

實測（9/14 兩趟對照，0–40 km/h 有前車）：

| | 舊 CB=2.0 | 新 CB=1.5 |
|---|---|---|
| 贏家換人 | 15.1 次/分 | **23.0 次/分** |
| \|jerk\| >2 佔比 | 2.2% | 2.8% |

換人那一幀的 |jerk| 中位 **0.55**，同一 source 連續時只有 **0.10**（差 5.5 倍）。
換人只佔 1.9% 的幀，卻貢獻 11–14% 的大 jerk 事件。

各 source 自己的 |jerk| 中位：`cruise` **0.03**（有 rate limit）、
`lead0` **0.20**、`e2e` **0.34**（>2 佔 15.4%）。

---

## 5. 三個覆寫（`min()` 之後）

| 誰 | 行 | 作用 | 方向 |
|---|---|---|---|
| force stop 的 `a_min` 回讀保護 | 421-422 | `output = max(output, −1.5)` | **上限**：不准煞太重 |
| `junction.a_floor` | 428-430 | `if a_floor < min(output,0): output = a_floor` | **下限**：至少要煞這麼多（可到 −2.5） |
| standstill creep | 446-450 | `output = min(output, 0)` | 停住後別蠕動 |

⚠ **前兩個方向相反，而且都掛在 `e2e` 這個 source 名下** ——
`stopLight` 在 HUD 上分不出是哪一個在作用（已知問題，memory 有紅線）。

### 5.1 `junction.a_floor` 已經是幾何式停止律（`junction_handoff.py:190-195`）

```python
want = -min(v_ego**2 / (2 * stop_x), MAX_FLOOR_DECEL)   # = 2.5
step = FLOOR_JERK * DT_MDL                              # 4.0 m/s³
a_floor = clip(want, a_floor - step, a_floor + step)    # 自己的 rate limit
```

**任何「要停在指定距離」的新機制都應該長這樣，而且這裡已經有一份實作。**
要加類似的東西（例如停在靜止前車後方固定距離）**先考慮擴充這個模組**，
不要再寫第二份幾何律——這正是 9/14 差點犯的錯。

---

## 6. 下游：`longcontrol.py`

```python
if state == stopping:
    output = last_output − 1.0·DT_CTRL   # 每秒加深 1 m/s²，直到 stopAccel = −2.0
    # ← a_target 在這個狀態被完全忽略
else:  # pid
    output = PID(a_target − aEgo, feedforward=a_target)
```

`should_stop = v_ego < 0.3 and a_target < 0.1`（`drive_helpers.py:17`）。

⚠ **stopping 只在 v < 1.1 km/h 才接管**，那時車距已經定了 —— 實測停住後只再前進 0.05 m。
所以它**不是**停等太近的原因（2026-09-14 一度誤判成是，已否決）。

---

## 7. 干擾點總表（動手前逐條檢查）

| # | 干擾 | 後果 |
|---|---|---|
| 1 | **jerk 限制只有 cruise 有** | MPC/e2e 的抖動直接穿透到車輪 |
| 2 | **`min()` 接縫沒有平滑** | 換贏家的那一幀 jerk 是平常的 5.5 倍 |
| 3 | **`COMFORT_BRAKE` 同時出現在三個地方** | 目標距離、`v_lead²/2CB`（雷達雜訊放大器）、HUD 的 followDistance。動它會同時改三件事 |
| 4 | **`A_CRUISE_MAX` 只管 cruise** | 調加速天花板，在跟車（63.7% 的幀）完全無效 |
| 5 | **轉彎有兩套判據** | `curve_speed` 用橫向加速度、`junction._turning` 用方向燈。同一個彎兩邊結論可以不同 |
| 6 | **force stop 與 junction a_floor 同名 `e2e`** | 一個設上限一個設下限，log 與 HUD 分不出誰在作用 |
| 7 | **`weak_lead` 把 `a_cruise` 壓到 0** | 加速過程中反覆閃現（9/14 某段佔 20.8% 幀），每次歸零 |
| 8 | **`STOP_DISTANCE` 是全速域常數** | 想靠它補停等距離，會等量加大所有速度的跟車距離 |

---

## 8. 已證實無效／已否決（FORBIDDEN 重走）

| 旋鈕 | 實測 | 日期 |
|---|---|---|
| `STOP_DISTANCE` 6.0 → 7.0 | 停等距離只動 0.1–0.3 m | 09-13 |
| `COMFORT_BRAKE` 2.0 → 1.5 | 停等距離**完全沒動**，換人 +52%、jerk 惡化 | 09-14 |
| 用定速律做紅綠燈停止 | 36/36 全部衝過停止線，中位超出 8.6 m | 09-05 |
| `longcontrol` 的 stopping 是停太近的原因 | 否決：只在 v<1.1 km/h 接管，停後只動 0.05 m | 09-14 |

---

## 9. 停等太近：目前的完整診斷

1. **不是算式錯**：目標距離確實是 `v²/2CB + 1.5v + 7`
2. **是那個距離在成本裡太便宜**（第 2.1 節：8.2 m 換 0.1 加速度變化）
3. **而且 26/32 次一進場就已經在目標之內**（中位少 6.5 m）——這時什麼目標值都補不回來
4. **兩個看似相關的旋鈕都已證實無效**（第 8 節）

→ 要解，需要一條**不參與成本競價的距離函數**（`−v²/2(d−D_MIN)` 形式），
而這條律 **`junction_handoff` 已經有一份實作**（第 5.1 節）。
離線量化（9/14，`E:/Temp/drive0914/analyze15.py`，D_MIN=5.0、clamp −3.0、
只在前車近靜止且車距 <30 m 時武裝）：29 次停車介入 16 次，
全部落在停太近那批（中位 3.99 m），本來停得好的 13 次一次沒碰，
全趟只壓住 0.69% 的幀，前車其實在動的誤判 0 幀。

⚠ 這是**開環估算**：算的是「規則會要求多重」，不是「車會停在哪」。
