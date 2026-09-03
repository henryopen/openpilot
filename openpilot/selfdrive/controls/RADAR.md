# Custin 雷達完整盤查

> **這份是雷達的單一真相來源。** 動雷達、跟車、靜止車、機車相關的東西之前先讀它。
> 每次都重新研究一遍是最貴的浪費——底下每個數字都是實測出來的，不要重驗，要推翻請附新數據。
> 縱向控制的全貌看隔壁 [`LONGITUDINAL.md`](LONGITUDINAL.md)。

---

## 1. 硬體與 CAN

| 項目 | 值 |
|---|---|
| 車型 | Hyundai Custin 2023，`HYUNDAI_CUSTIN_1ST_GEN`，一般 CAN |
| 雷達位址 | `0x238`–`0x255`（`CUSTIN_RADAR_ADDRS`），**33 Hz** |
| DBC 欄位 | `FLAG` / `LONG_DIST` / `LAT_DIST` / `V_ABS` / `SCORE` / `STATE` / `AZIMUTH` |
| **沒有的東西** | 車種、尺寸、RCS。**雷達分不出機車和汽車** |
| 本車速來源 | `WHL_SPD11`（動力總成匯流排）——雷達只給目標的**對地絕對速度** `V_ABS` |

解析在 `opendbc/car/hyundai/radar_interface.py`。手解 CAN 的欄位定義（沒有 opendbc 時可用）：
`LONG_DIST 5|10@0+ ×0.1`、`LAT_DIST 21|10@0- ×0.0475`、`V_ABS 37|11@0- ×0.02526 +2.587`、`SCORE 41|6@0+`

## 2. `0x238` 是什麼（最重要的一件事）

**`0x238` 不是「第一個 slot」，是原廠 ACC 當下鎖定的那個目標。它會換車。**

- 追蹤精度：與原廠 ACC 顯示的距離差 **0.10 m**，非常準
- 但**換人時不會通知**：2026-09-02 在兩台機車後方，它 9.0 → 5.5 m 一幀跳完，
  而 `REL_SPEED` 只讀到 0.8 km/h，同時另一個 track 就在 5.6 m
- 解析層本來就偵測得到（`CUSTIN_MAX_JUMP = 3.0`，註解寫著 "a different object took the slot"），
  會丟掉距離歷史重算，**但 2026-09-03 之前沒有換 `trackId`**
- 後果：radard 的 Kalman 速度估計、preferred-track 比對、MPC 的障礙物位置，
  全都以為是同一台車瞬移。車在機車後方一直「想走又停」

→ 2026-09-03 修正（opendbc `2779da09`）：slot 換人時發新 `trackId`。

## 3. 為什麼只放行一個 track

`CUSTIN_PRIMARY_ONLY = True`：`allPoints` 解全部（給 HUD、給旁車道觀察），
但 **radard 只拿得到 `0x238`**。

理由（已實測，不要輕易推翻）：其餘 track 是原始資料，**護欄被沿길掃描時會呈現
「等速保持在前方一個車道寬處的車」**，radard 會選它而不是真車，
量出來**比只用視覺更差**。門檻從 25% 收到 7% 才把誤選從 56% 壓到 4%。

其他過濾：`CUSTIN_MIN_RANGE 2.0`（保險桿雜訊）、`CUSTIN_MAX_ABS_Y 5.5`（本車道與兩側）、
`CUSTIN_MIN_SCORE 30`（31 是飽和）、`CUSTIN_MIN_HITS 8`（連續good幀才送出）。
**primary 不受 `MAX_ABS_Y` 限制**（那是車自己的選擇，不二次猜測）。

## 4. `vRel` 要讀 `V_ABS`，不要微分距離

距離微分在 30 m 外有 1.5 m 量測雜訊 → 0.36 秒窗口下等於好幾 m/s 的假速度。
實測（對照 ±1.5 s 置中擬合）：

| 方法 | 誤判「正在拉開」>2 m/s | p90 誤差 |
|---|---|---|
| 微分距離 | **5.1% 的幀** | 3.03 m/s |
| 讀 `V_ABS` | **0** | 1.26 m/s |

拉長窗口只是把誤差換成延遲（0.9 s：2.2% 拉開，但 8.5% 假性接近）。
→ `CustinSlot.solve()` 用 `V_ABS` 的中位數當 `vRel`，距離用短窗口平均。

## 5. radard 怎麼把雷達和視覺兜起來

```
modelV2.leadsV3[0]（視覺）           radarTracksSP（雷達，只有 primary 進 radard）
        │                                    │
        └──────── match_vision_to_track ─────┘
                   嚴格 gate：距離 7%/2m（或 same_object）、速度、
                   失配 → 對上次那個 track 用放寬 gate 再驗（preferred_track_id）
                        │
              有 → 用雷達的 dRel/vRel（radar=True）
              無 → 用視覺自己的估計（radar=False，vRel 由模型算）
                        │
                  radarState.leadOne → MPC
```

**`lead_prob > 0.5` 是總開關**：視覺沒把握時，雷達根本不會被拿出來看。
唯一例外 `potential_low_speed_lead`：`v_ego < 14.4 km/h` 且 `|yRel|<1.0`、`0.75<dRel<25`。

### 已驗證的兩次修正

| 日期 | 問題 | 修法 | 效果 |
|---|---|---|---|
| 2026-09-01 `4093f3cff` | gate 測的是視覺距離，而視覺距離在夜間最差 → 雷達在最需要時被丟掉 | 加 `same_object`（速度+橫向一致就算同一物） | 採用率 白天 80.1→84.8%、**夜間 66.7→91.6%** |
| 2026-09-03 `eade804b1` | gate 每幀重判，一幀雜訊就掉回視覺 | StarPilot 的 `preferred_track_id`：對上次那個 track 放寬再驗 | 來源切換 923→**175（−81%）**、雷達佔比 80.8→**88.4%**、最大跳幅 38.8→**27.6 km/h** |

## 6. 實測數據（2026-09-02，118 段 / 1.93 小時）

### 雷達與視覺誰在主導

| | |
|---|---|
| engaged 且有前車 | 63.0 分鐘 |
| 雷達確認 / 只有視覺 | **82.2% / 17.8%**（修正後 88.4%） |
| 兩者對同一台車的 `vRel` 落差 | 中位 **6.4 km/h**，最大 **45.8 km/h** |

### 靜止目標

| | |
|---|---|
| 雷達平均 track 數 | **2.6 個/幀**（不是 10 個，多數 slot 是空的） |
| 原廠 ACC 有鎖到目標的幀 | 53.5%（**另外 46.5% 連原廠都沒鎖到**） |
| primary 是對地靜止的 | 4.7%，持續 1 秒以上 **36 次** ← **原廠 ACC 確實會鎖靜止車** |
| 本車道有靜止目標的幀 | 5863（4.2%） |
| 　其中視覺也看到 | 68.6% |
| 　其中 radard 真的有 lead | **70.3%**（**30% 完全沒有 lead**） |

2026-08-29 的補充（30 段 57582 幀）：對地靜止目標中，**本車道**視覺抓到 77.5%，
漏掉的 22.5% 有一半是假目標 → **訊噪比約 1:1，不可直接拿來自動煞車**。
不分車道的「74.3% 漏掉」是嚇人但無用的數字——四分之三是路邊雜物（|y| 2–6 m）。

### 機車

- **雷達分不出機車**（沒有 RCS/尺寸欄位）
- 2026-09-02 14:41 實例（影片確認前方是兩台機車）：雷達**鎖得到**，
  但四個目標同時存在（4.4 / 4.7 / 6.4 / 11.5 m），`0x238` 在它們之間換人 → 走走停停
- 機車橫向散開快，`yRel` 在 −1.2 到 −3.1 m 之間游走
- → **要分辨機車只能靠 YOLO**（訓練集有 10006 個 motorcycle 框）

## 7. 還沒解決的

1. **`lead_prob > 0.5` 這道閘**：視覺沒把握時雷達完全失聲，而靜止車正是視覺最沒把握的情況。
   `carolpilot` 有現成做法可抄（`is_stopped_car_count` 累積 1 秒、`selected_count` 記憶、
   `lead.prob > 0.4` 的放寬分支），但**放行前必須先解決 1:1 的訊噪比**，
   否則會對著空路面煞車。
2. **多目標選擇**：radard 只拿到一個 track，「哪一台才是要跟的」沒有選擇餘地。
   放行更多 track 前要先解決護欄誤選（見第 3 節）。
3. **YOLO 融合**：類別資訊到位後，才有辦法回答「正前方那個是機車還是汽車」，
   再用雷達給它精確距離。這是三者各司其職的正解，也是目前的方向。

## 8. 別家怎麼做（動之前先看）

| fork | 位置 | 有什麼 |
|---|---|---|
| **StarPilot** | `E:/Documents/GitHub/StarPilot-StarPilot/selfdrive/controls/radard.py` | `preferred_track_id` 放寬再驗（已採用）、`lat_sane`、G90 專用濾除 |
| **carolpilot** | `E:/Documents/GitHub/carolpilot` | `stopped-car` 分支、`selected_count`、`is_stopped_car_count`、cut-in 用的第二組 gate |
| **open251021 / FrogPilot** | 同名目錄 | `potential_far_lead`（視覺沒 lead 時改用雷達）、`get_adjacent_lead`（旁車道） |
| open251021 / sunnypilot | — | `match_vision_to_track` 是上游原版（25% / 5.0 m） |

**Custin 特有、四家都沒有的**：`0x238` 會換車這件事。別家 Hyundai 用 `RADAR_START_ADDR`
那組固定 slot，每個 addr 就是一個目標，不會換人。

## 9. 分析工具（`openpilot/yolo/analysis/`）

| 腳本 | 回答 |
|---|---|
| `window.py --tracks` | 指定時段逐秒攤開，**加 `--tracks` 列出每個雷達目標** |
| `lead_source_audit_20260902.py` | 雷達/視覺切換頻率與代價 |
| `lead_stickiness_20260902.py` | 中斷後回來的是不是同一個 track |
| `verify_lead_hold.py --on-device` | 用真 `get_lead` 重放，比較改前後 |
| `verify_slot_swap.py` | 用真 `RadarInterface` 從 CAN 重解，驗證換人偵測 |
| `stationary_and_bikes_20260902.py` | 靜止目標的可見性 |
| `lead_range_jumps_20260902.py` | 距離變化與相對速矛不矛盾 |

**讀 rlog 用車機自己的 schema**：`F:/c4sunny/schema_hcop/`（含 `include/c++.capnp`）。
`radarTracksSP` 的欄位是 **`points`**（不是 `radarTracks`）；`leadOne` 是 **`present`**（不是 `status`）。

**驗證雷達解析層的改動只能在車機上跑**（要 opendbc 和 DBC），但腳本支援 `--on-device`，
**不要在車機上手改任何檔案**。
