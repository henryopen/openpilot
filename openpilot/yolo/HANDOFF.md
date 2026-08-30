# 接續：2026-08-29 縱向調校 + YOLO，待分析兩趟實車資料

寫給下一個 session。當天改了 8 項上機，兩趟資料錄好了但**還沒分析**。

---

## 1. 待分析的資料（這是主要任務）

車機 `/data/media/0/realdata/`：

| route | 段數 | 開始（台灣時間） | 內容 |
|---|---|---|---|
| `00000008--35b5e794c4` | 19 | 08/29 11:01 | 早上，**改動前**的基準，已分析過 |
| `00000009--8dba586109` | 7 | 08/29 13:26 | 短程試車，資料少 |
| **`0000000a--a7a82c46f5`** | **51** | **08/29 14:04** | **下午，市區＋高速** |
| **`0000000b--a769669911`** | **57** | 08/29 晚上 | **晚上，市區＋高速** |

⚠️ `0b` 首段檔案時間顯示 `07/28 15:06`，那是**開機時系統時鐘還沒同步**造成的（見第 5 節），不是資料有問題。

開機記錄 `/data/yolo/bootlog/`：`boot_20260829_142624.json`（下午）、`boot_current.json`（晚上，可能未正常結束所以沒改名）。
YOLO 偵測記錄 `/data/yolo/runs/`（共 30 MB）。

**使用者的原始訴求：這兩趟有市區也有高速，要做更詳細的分析。** 早上那趟最高只到 55 km/h、完全沒有高速路段，所以很多結論當時無法驗證。

---

## 2. 車機現況（8 個 commit，全部已上機並重啟驗證過）

連線：`ssh -i C:\Users\User\.ssh\c4key comma@192.168.2.143`（辦公室 WiFi 才連得到，開出去就斷）。
跑東西要 `source /usr/local/venv/bin/activate` + `export PYTHONPATH=/data/openpilot`。

| commit | 改了什麼 | 現在的值 |
|---|---|---|
| `c559242` | 加速曲線改用 open251021 的 eco（18 km/h 以上） | 0→1.20、10→1.17、18→1.00、36→0.50、90→0.30 |
| `856d953` | 定速 deadzone + 路口轉彎下限 | `V_CRUISE_DEADZONE=0.25` m/s、轉彎 15–30 km/h |
| `77f7aa7` | 彎道降速的啟動門檻與下限分離 | `MIN_V=20`、`V_FLOOR=15` km/h |
| `97bccb5` | 彎道 lat-acc 門檻依速度放寬 | `_LAT_TOL_V=[1.5,1.25,1.0] @ [0,10,20] m/s` |
| `dabebc5` | 補回移植時弄丟的彎道速度下限 | `v_ego<=V_FLOOR` 就不再要求減速 |
| `d5c940d` | 路口轉彎意圖（master 原本沒有） | 打方向燈 + 15–30 km/h → `Desire.turnLeft/Right` |
| `3f10ec7` | 起步依設定速度縮放 | 設40→0.89（原1.20）、54 以上不變 |
| `98e17b3` | （前一天）Custin 雷達橫向欄位 | — |

jerk 沒有動：`J_CRUISE_BP=[0,10,25,40]`、`J_CRUISE_VALS=[1.6,1.2,0.8,0.6]`、`J_CRUISE_COMFORT=0.16`。
`c559242` 有一併把 jerk 的斷點從加速曲線獨立出來（否則加點後兩個陣列長度不符）。

---

## 3. 每一項要驗什麼（使用者最在意的三件事）

**a) 定速抖動** — 使用者當天回報「deadzone 可以」，但那是主觀感受，**沒有資料佐證**。
早上那趟的現象是：設 40 → 41 → 39、設 50 → 51 → 49，15 秒內穿越設定速度 6–8 次。
要驗證的是**高速段**（早上完全沒有）。

**b) 加速曲線（eco）** — `c559242` 是當天最後一項改動，**兩趟都是改完之後跑的**。
要看：18 km/h 以上是否確實變緩、**高速再加速會不會太慢**（90 km/h 上限只剩 0.30，我當時明講這是風險）。

**c) 路口轉彎** — 改動前實測：engaged 中駕駛踩油門補油 5 次，**5 次全是方向盤 >90° 的路口**，踩下時中位 14.7 km/h。
要看這兩趟同樣的統計有沒有下降。

還有 **d) 開機時間** — 使用者原本說「快 10 分鐘」，下午回報「開機速度正常」。
`boot_20260829_142624.json` 裡有完整分段，**modeld 首次載入的時間點終於錄到了**（offroad 不啟動 modeld，所以先前兩次都沒有）。當天已排除：yolod 只要 1.9 秒就緒、scons 無改動只要 12 秒。

---

## 4. 現成工具（都在 `openpilot/sunnypilot/yolo/`，車機上同路徑 `/data/yolo/`）

```bash
python /data/yolo/drive_report.py 0000000a--a7a82c46f5    # 主力：定速/加速/轉彎/接管/減速來源
python /data/yolo/verify_deadzone.py 0000000a--a7a82c46f5 # deadzone 對真實速度序列的影響
python /data/yolo/summarize.py                            # YOLO 偵測結果，按速度分段
python /data/yolo/accel_compare.py                        # 本機跑：加速曲線 vs open251021
```

`drive_report.py` 的區塊：定速追蹤、振盪波形、設定速度穩定性、追到設定速度的超調、加速表現、轉彎、摩擦圓餘裕、減速事件、接管前 8 秒、**engaged 中踩油門的時刻**（最有用的一個）。

跑一趟 50 段大約要幾分鐘，車機 CPU 慢但跑得動。

---

## 5. 踩過的坑（不要重蹈）

1. **時鐘**：開機後約 49 秒才同步，之前的時間戳會顯示 **Jul 28**（差 31 天）。
   swaglog 時間戳、yolod run 目錄名、route 檔案時間都受影響。
   要精確時序**只能用** `/proc/<pid>/stat` 第 22 欄或 rlog 的 `logMonoTime`（都是開機起算）。

2. **儀表速度 ≠ vEgo**：這台 `vEgoCluster` 比 `vEgo` **高 4.5 km/h**（法規不准儀表讀低）。
   使用者看的是 `vEgoCluster` vs `vCruiseCluster`。
   用 `vEgo` vs `vCruise` 分析定速抖動，整趟只找到 1 次超速；換成儀表值有 10 次。**一定要比對使用者看到的那組。**

3. **`_A_LAT_REG_MAX` 在 master 是死參數**：只餵 `v_target`，而 `v_target` 沒有任何地方讀取
   （planner 只用 `curve_speed.a_target` 去 clamp）。要調彎道行為得改 lat_acc 門檻。

4. **`A_CRUISE_MAX_BP` 曾被 jerk 共用**：已在 `c559242` 分開。之後要加點記得檢查兩個陣列長度。

5. **裝置 ffmpeg 不吃 `select=eq(n\,N)`**，抽特定幀要串流讀取跳幀。本機 ffmpeg 可以，但 raw hevc 無法 `-ss` seek，要用 `select=not(mod(n\,N))`。

6. **多層 ssh + heredoc**：commit message 用 heredoc 會被本機 shell 吃掉，改成寫檔案再 `git commit -F`。

7. **改 Python 後要重啟** `sudo systemctl restart comma`，並確認 `grep -c exc_info` 最新 swaglog = 0。
   裝置 commit 用 `git -c user.name=henryccy -c user.email=tyson.motoway@gmail.com commit --no-verify`。

---

## 6. 未完成 / 待決定

- **車道線型辨識**（`lane_ipm.py` + `lane_type.py`）：已驗證可行——本車道的雙黃線和白實線都判對，
  但**掃全路寬會誤判車道外的線**（對向車道被電線桿陰影切斷 → 誤判虛線）。
  下一步是**改用 modelV2 的 `laneLines` 定位**，只判本車道那兩條。本機 `F:\c4sunny\rlog_20260828\` 有 16 段錄影可離線做。
  使用者原話：「本來就是要靠 YOLO 的啊」——我當時用傳統視覺（成本幾 ms vs YOLO 570 ms），
  但標線磨損／雨天／施工的場景仍需要分割模型，那是獨立工程。

- **jerk 未調**。當天已列出對照：我們有三層（變化率上限、sqrt 塑形、MPC cost），
  open251021 只有 MPC cost 但**動態乘 1.8（加速中）/ 1.5（減速中）**。
  他的 open251021 是自己改過的版本（中文註解），1.8/1.5 是他設的，不是 FrogPilot 預設。
  要複製那個平順度是把 `A_CHANGE_COST` 200 → 360，但我們多了它沒有的兩層，不會等比例。

- **YOLO HUD 圖示**已上線（dash.html 畫在路面圖上，只傳公尺座標約 11 kbps）。
  距離用單目反投影，對雷達實測偏差 +3% / +11% / +36%（三段），帶 0.9 修正。

---

## 7. 開場建議

先跑 `drive_report.py` 對 `0000000a` 和 `0000000b` 兩趟，**特別看高速段**（早上那趟完全沒有），
再把兩趟跟 `00000008`（改動前）對照。使用者要的是「更詳細的分析」，不是再改一輪參數——
**先給數據，讓他決定調哪裡**。他一整天的模式都是這樣：他看數字、他決定、我改。
