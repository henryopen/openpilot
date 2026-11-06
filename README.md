# Custin openpilot（hcop）

> ## ⚠️ 只供研究程式碼，請勿直接安裝到車上
>
> 你現在看的分支：**`c4-pi`**（comma four ＋ Raspberry Pi）。要安裝到車上請改用 **`c4-only`** 分支。

這是一份以 [openpilot](https://github.com/commaai/openpilot) 官方 master（2026-08-21，`084747c75`）為基礎，為一台 **2026 Hyundai Custin**（台灣／中國才有的車型）搭配 **comma four** 自行修改的版本。
這個分支公開出來是讓有興趣的人參考程式的寫法，**不是可以直接安裝使用的產品**。

---

## 請先讀完這一段

1. **只供研究程式碼，請勿直接安裝。** 這份程式只在作者自己的一台車上開發與使用，沒有在其他車輛、其他車機或其他人身上測試過。
2. **沒有設定介面。** 所有參數都直接寫在程式裡，沒有任何設定畫面可以調整，包含跟車距離、加速與減速、過彎速度、轉彎與換道的判斷、停車距離、紅綠燈的反應等。要改只能自己改程式碼。
3. **駕駛習慣完全依照創作者個人的喜好調整。** 不一定適合你，也不一定符合一般人的駕駛習慣。
4. **安全相關參數與官方不同。** 轉向扭力上限、扭力變化速度、駕駛接手的門檻等，都是依作者這一台車實測調整的；opendbc 與 panda 安全層也有修改。其他車輛的方向機條件不同，可能造成方向機故障或非預期的轉向。
5. **車型辨識只收錄兩組 ECU 版本：** openpilot 官方原有的一組（前鏡頭版本日期 2022-09），和作者這台 2026 年台灣版（前鏡頭版本日期 2025-04）。雷達、方向機、前鏡頭的零件號相同，而且前鏡頭版本日期在這兩者之間的 Custin，通常會被自動辨識（模糊辨識）。其他 Custin 可能認不出車，或認出來但行為異常。認不出來時見下方「ECU 版本對不上時」；零件號不同代表硬體可能不同，請勿自行加入。
6. **風險自行負責。** 這是 Level 2 駕駛輔助的實驗性修改。任何使用、修改、安裝所造成的後果與風險，均由使用者自行負責，作者不負任何責任，也不提供安裝或使用上的協助（「ECU 版本對不上時」的回報除外）。

---

## 分支：依你有的硬體選

每個分支都是單一快照（沒有開發歷程），差別只在拿掉了哪些硬體的程式。**只有 `c4-only` 可以直接安裝**，其他分支只供研究程式碼。

| 分支 | 需要的硬體 |
|---|---|
| `full` | comma four ＋ comma 3（第二台，當紅綠燈辨識機）＋ Raspberry Pi（HUD 顯示器、藍牙鍵盤、NavDash 導航轉接）＋ 後窗 LED 看板（BX-Y04 控制卡＋ESP32-C3 中繼） |
| `no-led` | comma four ＋ comma 3 ＋ Raspberry Pi（拿掉後窗 LED 看板） |
| `c4-pi` | comma four ＋ Raspberry Pi（再拿掉 comma 3） |
| `c4-only` | 只有 comma four 或 comma 3X（再拿掉 Raspberry Pi）；**可以直接安裝** |

### 各分支有哪些功能

| 功能 | `full` | `no-led` | `c4-pi` | `c4-only` |
|---|:---:|:---:|:---:|:---:|
| Custin 車型支援、讀取原廠雷達 | ✅ | ✅ | ✅ | ✅ |
| 橫向（轉向）與縱向（加減速）調校 | ✅ | ✅ | ✅ | ✅ |
| 打方向燈判斷「轉彎」或「換道」 | ✅ 地圖路口＋路段記憶 | ✅ 地圖路口＋路段記憶 | ✅ 地圖路口＋路段記憶 | ⚠️ 20 km/h 以下＝轉彎＋路段記憶（沒有地圖） |
| 路口停車接手（模型規劃要停時自動接手，不必開實驗模式） | ✅ | ✅ | ✅ | ✅ |
| 彎道減速、停車距離、打燈轉彎提早減速 | ✅ | ✅ | ✅ | ✅ |
| 紅綠燈 | ✅ comma 3 辨識（追蹤、鎖定燈頭） | ✅ comma 3 辨識（追蹤、鎖定燈頭） | ⚠️ comma four 自己辨識（較舊的做法，判錯較多，要自行安裝） | ⚠️ 只能用 openpilot 原本的實驗模式 |
| 停等時轉綠燈自動起步 | ✅ | ✅ | ❌ | ❌ |
| comma 3 語音指令 | ✅ | ✅ | ❌ | ❌ |
| Raspberry Pi HUD 顯示 | ✅ | ✅ | ✅ | ❌ |
| 藍牙鍵盤 | ✅ | ✅ | ✅ | ❌ |
| NavDash 導航（導航中依導航判斷轉彎；App 要自行編譯） | ✅ | ✅ | ✅ | ❌ |
| TDX 即時路況顯示（要自己申請 TDX 金鑰） | ✅ HUD＋LED | ✅ HUD | ✅ HUD | ❌ |
| 後窗 LED 看板字幕 | ✅ | ❌ | ❌ | ❌ |
| 「臨時停車」「路況不熟」字幕（亮著時打燈不判轉彎也不換道） | ✅ | ❌ | ❌ | ❌ |

---

## ECU 版本對不上時

車機顯示「無法辨識車輛」時，openpilot 只會錄影、不會控制車子。把 ECU 版本傳給作者，作者確認後加進車型辨識。到這個 repo 的 Issues 開一則「Custin ECU 版本回報」，下面兩種方法擇一。

**方法一：comma connect（不用 SSH）**

1. 依 comma 官方說明，用手機在 connect.comma.ai 配對你的 comma four。
2. 發動車子開一小段（認不出車也會照常錄影），熄火後讓車機連上 WiFi 上傳紀錄。
3. 在 connect 把那趟行程設成公開，把行程名稱貼到 Issue。
   **公開的行程任何人都看得到影片與 GPS 路線**，建議開一段不在住家附近的短程；作者讀完後你就可以關掉公開。

**方法二：SSH**

1. 在設定的開發者選項開啟 SSH，填入你的 GitHub 帳號（官方做法）。
2. 發動過車子一次以後，連進車機執行：
   ```
   cd /data/openpilot && python3 tools/custin_fw_report.py
   ```
3. 把完整輸出貼到 Issue。輸出不含 VIN 與位置。

作者這邊：方法一用 openpilot 官方的 `tools/car_porting/auto_fingerprint.py` 讀行程，方法二把輸出貼進 `opendbc_repo/opendbc/car/hyundai/fingerprints.py` 的 `HYUNDAI_CUSTIN_1ST_GEN`。只加入零件號相同的版本，加完更新 `c4-only` 分支。

---

## 程式放在哪裡

| 跑在哪 | 位置 | 哪些分支有 |
|---|---|---|
| comma four | 這個 repo 本身（openpilot 加上修改）；紅綠燈服務在 `openpilot/yolo/tlwatchd.py` | 全部 |
| comma four（車型定義、安全層） | `opendbc_repo/`（直接放檔案，不是子模組） | 全部 |
| comma 3（不跑 openpilot，只當辨識機，用 USB-C 線跟 comma four 直連） | `openpilot/c3link/` | `full`、`no-led` |
| Raspberry Pi | `openpilot/hud/pi/`；comma four 上給 Pi 的 HUD 資料伺服器在 `openpilot/hud/` | `full`、`no-led`、`c4-pi` |
| ESP32-C3（後窗 LED 中繼韌體） | `openpilot/hud/pi/esp32_led_relay/` | `full` |
| Android 手機 | `navdash/`（NavDash 導航 App 原始碼） | `full`、`no-led`、`c4-pi` |
| 紅綠燈辨識模型 | `models/yolo/` | 全部（comma 3 的模型只在 `full`、`no-led`） |

Raspberry Pi 的程式裡，WiFi 名稱（例如 `hud_agent.py` 的 `KNOWN_WIFI`、`car_hotspot.py`）已改成範例值，要換成你自己的。

---

## 沒有附的東西

- **onnxruntime**：跑紅綠燈模型用，comma four 與 comma 3 都要另外安裝。
- **離線地圖資料**：OSM 速限與路口資料。
- **TDX 即時路況的 API 金鑰**：要自己到交通部 TDX 運輸資料流通服務申請。
- **NavDash 的 Google 金鑰與編譯好的 APK**：見下方。
- **comma 3、Raspberry Pi 的系統安裝與設定步驟。**

---

## 紅綠燈辨識模型

| 資料夾 | 給誰用 | 模型 | 放到 |
|---|---|---|---|
| `models/yolo/c4/` | comma four | `c4band1`（找燈）、`tlcls_v2`（判燈色） | comma four 的 `/data/yolo/`（有 comma 3 時只當備援） |
| `models/yolo/c3/` | comma 3 | `c3light_ft1n`（找燈）、`lightcls_64`（判燈色）、`lampcls_96`（判箭頭） | comma 3 的 `c3link/models/` |

- 模型只用作者車輛在台灣錄的影像訓練，其他地區、其他鏡頭位置可能判錯。
- **授權是 AGPL-3.0**（以 Ultralytics YOLO11 訓練），跟程式碼的 MIT 授權不同，詳見 `models/README.md`。

---

## NavDash（要自行編譯）

不附 APK，也不附 Google 金鑰。要用的人需要自己：

1. 到 Google Cloud 建立專案，啟用 **Navigation SDK**、**Maps SDK for Android**、**Places API (New)**。Navigation SDK 要綁定付費帳戶，費用自行負擔。
2. 建立 API 金鑰，限制為 Android 應用程式，填入自己的套件名稱與自己簽章的 SHA-1，API 只勾上面三個。
3. 把 `navdash/local.properties.example` 複製成 `navdash/local.properties`，填入 `MAPS_API_KEY`。
4. 用 Android Studio 編譯安裝。

詳見 `navdash/README.md`（`c4-only` 分支沒有 NavDash）。

---

## 授權

- 程式碼：沿用 openpilot 的 MIT 授權（見 `LICENSE`）。
- 紅綠燈辨識模型：AGPL-3.0（見 `models/README.md` 與 `models/LICENSE-AGPL-3.0.txt`）。
- NavDash 使用 Google Navigation SDK，須遵守 Google 的服務條款。
- openpilot 原始專案：https://github.com/commaai/openpilot
