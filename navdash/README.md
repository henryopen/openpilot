# NavDash（Android 導航 App）

> 只供研究程式碼。不附 APK、不附 Google 金鑰，要用的人需要自己申請金鑰、自己編譯。

手機上跑 Google 導航（Navigation SDK），把目前的導航步驟、距離、車道與路線透過區網提供給 Raspberry Pi（`openpilot/hud/pi/hud_agent.py` 每秒讀取），Pi 再轉給 comma four：

- HUD 顯示導航卡片；
- comma four 導航中以導航判斷打方向燈是轉彎還是換道。

## 自己編譯

1. **Google Cloud**：建立專案，啟用 **Navigation SDK**、**Maps SDK for Android**、**Places API (New)**。Navigation SDK 必須綁定付費帳戶，費用自行負擔，也須遵守 Google 的服務條款。
2. **建立 API 金鑰**：應用程式限制選「Android 應用程式」，填入套件名稱（預設 `com.navdashn`，要改就一起改程式）與你自己簽章的 SHA-1；API 限制只勾上面三個。
3. **填金鑰**：把 `local.properties.example` 複製成 `local.properties`，填入 `MAPS_API_KEY=你的金鑰`，以及 Android SDK 路徑 `sdk.dir`。`local.properties` 已在 `.gitignore` 裡，不要提交。
4. **編譯**：用 Android Studio 開這個資料夾，或執行 `./gradlew assembleDebug`。

## 對外連線

- 區網 HTTP `:5002`：`/health`、`/nav`、`/route`，給 Raspberry Pi 讀取。
- 說明頁（鍵盤與語音說明）從網站載入，連不到時用 App 內建的 `assets/help.html`。
