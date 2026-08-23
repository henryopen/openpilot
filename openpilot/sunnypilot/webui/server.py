#!/usr/bin/env python3
"""sunnypilot local settings web UI (standalone, stdlib only).

Lives in /data/sp_webui/ (outside /data/openpilot) so openpilot updates and
branch switches never touch it. Renders the device's own settings_ui.json.
LAN-only by trust: binds 0.0.0.0, no auth (user's decision, home/office LAN).
"""
import json
import os
import re
import subprocess
import threading
import time
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

# capability context for THIS device (Hyundai Custin). Static parts here;
# has_icbm / has_longitudinal_control are refreshed from live params in build_model
# so the OP-long toggle appears once ICBM is turned off (they are mutually exclusive).
CAPABILITIES = {
  "has_longitudinal_control": False,
  "has_icbm": True,
  "icbm_available": True,
  "alpha_long_available": True,   # Custin: CarParams.alphaLongitudinalAvailable == True
  "brand": "hyundai",
  "is_sp_release": True,
}


VENV_PY = os.environ.get("SPWEB_VENV_PY", "/usr/local/venv/bin/python3")
OPENPILOT_DIR = os.environ.get("SPWEB_OPENPILOT", "/data/openpilot")
CAPS_TTL = 30.0
_caps_cache = {"t": -1e9, "data": None}


def load_device_capabilities():
  """Ask sunnypilot's own capability generator, so every field settings_ui.json can
  test (steer_control_type, torque_allowed, pcm_cruise, enable_bsm, device_type, ...)
  is present. Hardcoding a subset silently hid whole sections such as NNLC."""
  now = time.monotonic()
  if _caps_cache["data"] is not None and now - _caps_cache["t"] < CAPS_TTL:
    return _caps_cache["data"]
  code = ("from openpilot.sunnypilot.sunnylink.capabilities import generate_capabilities_json;"
          "print(generate_capabilities_json())")
  try:
    env = dict(os.environ, PYTHONPATH=OPENPILOT_DIR)
    out = subprocess.run([VENV_PY, "-c", code], cwd=OPENPILOT_DIR, env=env,
                         capture_output=True, text=True, timeout=20)
    caps = json.loads(out.stdout.strip().splitlines()[-1])
    _caps_cache.update(t=now, data=caps)
    return caps
  except Exception:
    return _caps_cache["data"]


def refresh_capabilities():
  caps = load_device_capabilities()
  if caps:
    CAPABILITIES.update(caps)
  CAPABILITIES["has_icbm"] = as_bool(read_param("IntelligentCruiseButtonManagement"))
  CAPABILITIES["has_longitudinal_control"] = as_bool(read_param("AlphaLongitudinalEnabled"))

OSM_KEYS = {"OsmLocal", "OsmLocationName", "OsmLocationTitle", "OsmStateName",
            "OsmStateTitle", "OsmDbUpdatesCheck", "OsmDownloadedDate"}
EXTRA_KEYS = {"IsMetric", "DisableUpdates", "DisableDriverMonitoring"}  # writable besides settings_ui items

# Traditional Chinese labels for the settings_ui.json strings (Taiwan usage).
# Anything not listed falls back to the original English.
TRANSLATIONS = {
  # panels
  "Steering": "轉向", "Cruise": "定速", "Display": "顯示", "Visuals": "畫面",
  "Toggles": "開關", "Device": "裝置", "Software": "軟體", "Developer": "開發者", "Models": "模型",
  # section / sub-panel titles
  "Modular Assistive Driving System (MADS)": "模組化輔助駕駛系統 (MADS)", "MADS Settings": "MADS 設定",
  "Lane Change": "變換車道", "Blinker Control": "方向燈控制", "Lateral Control": "橫向控制",
  "Neural Network Lateral Control (NNLC)": "神經網路橫向控制 (NNLC)", "Torque Settings": "扭力設定",
  "Speed Limits": "速限", "Speed Limit Settings": "速限設定", "Smart Cruise Control": "智慧定速",
  "Custom ACC Speed Intervals": "自訂定速增量", "Custom ACC Speed Intervals Settings": "自訂定速增量設定",
  "Model Behavior": "模型行為", "Self-Tune": "自動校調", "Advanced Settings": "進階設定",
  "HUD Elements": "抬頭顯示元素", "Alerts & Extras": "提醒與附加", "General": "一般",
  "Brightness & Timeout": "亮度與逾時", "Connectivity": "連線", "Recording": "錄影",
  "Updates": "更新", "Developer UI": "開發者介面", "Camera": "鏡頭", "Test Maneuvers": "測試動作",
  "Speed Limit Source": "速限來源",
  # item titles
  "Enable Modular Assistive Driving System (MADS)": "啟用模組化輔助駕駛系統 (MADS)",
  "Toggle with Main Cruise": "用主定速鍵切換", "Unified Engagement Mode (UEM)": "統一啟用模式 (UEM)",
  "Steering Mode on Brake Pedal": "踩煞車時的轉向模式",
  "Auto Lane Change by Blinker": "打燈自動變換車道", "Auto Lane Change: Delay with Blind Spot": "自動變換車道：盲點時延遲",
  "Pause Lateral Control with Blinker": "打燈時暫停橫向控制", "Wake Up Behavior": "喚醒行為",
  "Enforce Torque Lateral Control": "強制扭力橫向控制", "Neural Network Lateral Control (NNLC)": "神經網路橫向控制 (NNLC)",
  "Manual Real-Time Tuning": "手動即時校調", "Enable Custom Tuning": "啟用自訂校調",
  "Torque Control": "扭力控制", "Torque Control Tune Version": "扭力控制校調版本",
  "Lateral Acceleration Factor": "橫向加速度係數", "Friction": "摩擦係數",
  "Less Restrict Settings for Self-Tune (Beta)": "自動校調較寬鬆設定 (Beta)",
  "Intelligent Cruise Button Management (ICBM) (Alpha)": "智慧定速按鈕管理 (ICBM) (Alpha)",
  "sunnypilot Longitudinal Control (Alpha)": "sunnypilot 縱向控制 (Alpha)",
  "Dynamic Experimental Control": "動態實驗控制", "Smart Cruise Control": "智慧定速",
  "Vision": "視覺", "Map": "圖資",
  "Enable Custom ACC Speed Intervals": "啟用自訂定速增量", "Short Press Increment": "短按增量",
  "Long Press Increment": "長按增量", "Speed Limit Assist Mode": "速限輔助模式",
  "Speed Limit Offset Type": "速限偏移方式", "Speed Limit Offset Value": "速限偏移量",
  "Speed Limit Source": "速限來源", "Driving Personality": "駕駛風格",
  "Adjust Camera Offset": "調整鏡頭偏移", "Adjust Lane Turn Speed": "調整路口轉彎速度",
  "Use Lane Turn Desires": "使用路口轉彎意圖", "Adjust Software Delay": "調整軟體延遲",
  "Live Learning Steer Delay": "即時學習轉向延遲",
  "Display Metrics Below Chevron": "在跟車箭頭下顯示數據", "Display Road Name": "顯示道路名稱",
  "Display Turn Signals": "顯示方向燈", "Show Blind Spot Warnings": "顯示盲點警告",
  "Steering Arc": "轉向弧線", "Real-time Acceleration Bar": "即時加速條",
  "Speedometer: Always Display True Speed": "時速表：一律顯示真實車速",
  "Speedometer: Hide from Onroad Screen": "時速表：行車畫面隱藏",
  "Standstill Timer": "停等計時", "Green Traffic Light Alert (Beta)": "綠燈提醒 (Beta)",
  "Lead Departure Alert (Beta)": "前車起步提醒 (Beta)", "Tesla Rainbow Mode": "Tesla 彩虹模式",
  "Quiet Mode": "安靜模式", "Show Advanced Controls": "顯示進階控制",
  "Always-On Driver Monitoring": "常時駕駛監控", "Use Metric System": "使用公制單位",
  "Language": "語言", "Enable sunnypilot": "啟用 sunnypilot", "Enable SSH": "啟用 SSH",
  "Enable ADB": "啟用 ADB", "Enable Lane Departure Warnings": "啟用車道偏離警示",
  "Disengage Cruise on Accelerator Pedal": "踩油門取消定速",
  "Record and Upload Driver Camera": "錄製並上傳駕駛鏡頭",
  "Record and Upload Microphone Audio": "錄製並上傳麥克風", "Onroad Uploads": "行車中上傳",
  "Brightness & Timeout": "亮度與逾時", "Onroad Brightness": "行車亮度",
  "Onroad Brightness Delay": "行車亮度延遲", "Interactivity Timeout": "互動逾時",
  "Max Time Offroad": "熄火後最長待機", "Quickboot Mode": "快速開機模式",
  "Force Offroad Mode": "強制熄火模式", "Disable Updates": "停用更新",
  "GitHub Runner Service": "GitHub Runner 服務", "copyparty Service": "copyparty 服務",
  "Experimental Mode": "實驗模式", "Joystick Debug Mode": "搖桿除錯模式",
  "UI Debug Mode": "介面除錯模式", "Developer UI Info": "開發者介面資訊",
  "[TEST] Lateral Maneuver Mode": "[測試] 橫向動作模式",
  "[TEST] Longitudinal Maneuver Mode": "[測試] 縱向動作模式",
  # option labels
  "Aggressive": "積極", "Standard": "標準", "Relaxed": "輕鬆",
  "Off": "關閉", "Information": "資訊", "Warning": "警告", "Assist": "輔助",
  "Car State Only": "僅車輛狀態", "Map Data Only": "僅圖資", "Car State Priority": "車輛優先",
  "Map Data Priority": "圖資優先", "Combined": "綜合", "Fixed": "固定值", "Percentage": "百分比",
  "Nudge": "輕推", "Nudgeless": "免輕推",
  "0.5 second": "0.5 秒", "1 second": "1 秒", "2 seconds": "2 秒", "3 seconds": "3 秒", "Disengage": "取消", "Pause": "暫停",
  "Remain Active": "保持啟用", "Default": "預設", "All": "全部", "Always On": "常開",
  "Always Offroad": "常駐熄火", "Auto (Dark)": "自動 (深色)", "Auto (Default)": "自動 (預設)",
  "Bottom": "底部", "Right": "右側", "Right & Bottom": "右側及底部", "Screen Off": "螢幕關閉",
  "Distance": "距離", "Speed": "速度", "Time": "時間",
  # descriptions (section + item)
  "Automatic lane change timing and behavior": "自動變換車道的時機與行為",
  "Lateral pause behavior during turn signals": "打方向燈時的橫向暫停行為",
  "Steering torque tuning and lateral control method": "轉向扭力校調與橫向控制方式",
  "Speed limit detection and offset behavior": "速限偵測與偏移行為",
  "Lane desire and lead-vehicle awareness tuning": "路口轉彎意圖與前車感知校調",
  "Model Behavior": "模型行為", "Overlays shown on the driving screen": "行車畫面上的疊加顯示",
  "Traffic light alerts and visual flair": "號誌提醒與視覺效果",
  "Speedometer and debug display options": "時速表與除錯顯示選項",
  "Camera position and calibration": "鏡頭位置與校正",
  "Power, boot, and unit preferences": "電源、開機與單位偏好",
  "Screen dimming and sleep behavior while driving": "行車時螢幕變暗與休眠行為",
  "Camera and audio recording during drives": "行車時的影像與聲音錄製",
  "Remote access and debugging interfaces": "遠端存取與除錯介面",
  "Control software updates": "控制軟體更新",
  "Enable MADS. Disable toggle to revert back to stock sunnypilot engagement/disengagement.":
    "啟用 MADS。關閉後回到 sunnypilot 原本的啟用/取消方式。",
  "Note: For vehicles without LFA/LKAS button, disabling this will prevent lateral control engagement.":
    "注意：沒有 LFA/LKAS 按鈕的車，關閉此項會導致無法啟用橫向控制。",
  "Engage lateral and longitudinal control with cruise control engagement. Note: Once lateral control is engaged via UEM, it will remain engaged until it is manually disabled via the MADS button or car shut off.":
    "隨定速一併啟用橫向與縱向控制。注意：透過 UEM 啟用橫向控制後會保持啟用，直到用 MADS 按鈕手動關閉或熄火。",
  "Choose how Automatic Lane Centering (ALC) behaves after the brake pedal is manually pressed in sunnypilot.":
    "選擇手動踩煞車後，自動車道置中 (ALC) 的行為。",
  "Set a timer to delay the auto lane change operation when the blinker is used. No nudge on the steering wheel is required to auto lane change if a timer is set. Default is Nudge.":
    "設定打燈後自動變換車道的延遲計時。設了計時後不需推方向盤即可自動變換車道。預設為輕推。",
  "Toggle to enable a delay timer for lane changes when blind spot monitoring (BSM) detects a vehicle in your blind spot.":
    "當盲點偵測 (BSM) 偵測到盲點有車時，啟用變換車道的延遲計時。",
  "Pause lateral control with blinker when traveling below the desired speed selected.":
    "低於所選速度時，打方向燈會暫停橫向控制。",
  "Enable this to enforce sunnypilot to steer with Torque lateral control.":
    "啟用後強制 sunnypilot 使用扭力橫向控制轉向。",
  "Use a neural network for lateral control instead of the default torque controller.":
    "使用神經網路做橫向控制，取代預設的扭力控制器。",
  "Enables custom tuning for Torque lateral control. Modifying Lateral Acceleration Factor and Friction below will override the offline values indicated in the YAML files within \"opendbc/car/torque_data\". The values will also be used live when \"Manual Real-Time Tuning\" toggle is enabled.":
    "啟用扭力橫向控制的自訂校調。修改下方的橫向加速度係數與摩擦係數會覆蓋 \"opendbc/car/torque_data\" YAML 檔的離線值。啟用「手動即時校調」時這些值也會即時套用。",
  "Enforces the torque lateral controller to use the fixed values instead of the learned values from Self-Tune. Enabling this toggle overrides Self-Tune values.":
    "強制扭力橫向控制器使用固定值，而非自動校調學到的值。啟用此項會覆蓋自動校調的值。",
  "Enables self-tune for Torque lateral control for platforms that do not use Torque lateral control by default.":
    "為預設不使用扭力橫向控制的車型啟用自動校調。",
  "Less strict settings when using Self-Tune. This allows torqued to be more forgiving when learning values.":
    "自動校調時採較寬鬆的設定，讓 torqued 學習數值時更有彈性。",
  "Select the version of Torque Control Tune to use.": "選擇要使用的扭力控制校調版本。",
  "Intelligent Cruise Button Management (ICBM) (Alpha)": "智慧定速按鈕管理 (ICBM) (Alpha)",
  "Let the model decide when to use sunnypilot ACC or sunnypilot End to End Longitudinal.":
    "讓模型自行決定何時使用 sunnypilot ACC 或端到端縱向控制。",
  "Use vision path predictions to estimate the appropriate speed to drive through turns ahead.":
    "用視覺路徑預測估算前方彎道的合適速度。",
  "Use map data to estimate the appropriate speed to drive through turns ahead.":
    "用圖資估算前方彎道的合適速度。",
  "Standard is recommended. In aggressive mode, sunnypilot will follow lead cars closer and be more aggressive with the gas and brake. In relaxed mode sunnypilot will stay further away from lead cars. On supported cars, you can cycle through these personalities with your steering wheel distance button.":
    "建議用標準。積極模式會跟車更近、油門煞車更積極；輕鬆模式會與前車保持更遠距離。支援的車可用方向盤距離鍵切換這些風格。",
  "Virtually shift camera's perspective to move model's center to Left(+ values) or Right (- values)":
    "虛擬平移鏡頭視角，把模型中心往左(正值)或往右(負值)移。",
  "Set the maximum speed for lane turn desires.": "設定路口轉彎意圖的最高速度。",
  "If you are driving at 20 mph (32 km/h) or below and have your blinker on, the car will plan a turn in that direction at the nearest drivable path. This prevents situations (like at red lights) where the car might plan the wrong turn direction.":
    "時速 20 mph (32 km/h) 以下且打方向燈時，車會在最近的可行路徑規劃該方向的轉彎，避免（如紅燈時）規劃錯方向。",
  "Allow device to learn and adapt car's steering response time": "讓裝置學習並適應車輛的轉向反應時間",
  "Adjust the software delay when Live Learning Steer Delay is toggled off. The default software delay value is 0.2":
    "關閉「即時學習轉向延遲」時調整軟體延遲。預設值為 0.2。",
  "Speed limit detection and offset behavior.": "速限偵測與偏移行為。",
  "Displays the name of the road the car is traveling on. The OpenStreetMap database of the location must be downloaded to fetch the road name.":
    "顯示行駛道路的名稱。需先下載該地區的 OpenStreetMap 圖資才能取得道路名稱。",
  "Display steering arc on the driving screen when lateral control is enabled.":
    "啟用橫向控制時，在行車畫面顯示轉向弧線。",
  "Show an indicator on the left side of the screen to display real-time vehicle acceleration and deceleration. This displays what the car is currently doing, not what the planner is requesting.":
    "在畫面左側顯示即時加減速指示。顯示車輛當下的實際動作，而非規劃器的請求。",
  "Display real-time parameters and metrics from various sources.": "顯示來自各來源的即時參數與數據。",
  "For applicable vehicles, always display the true vehicle current speed from wheel speed sensors.":
    "適用車型一律顯示輪速感測器的真實車速。",
  "When enabled, the speedometer on the onroad screen is not displayed.": "啟用後，行車畫面不顯示時速表。",
  "Show a timer on the HUD when the car is at a standstill.": "車輛靜止時在抬頭顯示上顯示計時。",
  "Enabling this will display warnings when a vehicle is detected in your blind spot as long as your car has BSM supported.":
    "啟用後，只要車輛支援 BSM，偵測到盲點有車時會顯示警告。",
  "When enabled, visual turn indicators are drawn on the HUD.": "啟用後，在抬頭顯示上繪製方向燈指示。",
  "Display a rainbow effect on the path the model wants to take. It does not affect driving in any way.":
    "在模型預測路徑上顯示彩虹效果，完全不影響駕駛。",
  "A chime and on-screen alert will play when the traffic light you are waiting for turns green and you have no vehicle in front of you. On-screen visual alert is only available on comma 3X. Note: This chime is only designed as a notification. It is the driver's responsibility to observe their environment and make decisions accordingly.":
    "等待的號誌轉綠且前方無車時，會發出提示音與畫面提醒（畫面提醒僅 comma 3X 支援）。注意：此提示僅為通知，觀察環境與判斷仍是駕駛的責任。",
  "A chime and on-screen alert will play when you are stopped, and the vehicle in front of you start moving. On-screen visual alert is only available on comma 3X. Note: This chime is only designed as a notification. It is the driver's responsibility to observe their environment and make decisions accordingly.":
    "停車時前車起步，會發出提示音與畫面提醒（畫面提醒僅 comma 3X 支援）。注意：此提示僅為通知，觀察環境與判斷仍是駕駛的責任。",
  "Toggle visibility of advanced sunnypilot controls. This only changes the visibility of the toggles; it does not change the actual enabled/disabled state.":
    "切換進階 sunnypilot 控制項的顯示。只改變顯示與否，不改變實際啟用狀態。",
  "Display speed in km/h instead of mph.": "以 km/h 顯示速度，而非 mph。",
  "Enable driver monitoring even when sunnypilot is not engaged.": "即使未啟用 sunnypilot 也啟用駕駛監控。",
  "Use the sunnypilot system for adaptive cruise control and lane keep driver assistance. Your attention is required at all times to use this feature.":
    "使用 sunnypilot 進行主動定速與車道保持輔助。使用時須全程保持注意力。",
  "ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. See https://docs.comma.ai/how-to/connect-to-comma for more info.":
    "ADB (Android Debug Bridge) 可透過 USB 或網路連線到裝置。詳見 https://docs.comma.ai/how-to/connect-to-comma。",
  "Enable Lane Departure Warnings": "啟用車道偏離警示",
  "Receive alerts to steer back into the lane when your vehicle drifts over a detected lane line without a turn signal activated while driving over 31 mph (50 km/h).":
    "時速 31 mph (50 km/h) 以上，未打方向燈卻偏離偵測到的車道線時，提醒你轉回車道。",
  "When enabled, pressing the accelerator pedal will disengage longitudinal control.":
    "啟用後，踩油門會取消縱向控制。",
  "Upload data from the driver facing camera and help improve the driver monitoring algorithm.":
    "上傳駕駛鏡頭資料，協助改善駕駛監控演算法。",
  "Record and store microphone audio while driving. The audio will be included in the dashcam video in comma connect.":
    "行車時錄製並儲存麥克風聲音。聲音會併入 comma connect 的行車影像。",
  "Screen dimming and sleep behavior while driving.": "行車時螢幕變暗與休眠行為。",
  "Apply a custom timeout for settings UI. This is the time after which settings UI closes automatically if user is not interacting with the screen.":
    "設定介面的自訂逾時。使用者未操作畫面達此時間後，設定介面會自動關閉。",
  "Device will automatically shutdown after set time once the engine is turned off. 30h is the default.":
    "熄火後經過設定時間會自動關機，預設 30 小時。",
  "Controls state of the device after boot/sleep. Default: Device will boot/wake-up normally and will be ready to engage. Offroad: Device will be in Always Offroad mode after boot/wake-up.":
    "控制開機/喚醒後的裝置狀態。預設：正常開機並可啟用。熄火模式：開機/喚醒後進入常駐熄火模式。",
  "When enabled, software updates will be off. This requires a reboot to take effect.":
    "啟用後停用軟體更新，需重開機生效。",
  "Enables or disables the GitHub runner service.": "啟用或停用 GitHub runner 服務。",
  "copyparty is a very capable file server, you can use it to download your routes, view your logs and even make some edits on some files from your browser. Requires you to connect to your comma locally via its IP address.":
    "copyparty 是功能強大的檔案伺服器，可用瀏覽器下載行車紀錄、看 log、甚至編輯部分檔案。需透過區網 IP 連到你的 comma。",
  "DANGER: enabling these maneuvers replaces normal driving behavior with deterministic test sequences. Each toggle requires explicit confirmation per write. Use only in a closed environment.":
    "危險：啟用這些動作會用固定測試序列取代正常駕駛。每次寫入都需明確確認。僅限封閉環境使用。",
  "Replaces normal lateral control with a deterministic test sequence. NOT for road use.":
    "用固定測試序列取代正常橫向控制。不可上路使用。",
  "Replaces normal longitudinal control with a deterministic test sequence. NOT for road use.":
    "用固定測試序列取代正常縱向控制。不可上路使用。",
  "WARNING: sunnypilot longitudinal control is in alpha for this car and will disable Automatic Emergency Braking (AEB). On this car, sunnypilot defaults to the car's built-in ACC instead of sunnypilot's longitudinal control. Enable this to switch to sunnypilot longitudinal control. Enabling Experimental mode is recommended when enabling sunnypilot longitudinal control alpha. Changing this setting will restart sunnypilot if the car is powered on.":
    "警告：此車的 sunnypilot 縱向控制為 alpha，會停用自動緊急煞車 (AEB)。此車 sunnypilot 預設使用原廠 ACC 而非自家縱向控制。啟用此項可切換為 sunnypilot 縱向控制，建議同時開啟實驗模式。車輛開機時更改此設定會重啟 sunnypilot。",
}


DMONITORINGD_PATH = "/data/openpilot/openpilot/selfdrive/monitoring/dmonitoringd.py"


def dm_patch_unconditional():
  # the device patch neutralizes driverMonitoringState on every publish, with no param
  # toggle, because release-mici's Params deletes unknown keys like DisableDriverMonitoring
  try:
    with open(DMONITORINGD_PATH, encoding="utf-8") as f:
      src = f.read()
    return "def neutralize_dm(" in src and "\n    neutralize_dm(dat)" in src
  except OSError:
    return False


def lat_mode():
  """Which lateral setup is active. NNLC and the manual torque override are mutually
  exclusive in settings_ui.json (NNLC requires EnforceTorqueControl == false), so the
  page offers a one-tap switch instead of leaving one of them greyed out."""
  if as_bool(read_param("NeuralNetworkLateralControl")):
    return "nnlc"
  if as_bool(read_param("EnforceTorqueControl")) and as_bool(read_param("CustomTorqueParams")):
    return "custom"
  return "stock"


def dm_disabled():
  return dm_patch_unconditional() or as_bool(read_param("DisableDriverMonitoring"))


def tr(s):
  if s is None:
    return s
  return TRANSLATIONS.get(s) or TRANSLATIONS.get(s.strip() if isinstance(s, str) else s, s)


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
  refresh_capabilities()
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
            "title": tr(it.get("title", key)),
            "description": tr(it.get("description", "")),
            "widget": it.get("widget", "toggle"),
            "options": [
              {"label": tr(o.get("label")), "value": o.get("value"),
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
          "label": tr(sp.get("label", "")),
          "visible": eval_conds([sp.get("trigger_condition")] if sp.get("trigger_condition") else [], onroad),
          "items": conv(sp.get("items", [])),
        })
      sections.append({
        "title": tr(s.get("title", "")),
        "description": s.get("description", ""),
        "visible": eval_conds(s.get("enablement"), onroad),
        "items": conv(s.get("items", [])),
        "sub_panels": sub_panels,
      })
    panels.append({"id": p.get("id"), "label": tr(p.get("label")), "sections": sections})

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
      "disable_dm": dm_disabled(),
      "dm_unconditional": dm_patch_unconditional(),
      "lat_mode": lat_mode(),
      "lat_accel_factor": read_param("TorqueParamsOverrideLatAccelFactor"),
      "lat_friction": read_param("TorqueParamsOverrideFriction"),
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
<title>C4 設定</title>
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
<header><h1>C4 設定</h1><div id="status">連線中…</div></header>
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
// option values arrive as numbers (0.0) while params read back as strings ("0.0"),
// so compare numerically when both sides parse as numbers
function sameVal(a,b){const x=parseFloat(a),y=parseFloat(b);return (!isNaN(x)&&!isNaN(y))?x===y:String(a)===String(b??'')}
function itemRow(it){
  if(!it.visible)return '';
  const off=it.enabled?'':' off';
  let ctrl='';
  if(it.widget==='toggle'){
    ctrl=`<div class="sw ${boolVal(it.value)?'on':''}" onclick="setParam('${it.key}',${boolVal(it.value)?'0':'1'})"><i></i></div>`;
  }else if(it.widget==='multiple_button'){
    ctrl='<div class="seg">'+it.options.map(o=>
      `<button ${o.enabled?'':'disabled'} class="${sameVal(o.value,it.value)?'on':''}" onclick="setParam('${it.key}','${o.value}')">${o.label}</button>`).join('')+'</div>';
  }else if(it.widget==='option' && it.options && it.options.length){
    // enum option: step through the labelled choices, never show the raw enum number
    // (e.g. AutoLaneChangeTimer 2 means "0.5 second", not 2 seconds)
    const idx=it.options.findIndex(o=>sameVal(o.value,it.value));
    const lo=Math.max(0,idx-1), hi=Math.min(it.options.length-1,idx+1);
    const lbl=idx>=0?it.options[idx].label:(it.value??'—');
    ctrl=`<div class="num"><button ${idx<=0?'disabled':''} onclick="setParam('${it.key}','${it.options[lo].value}')">−</button>`
       + `<span>${lbl}</span>`
       + `<button ${idx<0||idx>=it.options.length-1?'disabled':''} onclick="setParam('${it.key}','${it.options[hi].value}')">+</button></div>`;
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
async function setLatMode(mode){
  const sets = mode==='nnlc'
    ? [['NeuralNetworkLateralControl','1'],['EnforceTorqueControl','0'],['CustomTorqueParams','0'],['TorqueParamsOverrideEnabled','0']]
    : mode==='custom'
    ? [['NeuralNetworkLateralControl','0'],['EnforceTorqueControl','1'],['CustomTorqueParams','1'],['TorqueParamsOverrideEnabled','1']]
    : [['NeuralNetworkLateralControl','0'],['EnforceTorqueControl','0'],['CustomTorqueParams','0'],['TorqueParamsOverrideEnabled','0']];
  for(const [k,v] of sets){ await api('/api/set',{key:k,value:v}); }
  toast('已切換，熄火再紅火一次才生效');
  await refresh(false);
}
function latSection(){
  const m=MODEL.status.lat_mode, st=MODEL.status;
  const btn=(id,label,desc)=>`<button class="${m===id?'on':''}" onclick="setLatMode('${id}')">${label}</button>`;
  return `<div class="sec">橫向控制模式（改完要熄火再紅火）</div><div class="card">
    <div class="item"><div class="grow"><div class="t">目前：${m==='nnlc'?'神經網路 NNLC':(m==='custom'?'手動調校':'原廠預設')}</div>
    <div class="d">手動調校 = 用你車學到的值（latAccelFactor ${st.lat_accel_factor||'—'} / friction ${st.lat_friction||'—'}）。NNLC = 用 HYUNDAI_TUCSON_4TH_GEN 的神經網路模型（Custin 無專屬模型，模糊比對）。兩者互斥。</div></div></div>
    <div class="item"><div class="grow"><div class="seg">
      ${btn('custom','手動調校')}${btn('nnlc','神經網路 NNLC')}${btn('stock','原廠預設')}
    </div></div></div></div>`;
}
function dmSection(){
  const on=MODEL.status.disable_dm, fixed=MODEL.status.dm_unconditional;
  const d=fixed?'已由裝置端 dmonitoringd patch 無條件關閉（不吃參數，release-mici 會刪掉自訂 param）。要改回請改裝置上的檔案。'
               :'關閉後不再偵測分心、不會發出注意力警告或強制減速。駕駛須自行全程注意路況並負責。';
  const sw=fixed?`<div class="sw on off"><i></i></div>`
                :`<div class="sw ${on?'on':''}" onclick="setParam('DisableDriverMonitoring',${on?'0':'1'})"><i></i></div>`;
  return `<div class="sec">駕駛監控 (DM)</div><div class="card">
    <div class="item${fixed?' off':''}"><div class="grow"><div class="t">關閉駕駛監控</div><div class="d">${d}</div></div>
    ${sw}</div></div>`;
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
  if(panel.id==='toggles')html+=dmSection();
  if(panel.id==='steering')html+=latSection();
  $('#main').innerHTML=html;
  const st=MODEL.status;
  $('#status').textContent=`${st.branch||''} ${st.version||''} · ${st.onroad?'⚠️ 行車中（多數設定鎖定）':'✅ 熄火中 可設定'}`;
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
