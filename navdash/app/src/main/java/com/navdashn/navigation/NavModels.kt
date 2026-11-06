package com.navdashn.navigation

import android.graphics.Bitmap

/**
 * 地點項目
 */
data class PlaceItem(
    val id: String = "",
    val name: String = "",
    val address: String = "",
    val icon: String = "📍",
    val lat: Double? = null,
    val lng: Double? = null,
    val timestamp: Long = System.currentTimeMillis()
)

/**
 * 導航資料 (傳送給 Simple Dash 的格式)
 */
data class NavData(
    val isNavigating: Boolean = false,
    val maneuverType: String = "",
    val maneuverModifier: String = "",
    val instruction: String = "",
    val roadName: String = "",
    val distanceToNext: Int = 0,
    val distanceRemaining: Int = 0,
    val timeRemaining: Int = 0,
    val eta: String = "",
    val speedLimit: Int = 0,
    val currentSpeed: Int = 0,
    // SDK 原生圖示
    val maneuverBitmap: Bitmap? = null,  // 轉向圖示 (SDK 提供)
    val lanesBitmap: Bitmap? = null,      // 車道指示圖 (SDK 提供)
    // 額外資訊
    val exitNumber: String = "",          // 出口編號
    val roundaboutTurnNumber: Int = 0,    // 圓環出口編號
    val drivingSide: String = "",         // 靠左/靠右行駛 (LEFT/RIGHT/UNSPECIFIED)
    val laneCount: Int = 0                // 車道數量
)

/**
 * 導航狀態
 */
enum class NavState {
    IDLE,           // 閒置
    ROUTING,        // 計算路線中
    NAVIGATING,     // 導航中
    REROUTING,      // 重新計算路線
    ARRIVED,        // 已到達
    ERROR           // 錯誤
}

/**
 * 轉向類型
 */
object ManeuverType {
    const val TURN = "turn"
    const val OFF_RAMP = "off_ramp"
    const val ON_RAMP = "on_ramp"
    const val FORK = "fork"
    const val MERGE = "merge"
    const val ROUNDABOUT = "roundabout"
    const val STRAIGHT = "straight"
    const val ARRIVE = "arrive"
    const val DEPART = "depart"
    const val FERRY = "ferry"
    const val CONTINUE = "continue"
}

/**
 * 轉向方向
 */
object ManeuverModifier {
    const val LEFT = "left"
    const val RIGHT = "right"
    const val SLIGHT_LEFT = "slight_left"
    const val SLIGHT_RIGHT = "slight_right"
    const val SHARP_LEFT = "sharp_left"
    const val SHARP_RIGHT = "sharp_right"
    const val STRAIGHT = "straight"
    const val UTURN = "uturn"
}
