package com.navdashn.utils

/**
 * 常數定義
 */
object Constants {

    // HTTP Server
    const val HTTP_SERVER_PORT = 5002

    // 更新間隔
    const val NAV_UPDATE_INTERVAL = 1000L  // 1 秒

    // WebView
    const val WEBVIEW_TIMEOUT = 30000L  // 30 秒

    // 導航
    const val MAX_FAVORITES = 50
    const val MAX_RECENTS = 10

    // 距離閾值 (公尺)
    const val DISTANCE_URGENT = 100
    const val DISTANCE_WARNING = 300
}
