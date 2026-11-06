package com.navdashn.navigation

import android.util.Log
import com.google.gson.GsonBuilder
import fi.iki.elonen.NanoHTTPD

/**
 * 給 PiBar 讀的導航 HTTP 服務（port 5002）
 *
 *  GET /health  → {"status":"ok","server":"NavDash-D531",...}   Pi 用它在手機熱點網段裡找到這台
 *  GET /nav     → 目前導航狀態（NavController.snapshot），Pi 每秒讀一次
 *  GET /route   → 整條路線座標 {"ver":N,"pts":[[lat,lng],...]}，Pi 看到 /nav 的 routeVer 變了才讀
 *
 * 只回應讀取，不接受任何控制指令。
 */
class NavHttpServer(port: Int, private val appVersion: String) : NanoHTTPD(port) {

    companion object {
        private const val TAG = "NavHttpServer"

        // 最後一次有人來讀 /nav（給主畫面顯示「PiBar 已連線」）
        @Volatile var lastNavReadElapsedMs: Long = 0L
            private set
        @Volatile var lastNavReader: String = ""
            private set
    }

    private val gson = GsonBuilder().serializeNulls().create()

    override fun serve(session: IHTTPSession): Response {
        if (session.method != Method.GET) {
            return newFixedLengthResponse(Response.Status.METHOD_NOT_ALLOWED, MIME_PLAINTEXT, "GET only")
        }
        return try {
            when (session.uri) {
                "/health" -> json(mapOf(
                    "status" to "ok",
                    "server" to "NavDash-D531",
                    "version" to appVersion,
                    "ready" to NavController.isReady()
                ))
                "/nav" -> {
                    lastNavReadElapsedMs = android.os.SystemClock.elapsedRealtime()
                    lastNavReader = session.remoteIpAddress ?: ""
                    json(NavController.snapshot())
                }
                "/route" -> json(NavController.routeJson())
                else -> newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, "Not Found")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error serving ${session.uri}", e)
            newFixedLengthResponse(Response.Status.INTERNAL_ERROR, MIME_PLAINTEXT, "Error: ${e.message}")
        }
    }

    private fun json(body: Any): Response =
        newFixedLengthResponse(Response.Status.OK, "application/json; charset=utf-8", gson.toJson(body)).apply {
            addHeader("Cache-Control", "no-store")
            addHeader("Access-Control-Allow-Origin", "*")
        }
}
