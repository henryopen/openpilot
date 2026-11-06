package com.navdashn.navigation

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Intent
import android.os.Build
import android.os.IBinder
import android.util.Log
import androidx.core.app.NotificationCompat
import fi.iki.elonen.NanoHTTPD
import com.navdashn.MainActivity
import com.navdashn.R

/**
 * 導航前台服務
 * 確保導航在背景執行時不會被系統終止
 */
class NavService : Service() {

    companion object {
        private const val TAG = "NavService"
        private const val NOTIFICATION_ID = 1001
        private const val CHANNEL_ID = "nav_channel"
        private const val CHANNEL_NAME = "導航服務"
    }

    private var httpServer: NavHttpServer? = null

    override fun onCreate() {
        super.onCreate()
        Log.d(TAG, "NavService created")
        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        Log.d(TAG, "NavService started")

        // 啟動前台服務（Android 14+ 必須帶 location 類型，且要已有定位權限）
        try {
            androidx.core.app.ServiceCompat.startForeground(
                this, NOTIFICATION_ID, createNotification(),
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q)
                    android.content.pm.ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION else 0
            )
        } catch (e: Exception) {
            Log.e(TAG, "startForeground failed (location permission?)", e)
            stopSelf()
            return START_NOT_STICKY
        }

        // 啟動 HTTP Server
        startHttpServer()

        return START_STICKY
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onDestroy() {
        super.onDestroy()
        stopHttpServer()
        Log.d(TAG, "NavService destroyed")
    }

    /**
     * 建立通知頻道
     */
    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                CHANNEL_ID,
                CHANNEL_NAME,
                NotificationManager.IMPORTANCE_LOW
            ).apply {
                description = "NavDash 導航服務"
                setShowBadge(false)
            }

            val manager = getSystemService(NotificationManager::class.java)
            manager.createNotificationChannel(channel)
        }
    }

    /**
     * 建立通知
     */
    private fun createNotification(): Notification {
        val pendingIntent = PendingIntent.getActivity(
            this,
            0,
            Intent(this, MainActivity::class.java),
            PendingIntent.FLAG_IMMUTABLE
        )

        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("NavDash")
            .setContentText("導航服務執行中")
            .setSmallIcon(R.drawable.ic_compass)
            .setContentIntent(pendingIntent)
            .setOngoing(true)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .build()
    }

    /**
     * 更新通知內容
     */
    fun updateNotification(title: String, text: String) {
        val notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle(title)
            .setContentText(text)
            .setSmallIcon(R.drawable.ic_compass)
            .setOngoing(true)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .build()

        val manager = getSystemService(NotificationManager::class.java)
        manager.notify(NOTIFICATION_ID, notification)
    }

    /**
     * 啟動 HTTP Server
     */
    private fun startHttpServer() {
        if (httpServer != null) return

        try {
            val version = try {
                packageManager.getPackageInfo(packageName, 0).versionName ?: ""
            } catch (e: Exception) {
                ""
            }
            httpServer = NavHttpServer(5002, version)
            httpServer?.start(NanoHTTPD.SOCKET_READ_TIMEOUT, false)
            Log.d(TAG, "HTTP Server started on port 5002")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start HTTP Server", e)
        }
    }

    /**
     * 停止 HTTP Server
     */
    private fun stopHttpServer() {
        httpServer?.stop()
        httpServer = null
        Log.d(TAG, "HTTP Server stopped")
    }
}
