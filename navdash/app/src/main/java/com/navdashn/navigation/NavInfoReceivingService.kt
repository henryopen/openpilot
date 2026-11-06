package com.navdashn.navigation

import android.app.Service
import android.content.Intent
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.os.Message
import android.os.Messenger
import android.util.Log
import com.google.android.libraries.mapsplatform.turnbyturn.TurnByTurnManager
import com.google.android.libraries.mapsplatform.turnbyturn.model.NavInfo
import com.google.android.libraries.mapsplatform.turnbyturn.model.StepInfo

/**
 * 接收 Turn-by-Turn 導航資料的 Service
 * 從 Navigation SDK 取得詳細的轉彎指示
 */
class NavInfoReceivingService : Service() {

    companion object {
        private const val TAG = "NavInfoReceivingService"

        // 最新的導航資訊（靜態，讓 NavController 可以存取）
        @Volatile
        var latestNavInfo: NavInfo? = null
            private set

        // 導航資料更新回調
        var onNavInfoUpdated: ((NavInfo) -> Unit)? = null
    }

    private lateinit var turnByTurnManager: TurnByTurnManager
    private lateinit var incomingMessenger: Messenger

    /**
     * 處理從 Navigation SDK 收到的訊息
     */
    private inner class IncomingNavInfoHandler : Handler(Looper.getMainLooper()) {
        override fun handleMessage(msg: Message) {
            try {
                if (msg.what == TurnByTurnManager.MSG_NAV_INFO) {
                    val navInfo = turnByTurnManager.readNavInfoFromBundle(msg.data)
                    if (navInfo != null) {
                        latestNavInfo = navInfo
                        logNavInfo(navInfo)
                        onNavInfoUpdated?.invoke(navInfo)
                    }
                } else {
                    super.handleMessage(msg)
                }
            } catch (e: Exception) {
                Log.e(TAG, "Error handling nav info message", e)
            }
        }
    }

    override fun onCreate() {
        super.onCreate()
        turnByTurnManager = TurnByTurnManager.createInstance()
        incomingMessenger = Messenger(IncomingNavInfoHandler())
        Log.d(TAG, "NavInfoReceivingService created")
    }

    override fun onBind(intent: Intent?): IBinder? {
        return incomingMessenger.binder
    }

    override fun onDestroy() {
        super.onDestroy()
        latestNavInfo = null
        Log.d(TAG, "NavInfoReceivingService destroyed")
    }

    /**
     * 記錄導航資訊（除錯用）
     */
    private fun logNavInfo(navInfo: NavInfo) {
        val currentStep = navInfo.currentStep
        val navState = navInfo.navState

        Log.d(TAG, "=== NavInfo Update ===")
        Log.d(TAG, "NavState: $navState")
        Log.d(TAG, "Distance to final dest: ${navInfo.distanceToFinalDestinationMeters} m")
        Log.d(TAG, "Time to final dest: ${navInfo.timeToFinalDestinationSeconds} s")

        if (currentStep != null) {
            Log.d(TAG, "--- Current Step ---")
            Log.d(TAG, "Full road name: ${currentStep.fullRoadName}")
            Log.d(TAG, "Full instruction: ${currentStep.fullInstructionText}")
            Log.d(TAG, "Distance to step: ${navInfo.distanceToCurrentStepMeters} m")
            Log.d(TAG, "Maneuver: ${currentStep.maneuver}")

            // ===== 除錯: 檢查 SDK 提供的 Bitmap =====
            val maneuverBitmap = currentStep.maneuverBitmap
            val lanesBitmap = currentStep.lanesBitmap
            Log.d(TAG, "ManeuverBitmap: ${maneuverBitmap != null} (${maneuverBitmap?.width}x${maneuverBitmap?.height})")
            Log.d(TAG, "LanesBitmap: ${lanesBitmap != null} (${lanesBitmap?.width}x${lanesBitmap?.height})")
            Log.d(TAG, "ExitNumber: ${currentStep.exitNumber}")
            Log.d(TAG, "RoundaboutTurnNumber: ${currentStep.roundaboutTurnNumber}")
            Log.d(TAG, "DrivingSide: ${currentStep.drivingSide}")
            Log.d(TAG, "Lanes count: ${currentStep.lanes?.size ?: 0}")
        }

        val remainingSteps = navInfo.remainingSteps
        if (remainingSteps != null && remainingSteps.isNotEmpty()) {
            Log.d(TAG, "Remaining steps: ${remainingSteps.size}")
            remainingSteps.take(3).forEachIndexed { index, step ->
                Log.d(TAG, "  Step $index: ${step.fullRoadName} - ${step.maneuver}")
            }
        }
    }
}
