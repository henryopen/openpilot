package com.navdashn.navigation

import android.app.Activity
import android.app.Application
import android.content.Context
import android.location.Location
import android.os.SystemClock
import android.util.Log
import com.google.android.libraries.mapsplatform.turnbyturn.model.LaneDirection
import com.google.android.libraries.mapsplatform.turnbyturn.model.Maneuver
import com.google.android.libraries.mapsplatform.turnbyturn.model.NavInfo
import com.google.android.libraries.mapsplatform.turnbyturn.model.StepInfo
import com.google.android.libraries.navigation.NavigationApi
import com.google.android.libraries.navigation.NavigationUpdatesOptions
import com.google.android.libraries.navigation.Navigator
import com.google.android.libraries.navigation.RoadSnappedLocationProvider
import com.google.android.libraries.navigation.SimulationOptions
import com.google.android.libraries.navigation.Waypoint
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * 導航控制器（D531 版）
 *
 * Google Navigation SDK 負責規劃路線與引導；這裡只把結果整理成 Pi 讀得懂的 JSON：
 *  - 目前這一步（轉彎種類、距離、路名、車道）與後續 3 步
 *  - 剩餘距離/時間、ETA
 *  - 貼路後的目前位置
 *  - 整條路線的座標（每次規劃/重新規劃 routeVersion +1）
 * 沒在導航時 state=idle，其它欄位都是空的。
 */
object NavController {

    private const val TAG = "NavController"
    private const val NUM_NEXT_STEPS_TO_PREVIEW = 25
    private const val NEXT_STEPS_IN_JSON = 3

    private var navigator: Navigator? = null
    private var state: NavState = NavState.IDLE
    private var isInitialized = false
    private var isTurnByTurnRegistered = false
    private var appContext: Context? = null
    private var displayMetrics: android.util.DisplayMetrics? = null
    private var locationProvider: RoadSnappedLocationProvider? = null

    // 目的地
    private var destName: String = ""
    private var destAddress: String = ""
    private var destLat: Double? = null
    private var destLng: Double? = null

    // 最後一次規劃失敗的原因（給畫面與 Pi 看）
    private var lastRouteError: String = ""

    // 路線座標：每次規劃/重新規劃換一版
    @Volatile private var routeVersion: Int = 0
    @Volatile private var routePoints: List<DoubleArray> = emptyList()

    // 貼路後的目前位置
    @Volatile private var lastLocation: Location? = null
    @Volatile private var lastLocationElapsedMs: Long = 0L

    @Volatile var isSimulating: Boolean = false
        private set

    // ------------------------------------------------------------------ 導航語音（11/02 使用者：「導航語音要可以關閉」）
    // 存在 SharedPreferences，預設開（跟 2.3.0 以前一樣）；開始導航時照這個設定，導航中切換立刻生效。
    private const val PREFS = "navdash"
    private const val KEY_VOICE = "voice_on"

    fun isVoiceOn(): Boolean =
        appContext?.getSharedPreferences(PREFS, Context.MODE_PRIVATE)?.getBoolean(KEY_VOICE, true) ?: true

    fun setVoiceOn(on: Boolean) {
        appContext?.getSharedPreferences(PREFS, Context.MODE_PRIVATE)?.edit()?.putBoolean(KEY_VOICE, on)?.apply()
        try {
            navigator?.setAudioGuidance(audioGuidance(on))
        } catch (e: Exception) {
            Log.e(TAG, "setAudioGuidance failed", e)
        }
        Log.d(TAG, "Voice guidance ${if (on) "on" else "off"}")
    }

    private fun audioGuidance(on: Boolean): Int =
        if (on) Navigator.AudioGuidance.VOICE_ALERTS_AND_GUIDANCE else Navigator.AudioGuidance.SILENT

    private var onNavigationStateChangedListener: ((NavState) -> Unit)? = null
    private var onInitErrorListener: ((String, Int) -> Unit)? = null

    private val arrivalListener = Navigator.ArrivalListener {
        Log.d(TAG, "Arrived at destination")
        setState(NavState.ARRIVED)
    }

    private val routeChangedListener = Navigator.RouteChangedListener {
        Log.d(TAG, "Route changed")
        captureRoute()
        if (state == NavState.REROUTING) setState(NavState.NAVIGATING)
    }

    private val reroutingListener = Navigator.ReroutingListener {
        Log.d(TAG, "Rerouting")
        if (state == NavState.NAVIGATING) setState(NavState.REROUTING)
    }

    private val locationListener = object : RoadSnappedLocationProvider.LocationListener {
        override fun onLocationChanged(location: Location) {
            lastLocation = location
            lastLocationElapsedMs = SystemClock.elapsedRealtime()
        }

        override fun onRawLocationUpdate(location: Location) {}
    }

    // ------------------------------------------------------------------ 初始化

    fun init(activity: Activity) {
        if (isInitialized) return

        appContext = activity.applicationContext
        displayMetrics = activity.resources.displayMetrics
        lastErrorCode = 0
        lastErrorMessage = ""

        try {
            NavigationApi.getNavigator(activity, object : NavigationApi.NavigatorListener {
                override fun onNavigatorReady(nav: Navigator) {
                    navigator = nav
                    isInitialized = true
                    lastErrorCode = 0
                    lastErrorMessage = ""
                    setupListeners()
                    registerTurnByTurnService()
                    startLocationUpdates(activity.application)
                    Log.d(TAG, "Navigator initialized")
                    onNavigationStateChangedListener?.invoke(state)
                }

                override fun onError(@NavigationApi.ErrorCode errorCode: Int) {
                    val errorMessage = when (errorCode) {
                        NavigationApi.ErrorCode.TERMS_NOT_ACCEPTED -> "尚未同意 Google 導航服務條款"
                        NavigationApi.ErrorCode.NETWORK_ERROR -> "網路錯誤，請確認已連上手機熱點"
                        NavigationApi.ErrorCode.NOT_AUTHORIZED -> "API Key 未授權：請確認 Google Cloud 已啟用 Navigation SDK"
                        NavigationApi.ErrorCode.LOCATION_PERMISSION_MISSING -> "缺少位置權限"
                        else -> "初始化失敗 (code: $errorCode)"
                    }
                    Log.e(TAG, "Navigator init failed: $errorMessage")
                    isInitialized = false
                    lastErrorCode = errorCode
                    lastErrorMessage = errorMessage
                    onInitErrorListener?.invoke(errorMessage, errorCode)
                }
            })
        } catch (e: Exception) {
            Log.e(TAG, "Exception during Navigation SDK init", e)
            isInitialized = false
            lastErrorCode = -1
            lastErrorMessage = "初始化異常: ${e.message}"
            onInitErrorListener?.invoke(lastErrorMessage, lastErrorCode)
        }
    }

    fun showTermsAndConditions(activity: Activity, onAccepted: () -> Unit, onRejected: () -> Unit) {
        try {
            NavigationApi.showTermsAndConditionsDialog(
                activity,
                "NavDash",
                object : NavigationApi.OnTermsResponseListener {
                    override fun onTermsResponse(accepted: Boolean) {
                        if (accepted) onAccepted() else onRejected()
                    }
                }
            )
        } catch (e: Exception) {
            Log.e(TAG, "Failed to show Terms dialog", e)
            onRejected()
        }
    }

    fun getLastError(): Pair<String, Int> = Pair(lastErrorMessage, lastErrorCode)

    private var lastErrorCode: Int = 0
    private var lastErrorMessage: String = ""

    private fun registerTurnByTurnService() {
        val nav = navigator ?: return
        val ctx = appContext ?: return
        val metrics = displayMetrics ?: return

        try {
            val options = NavigationUpdatesOptions.builder()
                .setNumNextStepsToPreview(NUM_NEXT_STEPS_TO_PREVIEW)
                .setGeneratedStepImagesType(NavigationUpdatesOptions.GeneratedStepImagesType.NONE)
                .setDisplayMetrics(metrics)
                .build()

            isTurnByTurnRegistered = nav.registerServiceForNavUpdates(
                ctx.packageName,
                NavInfoReceivingService::class.java.name,
                options
            )
            Log.d(TAG, "Turn-by-Turn service registration: $isTurnByTurnRegistered")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to register Turn-by-Turn service", e)
            isTurnByTurnRegistered = false
        }
    }

    private fun startLocationUpdates(app: Application) {
        try {
            val provider = NavigationApi.getRoadSnappedLocationProvider(app)
            provider?.addLocationListener(locationListener)
            locationProvider = provider
        } catch (e: Exception) {
            Log.e(TAG, "Road-snapped location unavailable", e)
        }
    }

    fun setOnNavigationStateChangedListener(listener: ((NavState) -> Unit)?) {
        onNavigationStateChangedListener = listener
    }

    fun setOnInitErrorListener(listener: ((String, Int) -> Unit)?) {
        onInitErrorListener = listener
    }

    private fun setupListeners() {
        navigator?.apply {
            addArrivalListener(arrivalListener)
            addRouteChangedListener(routeChangedListener)
            addReroutingListener(reroutingListener)
        }
    }

    private fun setState(newState: NavState) {
        state = newState
        onNavigationStateChangedListener?.invoke(state)
    }

    // ------------------------------------------------------------------ 導航

    /**
     * 開始導航：先等 Google 規劃出路線（沒網路會失敗），成功才開始引導。
     */
    fun startNavigation(lat: Double, lng: Double, name: String, address: String = "") {
        val nav = navigator ?: run {
            lastRouteError = "導航尚未初始化"
            setState(NavState.ERROR)
            return
        }

        if (isSimulating) stopSimulation()
        destName = name
        destAddress = address
        destLat = lat
        destLng = lng
        lastRouteError = ""
        setState(NavState.ROUTING)

        try {
            val waypoint = Waypoint.builder().setLatLng(lat, lng).setTitle(name).build()
            val future = nav.setDestination(waypoint)
            future.setOnResultListener { status ->
                if (status == Navigator.RouteStatus.OK) {
                    nav.setAudioGuidance(audioGuidance(isVoiceOn()))
                    nav.startGuidance()
                    captureRoute()
                    setState(NavState.NAVIGATING)
                    Log.d(TAG, "Navigation started to: $name ($lat, $lng)")
                } else {
                    lastRouteError = routeStatusText(status)
                    Log.e(TAG, "Route failed: $status")
                    nav.clearDestinations()
                    setState(NavState.ERROR)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start navigation", e)
            lastRouteError = "規劃失敗: ${e.message}"
            setState(NavState.ERROR)
        }
    }

    private fun routeStatusText(status: Navigator.RouteStatus): String = when (status) {
        Navigator.RouteStatus.NETWORK_ERROR -> "沒有網路，無法規劃路線（請確認已連上手機熱點）"
        Navigator.RouteStatus.NO_ROUTE_FOUND -> "找不到可行的路線"
        Navigator.RouteStatus.QUOTA_CHECK_FAILED -> "Google 導航額度檢查失敗"
        Navigator.RouteStatus.LOCATION_DISABLED, Navigator.RouteStatus.LOCATION_UNKNOWN -> "還沒有定位"
        Navigator.RouteStatus.WAYPOINT_ERROR -> "目的地無效"
        Navigator.RouteStatus.ROUTE_CANCELED -> "路線規劃被取消"
        else -> "規劃失敗 ($status)"
    }

    fun stopNavigation() {
        if (isSimulating) stopSimulation()
        navigator?.stopGuidance()
        navigator?.clearDestinations()
        destName = ""
        destAddress = ""
        destLat = null
        destLng = null
        routePoints = emptyList()
        routeVersion++
        setState(NavState.IDLE)
        Log.d(TAG, "Navigation stopped")
    }

    /** 測試用：沿目前路線模擬行駛（送給 Pi 的資料會標 simulated=true） */
    fun startSimulation(speedMultiplier: Float = 1f) {
        val nav = navigator ?: return
        nav.simulator.simulateLocationsAlongExistingRoute(SimulationOptions().speedMultiplier(speedMultiplier))
        isSimulating = true
    }

    fun stopSimulation() {
        navigator?.simulator?.unsetUserLocation()
        isSimulating = false
    }

    private fun captureRoute() {
        try {
            val pts = navigator?.currentRouteSegment?.latLngs ?: emptyList()
            routePoints = pts.map { doubleArrayOf(it.latitude, it.longitude) }
            routeVersion++
            Log.d(TAG, "Route captured: v$routeVersion, ${routePoints.size} points")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to capture route", e)
        }
    }

    // ------------------------------------------------------------------ 狀態查詢

    fun getState(): NavState = state

    fun getCurrentDestination(): String? = destName.ifBlank { null }

    fun getLastRouteError(): String = lastRouteError

    fun isNavigating(): Boolean = state == NavState.NAVIGATING || state == NavState.REROUTING

    fun isReady(): Boolean = isInitialized && navigator != null

    /** 給主畫面顯示用的剩餘距離/時間 */
    fun getRemaining(): Triple<Int, Int, String> {
        val navInfo = NavInfoReceivingService.latestNavInfo
        if (!isNavigating() || navInfo == null) return Triple(0, 0, "")
        val sec = navInfo.timeToFinalDestinationSeconds ?: 0
        return Triple(navInfo.distanceToFinalDestinationMeters ?: 0, sec, etaText(sec))
    }

    private fun etaText(seconds: Int): String {
        if (seconds <= 0) return ""
        return SimpleDateFormat("HH:mm", Locale.getDefault()).format(Date(System.currentTimeMillis() + seconds * 1000L))
    }

    // ------------------------------------------------------------------ 給 Pi 的 JSON

    fun getRouteVersion(): Int = routeVersion

    fun routeJson(): Map<String, Any?> = mapOf(
        "ver" to routeVersion,
        "pts" to routePoints
    )

    /**
     * 目前導航狀態（Pi 每秒讀一次 GET /nav）
     */
    fun snapshot(): Map<String, Any?> {
        val out = linkedMapOf<String, Any?>(
            "v" to 1,
            "ts" to System.currentTimeMillis(),
            "state" to state.name.lowercase(Locale.US),
            "simulated" to isSimulating,
            "error" to lastRouteError.ifBlank { null },
            "routeVer" to routeVersion
        )

        lastLocation?.let { loc ->
            out["loc"] = mapOf(
                "lat" to loc.latitude,
                "lng" to loc.longitude,
                "bearing" to if (loc.hasBearing()) loc.bearing else null,
                "speed" to if (loc.hasSpeed()) loc.speed else null,
                "acc" to if (loc.hasAccuracy()) loc.accuracy else null,
                "ageMs" to (SystemClock.elapsedRealtime() - lastLocationElapsedMs)
            )
        }

        if (!isNavigating()) return out

        out["dest"] = mapOf("name" to destName, "address" to destAddress, "lat" to destLat, "lng" to destLng)

        val navInfo = NavInfoReceivingService.latestNavInfo ?: return out
        val step = navInfo.currentStep
        if (step != null) {
            val (type, modifier) = getManeuverInfoFromStep(step)
            out["step"] = mapOf(
                "maneuver" to type,
                "modifier" to modifier,
                "sdkManeuver" to step.maneuver,
                "instruction" to (step.fullInstructionText ?: ""),
                "road" to (step.fullRoadName ?: ""),
                "dist" to (navInfo.distanceToCurrentStepMeters ?: 0),
                "time" to (navInfo.timeToCurrentStepSeconds ?: 0),
                "exit" to (step.exitNumber ?: ""),
                "roundabout" to step.roundaboutTurnNumber,
                "lanes" to lanesJson(step)
            )
        }

        out["next"] = (navInfo.remainingSteps?.toList() ?: emptyList()).take(NEXT_STEPS_IN_JSON).map { s ->
            val (type, modifier) = getManeuverInfoFromStep(s)
            mapOf(
                "maneuver" to type,
                "modifier" to modifier,
                "sdkManeuver" to s.maneuver,
                "instruction" to (s.fullInstructionText ?: ""),
                "road" to (s.fullRoadName ?: ""),
                "distFromPrev" to s.distanceFromPrevStepMeters
            )
        }

        val sec = navInfo.timeToFinalDestinationSeconds ?: 0
        out["remain"] = mapOf(
            "dist" to (navInfo.distanceToFinalDestinationMeters ?: 0),
            "time" to sec,
            "eta" to etaText(sec)
        )
        return out
    }

    private fun lanesJson(step: StepInfo): List<Map<String, Any?>> =
        (step.lanes ?: emptyList()).map { lane ->
            mapOf("dirs" to lane.laneDirections().map { d ->
                mapOf("shape" to laneShapeName(d.laneShape()), "rec" to (d.isRecommended() == true))
            })
        }

    private fun laneShapeName(shape: Int): String = when (shape) {
        LaneDirection.LaneShape.STRAIGHT -> "straight"
        LaneDirection.LaneShape.SLIGHT_LEFT -> "slight_left"
        LaneDirection.LaneShape.SLIGHT_RIGHT -> "slight_right"
        LaneDirection.LaneShape.NORMAL_LEFT -> "left"
        LaneDirection.LaneShape.NORMAL_RIGHT -> "right"
        LaneDirection.LaneShape.SHARP_LEFT -> "sharp_left"
        LaneDirection.LaneShape.SHARP_RIGHT -> "sharp_right"
        LaneDirection.LaneShape.U_TURN_LEFT -> "uturn_left"
        LaneDirection.LaneShape.U_TURN_RIGHT -> "uturn_right"
        else -> "unknown"
    }

    /**
     * 從 StepInfo 取得轉向資訊（沿用原 NavDash 的對照）
     */
    private fun getManeuverInfoFromStep(step: StepInfo?): Pair<String, String> {
        if (step == null) return Pair(ManeuverType.ARRIVE, ManeuverModifier.STRAIGHT)

        val maneuver = step.maneuver

        val type = when (maneuver) {
            Maneuver.TURN_LEFT, Maneuver.TURN_SHARP_LEFT, Maneuver.TURN_SLIGHT_LEFT,
            Maneuver.TURN_RIGHT, Maneuver.TURN_SHARP_RIGHT, Maneuver.TURN_SLIGHT_RIGHT,
            Maneuver.TURN_KEEP_LEFT, Maneuver.TURN_KEEP_RIGHT,
            Maneuver.TURN_U_TURN_CLOCKWISE, Maneuver.TURN_U_TURN_COUNTERCLOCKWISE -> ManeuverType.TURN

            Maneuver.STRAIGHT -> ManeuverType.TURN

            Maneuver.ON_RAMP_LEFT, Maneuver.ON_RAMP_RIGHT,
            Maneuver.ON_RAMP_SHARP_LEFT, Maneuver.ON_RAMP_SHARP_RIGHT,
            Maneuver.ON_RAMP_SLIGHT_LEFT, Maneuver.ON_RAMP_SLIGHT_RIGHT,
            Maneuver.ON_RAMP_KEEP_LEFT, Maneuver.ON_RAMP_KEEP_RIGHT,
            Maneuver.ON_RAMP_UNSPECIFIED -> ManeuverType.ON_RAMP

            Maneuver.OFF_RAMP_LEFT, Maneuver.OFF_RAMP_RIGHT,
            Maneuver.OFF_RAMP_SHARP_LEFT, Maneuver.OFF_RAMP_SHARP_RIGHT,
            Maneuver.OFF_RAMP_SLIGHT_LEFT, Maneuver.OFF_RAMP_SLIGHT_RIGHT,
            Maneuver.OFF_RAMP_KEEP_LEFT, Maneuver.OFF_RAMP_KEEP_RIGHT,
            Maneuver.OFF_RAMP_UNSPECIFIED -> ManeuverType.OFF_RAMP

            Maneuver.FORK_LEFT, Maneuver.FORK_RIGHT -> ManeuverType.FORK

            Maneuver.MERGE_LEFT, Maneuver.MERGE_RIGHT, Maneuver.MERGE_UNSPECIFIED -> ManeuverType.MERGE

            Maneuver.ROUNDABOUT_CLOCKWISE, Maneuver.ROUNDABOUT_COUNTERCLOCKWISE,
            Maneuver.ROUNDABOUT_EXIT_CLOCKWISE, Maneuver.ROUNDABOUT_EXIT_COUNTERCLOCKWISE -> ManeuverType.ROUNDABOUT

            Maneuver.DESTINATION, Maneuver.DESTINATION_LEFT, Maneuver.DESTINATION_RIGHT -> ManeuverType.ARRIVE

            Maneuver.FERRY_BOAT, Maneuver.FERRY_TRAIN -> ManeuverType.FERRY

            Maneuver.DEPART -> ManeuverType.DEPART

            Maneuver.NAME_CHANGE -> ManeuverType.CONTINUE

            else -> ManeuverType.TURN
        }

        val modifier = when (maneuver) {
            Maneuver.TURN_SHARP_LEFT, Maneuver.ON_RAMP_SHARP_LEFT, Maneuver.OFF_RAMP_SHARP_LEFT -> ManeuverModifier.SHARP_LEFT

            Maneuver.TURN_SHARP_RIGHT, Maneuver.ON_RAMP_SHARP_RIGHT, Maneuver.OFF_RAMP_SHARP_RIGHT -> ManeuverModifier.SHARP_RIGHT

            Maneuver.TURN_SLIGHT_LEFT, Maneuver.ON_RAMP_SLIGHT_LEFT, Maneuver.OFF_RAMP_SLIGHT_LEFT,
            Maneuver.ON_RAMP_KEEP_LEFT, Maneuver.OFF_RAMP_KEEP_LEFT, Maneuver.FORK_LEFT -> ManeuverModifier.SLIGHT_LEFT

            Maneuver.TURN_SLIGHT_RIGHT, Maneuver.ON_RAMP_SLIGHT_RIGHT, Maneuver.OFF_RAMP_SLIGHT_RIGHT,
            Maneuver.ON_RAMP_KEEP_RIGHT, Maneuver.OFF_RAMP_KEEP_RIGHT, Maneuver.FORK_RIGHT -> ManeuverModifier.SLIGHT_RIGHT

            Maneuver.TURN_LEFT, Maneuver.ON_RAMP_LEFT, Maneuver.OFF_RAMP_LEFT,
            Maneuver.TURN_KEEP_LEFT, Maneuver.MERGE_LEFT, Maneuver.DESTINATION_LEFT -> ManeuverModifier.LEFT

            Maneuver.TURN_RIGHT, Maneuver.ON_RAMP_RIGHT, Maneuver.OFF_RAMP_RIGHT,
            Maneuver.TURN_KEEP_RIGHT, Maneuver.MERGE_RIGHT, Maneuver.DESTINATION_RIGHT -> ManeuverModifier.RIGHT

            Maneuver.TURN_U_TURN_CLOCKWISE, Maneuver.TURN_U_TURN_COUNTERCLOCKWISE,
            Maneuver.ON_RAMP_U_TURN_CLOCKWISE, Maneuver.ON_RAMP_U_TURN_COUNTERCLOCKWISE,
            Maneuver.OFF_RAMP_U_TURN_CLOCKWISE, Maneuver.OFF_RAMP_U_TURN_COUNTERCLOCKWISE -> ManeuverModifier.UTURN

            else -> ManeuverModifier.STRAIGHT
        }

        return Pair(type, modifier)
    }

    fun release() {
        if (isTurnByTurnRegistered) {
            try {
                navigator?.unregisterServiceForNavUpdates()
                isTurnByTurnRegistered = false
            } catch (e: Exception) {
                Log.e(TAG, "Failed to unregister Turn-by-Turn service", e)
            }
        }
        locationProvider?.removeLocationListener(locationListener)
        locationProvider = null
        navigator?.apply {
            removeArrivalListener(arrivalListener)
            removeRouteChangedListener(routeChangedListener)
            removeReroutingListener(reroutingListener)
            stopGuidance()
            clearDestinations()
        }
        navigator = null
        appContext = null
        isInitialized = false
        state = NavState.IDLE
    }
}
