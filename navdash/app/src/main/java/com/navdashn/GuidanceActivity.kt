package com.navdashn

import android.os.Bundle
import android.view.WindowManager
import android.widget.ImageButton
import androidx.appcompat.app.AppCompatActivity
import com.google.android.libraries.navigation.SupportNavigationFragment
import com.navdashn.navigation.NavController
import com.navdashn.navigation.NavState

/**
 * Google 導航畫面（地圖、轉彎提示、車道）。
 *
 * Manifest 宣告了完整的 configChanges：D531 換解析度/方向時 Activity 不會重建，
 * 導航畫面跟著新尺寸重新排版，引導不中斷。
 */
class GuidanceActivity : AppCompatActivity() {

    private var navFragment: SupportNavigationFragment? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_guidance)

        navFragment = supportFragmentManager.findFragmentById(R.id.navFragmentContainer) as? SupportNavigationFragment
        if (navFragment == null) {
            val fragment = SupportNavigationFragment.newInstance()
            supportFragmentManager.beginTransaction()
                .replace(R.id.navFragmentContainer, fragment)
                .commitNow()
            navFragment = fragment
        }

    }

    private var controlsWired = false

    override fun onStart() {
        super.onStart()
        // 2.5.0：圖示鍵直接疊在 activity_guidance 左側（不再用 setCustomControl 佔地圖下方）
        if (controlsWired) return
        findViewById<ImageButton>(R.id.guidanceBackButton).setOnClickListener { finish() }
        findViewById<ImageButton>(R.id.guidanceHelpButton).setOnClickListener {
            startActivity(android.content.Intent(this, HelpActivity::class.java))
        }
        // 導航語音開／關（11/02 使用者：「導航語音要可以關閉」）
        val voice = findViewById<ImageButton>(R.id.guidanceVoiceButton)
        showVoice(voice, NavController.isVoiceOn())
        voice.setOnClickListener {
            val on = !NavController.isVoiceOn()
            NavController.setVoiceOn(on)
            showVoice(voice, on)
        }
        findViewById<ImageButton>(R.id.guidanceStopButton).setOnClickListener {
            NavController.stopNavigation()
            finish()
        }
        // 即時路況（道路紅黃綠）。10/29 使用者：「顯示的地圖沒有路況的顯示」
        navFragment?.getMapAsync { map -> map.isTrafficEnabled = true }
        fitControls(resources.configuration.orientation)
        controlsWired = true
    }

    // configChanges 全列＝轉向不重建 Activity，values-land 不會自己套用，要在這裡手動換尺寸
    override fun onConfigurationChanged(newConfig: android.content.res.Configuration) {
        super.onConfigurationChanged(newConfig)
        fitControls(newConfig.orientation)
    }

    /** 直式 48dp／間距 8dp；橫式地圖只剩約 260dp 高，縮成 40dp／間距 6dp，才不會壓到轉向指示卡和 Google 標誌 */
    private fun fitControls(orientation: Int) {
        val land = orientation == android.content.res.Configuration.ORIENTATION_LANDSCAPE
        val dp = resources.displayMetrics.density
        val size = ((if (land) 40 else 48) * dp).toInt()
        val gap = ((if (land) 6 else 8) * dp).toInt()
        val ids = intArrayOf(R.id.guidanceBackButton, R.id.guidanceHelpButton, R.id.guidanceVoiceButton, R.id.guidanceStopButton)
        ids.forEachIndexed { i, id ->
            val b = findViewById<ImageButton>(id)
            val lp = b.layoutParams as android.widget.LinearLayout.LayoutParams
            lp.width = size
            lp.height = size
            lp.topMargin = if (i == 0) 0 else gap
            b.layoutParams = lp
            val pad = (((if (id == R.id.guidanceStopButton) 12 else 11) * dp) * size / (48 * dp)).toInt()
            b.setPadding(pad, pad, pad, pad)
        }
        // 橫式：整排往下 16dp，跟轉向指示卡（含「Then」小卡）隔開
        val column = findViewById<android.view.View>(R.id.guidanceControls)
        val clp = column.layoutParams as android.widget.FrameLayout.LayoutParams
        clp.topMargin = if (land) (16 * dp).toInt() else 0
        column.layoutParams = clp
        // 橫式：地圖左側讓出圖示欄寬度，Google 標誌移到圖示右邊不被蓋住；直式標誌在圖示下方，不用讓
        val left = if (land) size + (12 * dp).toInt() else 0
        navFragment?.getMapAsync { map -> map.setPadding(left, 0, 0, 0) }
    }

    /** 語音鍵：開＝白色喇叭，關＝灰色靜音喇叭；contentDescription 跟著改（手機夥計靠它認） */
    private fun showVoice(button: ImageButton, on: Boolean) {
        button.setImageResource(if (on) R.drawable.ic_volume_up else R.drawable.ic_volume_off)
        button.contentDescription = getString(if (on) R.string.voice_on else R.string.voice_off)
    }

    override fun onResume() {
        super.onResume()
        // 導航已結束（到達/停止）就回主畫面
        val state = NavController.getState()
        if (state == NavState.IDLE || state == NavState.ERROR) finish()
    }
}
