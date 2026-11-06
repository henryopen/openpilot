package com.navdashn

import android.annotation.SuppressLint
import android.net.Uri
import android.os.Bundle
import android.webkit.WebResourceError
import android.webkit.WebResourceRequest
import android.webkit.WebResourceResponse
import android.webkit.WebView
import android.webkit.WebViewClient
import androidx.activity.OnBackPressedCallback
import androidx.appcompat.app.AppCompatActivity

/**
 * 藍牙鍵盤／C3 語音說明。
 * 2026-10-26：內建在 App 的 assets/help.html。
 * 2026-10-27 使用者：「改用個網站的連結，不然改圖又要重新安裝 APK」→ 先載入網站上的版本（GitHub Pages，
 * 原始檔在 openpilot repo 的 openpilot/hud/navdash_help.html），沒網路或網站開不了才退回內建的 assets/help.html。
 * 2026-11-06 使用者：「我們自己的對外網站那麼多說明頁為什麼就一定要連GITHUB」→ 改放公司網站 www.motoway.tw/navdash/
 *   （原始檔不變，說明頁原始檔在 openpilot repo 的 openpilot/hud/navdash_help.html）。
 * 只允許開說明網站和內建頁，其他連結一律不開（這頁有開 JavaScript）。
 */
class HelpActivity : AppCompatActivity() {

    companion object {
        const val HELP_URL = "https://www.motoway.tw/navdash/index.html"
        private const val HELP_HOST = "www.motoway.tw"
        private const val HELP_PATH = "/navdash/"
        private const val FALLBACK_URL = "file:///android_asset/help.html"
    }

    private lateinit var web: WebView
    private var fellBack = false

    private fun allowed(uri: Uri): Boolean =
        (uri.scheme == "https" && uri.host == HELP_HOST && (uri.path ?: "").startsWith(HELP_PATH)) ||
            uri.toString().startsWith("file:///android_asset/")

    private fun fallBack(view: WebView) {
        if (fellBack) return
        fellBack = true
        view.stopLoading()
        view.loadUrl(FALLBACK_URL)
    }

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        web = WebView(this).apply {
            setBackgroundColor(0xFF0E1116.toInt())
            settings.javaScriptEnabled = true          // 分頁切換、點按鍵看說明；只載入說明網站與 App 自己的 asset
            settings.allowFileAccess = false
            settings.allowContentAccess = false
            webViewClient = object : WebViewClient() {
                override fun shouldOverrideUrlLoading(view: WebView, request: WebResourceRequest): Boolean =
                    !allowed(request.url)              // true＝攔下不開

                // 沒網路、DNS 失敗、逾時：主頁載不到就改看內建版
                override fun onReceivedError(view: WebView, request: WebResourceRequest, error: WebResourceError) {
                    if (request.isForMainFrame && request.url.toString().startsWith(HELP_URL)) fallBack(view)
                }

                // 網站回 404／5xx（例如頁面被移走）也改看內建版
                override fun onReceivedHttpError(view: WebView, request: WebResourceRequest, errorResponse: WebResourceResponse) {
                    if (request.isForMainFrame && errorResponse.statusCode >= 400) fallBack(view)
                }
            }
        }
        setContentView(web)
        if (savedInstanceState == null) web.loadUrl(HELP_URL)
        else web.restoreState(savedInstanceState)

        onBackPressedDispatcher.addCallback(this, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() = finish()
        })
    }

    override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        web.saveState(outState)
    }

    override fun onDestroy() {
        web.destroy()
        super.onDestroy()
    }
}
