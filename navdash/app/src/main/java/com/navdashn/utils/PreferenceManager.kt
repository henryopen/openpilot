package com.navdashn.utils

import android.content.Context
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import com.navdashn.navigation.PlaceItem
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

/**
 * DataStore 擴展
 */
private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "navdash_prefs")

/**
 * 偏好設定管理器
 */
class PreferenceManager(private val context: Context) {

    companion object {
        // Keys
        private val KEY_FROGPILOT_IP = stringPreferencesKey("frogpilot_ip")
        private val KEY_FROGPILOT_PORT = intPreferencesKey("frogpilot_port")
        private val KEY_API_KEY = stringPreferencesKey("api_key")
        private val KEY_FAVORITES = stringPreferencesKey("favorites")
        private val KEY_RECENTS = stringPreferencesKey("recents")
        private val KEY_AVOID_TOLLS = stringPreferencesKey("avoid_tolls")
        private val KEY_AVOID_HIGHWAYS = stringPreferencesKey("avoid_highways")

        // 預設值 (WebRTC port)
        private const val DEFAULT_PORT = 5001
        private const val MAX_RECENTS = 10
    }

    private val gson = Gson()
    private val placeListType = object : TypeToken<List<PlaceItem>>() {}.type

    // ==================== FrogPilot 設定 ====================

    /**
     * FrogPilot IP
     */
    val frogpilotIp: Flow<String> = context.dataStore.data.map { prefs ->
        prefs[KEY_FROGPILOT_IP] ?: ""
    }

    /**
     * 設定 FrogPilot IP
     */
    suspend fun setFrogpilotIp(ip: String) {
        context.dataStore.edit { prefs ->
            prefs[KEY_FROGPILOT_IP] = ip
        }
    }

    /**
     * FrogPilot Port
     */
    val frogpilotPort: Flow<Int> = context.dataStore.data.map { prefs ->
        prefs[KEY_FROGPILOT_PORT] ?: DEFAULT_PORT
    }

    /**
     * 設定 FrogPilot Port
     */
    suspend fun setFrogpilotPort(port: Int) {
        context.dataStore.edit { prefs ->
            prefs[KEY_FROGPILOT_PORT] = port
        }
    }

    // ==================== API 設定 ====================

    /**
     * Google API Key
     */
    val apiKey: Flow<String> = context.dataStore.data.map { prefs ->
        prefs[KEY_API_KEY] ?: ""
    }

    /**
     * 設定 API Key
     */
    suspend fun setApiKey(key: String) {
        context.dataStore.edit { prefs ->
            prefs[KEY_API_KEY] = key
        }
    }

    // ==================== 地點管理 ====================

    /**
     * 常用地點
     */
    val favorites: Flow<List<PlaceItem>> = context.dataStore.data.map { prefs ->
        val jsonString = prefs[KEY_FAVORITES] ?: "[]"
        try {
            gson.fromJson<List<PlaceItem>>(jsonString, placeListType) ?: emptyList()
        } catch (e: Exception) {
            emptyList()
        }
    }

    /**
     * 新增常用地點
     */
    suspend fun addFavorite(place: PlaceItem) {
        context.dataStore.edit { prefs ->
            val current: List<PlaceItem> = try {
                gson.fromJson(prefs[KEY_FAVORITES] ?: "[]", placeListType) ?: emptyList()
            } catch (e: Exception) {
                emptyList()
            }

            // 避免重複
            val filtered = current.filter { item -> item.id != place.id }
            val updated = listOf(place) + filtered

            prefs[KEY_FAVORITES] = gson.toJson(updated)
        }
    }

    /**
     * 移除常用地點
     */
    suspend fun removeFavorite(placeId: String) {
        context.dataStore.edit { prefs ->
            val current: List<PlaceItem> = try {
                gson.fromJson(prefs[KEY_FAVORITES] ?: "[]", placeListType) ?: emptyList()
            } catch (e: Exception) {
                emptyList()
            }

            val updated = current.filter { item -> item.id != placeId }
            prefs[KEY_FAVORITES] = gson.toJson(updated)
        }
    }

    /**
     * 最近導航
     */
    val recents: Flow<List<PlaceItem>> = context.dataStore.data.map { prefs ->
        val jsonString = prefs[KEY_RECENTS] ?: "[]"
        try {
            gson.fromJson<List<PlaceItem>>(jsonString, placeListType) ?: emptyList()
        } catch (e: Exception) {
            emptyList()
        }
    }

    /**
     * 新增最近導航
     */
    suspend fun addRecent(place: PlaceItem) {
        context.dataStore.edit { prefs ->
            val current: List<PlaceItem> = try {
                gson.fromJson(prefs[KEY_RECENTS] ?: "[]", placeListType) ?: emptyList()
            } catch (e: Exception) {
                emptyList()
            }

            // 避免重複，並限制數量
            val filtered = current.filter { item -> item.id != place.id }
            val updated = (listOf(place.copy(timestamp = System.currentTimeMillis())) + filtered)
                .take(MAX_RECENTS)

            prefs[KEY_RECENTS] = gson.toJson(updated)
        }
    }

    /**
     * 清除最近導航
     */
    suspend fun clearRecents() {
        context.dataStore.edit { prefs ->
            prefs[KEY_RECENTS] = "[]"
        }
    }

    // ==================== 導航偏好 ====================

    /**
     * 避開收費道路
     */
    val avoidTolls: Flow<Boolean> = context.dataStore.data.map { prefs ->
        prefs[KEY_AVOID_TOLLS] == "true"
    }

    /**
     * 設定避開收費道路
     */
    suspend fun setAvoidTolls(avoid: Boolean) {
        context.dataStore.edit { prefs ->
            prefs[KEY_AVOID_TOLLS] = avoid.toString()
        }
    }

    /**
     * 避開高速公路
     */
    val avoidHighways: Flow<Boolean> = context.dataStore.data.map { prefs ->
        prefs[KEY_AVOID_HIGHWAYS] == "true"
    }

    /**
     * 設定避開高速公路
     */
    suspend fun setAvoidHighways(avoid: Boolean) {
        context.dataStore.edit { prefs ->
            prefs[KEY_AVOID_HIGHWAYS] = avoid.toString()
        }
    }
}
