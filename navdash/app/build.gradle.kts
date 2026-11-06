import java.util.Properties

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

// 讀取 local.properties 中的 API Key
val localProperties = Properties()
val localPropertiesFile = rootProject.file("local.properties")
if (localPropertiesFile.exists()) {
    localPropertiesFile.inputStream().use { localProperties.load(it) }
}

android {
    namespace = "com.navdashn"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.navdashn"
        minSdk = 26
        targetSdk = 34
        versionCode = 9
        versionName = "2.5.1"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"

        // 從 local.properties 讀取 API Key，注入到 Manifest
        val mapsApiKey = localProperties.getProperty("MAPS_API_KEY") ?: ""
        manifestPlaceholders["MAPS_API_KEY"] = mapsApiKey
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }
    buildFeatures {
        viewBinding = true
    }
}

configurations.all {
    // 排除 play-services-maps，因為 Navigation SDK 已經包含
    exclude(group = "com.google.android.gms", module = "play-services-maps")
}

dependencies {
    // Android Core
    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    implementation("androidx.activity:activity-ktx:1.8.2")

    // Navigation SDK (Google Maps)
    // 注意：需要在 Google Cloud Console 啟用 Navigation SDK 並取得 API Key
    // Navigation SDK 已經包含 Maps SDK，不需另外加入
    implementation("com.google.android.libraries.navigation:navigation:5.0.0")

    // Places SDK (地點搜尋) - 排除內建的 maps 依賴
    implementation("com.google.android.libraries.places:places:3.5.0")

    // HTTP Server (備用相容 NavBridge)
    implementation("org.nanohttpd:nanohttpd:2.3.1")

    // WebRTC (直連 webrtcd)

    // OkHttp (用於 WebRTC signaling)

    // JSON
    implementation("com.google.code.gson:gson:2.10.1")

    // Coroutines
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-android:1.7.3")
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.7.3")

    // Lifecycle
    implementation("androidx.lifecycle:lifecycle-viewmodel-ktx:2.7.0")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.7.0")
    implementation("androidx.lifecycle:lifecycle-livedata-ktx:2.7.0")

    // DataStore (設定儲存)
    implementation("androidx.datastore:datastore-preferences:1.0.0")

    // RecyclerView
    implementation("androidx.recyclerview:recyclerview:1.3.2")

    // Testing
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
}
