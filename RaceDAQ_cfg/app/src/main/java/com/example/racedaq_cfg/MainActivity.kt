package com.example.racedaq_cfg

import android.Manifest
import android.content.Context
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.imePadding
import androidx.compose.foundation.layout.systemBarsPadding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.core.content.ContextCompat

object BleManagerHolder {
    @Volatile
    private var instance: RaceDaqBleManager? = null

    fun get(context: Context): RaceDaqBleManager {
        return instance ?: synchronized(this) {
            instance ?: RaceDaqBleManager(context.applicationContext).also { instance = it }
        }
    }
}

object SensorProfileStorage {
    private const val PREFS_NAME = "racedaq_sensor_profiles"

    fun loadProfile(context: Context, prefix: String, defaultProfile: AnalogSensorProfile): AnalogSensorProfile {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        return AnalogSensorProfile(
            name = prefs.getString("${prefix}_name", defaultProfile.name) ?: defaultProfile.name,
            unit = prefs.getString("${prefix}_unit", defaultProfile.unit) ?: defaultProfile.unit,
            v1Mv = prefs.getInt("${prefix}_v1", defaultProfile.v1Mv),
            val1 = prefs.getFloat("${prefix}_val1", defaultProfile.val1),
            v2Mv = prefs.getInt("${prefix}_v2", defaultProfile.v2Mv),
            val2 = prefs.getFloat("${prefix}_val2", defaultProfile.val2)
        )
    }

    fun saveProfile(context: Context, prefix: String, profile: AnalogSensorProfile) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE).edit()
            .putString("${prefix}_name", profile.name)
            .putString("${prefix}_unit", profile.unit)
            .putInt("${prefix}_v1", profile.v1Mv)
            .putFloat("${prefix}_val1", profile.val1)
            .putInt("${prefix}_v2", profile.v2Mv)
            .putFloat("${prefix}_val2", profile.val2)
            .apply()
    }
    private fun SharedPreferences.Editor.putNullableFloat(key: String, value: Float?): SharedPreferences.Editor {
        return if (value != null) putFloat(key, value) else remove(key)
    }

    private fun loadLimit(prefs: android.content.SharedPreferences, prefix: String, def: AlarmLimit): AlarmLimit {
        val enabled = prefs.getBoolean("${prefix}_en", def.enabled)
        val min = if (prefs.contains("${prefix}_min")) prefs.getFloat("${prefix}_min", 0f) else if (!prefs.contains("${prefix}_en")) def.min else null
        val max = if (prefs.contains("${prefix}_max")) prefs.getFloat("${prefix}_max", 0f) else if (!prefs.contains("${prefix}_en")) def.max else null
        return AlarmLimit(enabled = enabled, min = min, max = max)
    }

    private fun saveLimit(editor: android.content.SharedPreferences.Editor, prefix: String, limit: AlarmLimit) {
        editor.putBoolean("${prefix}_en", limit.enabled)
        editor.putNullableFloat("${prefix}_min", limit.min)
        editor.putNullableFloat("${prefix}_max", limit.max)
    }

    fun loadAlarmLimits(context: Context): DashboardAlarmLimits {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        val def = DashboardAlarmLimits()
        return DashboardAlarmLimits(
            tempWat = loadLimit(prefs, "lim_wat", def.tempWat),
            tempOil = loadLimit(prefs, "lim_oil", def.tempOil),
            egt = loadLimit(prefs, "lim_egt", def.egt),
            ain1 = loadLimit(prefs, "lim_ain1", def.ain1),
            ain2 = loadLimit(prefs, "lim_ain2", def.ain2),
            ign = loadLimit(prefs, "lim_ign", def.ign),
            shiftRpm = prefs.getInt("lim_shift_rpm", def.shiftRpm),
            maxRpm = prefs.getInt("lim_max_rpm", def.maxRpm)
        )
    }

    fun saveAlarmLimits(context: Context, limits: DashboardAlarmLimits) {
        val editor = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE).edit()
        saveLimit(editor, "lim_wat", limits.tempWat)
        saveLimit(editor, "lim_oil", limits.tempOil)
        saveLimit(editor, "lim_egt", limits.egt)
        saveLimit(editor, "lim_ain1", limits.ain1)
        saveLimit(editor, "lim_ain2", limits.ain2)
        saveLimit(editor, "lim_ign", limits.ign)
        editor.putInt("lim_shift_rpm", limits.shiftRpm)
        editor.putInt("lim_max_rpm", limits.maxRpm)
        editor.apply()
    }
    fun loadDashboardVisibility(context: Context): DashboardVisibility {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        return DashboardVisibility(
            showRpm = prefs.getBoolean("vis_rpm", true),
            showTempWat = prefs.getBoolean("vis_wat", true),
            showTempOil = prefs.getBoolean("vis_oil", true),
            showEgt = prefs.getBoolean("vis_egt", true),
            showAin1 = prefs.getBoolean("vis_ain1", true),
            showAin2 = prefs.getBoolean("vis_ain2", true),
            showGps = prefs.getBoolean("vis_gps", true),
            showDout = prefs.getBoolean("vis_dout", true)
        )
    }

    fun saveDashboardVisibility(context: Context, vis: DashboardVisibility) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE).edit()
            .putBoolean("vis_rpm", vis.showRpm)
            .putBoolean("vis_wat", vis.showTempWat)
            .putBoolean("vis_oil", vis.showTempOil)
            .putBoolean("vis_egt", vis.showEgt)
            .putBoolean("vis_ain1", vis.showAin1)
            .putBoolean("vis_ain2", vis.showAin2)
            .putBoolean("vis_gps", vis.showGps)
            .putBoolean("vis_dout", vis.showDout)
            .apply()
    }
}

class MainActivity : ComponentActivity() {

    private lateinit var bleManager: RaceDaqBleManager

    override fun onCreate(savedInstanceState: Bundle?) {
        enableEdgeToEdge()
        super.onCreate(savedInstanceState)

        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        bleManager = BleManagerHolder.get(this)

        setContent {
            MaterialTheme(colorScheme = darkColorScheme()) {
                Box(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(Color(0xFF0D0F12))
                        .systemBarsPadding()
                        .imePadding()
                ) {
                    RaceDaqApp(
                        context = this@MainActivity,
                        bleManager = bleManager
                    )
                }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        if (!bleManager.telemetry.value.connected && hasBlePermissions()) {
            bleManager.startAutoConnect()
        }
    }

    private fun hasBlePermissions(): Boolean {
        val perms = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            )
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }
        return perms.all {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
    }
}

@Composable
private fun RaceDaqApp(
    context: Context,
    bleManager: RaceDaqBleManager
) {
    val requiredPermissions = remember {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        }
    }

    val permissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        if (grants.values.all { it }) {
            bleManager.startAutoConnect()
        }
    }

    LaunchedEffect(Unit) {
        val allGranted = requiredPermissions.all {
            ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
        }
        if (allGranted) {
            if (!bleManager.telemetry.value.connected) {
                bleManager.startAutoConnect()
            }
        } else {
            permissionLauncher.launch(requiredPermissions)
        }
    }

    var showConfig by remember { mutableStateOf(false) }

    var ain1Profile by remember {
        mutableStateOf(
            SensorProfileStorage.loadProfile(
                context = context,
                prefix = "ain1",
                defaultProfile = AnalogSensorProfile("Давление масла", "бар", 500, 0.0f, 4500, 10.0f)
            )
        )
    }

    var ain2Profile by remember {
        mutableStateOf(
            SensorProfileStorage.loadProfile(
                context = context,
                prefix = "ain2",
                defaultProfile = AnalogSensorProfile("Буст", "бар", 500, -1.0f, 4500, 2.0f)
            )
        )
    }
    var alarmLimits by remember {
        mutableStateOf(SensorProfileStorage.loadAlarmLimits(context))
    }
    var dashboardVisibility by remember {
        mutableStateOf(SensorProfileStorage.loadDashboardVisibility(context))
    }

    val telemetry by bleManager.telemetry.collectAsState()
    val sysConfig by bleManager.config.collectAsState()
    val gpsHz by bleManager.gpsRateHz.collectAsState()
    val deviceInfo by bleManager.deviceInfo.collectAsState()

    if (showConfig) {
        ConfigScreen(
            telemetry = telemetry,
            currentConfig = sysConfig,
            currentGpsHz = gpsHz,
            ain1Profile = ain1Profile,
            ain2Profile = ain2Profile,
            alarmLimits = alarmLimits,
            visibility = dashboardVisibility,
            deviceInfo = deviceInfo,
            onSaveSystemConfig = { newCfg ->
                bleManager.writeSystemConfig(newCfg)
            },
            onSaveAinProfiles = { newAin1, newAin2 ->
                ain1Profile = newAin1
                ain2Profile = newAin2
                SensorProfileStorage.saveProfile(context, "ain1", newAin1)
                SensorProfileStorage.saveProfile(context, "ain2", newAin2)
            },
            onSaveAlarmLimits = { newLimits ->
                alarmLimits = newLimits
                SensorProfileStorage.saveAlarmLimits(context, newLimits)
            },
            onSaveVisibility = { newVis ->
                dashboardVisibility = newVis
                SensorProfileStorage.saveDashboardVisibility(context, newVis)
            },
            onSetGpsRate = { hz ->
                bleManager.setGpsRate(hz)
            },
            onBack = { showConfig = false }
        )
    } else {
        DashboardScreen(
            state = telemetry,
            config = sysConfig,
            ain1Profile = ain1Profile,
            ain2Profile = ain2Profile,
            alarmLimits = alarmLimits,
            visibility = dashboardVisibility,
            deviceInfo = deviceInfo,
            onToggleDout = { channel, enable ->
                bleManager.toggleDout(channel, enable)
            },
            onOpenConfig = { showConfig = true }
        )
    }
}