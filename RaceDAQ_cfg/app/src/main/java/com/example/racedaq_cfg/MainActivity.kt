package com.example.racedaq_cfg

import android.Manifest
import android.content.Context
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

object SensorProfileStorage {
    private const val PREFS_NAME = "racedaq_sensor_profiles"

    fun loadProfile(context: Context, prefix: String, defaultProfile: AnalogSensorProfile): AnalogSensorProfile {
        val prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
        return AnalogSensorProfile(
            name = prefs.getString("\${prefix}_name", defaultProfile.name) ?: defaultProfile.name,
            unit = prefs.getString("\${prefix}_unit", defaultProfile.unit) ?: defaultProfile.unit,
            v1Mv = prefs.getInt("\${prefix}_v1", defaultProfile.v1Mv),
            val1 = prefs.getFloat("\${prefix}_val1", defaultProfile.val1),
            v2Mv = prefs.getInt("\${prefix}_v2", defaultProfile.v2Mv),
            val2 = prefs.getFloat("\${prefix}_val2", defaultProfile.val2)
        )
    }

    fun saveProfile(context: Context, prefix: String, profile: AnalogSensorProfile) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE).edit()
            .putString("\${prefix}_name", profile.name)
            .putString("\${prefix}_unit", profile.unit)
            .putInt("\${prefix}_v1", profile.v1Mv)
            .putFloat("\${prefix}_val1", profile.val1)
            .putInt("\${prefix}_v2", profile.v2Mv)
            .putFloat("\${prefix}_val2", profile.val2)
            .apply()
    }
}

class MainActivity : ComponentActivity() {

    private lateinit var bleManager: RaceDaqBleManager

    override fun onCreate(savedInstanceState: Bundle?) {
        enableEdgeToEdge()
        super.onCreate(savedInstanceState)

        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        bleManager = RaceDaqBleManager(applicationContext)

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
            bleManager.startAutoConnect()
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
                defaultProfile = AnalogSensorProfile("Наддув (MAP)", "бар", 500, -1.0f, 4500, 2.0f)
            )
        )
    }

    val telemetry by bleManager.telemetry.collectAsState()
    val sysConfig by bleManager.config.collectAsState()
    val gpsHz by bleManager.gpsRateHz.collectAsState()

    if (showConfig) {
        ConfigScreen(
            telemetry = telemetry,
            currentConfig = sysConfig,
            currentGpsHz = gpsHz,
            ain1Profile = ain1Profile,
            ain2Profile = ain2Profile,
            onSaveSystemConfig = { newCfg ->
                bleManager.writeSystemConfig(newCfg)
            },
            onSaveAinProfiles = { newAin1, newAin2 ->
                ain1Profile = newAin1
                ain2Profile = newAin2
                SensorProfileStorage.saveProfile(context, "ain1", newAin1)
                SensorProfileStorage.saveProfile(context, "ain2", newAin2)
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
            onToggleDout = { channel, enable ->
                bleManager.toggleDout(channel, enable)
            },
            onOpenConfig = { showConfig = true }
        )
    }
}