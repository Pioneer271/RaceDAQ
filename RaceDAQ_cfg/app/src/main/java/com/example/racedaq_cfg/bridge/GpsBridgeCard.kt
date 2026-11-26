package com.example.racedaq_cfg.bridge

import android.Manifest
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.provider.Settings
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner

@Composable
fun GpsBridgeCard(modifier: Modifier = Modifier) {
    val context = LocalContext.current
    val lifecycleOwner = LocalLifecycleOwner.current

    LaunchedEffect(Unit) {
        BridgeStateHolder.init(context)
    }

    val state by BridgeStateHolder.state.collectAsState()
    var pendingAutoStart by remember { mutableStateOf(false) }

    val requiredPerms = remember {
        buildList {
            add(Manifest.permission.ACCESS_FINE_LOCATION)
            add(Manifest.permission.ACCESS_COARSE_LOCATION)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                add(Manifest.permission.POST_NOTIFICATIONS)
            }
        }.toTypedArray()
    }

    val permLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestMultiplePermissions()
    ) { grants ->
        val locationGranted = grants[Manifest.permission.ACCESS_FINE_LOCATION] == true ||
                ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
        if (locationGranted) {
            toggleBridgeService(context, true)
        } else {
            pendingAutoStart = false
            BridgeStateHolder.update {
                it.copy(isRunning = false, errorMessage = "Разрешите доступ к местоположению для работы GPS-моста")
            }
        }
    }

    fun startWithPermissionCheck() {
        val hasLocation = ContextCompat.checkSelfPermission(
            context, Manifest.permission.ACCESS_FINE_LOCATION
        ) == PackageManager.PERMISSION_GRANTED

        if (hasLocation) {
            toggleBridgeService(context, true)
        } else {
            permLauncher.launch(requiredPerms)
        }
    }

    DisposableEffect(lifecycleOwner, pendingAutoStart) {
        val observer = LifecycleEventObserver { _, event ->
            if (event == Lifecycle.Event.ON_RESUME && pendingAutoStart && !state.isRunning) {
                pendingAutoStart = false
                startWithPermissionCheck()
            }
        }
        lifecycleOwner.lifecycle.addObserver(observer)
        onDispose { lifecycleOwner.lifecycle.removeObserver(observer) }
    }

    val hasError = state.errorMessage != null
    val borderColor = when {
        hasError -> Color(0xFFFF5252)
        state.isRunning -> Color(0xFF00E676)
        else -> Color(0xFF2A313D)
    }

    Column(
        modifier = modifier
            .fillMaxWidth()
            .border(1.dp, borderColor, RoundedCornerShape(8.dp))
            .padding(12.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column(modifier = Modifier.weight(1f)) {
                Text(
                    text = "СЕРВИС ФИКТИВНЫХ МЕСТОПОЛОЖЕНИЙ",
                    color = Color(0xFF4FC3F7),
                    fontWeight = FontWeight.Bold,
                    fontSize = 14.sp
                )
                Spacer(modifier = Modifier.height(2.dp))
                Text(
                    text = when {
                        state.isRunning -> "● АКТИВЕН: ${"%.1f".format(state.updateRateHz)} Гц • Спутников: ${state.satellites}"
                        hasError -> "⚠ ${state.errorMessage}"
                        else -> "○ ВЫКЛЮЧЕН (передача внешнего GPS в RaceChrono)"
                    },
                    color = when {
                        state.isRunning -> Color(0xFF00E676)
                        hasError -> Color(0xFFFF5252)
                        else -> Color(0xFF90A4AE)
                    },
                    fontWeight = FontWeight.SemiBold,
                    fontSize = 12.sp
                )
            }

            Switch(
                checked = state.isRunning,
                onCheckedChange = { enable ->
                    if (enable) {
                        pendingAutoStart = true
                        startWithPermissionCheck()
                    } else {
                        pendingAutoStart = false
                        toggleBridgeService(context, false)
                    }
                },
                colors = SwitchDefaults.colors(
                    checkedThumbColor = Color.White,
                    checkedTrackColor = Color(0xFF00C853),
                    uncheckedThumbColor = Color(0xFFB0BEC5),
                    uncheckedTrackColor = Color(0xFF263238)
                )
            )
        }

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            BridgeProtocol.entries.forEach { proto ->
                val selected = state.protocol == proto
                Box(
                    modifier = Modifier
                        .weight(1f)
                        .clip(RoundedCornerShape(8.dp))
                        .background(if (selected) Color(0xFF0277BD) else Color(0xFF1E242D))
                        .border(
                            1.dp,
                            if (selected) Color(0xFF4FC3F7) else Color(0xFF37474F),
                            RoundedCornerShape(8.dp)
                        )
                        .clickable { BridgeStateHolder.setProtocol(context, proto) }
                        .padding(vertical = 8.dp, horizontal = 10.dp),
                    contentAlignment = Alignment.Center
                ) {
                    Text(
                        text = proto.title,
                        color = Color.White,
                        fontSize = 12.sp,
                        fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal,
                        maxLines = 1
                    )
                }
            }
        }

        if (hasError) {
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .background(Color(0xFF2B1215), RoundedCornerShape(8.dp))
                    .padding(10.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Text(
                    text = "Если разрешение на геолокацию выдано, выберите приложение для подмены GPS:\n" +
                            "1. Нажмите кнопку ниже.\n" +
                            "2. Откройте «Выбрать приложение для фиктивных местоположений».\n" +
                            "3. Выберите RaceDAQ_cfg и вернитесь назад.",
                    color = Color(0xFFFFCDD2),
                    fontSize = 12.sp
                )
                Button(
                    onClick = {
                        pendingAutoStart = true
                        openDeveloperOrDeviceSettings(context)
                    },
                    modifier = Modifier.fillMaxWidth(),
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFD32F2F))
                ) {
                    Text(
                        text = "ОТКРЫТЬ НАСТРОЙКИ РАЗРАБОТЧИКА",
                        color = Color.White,
                        fontWeight = FontWeight.Bold,
                        fontSize = 12.sp
                    )
                }
            }
        } else {
            Text(
                text = "После включения откройте RaceChrono и выберите тип приёмника «Встроенный GPS».",
                color = Color(0xFF90A4AE),
                fontSize = 12.sp
            )
        }
    }
}

private fun openDeveloperOrDeviceSettings(context: Context) {
    val devIntent = Intent(Settings.ACTION_APPLICATION_DEVELOPMENT_SETTINGS).apply {
        flags = Intent.FLAG_ACTIVITY_NEW_TASK
    }
    runCatching {
        context.startActivity(devIntent)
    }.onFailure {
        val aboutIntent = Intent(Settings.ACTION_DEVICE_INFO_SETTINGS).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK
        }
        runCatching { context.startActivity(aboutIntent) }
    }
}

private fun toggleBridgeService(context: Context, enable: Boolean) {
    val intent = Intent(context, RaceGpsBridgeService::class.java)
    if (enable) {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            context.startForegroundService(intent)
        } else {
            context.startService(intent)
        }
    } else {
        context.stopService(intent)
    }
}