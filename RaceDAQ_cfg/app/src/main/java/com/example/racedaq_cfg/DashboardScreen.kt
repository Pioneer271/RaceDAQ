package com.example.racedaq_cfg

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
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp


@Composable
fun DashboardScreen(
    state: TelemetryState,
    config: SystemConfig,
    ain1Profile: AnalogSensorProfile,
    ain2Profile: AnalogSensorProfile,
    onToggleDout: (channel: Int, enable: Boolean) -> Unit,
    onOpenConfig: () -> Unit
) {
    val oilPressure = ain1Profile.mvToValue(state.mvAin1)
    val boostBar = ain2Profile.mvToValue(state.mvAin2)

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0D0F12))
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        // 1. ВЕРХНЯЯ ПАНЕЛЬ: ШКАЛА ОБОРОТОВ + СТАТУСЫ
        RpmShiftBar(rpm = state.rpm, maxRpm = 8000, shiftRpm = 6800)

        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = if (state.connected) "● RACEDAQ ONLINE" else "○ ПОИСК RACEDAQ...",
                color = if (state.connected) Color(0xFF00E676) else Color(0xFFFF5252),
                fontWeight = FontWeight.Bold,
                fontSize = 14.sp
            )
            Text(
                text = "GPS: ${state.gpsSats} спутн. | IGN: ${"%.1f".format(state.ignVolts)} В",
                color = Color(0xFFB0BEC5),
                fontSize = 14.sp
            )
            Button(
                onClick = onOpenConfig,
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF263238))
            ) {
                Text("⚙ НАСТРОЙКА ВХОДОВ И СЦЕНАРИЕВ", color = Color.White)
            }
        }

        // 2. ОСНОВНАЯ СЕТКА КРУПНЫХ ПРИБОРОВ (2 РЯДА ПО 3 ПЛИТКИ)
        Row(
            modifier = Modifier.weight(1f).fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            GaugeTile(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                title = "ТЕМП. ОЖ (T_WATR)",
                value = if (state.tempWatrC <= -39f) "ОБРЫВ" else "${"%.1f".format(state.tempWatrC)}",
                unit = "°C",
                isAlarm = state.tempWatrC > 102f
            )
            GaugeTile(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                title = "ТЕМП. МАСЛА (T_OIL)",
                value = if (state.tempOilC <= -39f) "ОБРЫВ" else "${"%.1f".format(state.tempOilC)}",
                unit = "°C",
                isAlarm = state.tempOilC > 125f
            )
            GaugeTile(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                title = "ВЫХЛОП (EGT)",
                value = state.egtC?.let { "${it.toInt()}" } ?: "ОБРЫВ",
                unit = "°C",
                isAlarm = (state.egtC ?: 0f) > 900f
            )
        }

        Row(
            modifier = Modifier.weight(1f).fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            GaugeTile(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                title = ain1Profile.name.uppercase(),
                value = "${"%.2f".format(oilPressure)}",
                unit = ain1Profile.unit,
                isAlarm = state.rpm > 1500 && oilPressure < 1.2f
            )
            GaugeTile(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                title = ain2Profile.name.uppercase(),
                value = "${"%.2f".format(boostBar)}",
                unit = ain2Profile.unit,
                isAlarm = false
            )
            GaugeTile(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                title = "СКОРОСТЬ GPS / G-FORCE",
                value = "${state.gpsSpeedKmh.toInt()}",
                unit = "км/ч  (${"%.2f".format(state.accXg)}G / ${"%.2f".format(state.accYg)}G)",
                isAlarm = false
            )
        }

        // 3. НИЖНЯЯ ПАНЕЛЬ УПРАВЛЕНИЯ СИЛОВЫМИ ВЫХОДАМИ DOUT1 И DOUT2
        Row(
            modifier = Modifier.fillMaxWidth().height(64.dp),
            horizontalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            DoutControlButton(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                label = "DOUT 1",
                isActive = state.dout1,
                isAuto = config.dout1.mode != OutputMode.MANUAL,
                onClick = { onToggleDout(1, !state.dout1) }
            )
            DoutControlButton(
                modifier = Modifier.weight(1f).fillMaxHeight(),
                label = "DOUT 2",
                isActive = state.dout2,
                isAuto = config.dout2.mode != OutputMode.MANUAL,
                onClick = { onToggleDout(2, !state.dout2) }
            )
        }
    }
}

@Composable
private fun RpmShiftBar(rpm: Int, maxRpm: Int, shiftRpm: Int) {
    val fraction = (rpm.toFloat() / maxRpm).coerceIn(0f, 1f)
    val barColor = when {
        rpm >= shiftRpm -> Color(0xFFFF1744)
        rpm >= shiftRpm - 1000 -> Color(0xFFFFEA00)
        else -> Color(0xFF00E5FF)
    }
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(48.dp)
            .clip(RoundedCornerShape(8.dp))
            .background(Color(0xFF1A1E24))
    ) {
        Box(
            modifier = Modifier
                .fillMaxWidth(fraction)
                .fillMaxHeight()
                .background(barColor)
        )
        Text(
            text = "$rpm RPM",
            modifier = Modifier.align(Alignment.Center),
            color = Color.White,
            fontWeight = FontWeight.Black,
            fontSize = 24.sp
        )
    }
}

@Composable
private fun GaugeTile(
    modifier: Modifier,
    title: String,
    value: String,
    unit: String,
    isAlarm: Boolean
) {
    val bg = if (isAlarm) Color(0xFF7F0000) else Color(0xFF161A20)
    val border = if (isAlarm) Color(0xFFFF5252) else Color(0xFF2A313D)

    Column(
        modifier = modifier
            .clip(RoundedCornerShape(12.dp))
            .background(bg)
            .border(2.dp, border, RoundedCornerShape(12.dp))
            .padding(14.dp),
        verticalArrangement = Arrangement.SpaceBetween
    ) {
        Text(text = title, color = Color(0xFF90A4AE), fontSize = 14.sp, fontWeight = FontWeight.SemiBold)
        Row(verticalAlignment = Alignment.Bottom) {
            Text(
                text = value,
                color = Color.White,
                fontSize = 46.sp,
                fontWeight = FontWeight.Bold
            )
            Spacer(Modifier.width(8.dp))
            Text(
                text = unit,
                color = Color(0xFF90A4AE),
                fontSize = 18.sp,
                modifier = Modifier.padding(bottom = 8.dp)
            )
        }
    }
}

@Composable
private fun DoutControlButton(
    modifier: Modifier,
    label: String,
    isActive: Boolean,
    isAuto: Boolean,
    onClick: () -> Unit
) {
    val bg = when {
        isActive -> Color(0xFF00C853)
        isAuto -> Color(0xFF1E272E)
        else -> Color(0xFF263238)
    }
    Row(
        modifier = modifier
            .clip(RoundedCornerShape(10.dp))
            .background(bg)
            .clickable(enabled = !isAuto) { onClick() }
            .padding(horizontal = 20.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(text = label, color = Color.White, fontWeight = FontWeight.Bold, fontSize = 20.sp)
        Text(
            text = if (isAuto) (if (isActive) "AUTO: ВКЛ" else "AUTO: ОЖИДАНИЕ") else (if (isActive) "ВКЛ" else "ВЫКЛ"),
            color = Color.White,
            fontWeight = FontWeight.SemiBold,
            fontSize = 16.sp
        )
    }
}