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
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

private data class GaugeItem(
    val title: String,
    val value: String,
    val unit: String,
    val isAlarm: Boolean
)

private fun checkAnalogAlarm(profile: AnalogSensorProfile, value: Float, rpm: Int): Boolean {
    val nameLower = profile.name.lowercase()
    return when {
        "масл" in nameLower -> rpm > 1500 && value < 1.2f
        "буст" in nameLower -> value > 1.8f
        else -> false
    }
}

@Composable
fun DashboardScreen(
    state: TelemetryState,
    config: SystemConfig,
    ain1Profile: AnalogSensorProfile,
    ain2Profile: AnalogSensorProfile,
    deviceInfo: DeviceInfoPacket = DeviceInfoPacket(),
    onToggleDout: (channel: Int, enable: Boolean) -> Unit,
    onOpenConfig: () -> Unit
) {
    val ain1Val = ain1Profile.mvToValue(state.mvAin1)
    val ain2Val = ain2Profile.mvToValue(state.mvAin2)

    val gauges = listOf(
        GaugeItem(
            title = "ТЕМП. ОЖ (T_WATR)",
            value = if (state.tempWatrC <= -39f) "ОБРЫВ" else "${"%.1f".format(state.tempWatrC)}",
            unit = "°C",
            isAlarm = state.tempWatrC > 102f
        ),
        GaugeItem(
            title = "ТЕМП. МАСЛА (T_OIL)",
            value = if (state.tempOilC <= -39f) "ОБРЫВ" else "${"%.1f".format(state.tempOilC)}",
            unit = "°C",
            isAlarm = state.tempOilC > 125f
        ),
        GaugeItem(
            title = "ВЫХЛОП (EGT)",
            value = state.egtC?.let { "${it.toInt()}" } ?: "ОБРЫВ",
            unit = "°C",
            isAlarm = (state.egtC ?: 0f) > 900f
        ),
        GaugeItem(
            title = ain1Profile.name.uppercase(),
            value = "${"%.2f".format(ain1Val)}",
            unit = ain1Profile.unit,
            isAlarm = checkAnalogAlarm(ain1Profile, ain1Val, state.rpm)
        ),
        GaugeItem(
            title = ain2Profile.name.uppercase(),
            value = "${"%.2f".format(ain2Val)}",
            unit = ain2Profile.unit,
            isAlarm = checkAnalogAlarm(ain2Profile, ain2Val, state.rpm)
        ),
        GaugeItem(
            title = "GPS / G-FORCE",
            value = "${state.gpsSpeedKmh.toInt()}",
            unit = "км/ч (${"%.1f".format(state.accXg)}/${"%.1f".format(state.accYg)}G)",
            isAlarm = false
        )
    )

    val fwShort = deviceInfo.fwVersion.substringBefore(" (")
    val versionBadge = if (state.connected) {
        "HW ${deviceInfo.hwVersion} | FW $fwShort"
    } else {
        ""
    }

    BoxWithConstraints(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0D0F12))
    ) {
        val isPortraitPhone = maxWidth < 600.dp
        val isCompactHeight = maxHeight < 480.dp

        val screenPad = if (isPortraitPhone || isCompactHeight) 8.dp else 16.dp
        val gridGap = if (isPortraitPhone || isCompactHeight) 8.dp else 12.dp

        val titleSize = when {
            isCompactHeight -> 11.sp
            isPortraitPhone -> 12.sp
            else -> 14.sp
        }
        val valueSize = when {
            isCompactHeight -> 26.sp
            isPortraitPhone -> 32.sp
            else -> 46.sp
        }
        val unitSize = when {
            isCompactHeight -> 12.sp
            isPortraitPhone -> 13.sp
            else -> 18.sp
        }
        val tilePad = when {
            isCompactHeight -> 8.dp
            isPortraitPhone -> 10.dp
            else -> 14.dp
        }

        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(screenPad),
            verticalArrangement = Arrangement.spacedBy(gridGap)
        ) {
            // 1. ВЕРХНЯЯ ПАНЕЛЬ: ШКАЛА ОБОРОТОВ + СТАТУСЫ И ВЕРСИИ HW/FW
            RpmShiftBar(
                rpm = state.rpm,
                maxRpm = 8000,
                shiftRpm = 6800,
                barHeight = if (isCompactHeight) 34.dp else if (isPortraitPhone) 42.dp else 48.dp,
                fontSize = if (isCompactHeight) 18.sp else if (isPortraitPhone) 20.sp else 24.sp
            )

            if (isPortraitPhone) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Column(modifier = Modifier.weight(1f)) {
                        Text(
                            text = if (state.connected) "● ONLINE • $versionBadge" else "○ ПОИСК RACEDAQ...",
                            color = if (state.connected) Color(0xFF00E676) else Color(0xFFFF5252),
                            fontWeight = FontWeight.Bold,
                            fontSize = 12.sp,
                            maxLines = 1,
                            overflow = TextOverflow.Ellipsis
                        )
                        Text(
                            text = "GPS: ${state.gpsSats} спутн. | IGN: ${"%.1f".format(state.ignVolts)} В",
                            color = Color(0xFFB0BEC5),
                            fontSize = 12.sp,
                            maxLines = 1,
                            overflow = TextOverflow.Ellipsis
                        )
                    }
                    Spacer(Modifier.width(8.dp))
                    Button(
                        onClick = onOpenConfig,
                        contentPadding = PaddingValues(horizontal = 12.dp, vertical = 6.dp),
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF263238))
                    ) {
                        Text("⚙ НАСТРОЙКИ", color = Color.White, fontSize = 12.sp, fontWeight = FontWeight.Bold)
                    }
                }
            } else {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = if (state.connected) "● RACEDAQ ONLINE ($versionBadge)" else "○ ПОИСК RACEDAQ...",
                        color = if (state.connected) Color(0xFF00E676) else Color(0xFFFF5252),
                        fontWeight = FontWeight.Bold,
                        fontSize = if (isCompactHeight) 12.sp else 14.sp,
                        maxLines = 1,
                        overflow = TextOverflow.Ellipsis
                    )
                    Text(
                        text = "GPS: ${state.gpsSats} спутн. | IGN: ${"%.1f".format(state.ignVolts)} В",
                        color = Color(0xFFB0BEC5),
                        fontSize = if (isCompactHeight) 12.sp else 14.sp,
                        maxLines = 1
                    )
                    Button(
                        onClick = onOpenConfig,
                        contentPadding = PaddingValues(
                            horizontal = if (isCompactHeight) 10.dp else 16.dp,
                            vertical = if (isCompactHeight) 4.dp else 8.dp
                        ),
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF263238))
                    ) {
                        Text(
                            text = if (isCompactHeight) "⚙ НАСТРОЙКИ" else "⚙ НАСТРОЙКА ВХОДОВ И СЦЕНАРИЕВ",
                            color = Color.White,
                            fontSize = if (isCompactHeight) 12.sp else 14.sp,
                            maxLines = 1
                        )
                    }
                }
            }

            // 2. СЕТКА ПРИБОРОВ: 2 КОЛОНКИ x 3 РЯДА НА ТЕЛЕФОНЕ ИЛИ 3 КОЛОНКИ x 2 РЯДА НА ПЛАНШЕТЕ
            val columns = if (isPortraitPhone) 2 else 3
            val rows = gauges.chunked(columns)

            Column(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth(),
                verticalArrangement = Arrangement.spacedBy(gridGap)
            ) {
                rows.forEach { rowItems ->
                    Row(
                        modifier = Modifier
                            .weight(1f)
                            .fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(gridGap)
                    ) {
                        rowItems.forEach { item ->
                            GaugeTile(
                                modifier = Modifier
                                    .weight(1f)
                                    .fillMaxHeight(),
                                title = item.title,
                                value = item.value,
                                unit = item.unit,
                                isAlarm = item.isAlarm,
                                tilePad = tilePad,
                                titleSize = titleSize,
                                valueSize = valueSize,
                                unitSize = unitSize
                            )
                        }
                    }
                }
            }

            // 3. НИЖНЯЯ ПАНЕЛЬ УПРАВЛЕНИЯ СИЛОВЫМИ ВЫХОДАМИ DOUT1 И DOUT2
            val doutHeight = when {
                isCompactHeight -> 42.dp
                isPortraitPhone -> 52.dp
                else -> 64.dp
            }
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(doutHeight),
                horizontalArrangement = Arrangement.spacedBy(gridGap)
            ) {
                DoutControlButton(
                    modifier = Modifier
                        .weight(1f)
                        .fillMaxHeight(),
                    label = "DOUT 1",
                    isActive = state.dout1,
                    isAuto = config.dout1.mode != OutputMode.MANUAL,
                    compact = isPortraitPhone || isCompactHeight,
                    onClick = { onToggleDout(1, !state.dout1) }
                )
                DoutControlButton(
                    modifier = Modifier
                        .weight(1f)
                        .fillMaxHeight(),
                    label = "DOUT 2",
                    isActive = state.dout2,
                    isAuto = config.dout2.mode != OutputMode.MANUAL,
                    compact = isPortraitPhone || isCompactHeight,
                    onClick = { onToggleDout(2, !state.dout2) }
                )
            }
        }
    }
}

@Composable
private fun RpmShiftBar(
    rpm: Int,
    maxRpm: Int,
    shiftRpm: Int,
    barHeight: androidx.compose.ui.unit.Dp,
    fontSize: TextUnit
) {
    val fraction = (rpm.toFloat() / maxRpm).coerceIn(0f, 1f)
    val barColor = when {
        rpm >= shiftRpm -> Color(0xFFFF1744)
        rpm >= shiftRpm - 1000 -> Color(0xFFFFEA00)
        else -> Color(0xFF00E5FF)
    }
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(barHeight)
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
            fontSize = fontSize,
            maxLines = 1
        )
    }
}

@Composable
private fun GaugeTile(
    modifier: Modifier,
    title: String,
    value: String,
    unit: String,
    isAlarm: Boolean,
    tilePad: androidx.compose.ui.unit.Dp,
    titleSize: TextUnit,
    valueSize: TextUnit,
    unitSize: TextUnit
) {
    val bg = if (isAlarm) Color(0xFF7F0000) else Color(0xFF161A20)
    val border = if (isAlarm) Color(0xFFFF5252) else Color(0xFF2A313D)

    Column(
        modifier = modifier
            .clip(RoundedCornerShape(12.dp))
            .background(bg)
            .border(2.dp, border, RoundedCornerShape(12.dp))
            .padding(tilePad),
        verticalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = title,
            color = Color(0xFF90A4AE),
            fontSize = titleSize,
            fontWeight = FontWeight.SemiBold,
            maxLines = 1,
            softWrap = false,
            overflow = TextOverflow.Ellipsis
        )
        Row(
            modifier = Modifier.fillMaxWidth(),
            verticalAlignment = Alignment.Bottom
        ) {
            Text(
                text = value,
                color = Color.White,
                fontSize = valueSize,
                fontWeight = FontWeight.Bold,
                maxLines = 1,
                softWrap = false,
                overflow = TextOverflow.Ellipsis
            )
            Spacer(Modifier.width(6.dp))
            Text(
                text = unit,
                color = Color(0xFF90A4AE),
                fontSize = unitSize,
                maxLines = 1,
                softWrap = false,
                overflow = TextOverflow.Ellipsis,
                modifier = Modifier.padding(bottom = 4.dp)
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
    compact: Boolean,
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
            .padding(horizontal = if (compact) 12.dp else 20.dp),
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = label,
            color = Color.White,
            fontWeight = FontWeight.Bold,
            fontSize = if (compact) 15.sp else 20.sp,
            maxLines = 1
        )
        Text(
            text = if (isAuto) (if (isActive) "AUTO: ВКЛ" else "AUTO: ОЖИД.") else (if (isActive) "ВКЛ" else "ВЫКЛ"),
            color = Color.White,
            fontWeight = FontWeight.SemiBold,
            fontSize = if (compact) 12.sp else 16.sp,
            maxLines = 1
        )
    }
}