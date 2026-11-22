package com.example.racedaq_cfg

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.abs

@Composable
fun ConfigScreen(
    telemetry: TelemetryState,
    currentConfig: SystemConfig,
    currentGpsHz: Int,
    ain1Profile: AnalogSensorProfile,
    ain2Profile: AnalogSensorProfile,
    deviceInfo: DeviceInfoPacket = DeviceInfoPacket(),
    onSaveSystemConfig: (SystemConfig) -> Unit,
    onSaveAinProfiles: (AnalogSensorProfile, AnalogSensorProfile) -> Unit,
    onSetGpsRate: (Int) -> Unit,
    onBack: () -> Unit
) {
    val latestTelemetry by rememberUpdatedState(telemetry)

    var draftConfig by remember(currentConfig) { mutableStateOf(currentConfig) }
    var draftAin1 by remember(ain1Profile) { mutableStateOf(ain1Profile) }
    var draftAin2 by remember(ain2Profile) { mutableStateOf(ain2Profile) }
    var selectedTab by remember { mutableIntStateOf(0) }

    BoxWithConstraints(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0D0F12))
    ) {
        val isPhone = maxWidth < 700.dp
        val isNarrowPhone = maxWidth < 420.dp
        val pad = if (isPhone) 8.dp else 16.dp
        val gap = if (isPhone) 8.dp else 12.dp

        val tabs = if (isNarrowPhone) {
            listOf("1. ВХОДЫ", "2. ВЫХОДЫ", "3. CAN", "4. СИСТЕМА")
        } else {
            listOf("1. ВХОДЫ", "2. ВЫХОДЫ", "3. CAN-ШИНА", "4. СИСТЕМА")
        }

        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(pad),
            verticalArrangement = Arrangement.spacedBy(gap)
        ) {
            // ВЕРХНЯЯ ПАНЕЛЬ НАВИГАЦИИ И СОХРАНЕНИЯ
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                OutlinedButton(
                    onClick = onBack,
                    contentPadding = PaddingValues(horizontal = 12.dp, vertical = 8.dp)
                ) {
                    Text(
                        text = if (isPhone) "← НАЗАД" else "← НА ПРИБОРКУ",
                        color = Color.White,
                        fontWeight = FontWeight.Bold,
                        fontSize = if (isPhone) 12.sp else 14.sp
                    )
                }

                if (!isPhone) {
                    Text(
                        text = "КОНФИГУРАТОР RACEDAQ",
                        color = Color.White,
                        fontSize = 20.sp,
                        fontWeight = FontWeight.Black
                    )
                }

                Button(
                    onClick = {
                        onSaveAinProfiles(draftAin1, draftAin2)
                        onSaveSystemConfig(draftConfig)
                    },
                    contentPadding = PaddingValues(horizontal = 14.dp, vertical = 8.dp),
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF00C853))
                ) {
                    Text(
                        text = if (isPhone) "💾 СОХРАНИТЬ" else "💾 ЗАПИСАТЬ В ПРИБОР",
                        color = Color.White,
                        fontWeight = FontWeight.Bold,
                        fontSize = if (isPhone) 12.sp else 14.sp
                    )
                }
            }

            // ВКЛАДКИ
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                tabs.forEachIndexed { index, title ->
                    val isSelected = selectedTab == index
                    Box(
                        modifier = Modifier
                            .weight(1f)
                            .clip(RoundedCornerShape(8.dp))
                            .background(if (isSelected) Color(0xFF0288D1) else Color(0xFF1A1E24))
                            .clickable { selectedTab = index }
                            .padding(vertical = if (isPhone) 10.dp else 12.dp, horizontal = 4.dp),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = title,
                            color = Color.White,
                            fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal,
                            fontSize = if (isNarrowPhone) 11.sp else if (isPhone) 12.sp else 14.sp,
                            maxLines = 1,
                            softWrap = false,
                            overflow = TextOverflow.Ellipsis
                        )
                    }
                }
            }

            // КОНТЕНТ ВЫБРАННОЙ ВКЛАДКИ
            Box(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(12.dp))
                    .background(Color(0xFF161A20))
                    .padding(if (isPhone) 10.dp else 16.dp)
            ) {
                when (selectedTab) {
                    0 -> InputsTab(
                        isPhone = isPhone,
                        telemetryProvider = { latestTelemetry },
                        din1 = draftConfig.din1,
                        din2 = draftConfig.din2,
                        tempWat = draftConfig.tempWat,
                        tempOil = draftConfig.tempOil,
                        ain1 = draftAin1,
                        ain2 = draftAin2,
                        onDin1Change = { draftConfig = draftConfig.copy(din1 = it) },
                        onDin2Change = { draftConfig = draftConfig.copy(din2 = it) },
                        onTempWatChange = { draftConfig = draftConfig.copy(tempWat = it) },
                        onTempOilChange = { draftConfig = draftConfig.copy(tempOil = it) },
                        onAin1Change = { draftAin1 = it },
                        onAin2Change = { draftAin2 = it }
                    )
                    1 -> OutputsTab(
                        isPhone = isPhone,
                        dout1 = draftConfig.dout1,
                        dout2 = draftConfig.dout2,
                        ain1Profile = draftAin1,
                        ain2Profile = draftAin2,
                        onDout1Change = { draftConfig = draftConfig.copy(dout1 = it) },
                        onDout2Change = { draftConfig = draftConfig.copy(dout2 = it) }
                    )
                    2 -> CanTab(
                        isPhone = isPhone,
                        canSpeedKbps = draftConfig.canSpeedKbps,
                        slots = draftConfig.canSlots,
                        onSpeedChange = { draftConfig = draftConfig.copy(canSpeedKbps = it) },
                        onSlotChange = { idx, slot ->
                            val updated = draftConfig.canSlots.toMutableList()
                            updated[idx] = slot
                            draftConfig = draftConfig.copy(canSlots = updated)
                        }
                    )
                    3 -> SystemTab(
                        connected = latestTelemetry.connected,
                        currentGpsHz = currentGpsHz,
                        deviceInfo = deviceInfo,
                        onSetGpsRate = onSetGpsRate
                    )
                }
            }
        }
    }
}

// ============================================================================
// ВКЛАДКА 1: НАСТРОЙКА ВХОДОВ DIN1/DIN2, NTC ТЕМПЕРАТУРЫ И AIN1/AIN2
// ============================================================================
@Composable
private fun InputsTab(
    isPhone: Boolean,
    telemetryProvider: () -> TelemetryState,
    din1: DinConfig,
    din2: DinConfig,
    tempWat: TempChannelConfig,
    tempOil: TempChannelConfig,
    ain1: AnalogSensorProfile,
    ain2: AnalogSensorProfile,
    onDin1Change: (DinConfig) -> Unit,
    onDin2Change: (DinConfig) -> Unit,
    onTempWatChange: (TempChannelConfig) -> Unit,
    onTempOilChange: (TempChannelConfig) -> Unit,
    onAin1Change: (AnalogSensorProfile) -> Unit,
    onAin2Change: (AnalogSensorProfile) -> Unit
) {
    val leftColumn: @Composable (Modifier) -> Unit = { mod ->
        Column(modifier = mod, verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Text("ДАТЧИКИ ТЕМПЕРАТУРЫ (NTC)", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold, fontSize = 14.sp)
            TempChannelCard(
                labelPrefix = "T_WATR (ОЖ)",
                liveTextProvider = {
                    val t = telemetryProvider()
                    val s = if (t.tempWatrC <= -39.5f) "ОБРЫВ" else "${"%.1f".format(t.tempWatrC)} °C"
                    "${t.mvWatr} мВ -> $s"
                },
                config = tempWat,
                onChange = onTempWatChange
            )
            TempChannelCard(
                labelPrefix = "T_OIL (Масло)",
                liveTextProvider = {
                    val t = telemetryProvider()
                    val s = if (t.tempOilC <= -39.5f) "ОБРЫВ" else "${"%.1f".format(t.tempOilC)} °C"
                    "${t.mvOil} мВ -> $s"
                },
                config = tempOil,
                onChange = onTempOilChange
            )

            Spacer(modifier = Modifier.height(4.dp))
            Text("ЦИФРОВЫЕ ВХОДЫ (DIN1 / DIN2)", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold, fontSize = 14.sp)
            DinCard(
                title = "Вход DIN 1 (P0.11)",
                liveTextProvider = {
                    val t = telemetryProvider()
                    if (din1.mode == DinMode.RPM) "${t.rpm} RPM" else if (t.din1) "1 (GND)" else "0"
                },
                config = din1,
                onChange = { newDin1 ->
                    if (newDin1.mode == DinMode.RPM && din2.mode == DinMode.RPM) {
                        onDin2Change(din2.copy(mode = DinMode.DIGITAL))
                    }
                    onDin1Change(newDin1)
                }
            )
            DinCard(
                title = "Вход DIN 2 (P0.09)",
                liveTextProvider = {
                    val t = telemetryProvider()
                    if (din2.mode == DinMode.RPM) "${t.rpm} RPM" else if (t.din2) "1 (GND)" else "0"
                },
                config = din2,
                onChange = { newDin2 ->
                    if (newDin2.mode == DinMode.RPM && din1.mode == DinMode.RPM) {
                        onDin1Change(din1.copy(mode = DinMode.DIGITAL))
                    }
                    onDin2Change(newDin2)
                }
            )
        }
    }

    val rightColumn: @Composable (Modifier) -> Unit = { mod ->
        Column(modifier = mod, verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Text("КАЛИБРОВКА АНАЛОГОВЫХ ДАТЧИКОВ", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold, fontSize = 14.sp)
            AinCard(
                isPhone = isPhone,
                labelPrefix = "AIN 1",
                liveTextProvider = {
                    val mv = telemetryProvider().mvAin1
                    "$mv мВ -> ${"%.2f".format(ain1.mvToValue(mv))} ${ain1.unit}"
                },
                profile = ain1,
                onChange = onAin1Change
            )
            AinCard(
                isPhone = isPhone,
                labelPrefix = "AIN 2",
                liveTextProvider = {
                    val mv = telemetryProvider().mvAin2
                    "$mv мВ -> ${"%.2f".format(ain2.mvToValue(mv))} ${ain2.unit}"
                },
                profile = ain2,
                onChange = onAin2Change
            )
        }
    }

    if (isPhone) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(14.dp)
        ) {
            leftColumn(Modifier.fillMaxWidth())
            rightColumn(Modifier.fillMaxWidth())
        }
    } else {
        Row(
            modifier = Modifier.fillMaxSize(),
            horizontalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            leftColumn(Modifier.weight(1f).verticalScroll(rememberScrollState()))
            rightColumn(Modifier.weight(1f).verticalScroll(rememberScrollState()))
        }
    }
}

@Composable
private fun TempChannelCard(
    labelPrefix: String,
    liveTextProvider: () -> String,
    config: TempChannelConfig,
    onChange: (TempChannelConfig) -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(labelPrefix, color = Color.White, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            Text(liveTextProvider(), color = Color(0xFF00E676), fontWeight = FontWeight.SemiBold, fontSize = 12.sp)
        }

        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            TEMP_SENSOR_PRESETS.forEach { preset ->
                val isSelected = config.pullupMode == preset.config.pullupMode &&
                        abs(config.r25Ohm - preset.config.r25Ohm) < 1f &&
                        config.betaK == preset.config.betaK
                FastChip(
                    selected = isSelected,
                    text = preset.name,
                    fontSize = 11.sp,
                    onClick = { onChange(preset.config) }
                )
            }
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            DarkTextField(
                value = config.r25Ohm.toInt().toString(),
                label = "R при 25°C (Ом)",
                isNumber = true,
                modifier = Modifier.weight(1f),
                onValueChange = { it.toFloatOrNull()?.let { v -> onChange(config.copy(r25Ohm = v)) } }
            )
            DarkTextField(
                value = config.betaK.toString(),
                label = "Beta (K)",
                isNumber = true,
                modifier = Modifier.weight(1f),
                onValueChange = { it.toIntOrNull()?.let { v -> onChange(config.copy(betaK = v)) } }
            )
        }

        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            TempPullupMode.entries.forEach { mode ->
                FastChip(
                    selected = config.pullupMode == mode,
                    text = mode.label,
                    fontSize = 11.sp,
                    onClick = { onChange(config.copy(pullupMode = mode)) }
                )
            }
        }
    }
}

@Composable
private fun DinCard(
    title: String,
    liveTextProvider: () -> String,
    config: DinConfig,
    onChange: (DinConfig) -> Unit
) {
    val pulsePresets = remember {
        listOf(
            1 to "1 цил. 4T",
            2 to "2 цил. / 1 имп",
            4 to "4 цил. 4T",
            6 to "6 цил. 4T",
            8 to "8 цил. 4T"
        )
    }

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(title, color = Color.White, fontWeight = FontWeight.Bold, fontSize = 14.sp)
            Text(liveTextProvider(), color = Color(0xFF00E676), fontWeight = FontWeight.SemiBold, fontSize = 13.sp)
        }

        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            DinMode.entries.forEach { mode ->
                FastChip(
                    selected = config.mode == mode,
                    text = mode.title,
                    fontSize = 12.sp,
                    onClick = { onChange(config.copy(mode = mode)) }
                )
            }
            FastChip(
                selected = config.pullup,
                text = "Pull-Up",
                fontSize = 12.sp,
                onClick = { onChange(config.copy(pullup = !config.pullup)) }
            )
        }

        if (config.mode == DinMode.RPM) {
            Text("Делитель тахометра:", color = Color(0xFFB0BEC5), fontSize = 12.sp)
            Row(
                modifier = Modifier.horizontalScroll(rememberScrollState()),
                horizontalArrangement = Arrangement.spacedBy(6.dp)
            ) {
                pulsePresets.forEach { (p2r, label) ->
                    FastChip(
                        selected = config.pulsesPer2Rev == p2r,
                        text = label,
                        fontSize = 11.sp,
                        onClick = { onChange(config.copy(pulsesPer2Rev = p2r)) }
                    )
                }
            }
        }
    }
}

@Composable
private fun AinCard(
    isPhone: Boolean,
    labelPrefix: String,
    liveTextProvider: () -> String,
    profile: AnalogSensorProfile,
    onChange: (AnalogSensorProfile) -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(labelPrefix, color = Color.White, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            Text(liveTextProvider(), color = Color(0xFF00E676), fontWeight = FontWeight.SemiBold, fontSize = 12.sp)
        }

        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            ANALOG_SENSOR_PRESETS.forEach { (chipLabel, preset) ->
                val isSelected = profile.name.equals(preset.name, ignoreCase = true)
                FastChip(
                    selected = isSelected,
                    text = chipLabel,
                    fontSize = 11.sp,
                    onClick = { onChange(preset) }
                )
            }
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            DarkTextField(
                value = profile.name,
                label = "Название",
                modifier = Modifier.weight(2f),
                onValueChange = { onChange(profile.copy(name = it)) }
            )
            DarkTextField(
                value = profile.unit,
                label = "Ед. изм.",
                modifier = Modifier.weight(1f),
                onValueChange = { onChange(profile.copy(unit = it)) }
            )
        }

        if (isPhone) {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                DarkTextField(
                    value = profile.v1Mv.toString(),
                    label = "Точка 1 (мВ)",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.toIntOrNull()?.let { v -> onChange(profile.copy(v1Mv = v)) } }
                )
                DarkTextField(
                    value = profile.val1.toString(),
                    label = "Знач. 1 (${profile.unit})",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.replace(',', '.').toFloatOrNull()?.let { v -> onChange(profile.copy(val1 = v)) } }
                )
            }
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                DarkTextField(
                    value = profile.v2Mv.toString(),
                    label = "Точка 2 (мВ)",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.toIntOrNull()?.let { v -> onChange(profile.copy(v2Mv = v)) } }
                )
                DarkTextField(
                    value = profile.val2.toString(),
                    label = "Знач. 2 (${profile.unit})",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.replace(',', '.').toFloatOrNull()?.let { v -> onChange(profile.copy(val2 = v)) } }
                )
            }
        } else {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                DarkTextField(
                    value = profile.v1Mv.toString(),
                    label = "Точка 1 (мВ)",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.toIntOrNull()?.let { v -> onChange(profile.copy(v1Mv = v)) } }
                )
                DarkTextField(
                    value = profile.val1.toString(),
                    label = "Знач. 1",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.replace(',', '.').toFloatOrNull()?.let { v -> onChange(profile.copy(val1 = v)) } }
                )
                DarkTextField(
                    value = profile.v2Mv.toString(),
                    label = "Точка 2 (мВ)",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.toIntOrNull()?.let { v -> onChange(profile.copy(v2Mv = v)) } }
                )
                DarkTextField(
                    value = profile.val2.toString(),
                    label = "Знач. 2",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.replace(',', '.').toFloatOrNull()?.let { v -> onChange(profile.copy(val2 = v)) } }
                )
            }
        }
    }
}

// ============================================================================
// ВКЛАДКА 2: КОНСТРУКТОР СЦЕНАРИЕВ ВЫХОДОВ DOUT1 И DOUT2
// ============================================================================
@Composable
private fun OutputsTab(
    isPhone: Boolean,
    dout1: DoutConfig,
    dout2: DoutConfig,
    ain1Profile: AnalogSensorProfile,
    ain2Profile: AnalogSensorProfile,
    onDout1Change: (DoutConfig) -> Unit,
    onDout2Change: (DoutConfig) -> Unit
) {
    if (isPhone) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            DoutRuleEditor(
                isPhone = true,
                modifier = Modifier.fillMaxWidth(),
                title = "СИЛОВОЙ ВЫХОД DOUT 1 (P0.29)",
                config = dout1,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                onChange = onDout1Change
            )
            DoutRuleEditor(
                isPhone = true,
                modifier = Modifier.fillMaxWidth(),
                title = "СИЛОВОЙ ВЫХОД DOUT 2 (P0.31)",
                config = dout2,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                onChange = onDout2Change
            )
        }
    } else {
        Row(
            modifier = Modifier.fillMaxSize(),
            horizontalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            DoutRuleEditor(
                isPhone = false,
                modifier = Modifier.weight(1f).verticalScroll(rememberScrollState()),
                title = "СИЛОВОЙ ВЫХОД DOUT 1 (P0.29)",
                config = dout1,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                onChange = onDout1Change
            )
            DoutRuleEditor(
                isPhone = false,
                modifier = Modifier.weight(1f).verticalScroll(rememberScrollState()),
                title = "СИЛОВОЙ ВЫХОД DOUT 2 (P0.31)",
                config = dout2,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                onChange = onDout2Change
            )
        }
    }
}

@Composable
private fun DoutRuleEditor(
    isPhone: Boolean,
    modifier: Modifier,
    title: String,
    config: DoutConfig,
    ain1Profile: AnalogSensorProfile,
    ain2Profile: AnalogSensorProfile,
    onChange: (DoutConfig) -> Unit
) {
    Column(
        modifier = modifier
            .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(title, color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold, fontSize = 14.sp)

        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            OutputMode.entries.forEach { mode ->
                FastChip(
                    selected = config.mode == mode,
                    text = mode.title,
                    fontSize = 12.sp,
                    onClick = { onChange(config.copy(mode = mode)) }
                )
            }
        }

        if (config.mode != OutputMode.MANUAL) {
            FastChip(
                selected = config.invert,
                text = "Инверсия выхода (выключать при срабатывании)",
                fontSize = 12.sp,
                onClick = { onChange(config.copy(invert = !config.invert)) }
            )

            ConditionEditor(
                isPhone = isPhone,
                label = "Условие A (основное)",
                cond = config.condA,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                showDelay = true,
                onChange = { onChange(config.copy(condA = it)) }
            )

            if (config.mode == OutputMode.LOGIC_OR || config.mode == OutputMode.LOGIC_AND) {
                ConditionEditor(
                    isPhone = isPhone,
                    label = if (config.mode == OutputMode.LOGIC_OR) "Условие B (ИЛИ)" else "Условие B (И)",
                    cond = config.condB,
                    ain1Profile = ain1Profile,
                    ain2Profile = ain2Profile,
                    showDelay = false,
                    onChange = { onChange(config.copy(condB = it)) }
                )
            }
        }
    }
}

@Composable
private fun ConditionEditor(
    isPhone: Boolean,
    label: String,
    cond: RuleCondition,
    ain1Profile: AnalogSensorProfile,
    ain2Profile: AnalogSensorProfile,
    showDelay: Boolean,
    onChange: (RuleCondition) -> Unit
) {
    var expandedSource by remember { mutableStateOf(false) }

    val unitLabel = when (cond.source) {
        SignalSource.AIN1_MV -> ain1Profile.unit
        SignalSource.AIN2_MV -> ain2Profile.unit
        SignalSource.TWATR_C10, SignalSource.TOIL_C10 -> "°C"
        else -> cond.source.unit
    }

    val displayThreshold = when (cond.source) {
        SignalSource.AIN1_MV -> "%.2f".format(ain1Profile.mvToValue(cond.threshold))
        SignalSource.AIN2_MV -> "%.2f".format(ain2Profile.mvToValue(cond.threshold))
        SignalSource.TWATR_C10, SignalSource.TOIL_C10 -> "%.1f".format(cond.threshold / 10f)
        else -> cond.threshold.toString()
    }

    var thresholdText by remember(cond.source, cond.threshold) { mutableStateOf(displayThreshold) }

    val onThresholdTextChange: (String) -> Unit = { txt ->
        thresholdText = txt
        val parsed = txt.replace(',', '.').toFloatOrNull()
        if (parsed != null) {
            val rawThreshold = when (cond.source) {
                SignalSource.AIN1_MV -> ain1Profile.valueToMv(parsed)
                SignalSource.AIN2_MV -> ain2Profile.valueToMv(parsed)
                SignalSource.TWATR_C10, SignalSource.TOIL_C10 -> (parsed * 10f).toInt()
                else -> parsed.toInt()
            }
            onChange(cond.copy(threshold = rawThreshold))
        }
    }

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(Color(0xFF1E232B), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(label, color = Color(0xFFFFD54F), fontSize = 13.sp, fontWeight = FontWeight.SemiBold)

        if (isPhone) {
            Box(modifier = Modifier.fillMaxWidth()) {
                OutlinedButton(
                    onClick = { expandedSource = true },
                    modifier = Modifier.fillMaxWidth(),
                    contentPadding = PaddingValues(horizontal = 10.dp, vertical = 8.dp)
                ) {
                    Text(cond.source.title, color = Color.White, maxLines = 1, fontSize = 13.sp)
                }
                if (expandedSource) {
                    DropdownMenu(expanded = true, onDismissRequest = { expandedSource = false }) {
                        SignalSource.entries.forEach { src ->
                            DropdownMenuItem(
                                text = { Text(src.title) },
                                onClick = {
                                    expandedSource = false
                                    onChange(cond.copy(source = src))
                                }
                            )
                        }
                    }
                }
            }
            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                CompareOp.entries.forEach { op ->
                    FastChip(
                        selected = cond.op == op,
                        text = op.symbol,
                        fontSize = 13.sp,
                        bold = true,
                        onClick = { onChange(cond.copy(op = op)) }
                    )
                }
            }

            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                DarkTextField(
                    value = thresholdText,
                    label = "Порог ($unitLabel)",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = onThresholdTextChange
                )
                DarkTextField(
                    value = cond.hysteresis.toString(),
                    label = "Гистерезис",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.toIntOrNull()?.let { h -> onChange(cond.copy(hysteresis = h)) } }
                )
            }
            if (showDelay) {
                DarkTextField(
                    value = cond.delayMs.toString(),
                    label = "Задержка срабатывания (мс)",
                    isNumber = true,
                    modifier = Modifier.fillMaxWidth(),
                    onValueChange = { it.toIntOrNull()?.let { d -> onChange(cond.copy(delayMs = d)) } }
                )
            }
        } else {
            Row(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalAlignment = Alignment.CenterVertically) {
                Box(modifier = Modifier.weight(1f)) {
                    OutlinedButton(
                        onClick = { expandedSource = true },
                        modifier = Modifier.fillMaxWidth(),
                        contentPadding = PaddingValues(horizontal = 10.dp, vertical = 6.dp)
                    ) {
                        Text(cond.source.title, color = Color.White, maxLines = 1, fontSize = 12.sp)
                    }
                    if (expandedSource) {
                        DropdownMenu(expanded = true, onDismissRequest = { expandedSource = false }) {
                            SignalSource.entries.forEach { src ->
                                DropdownMenuItem(
                                    text = { Text(src.title) },
                                    onClick = {
                                        expandedSource = false
                                        onChange(cond.copy(source = src))
                                    }
                                )
                            }
                        }
                    }
                }

                Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                    CompareOp.entries.forEach { op ->
                        FastChip(
                            selected = cond.op == op,
                            text = op.symbol,
                            fontSize = 13.sp,
                            bold = true,
                            onClick = { onChange(cond.copy(op = op)) }
                        )
                    }
                }
            }

            Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                DarkTextField(
                    value = thresholdText,
                    label = "Порог ($unitLabel)",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = onThresholdTextChange
                )
                DarkTextField(
                    value = cond.hysteresis.toString(),
                    label = "Гистерезис",
                    isNumber = true,
                    modifier = Modifier.weight(1f),
                    onValueChange = { it.toIntOrNull()?.let { h -> onChange(cond.copy(hysteresis = h)) } }
                )
                if (showDelay) {
                    DarkTextField(
                        value = cond.delayMs.toString(),
                        label = "Задержка (мс)",
                        isNumber = true,
                        modifier = Modifier.weight(1f),
                        onValueChange = { it.toIntOrNull()?.let { d -> onChange(cond.copy(delayMs = d)) } }
                    )
                }
            }
        }
    }
}

// ============================================================================
// ВКЛАДКА 3: НАСТРОЙКА CAN-ШИНЫ И 4 АВТОНОМНЫХ СЛОТОВ
// ============================================================================
@Composable
private fun CanTab(
    isPhone: Boolean,
    canSpeedKbps: Int,
    slots: List<CanSignalSlot>,
    onSpeedChange: (Int) -> Unit,
    onSlotChange: (Int, CanSignalSlot) -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text("СКОРОСТЬ ШИНЫ MCP2515:", color = Color.White, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                listOf(250, 500, 1000).forEach { speed ->
                    FastChip(
                        selected = canSpeedKbps == speed,
                        text = "$speed кбит/с",
                        fontSize = 12.sp,
                        onClick = { onSpeedChange(speed) }
                    )
                }
            }
        }

        slots.forEachIndexed { index, slot ->
            var hexId by remember(slot.canId) { mutableStateOf(slot.canId.toString(16).uppercase()) }

            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
                    .padding(10.dp),
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    FastChip(
                        selected = slot.enabled,
                        text = if (slot.enabled) "✓ CAN Слот $index" else "CAN Слот $index",
                        fontSize = 12.sp,
                        bold = true,
                        onClick = { onSlotChange(index, slot.copy(enabled = !slot.enabled)) }
                    )
                    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        FastChip(
                            selected = slot.bigEndian,
                            text = if (slot.bigEndian) "Big-End" else "Little-End",
                            fontSize = 11.sp,
                            onClick = { onSlotChange(index, slot.copy(bigEndian = !slot.bigEndian)) }
                        )
                        FastChip(
                            selected = slot.signed,
                            text = if (slot.signed) "Signed" else "Unsigned",
                            fontSize = 11.sp,
                            onClick = { onSlotChange(index, slot.copy(signed = !slot.signed)) }
                        )
                    }
                }

                if (isPhone) {
                    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        DarkTextField(
                            value = hexId,
                            label = "CAN ID (HEX)",
                            modifier = Modifier.weight(1.2f),
                            onValueChange = { txt ->
                                hexId = txt
                                txt.toLongOrNull(16)?.let { id -> onSlotChange(index, slot.copy(canId = id)) }
                            }
                        )
                        DarkTextField(
                            value = slot.startByte.toString(),
                            label = "Байт (0..7)",
                            isNumber = true,
                            modifier = Modifier.weight(1f),
                            onValueChange = { it.toIntOrNull()?.let { b -> onSlotChange(index, slot.copy(startByte = b.coerceIn(0, 7))) } }
                        )
                        DarkTextField(
                            value = slot.length.toString(),
                            label = "Длина (1/2)",
                            isNumber = true,
                            modifier = Modifier.weight(1f),
                            onValueChange = { it.toIntOrNull()?.let { l -> onSlotChange(index, slot.copy(length = l.coerceIn(1, 2))) } }
                        )
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        DarkTextField(
                            value = slot.mul.toString(),
                            label = "Множитель (100 = x1.0)",
                            isNumber = true,
                            modifier = Modifier.weight(1f),
                            onValueChange = { it.toIntOrNull()?.let { m -> onSlotChange(index, slot.copy(mul = m)) } }
                        )
                        DarkTextField(
                            value = slot.offset.toString(),
                            label = "Смещение",
                            isNumber = true,
                            modifier = Modifier.weight(1f),
                            onValueChange = { it.toIntOrNull()?.let { o -> onSlotChange(index, slot.copy(offset = o)) } }
                        )
                    }
                } else {
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        DarkTextField(
                            value = hexId,
                            label = "CAN ID (HEX)",
                            modifier = Modifier.weight(1.2f),
                            onValueChange = { txt ->
                                hexId = txt
                                txt.toLongOrNull(16)?.let { id -> onSlotChange(index, slot.copy(canId = id)) }
                            }
                        )
                        DarkTextField(
                            value = slot.startByte.toString(),
                            label = "Байт (0..7)",
                            isNumber = true,
                            modifier = Modifier.weight(1f),
                            onValueChange = { it.toIntOrNull()?.let { b -> onSlotChange(index, slot.copy(startByte = b.coerceIn(0, 7))) } }
                        )
                        DarkTextField(
                            value = slot.length.toString(),
                            label = "Длина (1/2)",
                            isNumber = true,
                            modifier = Modifier.weight(1f),
                            onValueChange = { it.toIntOrNull()?.let { l -> onSlotChange(index, slot.copy(length = l.coerceIn(1, 2))) } }
                        )
                        DarkTextField(
                            value = slot.mul.toString(),
                            label = "Множ. (100=x1)",
                            isNumber = true,
                            modifier = Modifier.weight(1.2f),
                            onValueChange = { it.toIntOrNull()?.let { m -> onSlotChange(index, slot.copy(mul = m)) } }
                        )
                        DarkTextField(
                            value = slot.offset.toString(),
                            label = "Смещение",
                            isNumber = true,
                            modifier = Modifier.weight(1f),
                            onValueChange = { it.toIntOrNull()?.let { o -> onSlotChange(index, slot.copy(offset = o)) } }
                        )
                    }
                }
            }
        }
    }
}

// ============================================================================
// ВКЛАДКА 4: ЧАСТОТА GPS, ВЕРСИИ ПЛАТЫ/ПО И ОБСЛУЖИВАНИЕ
// ============================================================================
@Composable
private fun SystemTab(
    connected: Boolean,
    currentGpsHz: Int,
    deviceInfo: DeviceInfoPacket,
    onSetGpsRate: (Int) -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
                .padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            Text("ИНФОРМАЦИЯ ОБ УСТРОЙСТВЕ", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold, fontSize = 14.sp)
            Text(
                text = "Аппаратная ревизия (HW): ${if (connected) deviceInfo.hwVersion else "Нет подключения"}",
                color = Color.White,
                fontSize = 13.sp
            )
            Text(
                text = "Версия прошивки (FW): ${if (connected) deviceInfo.fwVersion else "Нет подключения"}",
                color = Color.White,
                fontSize = 13.sp
            )
            if (connected) {
                Text(
                    text = "Активные модули: ${deviceInfo.capsSummary()}",
                    color = Color(0xFF00E676),
                    fontSize = 12.sp
                )
            }
        }

        Text("ЧАСТОТА ОБНОВЛЕНИЯ GPS", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold)
        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            listOf(1, 5, 10, 20, 25).forEach { hz ->
                FastChip(
                    selected = currentGpsHz == hz,
                    enabled = connected,
                    text = "$hz Гц",
                    fontSize = 14.sp,
                    bold = true,
                    onClick = { onSetGpsRate(hz) }
                )
            }
        }
        Text(
            text = "Выбранная частота сразу записывается во внутреннюю флеш-память nRF52832 (/gps_hz.bin) и применяется при каждом включении зажигания.",
            color = Color(0xFF90A4AE),
            fontSize = 13.sp
        )
    }
}

// Быстрый плоский чип без накладных расходов Material3 Surface
@Composable
private fun FastChip(
    selected: Boolean,
    text: String,
    fontSize: TextUnit = 12.sp,
    bold: Boolean = false,
    enabled: Boolean = true,
    onClick: () -> Unit
) {
    val bgColor = when {
        !enabled -> Color(0xFF1A1E24)
        selected -> Color(0xFF0277BD)
        else -> Color(0xFF1E242D)
    }
    val borderColor = when {
        !enabled -> Color(0xFF263238)
        selected -> Color(0xFF4FC3F7)
        else -> Color(0xFF37474F)
    }
    val textColor = if (enabled) Color.White else Color(0xFF607D8B)

    Box(
        modifier = Modifier
            .clip(RoundedCornerShape(8.dp))
            .background(bgColor)
            .border(1.dp, borderColor, RoundedCornerShape(8.dp))
            .clickable(enabled = enabled, onClick = onClick)
            .padding(horizontal = 10.dp, vertical = 7.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = text,
            color = textColor,
            fontSize = fontSize,
            fontWeight = if (selected || bold) FontWeight.Bold else FontWeight.Normal,
            maxLines = 1,
            softWrap = false
        )
    }
}

@Composable
private fun DarkTextField(
    value: String,
    label: String,
    modifier: Modifier = Modifier,
    isNumber: Boolean = false,
    onValueChange: (String) -> Unit
) {
    Column(
        modifier = modifier
            .background(Color(0xFF14181F), RoundedCornerShape(6.dp))
            .border(1.dp, Color(0xFF37474F), RoundedCornerShape(6.dp))
            .padding(horizontal = 8.dp, vertical = 6.dp)
    ) {
        Text(
            text = label,
            color = Color(0xFF90A4AE),
            fontSize = 10.sp,
            maxLines = 1,
            overflow = TextOverflow.Ellipsis
        )
        Spacer(modifier = Modifier.height(2.dp))
        BasicTextField(
            value = value,
            onValueChange = onValueChange,
            singleLine = true,
            textStyle = TextStyle(color = Color.White, fontSize = 14.sp),
            cursorBrush = SolidColor(Color(0xFF4FC3F7)),
            keyboardOptions = if (isNumber) {
                KeyboardOptions(keyboardType = KeyboardType.Number)
            } else {
                KeyboardOptions.Default
            },
            modifier = Modifier.fillMaxWidth()
        )
    }
}