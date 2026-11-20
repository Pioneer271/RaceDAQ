package com.example.racedaq_cfg

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

@Composable
fun ConfigScreen(
    telemetry: TelemetryState,
    currentConfig: SystemConfig,
    currentGpsHz: Int,
    ain1Profile: AnalogSensorProfile,
    ain2Profile: AnalogSensorProfile,
    onSaveSystemConfig: (SystemConfig) -> Unit,
    onSaveAinProfiles: (AnalogSensorProfile, AnalogSensorProfile) -> Unit,
    onSetGpsRate: (Int) -> Unit,
    onBack: () -> Unit
) {
    var draftConfig by remember(currentConfig) { mutableStateOf(currentConfig) }
    var draftAin1 by remember(ain1Profile) { mutableStateOf(ain1Profile) }
    var draftAin2 by remember(ain2Profile) { mutableStateOf(ain2Profile) }
    var selectedTab by remember { mutableIntStateOf(0) }

    val tabs = listOf("1. ВХОДЫ", "2. ВЫХОДЫ", "3. CAN-ШИНА", "4. СИСТЕМА")

    BoxWithConstraints(
        modifier = Modifier
            .fillMaxSize()
            .background(Color(0xFF0D0F12))
    ) {
        val isPhone = maxWidth < 700.dp
        val pad = if (isPhone) 8.dp else 16.dp
        val gap = if (isPhone) 8.dp else 12.dp

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
                horizontalArrangement = Arrangement.spacedBy(6.dp)
            ) {
                tabs.forEachIndexed { index, title ->
                    val isSelected = selectedTab == index
                    Box(
                        modifier = Modifier
                            .weight(1f)
                            .clip(RoundedCornerShape(8.dp))
                            .background(if (isSelected) Color(0xFF0288D1) else Color(0xFF1A1E24))
                            .clickable { selectedTab = index }
                            .padding(vertical = if (isPhone) 10.dp else 12.dp),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = title,
                            color = Color.White,
                            fontWeight = if (isSelected) FontWeight.Bold else FontWeight.Normal,
                            fontSize = if (isPhone) 12.sp else 14.sp,
                            maxLines = 1
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
                        telemetry = telemetry,
                        din1 = draftConfig.din1,
                        din2 = draftConfig.din2,
                        ain1 = draftAin1,
                        ain2 = draftAin2,
                        onDin1Change = { draftConfig = draftConfig.copy(din1 = it) },
                        onDin2Change = { draftConfig = draftConfig.copy(din2 = it) },
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
                        connected = telemetry.connected,
                        currentGpsHz = currentGpsHz,
                        onSetGpsRate = onSetGpsRate
                    )
                }
            }
        }
    }
}

// ============================================================================
// ВКЛАДКА 1: НАСТРОЙКА ВХОДОВ DIN1/DIN2 И КАЛИБРОВКА AIN1/AIN2
// ============================================================================
@Composable
private fun InputsTab(
    isPhone: Boolean,
    telemetry: TelemetryState,
    din1: DinConfig,
    din2: DinConfig,
    ain1: AnalogSensorProfile,
    ain2: AnalogSensorProfile,
    onDin1Change: (DinConfig) -> Unit,
    onDin2Change: (DinConfig) -> Unit,
    onAin1Change: (AnalogSensorProfile) -> Unit,
    onAin2Change: (AnalogSensorProfile) -> Unit
) {
    val dinSection: @Composable (Modifier) -> Unit = { mod ->
        Column(modifier = mod, verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Text("ЦИФРОВЫЕ ВХОДЫ (DIN1 / DIN2)", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold, fontSize = 14.sp)
            DinCard(
                title = "Вход DIN 1 (P0.11)",
                liveText = if (din1.mode == DinMode.RPM) "${telemetry.rpm} RPM" else if (telemetry.din1) "1 (GND)" else "0",
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
                liveText = if (din2.mode == DinMode.RPM) "${telemetry.rpm} RPM" else if (telemetry.din2) "1 (GND)" else "0",
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

    val ainSection: @Composable (Modifier) -> Unit = { mod ->
        Column(modifier = mod, verticalArrangement = Arrangement.spacedBy(10.dp)) {
            Text("КАЛИБРОВКА АНАЛОГОВЫХ ДАТЧИКОВ", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold, fontSize = 14.sp)
            AinCard(
                isPhone = isPhone,
                title = "AIN 1: ${telemetry.mvAin1} мВ -> ${"%.2f".format(ain1.mvToValue(telemetry.mvAin1))} ${ain1.unit}",
                profile = ain1,
                onChange = onAin1Change
            )
            AinCard(
                isPhone = isPhone,
                title = "AIN 2: ${telemetry.mvAin2} мВ -> ${"%.2f".format(ain2.mvToValue(telemetry.mvAin2))} ${ain2.unit}",
                profile = ain2,
                onChange = onAin2Change
            )
        }
    }

    if (isPhone) {
        Column(
            modifier = Modifier.fillMaxSize().verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(14.dp)
        ) {
            dinSection(Modifier.fillMaxWidth())
            ainSection(Modifier.fillMaxWidth())
        }
    } else {
        Row(
            modifier = Modifier.fillMaxSize(),
            horizontalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            dinSection(Modifier.weight(1f).verticalScroll(rememberScrollState()))
            ainSection(Modifier.weight(1f).verticalScroll(rememberScrollState()))
        }
    }
}

@Composable
private fun DinCard(title: String, liveText: String, config: DinConfig, onChange: (DinConfig) -> Unit) {
    val pulsePresets = listOf(
        1 to "1 цил. 4T",
        2 to "2 цил. / 1 имп",
        4 to "4 цил. 4T",
        6 to "6 цил. 4T",
        8 to "8 цил. 4T"
    )

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(title, color = Color.White, fontWeight = FontWeight.Bold, fontSize = 14.sp)
            Text(liveText, color = Color(0xFF00E676), fontWeight = FontWeight.SemiBold, fontSize = 13.sp)
        }

        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(6.dp)
        ) {
            DinMode.entries.forEach { mode ->
                FilterChip(
                    selected = config.mode == mode,
                    onClick = { onChange(config.copy(mode = mode)) },
                    label = { Text(mode.title, fontSize = 12.sp) }
                )
            }
            FilterChip(
                selected = config.pullup,
                onClick = { onChange(config.copy(pullup = !config.pullup)) },
                label = { Text("Pull-Up", fontSize = 12.sp) }
            )
        }

        if (config.mode == DinMode.RPM) {
            Text("Делитель тахометра:", color = Color(0xFFB0BEC5), fontSize = 12.sp)
            Row(
                modifier = Modifier.horizontalScroll(rememberScrollState()),
                horizontalArrangement = Arrangement.spacedBy(6.dp)
            ) {
                pulsePresets.forEach { (p2r, label) ->
                    FilterChip(
                        selected = config.pulsesPer2Rev == p2r,
                        onClick = { onChange(config.copy(pulsesPer2Rev = p2r)) },
                        label = { Text(label, fontSize = 11.sp) }
                    )
                }
            }
        }
    }
}

@Composable
private fun AinCard(isPhone: Boolean, title: String, profile: AnalogSensorProfile, onChange: (AnalogSensorProfile) -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .border(1.dp, Color(0xFF2A313D), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(title, color = Color.White, fontWeight = FontWeight.Bold, fontSize = 13.sp)
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
                    onValueChange = { it.toFloatOrNull()?.let { v -> onChange(profile.copy(val1 = v)) } }
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
                    onValueChange = { it.toFloatOrNull()?.let { v -> onChange(profile.copy(val2 = v)) } }
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
                    onValueChange = { it.toFloatOrNull()?.let { v -> onChange(profile.copy(val1 = v)) } }
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
                    onValueChange = { it.toFloatOrNull()?.let { v -> onChange(profile.copy(val2 = v)) } }
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
            modifier = Modifier.fillMaxSize().verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            DoutRuleEditor(
                modifier = Modifier.fillMaxWidth(),
                title = "СИЛОВОЙ ВЫХОД DOUT 1 (P0.29)",
                config = dout1,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                onChange = onDout1Change
            )
            DoutRuleEditor(
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
                modifier = Modifier.weight(1f).verticalScroll(rememberScrollState()),
                title = "СИЛОВОЙ ВЫХОД DOUT 1 (P0.29)",
                config = dout1,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                onChange = onDout1Change
            )
            DoutRuleEditor(
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
                FilterChip(
                    selected = config.mode == mode,
                    onClick = { onChange(config.copy(mode = mode)) },
                    label = { Text(mode.title, fontSize = 12.sp) }
                )
            }
        }

        if (config.mode != OutputMode.MANUAL) {
            FilterChip(
                selected = config.invert,
                onClick = { onChange(config.copy(invert = !config.invert)) },
                label = { Text("Инверсия выхода (выключать при срабатывании)", fontSize = 12.sp) }
            )

            ConditionEditor(
                label = "Условие A (основное)",
                cond = config.condA,
                ain1Profile = ain1Profile,
                ain2Profile = ain2Profile,
                showDelay = true,
                onChange = { onChange(config.copy(condA = it)) }
            )

            if (config.mode == OutputMode.LOGIC_OR || config.mode == OutputMode.LOGIC_AND) {
                ConditionEditor(
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

    Column(
        modifier = Modifier
            .fillMaxWidth()
            .background(Color(0xFF1E232B), RoundedCornerShape(8.dp))
            .padding(10.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        Text(label, color = Color(0xFFFFD54F), fontSize = 13.sp, fontWeight = FontWeight.SemiBold)

        Row(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalAlignment = Alignment.CenterVertically) {
            Box(modifier = Modifier.weight(1f)) {
                OutlinedButton(
                    onClick = { expandedSource = true },
                    modifier = Modifier.fillMaxWidth(),
                    contentPadding = PaddingValues(horizontal = 10.dp, vertical = 6.dp)
                ) {
                    Text(cond.source.title, color = Color.White, maxLines = 1, fontSize = 12.sp)
                }
                DropdownMenu(expanded = expandedSource, onDismissRequest = { expandedSource = false }) {
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

            Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                CompareOp.entries.forEach { op ->
                    FilterChip(
                        selected = cond.op == op,
                        onClick = { onChange(cond.copy(op = op)) },
                        label = { Text(op.symbol, fontWeight = FontWeight.Bold) }
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
                onValueChange = { txt ->
                    thresholdText = txt
                    val parsed = txt.replace(',', '.').toFloatOrNull() ?: return@DarkTextField
                    val rawThreshold = when (cond.source) {
                        SignalSource.AIN1_MV -> ain1Profile.valueToMv(parsed)
                        SignalSource.AIN2_MV -> ain2Profile.valueToMv(parsed)
                        SignalSource.TWATR_C10, SignalSource.TOIL_C10 -> (parsed * 10f).toInt()
                        else -> parsed.toInt()
                    }
                    onChange(cond.copy(threshold = rawThreshold))
                }
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
        modifier = Modifier.fillMaxSize().verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(12.dp)
    ) {
        Column(verticalArrangement = Arrangement.spacedBy(6.dp)) {
            Text("СКОРОСТЬ ШИНЫ MCP2515:", color = Color.White, fontWeight = FontWeight.Bold, fontSize = 13.sp)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                listOf(250, 500, 1000).forEach { speed ->
                    FilterChip(
                        selected = canSpeedKbps == speed,
                        onClick = { onSpeedChange(speed) },
                        label = { Text("$speed кбит/с", fontSize = 12.sp) }
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
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Checkbox(
                            checked = slot.enabled,
                            onCheckedChange = { onSlotChange(index, slot.copy(enabled = it)) }
                        )
                        Text("CAN Слот $index", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold)
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                        FilterChip(
                            selected = slot.bigEndian,
                            onClick = { onSlotChange(index, slot.copy(bigEndian = !slot.bigEndian)) },
                            label = { Text(if (slot.bigEndian) "Big-End" else "Little-End", fontSize = 11.sp) }
                        )
                        FilterChip(
                            selected = slot.signed,
                            onClick = { onSlotChange(index, slot.copy(signed = !slot.signed)) },
                            label = { Text(if (slot.signed) "Signed" else "Unsigned", fontSize = 11.sp) }
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
// ВКЛАДКА 4: ЧАСТОТА GPS И ОБСЛУЖИВАНИЕ
// ============================================================================
@Composable
private fun SystemTab(
    connected: Boolean,
    currentGpsHz: Int,
    onSetGpsRate: (Int) -> Unit
) {
    Column(
        modifier = Modifier.fillMaxSize().verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        Text("ЧАСТОТА ОБНОВЛЕНИЯ GPS-МОДУЛЯ BE-880", color = Color(0xFF4FC3F7), fontWeight = FontWeight.Bold)
        Row(
            modifier = Modifier.horizontalScroll(rememberScrollState()),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            listOf(1, 5, 10, 20, 25).forEach { hz ->
                FilterChip(
                    selected = currentGpsHz == hz,
                    enabled = connected,
                    onClick = { onSetGpsRate(hz) },
                    label = { Text("$hz Гц", fontSize = 15.sp, fontWeight = FontWeight.Bold) }
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

@Composable
private fun DarkTextField(
    value: String,
    label: String,
    modifier: Modifier = Modifier,
    isNumber: Boolean = false,
    onValueChange: (String) -> Unit
) {
    OutlinedTextField(
        value = value,
        onValueChange = onValueChange,
        label = { Text(label, fontSize = 11.sp, maxLines = 1) },
        singleLine = true,
        modifier = modifier,
        keyboardOptions = if (isNumber) KeyboardOptions(keyboardType = KeyboardType.Number) else KeyboardOptions.Default,
        colors = OutlinedTextFieldDefaults.colors(
            focusedTextColor = Color.White,
            unfocusedTextColor = Color.White,
            focusedBorderColor = Color(0xFF4FC3F7),
            unfocusedBorderColor = Color(0xFF37474F),
            focusedLabelColor = Color(0xFF4FC3F7),
            unfocusedLabelColor = Color(0xFF90A4AE)
        )
    )
}