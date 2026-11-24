package com.example.racedaq_cfg

import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.roundToInt

enum class DinMode(val code: Byte, val title: String) {
    DIGITAL(0, "Дискретный (0/1)"),
    RPM(1, "Тахометр (RPM)");

    companion object {
        fun fromCode(code: Byte): DinMode = entries.firstOrNull { it.code == code } ?: DIGITAL
    }
}

data class DinConfig(
    val mode: DinMode = DinMode.DIGITAL,
    val pulsesPer2Rev: Int = 4,
    val pullup: Boolean = true
) {
    fun writeTo(buf: ByteBuffer) {
        buf.put(mode.code)
        buf.put(pulsesPer2Rev.toByte())
        buf.put(if (pullup) 1.toByte() else 0.toByte())
    }

    companion object {
        fun readFrom(buf: ByteBuffer): DinConfig {
            val mode = DinMode.fromCode(buf.get())
            val p2r = buf.get().toInt() and 0xFF
            val pullup = buf.get().toInt() != 0
            return DinConfig(mode = mode, pulsesPer2Rev = if (p2r == 0) 4 else p2r, pullup = pullup)
        }
    }
}

enum class TempPullupMode(val code: Byte, val label: String) {
    LOW_OHM(0, "Низкоомный (1–5 кОм)"),
    HIGH_OHM(1, "Высокоомный (20–100 кОм)");

    companion object {
        fun fromCode(code: Byte): TempPullupMode = entries.firstOrNull { it.code == code } ?: LOW_OHM
    }
}

data class TempChannelConfig(
    val pullupMode: TempPullupMode = TempPullupMode.LOW_OHM,
    val r25Ohm: Float = 2750f,
    val betaK: Int = 3950
) {
    fun writeTo(buf: ByteBuffer) {
        buf.put(pullupMode.code)
        buf.putFloat(r25Ohm)
        buf.putShort(betaK.toShort())
    }

    companion object {
        fun readFrom(buf: ByteBuffer): TempChannelConfig {
            val mode = TempPullupMode.fromCode(buf.get())
            val r25 = buf.float
            val beta = buf.short.toInt() and 0xFFFF
            return TempChannelConfig(pullupMode = mode, r25Ohm = r25, betaK = beta)
        }
    }
}

data class TempSensorPreset(
    val name: String,
    val config: TempChannelConfig
)

val TEMP_SENSOR_PRESETS = listOf(
    TempSensorPreset("GM / Штатный", TempChannelConfig(TempPullupMode.LOW_OHM, 2435f, 3840)),
    TempSensorPreset("Bosch NTC",    TempChannelConfig(TempPullupMode.LOW_OHM, 2050f, 3450)),
    TempSensorPreset("VAG / BMW",    TempChannelConfig(TempPullupMode.LOW_OHM, 2500f, 3900)),
    TempSensorPreset("Defi (55 кОм)", TempChannelConfig(TempPullupMode.HIGH_OHM, 47100f, 3900)),
    TempSensorPreset("Свой NTC",     TempChannelConfig(TempPullupMode.LOW_OHM, 2400f, 3880))
)

enum class OutputMode(val code: Byte, val title: String) {
    MANUAL(0, "Ручной"),
    SINGLE(1, "Условие A"),
    LOGIC_OR(2, "A ИЛИ B"),
    LOGIC_AND(3, "A И B");

    companion object {
        fun fromCode(code: Byte): OutputMode = entries.firstOrNull { it.code == code } ?: MANUAL
    }
}

enum class SignalSource(val code: Byte, val title: String, val unit: String) {
    NONE(0, "Выключено", ""),
    AIN1_MV(1, "Аналоговый вход 1 (AIN1)", "мВ"),
    AIN2_MV(2, "Аналоговый вход 2 (AIN2)", "мВ"),
    TWATR_C10(3, "Температура воды (T_WATR)", "°C"),
    TOIL_C10(4, "Температура масла (T_OIL)", "°C"),
    EGT_C(5, "Температура выхлопа (EGT)", "°C"),
    IGN_MV(6, "Напряжение зажигания (IGN)", "мВ"),
    DIN1(7, "Цифровой вход 1 (DIN1)", "0/1"),
    DIN2(8, "Цифровой вход 2 (DIN2)", "0/1"),
    RPM(9, "Обороты двигателя (RPM)", "об/мин"),
    GPS_SPEED(10, "Скорость GPS", "км/ч"),
    CAN_SLOT0(11, "CAN Слот 0", "ед."),
    CAN_SLOT1(12, "CAN Слот 1", "ед."),
    CAN_SLOT2(13, "CAN Слот 2", "ед."),
    CAN_SLOT3(14, "CAN Слот 3", "ед.");

    companion object {
        fun fromCode(code: Byte): SignalSource = entries.firstOrNull { it.code == code } ?: NONE
    }
}

enum class CompareOp(val code: Byte, val symbol: String) {
    GT(0, ">"),
    LT(1, "<"),
    EQ(2, "==");

    companion object {
        fun fromCode(code: Byte): CompareOp = entries.firstOrNull { it.code == code } ?: GT
    }
}

data class RuleCondition(
    val source: SignalSource = SignalSource.NONE,
    val op: CompareOp = CompareOp.GT,
    val threshold: Int = 0,
    val hysteresis: Int = 0,
    val delayMs: Int = 0
) {
    fun writeTo(buf: ByteBuffer) {
        buf.put(source.code)
        buf.put(op.code)
        buf.putInt(threshold)
        buf.putShort(hysteresis.toShort())
        buf.putShort(delayMs.toShort())
    }

    companion object {
        fun readFrom(buf: ByteBuffer): RuleCondition {
            val src = SignalSource.fromCode(buf.get())
            val op = CompareOp.fromCode(buf.get())
            val thr = buf.int
            val hyst = buf.short.toInt() and 0xFFFF
            val delay = buf.short.toInt() and 0xFFFF
            return RuleCondition(source = src, op = op, threshold = thr, hysteresis = hyst, delayMs = delay)
        }
    }
}

data class DoutConfig(
    val mode: OutputMode = OutputMode.MANUAL,
    val invert: Boolean = false,
    val condA: RuleCondition = RuleCondition(),
    val condB: RuleCondition = RuleCondition()
) {
    fun writeTo(buf: ByteBuffer) {
        buf.put(mode.code)
        buf.put(if (invert) 1.toByte() else 0.toByte())
        condA.writeTo(buf)
        condB.writeTo(buf)
    }

    companion object {
        fun readFrom(buf: ByteBuffer): DoutConfig {
            val mode = OutputMode.fromCode(buf.get())
            val invert = buf.get().toInt() != 0
            val condA = RuleCondition.readFrom(buf)
            val condB = RuleCondition.readFrom(buf)
            return DoutConfig(mode = mode, invert = invert, condA = condA, condB = condB)
        }
    }
}

data class CanSignalSlot(
    val enabled: Boolean = false,
    val canId: Long = 0,
    val startByte: Int = 0,
    val length: Int = 2,
    val bigEndian: Boolean = true,
    val signed: Boolean = false,
    val mul: Int = 100,
    val offset: Int = 0
) {
    fun writeTo(buf: ByteBuffer) {
        buf.put(if (enabled) 1.toByte() else 0.toByte())
        buf.putInt(canId.toInt())
        buf.put(startByte.toByte())
        buf.put(length.toByte())
        var flags = 0
        if (bigEndian) flags = flags or 0x01
        if (signed) flags = flags or 0x02
        buf.put(flags.toByte())
        buf.putShort(mul.toShort())
        buf.putShort(offset.toShort())
    }

    companion object {
        fun readFrom(buf: ByteBuffer): CanSignalSlot {
            val enabled = buf.get().toInt() != 0
            val canId = buf.int.toLong() and 0xFFFFFFFFL
            val startByte = buf.get().toInt() and 0xFF
            val length = buf.get().toInt() and 0xFF
            val flags = buf.get().toInt() and 0xFF
            val mul = buf.short.toInt()
            val offset = buf.short.toInt()
            return CanSignalSlot(
                enabled = enabled,
                canId = canId,
                startByte = startByte,
                length = if (length in 1..2) length else 2,
                bigEndian = (flags and 0x01) != 0,
                signed = (flags and 0x02) != 0,
                mul = mul,
                offset = offset
            )
        }
    }
}

data class SystemConfig(
    val din1: DinConfig = DinConfig(),
    val din2: DinConfig = DinConfig(),
    val dout1: DoutConfig = DoutConfig(),
    val dout2: DoutConfig = DoutConfig(),
    val canSpeedKbps: Int = 500,
    val canSlots: List<CanSignalSlot> = List(4) { CanSignalSlot() },
    val tempWat: TempChannelConfig = TempChannelConfig(),
    val tempOil: TempChannelConfig = TempChannelConfig()
) {
    fun toBytes(): ByteArray {
        val buf = ByteBuffer.allocate(PACKET_SIZE).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(0xDA.toByte())
        buf.put(3.toByte())
        din1.writeTo(buf)
        din2.writeTo(buf)
        dout1.writeTo(buf)
        dout2.writeTo(buf)
        buf.putShort(canSpeedKbps.toShort())
        for (i in 0 until 4) {
            canSlots.getOrElse(i) { CanSignalSlot() }.writeTo(buf)
        }
        tempWat.writeTo(buf)
        tempOil.writeTo(buf)
        return buf.array()
    }

    companion object {
        const val PACKET_SIZE = 116

        fun fromBytes(bytes: ByteArray?): SystemConfig? {
            if (bytes == null || bytes.size < PACKET_SIZE) return null
            val buf = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
            val magic = buf.get()
            val version = buf.get().toInt() and 0xFF
            if (magic != 0xDA.toByte() || version != 3) return null

            return SystemConfig(
                din1 = DinConfig.readFrom(buf),
                din2 = DinConfig.readFrom(buf),
                dout1 = DoutConfig.readFrom(buf),
                dout2 = DoutConfig.readFrom(buf),
                canSpeedKbps = buf.short.toInt() and 0xFFFF,
                canSlots = List(4) { CanSignalSlot.readFrom(buf) },
                tempWat = TempChannelConfig.readFrom(buf),
                tempOil = TempChannelConfig.readFrom(buf)
            )
        }
    }
}

data class AnalogSensorProfile(
    val name: String = "Буст",
    val unit: String = "бар",
    val v1Mv: Int = 500,
    val val1: Float = -1.0f,
    val v2Mv: Int = 4500,
    val val2: Float = 2.0f
) {
    fun mvToValue(mv: Int): Float {
        if (v2Mv == v1Mv) return val1
        return val1 + (mv - v1Mv).toFloat() * (val2 - val1) / (v2Mv - v1Mv).toFloat()
    }

    fun valueToMv(value: Float): Int {
        if (val2 == val1) return v1Mv
        return (v1Mv + (value - val1) * (v2Mv - v1Mv).toFloat() / (val2 - val1)).roundToInt()
    }
}

data class AnalogSensorCategory(
    val title: String,
    val sensors: List<Pair<String, AnalogSensorProfile>>
)

val ANALOG_SENSOR_CATEGORIES = listOf(
    AnalogSensorCategory(
        title = "Давл. впуск",
        sensors = listOf(
            "Bosch 3 bar" to AnalogSensorProfile("Давление наддува", "бар", 400, -0.8f, 4650, 2.0f),
            "MPX4250" to AnalogSensorProfile("Давление наддува", "бар", 200, -0.8f, 4800, 1.5f),
            "Belais 3 bar" to AnalogSensorProfile("Давление наддува", "бар", 1000, 0.0f, 4500, 3.0f),
            "Атмо 1 bar" to AnalogSensorProfile("ДАД (впуск)", "кПа", 400, 10.0f, 4650, 105.0f)
        )
    ),
    AnalogSensorCategory(
        title = "ШЛЗ (AFR)",
        sensors = listOf(
            "Innovate LC-2" to AnalogSensorProfile("AFR (Innovate)", "AFR", 0, 7.35f, 5000, 22.39f),
            "AEM X-Series" to AnalogSensorProfile("AFR (AEM)", "AFR", 500, 8.5f, 4500, 18.0f)
        )
    ),
    AnalogSensorCategory(
        title = "Давл. масла / топлива",
        sensors = listOf(
            "0–10 бар (0.5–4.5В)" to AnalogSensorProfile("Давление масла", "бар", 500, 0.0f, 4500, 10.0f),
            "0–5 бар (0.5–4.5В)" to AnalogSensorProfile("Давление топлива", "бар", 500, 0.0f, 4500, 5.0f)
        )
    )
)

data class DeviceInfoPacket(
    val hwVersion: String = "Ожидание...",
    val fwVersion: String = "Ожидание...",
    val hwCaps: Int = 0,
    val buildId: Int = 0
) {
    fun capsSummary(): String {
        val list = mutableListOf<String>()
        if ((hwCaps and (1 shl 0)) != 0) list.add("Switch-NTC")
        if ((hwCaps and (1 shl 1)) != 0) list.add("EGT MAX6675")
        if ((hwCaps and (1 shl 2)) != 0) list.add("IMU LSM6DS3")
        if ((hwCaps and (1 shl 3)) != 0) list.add("CAN MCP2515")
        if ((hwCaps and (1 shl 4)) != 0) list.add("GPS")
        return if (list.isEmpty()) "Базовая конфигурация" else list.joinToString(", ")
    }

    companion object {
        const val PACKET_SIZE = 9

        fun fromBytes(bytes: ByteArray?): DeviceInfoPacket? {
            if (bytes == null || bytes.size < PACKET_SIZE) return null
            val buf = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
            val hwMaj = buf.get().toInt() and 0xFF
            val hwMin = buf.get().toInt() and 0xFF
            val hwPat = buf.get().toInt() and 0xFF
            val fwMaj = buf.get().toInt() and 0xFF
            val fwMin = buf.get().toInt() and 0xFF
            val fwPat = buf.get().toInt() and 0xFF
            val caps = buf.get().toInt() and 0xFF
            val build = buf.short.toInt() and 0xFFFF

            val hwStr = if (hwMaj == 0 && hwMin == 0 && hwPat == 0) {
                "Не задана (UICR 0x10001080)"
            } else {
                "V$hwMaj.$hwMin.$hwPat"
            }
            return DeviceInfoPacket(
                hwVersion = hwStr,
                fwVersion = "V$fwMaj.$fwMin.$fwPat (build $build)",
                hwCaps = caps,
                buildId = build
            )
        }
    }
}
data class AlarmLimit(
    val enabled: Boolean = true,
    val min: Float? = null,
    val max: Float? = null
) {
    fun isTriggered(value: Float?): Boolean {
        if (!enabled || value == null || value.isNaN()) return false
        if (min != null && value < min) return true
        if (max != null && value > max) return true
        return false
    }
}

data class DashboardAlarmLimits(
    val tempWat: AlarmLimit = AlarmLimit(enabled = true, min = null, max = 102f),
    val tempOil: AlarmLimit = AlarmLimit(enabled = true, min = null, max = 125f),
    val egt: AlarmLimit = AlarmLimit(enabled = true, min = null, max = 900f),
    val ain1: AlarmLimit = AlarmLimit(enabled = true, min = 1.2f, max = null),
    val ain2: AlarmLimit = AlarmLimit(enabled = true, min = null, max = 1.8f),
    val ign: AlarmLimit = AlarmLimit(enabled = true, min = 12.0f, max = 15.2f),
    val shiftRpm: Int = 6800,
    val maxRpm: Int = 8000
)
data class DashboardVisibility(
    val showRpm: Boolean = true,
    val showTempWat: Boolean = true,
    val showTempOil: Boolean = true,
    val showEgt: Boolean = true,
    val showAin1: Boolean = true,
    val showAin2: Boolean = true,
    val showGps: Boolean = true,
    val showDout: Boolean = true
)