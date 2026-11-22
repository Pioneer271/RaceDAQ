package com.example.racedaq_cfg

import java.nio.ByteBuffer
import java.nio.ByteOrder

enum class DinMode(val id: Int, val title: String) {
    DIGITAL(0, "Дискретный (0/1)"),
    RPM(1, "Тахометр (RPM)");
    companion object { fun fromId(id: Int) = entries.firstOrNull { it.id == id } ?: DIGITAL }
}

enum class OutputMode(val id: Int, val title: String) {
    MANUAL(0, "Ручной (BLE)"),
    SINGLE(1, "Одно условие (A)"),
    LOGIC_OR(2, "Условие A ИЛИ B"),
    LOGIC_AND(3, "Условие A И B");
    companion object { fun fromId(id: Int) = entries.firstOrNull { it.id == id } ?: MANUAL }
}

enum class SignalSource(val id: Int, val title: String, val unit: String) {
    NONE(0, "Отключено", ""),
    AIN1_MV(1, "Аналог 1 (AIN1)", "мВ"),
    AIN2_MV(2, "Аналог 2 (AIN2)", "мВ"),
    TWATR_C10(3, "Темп. ОЖ (T_WATR)", "0.1 °C"),
    TOIL_C10(4, "Темп. масла (T_OIL)", "0.1 °C"),
    EGT_C(5, "Выхлоп (EGT)", "°C"),
    IGN_MV(6, "Бортсеть (IGN)", "мВ"),
    DIN1(7, "Вход DIN1", "0/1"),
    DIN2(8, "Вход DIN2", "0/1"),
    RPM(9, "Обороты (DIN RPM)", "об/мин"),
    GPS_SPEED(10, "Скорость GPS", "км/ч"),
    CAN_SLOT0(11, "CAN Слот 0", "ед."),
    CAN_SLOT1(12, "CAN Слот 1", "ед."),
    CAN_SLOT2(13, "CAN Слот 2", "ед."),
    CAN_SLOT3(14, "CAN Слот 3", "ед.");
    companion object { fun fromId(id: Int) = entries.firstOrNull { it.id == id } ?: NONE }
}

enum class CompareOp(val id: Int, val symbol: String) {
    GT(0, ">"),
    LT(1, "<"),
    EQ(2, "=");
    companion object { fun fromId(id: Int) = entries.firstOrNull { it.id == id } ?: GT }
}

data class DinConfig(
    val mode: DinMode = DinMode.DIGITAL,
    val pulsesPer2Rev: Int = 4, // 4 = 4 цил. 4T (2 имп/об)
    val pullup: Boolean = true
)

data class RuleCondition(
    val source: SignalSource = SignalSource.NONE,
    val op: CompareOp = CompareOp.GT,
    val threshold: Int = 0,
    val hysteresis: Int = 0,
    val delayMs: Int = 0
)

data class DoutConfig(
    val mode: OutputMode = OutputMode.MANUAL,
    val invert: Boolean = false,
    val condA: RuleCondition = RuleCondition(),
    val condB: RuleCondition = RuleCondition()
)

data class CanSignalSlot(
    val enabled: Boolean = false,
    val canId: Long = 0x280,
    val startByte: Int = 0,
    val length: Int = 2,
    val bigEndian: Boolean = true,
    val signed: Boolean = false,
    val mul: Int = 100,   // 100 = x1.0
    val offset: Int = 0
)

data class SystemConfig(
    val din1: DinConfig = DinConfig(),
    val din2: DinConfig = DinConfig(),
    val dout1: DoutConfig = DoutConfig(),
    val dout2: DoutConfig = DoutConfig(),
    val canSpeedKbps: Int = 500,
    val canSlots: List<CanSignalSlot> = List(4) { CanSignalSlot() }
) {
    companion object {
        const val MAGIC: Byte = 0xDA.toByte()
        const val VERSION: Byte = 2
        const val PACKET_SIZE = 102

        fun fromBytes(bytes: ByteArray): SystemConfig? {
            if (bytes.size != PACKET_SIZE) return null
            val buf = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
            if (buf.get() != MAGIC || buf.get() != VERSION) return null

            fun readDin() = DinConfig(
                mode = DinMode.fromId(buf.get().toInt() and 0xFF),
                pulsesPer2Rev = buf.get().toInt() and 0xFF,
                pullup = (buf.get().toInt() and 0xFF) != 0
            )

            fun readCond() = RuleCondition(
                source = SignalSource.fromId(buf.get().toInt() and 0xFF),
                op = CompareOp.fromId(buf.get().toInt() and 0xFF),
                threshold = buf.int,
                hysteresis = buf.short.toInt() and 0xFFFF,
                delayMs = buf.short.toInt() and 0xFFFF
            )

            fun readDout() = DoutConfig(
                mode = OutputMode.fromId(buf.get().toInt() and 0xFF),
                invert = (buf.get().toInt() and 0xFF) != 0,
                condA = readCond(),
                condB = readCond()
            )

            val d1 = readDin()
            val d2 = readDin()
            val out1 = readDout()
            val out2 = readDout()
            val speed = buf.short.toInt() and 0xFFFF

            val slots = List(4) {
                val en = (buf.get().toInt() and 0xFF) != 0
                val id = buf.int.toLong() and 0xFFFFFFFFL
                val start = buf.get().toInt() and 0xFF
                val len = buf.get().toInt() and 0xFF
                val flags = buf.get().toInt() and 0xFF
                val mul = buf.short.toInt()
                val off = buf.short.toInt()
                CanSignalSlot(
                    enabled = en,
                    canId = id,
                    startByte = start,
                    length = len,
                    bigEndian = (flags and 0x01) != 0,
                    signed = (flags and 0x02) != 0,
                    mul = mul,
                    offset = off
                )
            }
            return SystemConfig(d1, d2, out1, out2, speed, slots)
        }
    }

    fun toBytes(): ByteArray {
        val buf = ByteBuffer.allocate(PACKET_SIZE).order(ByteOrder.LITTLE_ENDIAN)
        buf.put(MAGIC)
        buf.put(VERSION)

        fun writeDin(d: DinConfig) {
            buf.put(d.mode.id.toByte())
            buf.put(d.pulsesPer2Rev.toByte())
            buf.put(if (d.pullup) 1 else 0)
        }

        fun writeCond(c: RuleCondition) {
            buf.put(c.source.id.toByte())
            buf.put(c.op.id.toByte())
            buf.putInt(c.threshold)
            buf.putShort(c.hysteresis.toShort())
            buf.putShort(c.delayMs.toShort())
        }

        fun writeDout(o: DoutConfig) {
            buf.put(o.mode.id.toByte())
            buf.put(if (o.invert) 1 else 0)
            writeCond(o.condA)
            writeCond(o.condB)
        }

        writeDin(din1)
        writeDin(din2)
        writeDout(dout1)
        writeDout(dout2)
        buf.putShort(canSpeedKbps.toShort())

        for (i in 0 until 4) {
            val s = canSlots.getOrElse(i) { CanSignalSlot() }
            buf.put(if (s.enabled) 1 else 0)
            buf.putInt(s.canId.toInt())
            buf.put(s.startByte.toByte())
            buf.put(s.length.toByte())
            var flags = 0
            if (s.bigEndian) flags = flags or 0x01
            if (s.signed) flags = flags or 0x02
            buf.put(flags.toByte())
            buf.putShort(s.mul.toShort())
            buf.putShort(s.offset.toShort())
        }
        return buf.array()
    }
}

// Локальная калибровка аналогового входа на планшете (по 2 точкам)
data class AnalogSensorProfile(
    val name: String = "Давление масла",
    val unit: String = "бар",
    val v1Mv: Int = 500,
    val val1: Float = 0.0f,
    val v2Mv: Int = 4500,
    val val2: Float = 10.0f
) {
    fun mvToValue(mv: Int): Float {
        if (v2Mv == v1Mv) return val1
        return val1 + (mv - v1Mv).toFloat() * (val2 - val1) / (v2Mv - v1Mv).toFloat()
    }

    fun valueToMv(targetVal: Float): Int {
        if (val2 == val1) return v1Mv
        return (v1Mv + (targetVal - val1) * (v2Mv - v1Mv) / (val2 - val1)).toInt().coerceIn(0, 5000)
    }
}