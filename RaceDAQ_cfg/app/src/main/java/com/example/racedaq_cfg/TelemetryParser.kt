package com.example.racedaq_cfg

import java.nio.ByteBuffer
import java.nio.ByteOrder

data class TelemetryState(
    val connected: Boolean = false,
    val mvWatr: Int = 0,
    val mvOil: Int = 0,
    val mvAin1: Int = 0,
    val mvAin2: Int = 0,
    val tempWatrC: Float = -40f,
    val tempOilC: Float = -40f,
    val egtC: Float? = null, // null = обрыв термопары
    val ignVolts: Float = 0f,
    val din1: Boolean = false,
    val din2: Boolean = false,
    val dout1: Boolean = false,
    val dout2: Boolean = false,
    val rpm: Int = 0,
    val gpsSpeedKmh: Float = 0f,
    val gpsSats: Int = 0,
    val accXg: Float = 0f,
    val accYg: Float = 0f,
    val accZg: Float = 0f
)

object TelemetryParser {
    fun parseCanPacket(data: ByteArray, current: TelemetryState): TelemetryState {
        if (data.size < 4) return current
        val le = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        val pid = le.int.toLong() and 0xFFFFFFFFL
        val payload = data.copyOfRange(4, data.size)
        val be = ByteBuffer.wrap(payload).order(ByteOrder.BIG_ENDIAN)

        return when (pid) {
            0x100L -> {
                if (payload.size < 8) return current
                current.copy(
                    mvWatr = be.short.toInt() and 0xFFFF,
                    mvOil  = be.short.toInt() and 0xFFFF,
                    mvAin1 = be.short.toInt() and 0xFFFF,
                    mvAin2 = be.short.toInt() and 0xFFFF
                )
            }
            0x101L -> {
                if (payload.size < 6) return current
                val egtRaw = be.short.toInt() and 0xFFFF
                val ignMv  = be.short.toInt() and 0xFFFF
                val dinMask  = be.get().toInt() and 0xFF
                val doutMask = be.get().toInt() and 0xFF
                current.copy(
                    egtC = if (egtRaw == 0) null else egtRaw * 0.25f,
                    ignVolts = ignMv / 1000f,
                    din1 = (dinMask and 0x01) != 0,
                    din2 = (dinMask and 0x02) != 0,
                    dout1 = (doutMask and 0x01) != 0,
                    dout2 = (doutMask and 0x02) != 0
                )
            }
            0x102L -> {
                if (payload.size < 7) return current
                val d1 = payload[0].toInt() != 0
                val d2 = payload[1].toInt() != 0
                val o1 = payload[2].toInt() != 0
                val o2 = payload[3].toInt() != 0
                val rpm = ((payload[5].toInt() and 0xFF) shl 8) or (payload[6].toInt() and 0xFF)
                current.copy(din1 = d1, din2 = d2, dout1 = o1, dout2 = o2, rpm = rpm)
            }
            0x103L -> {
                if (payload.size < 8) return current
                current.copy(
                    accXg = be.short / 1000f,
                    accYg = be.short / 1000f,
                    accZg = be.short / 1000f
                )
            }
            0x104L -> {
                if (payload.size < 8) return current
                current.copy(
                    tempWatrC = be.short / 10f,
                    tempOilC  = be.short / 10f
                )
            }
            else -> current
        }
    }

    fun parseGpsPacket(data: ByteArray, current: TelemetryState): TelemetryState {
        if (data.size < 20) return current
        val sats = data[3].toInt() and 0x3F
        val rawSpeed = ((data[14].toInt() and 0xFF) shl 8) or (data[15].toInt() and 0xFF)
        val speedKmh = if ((rawSpeed and 0x8000) != 0) {
            (rawSpeed and 0x7FFF) / 10f
        } else {
            rawSpeed / 100f
        }
        return current.copy(gpsSpeedKmh = speedKmh, gpsSats = sats)
    }
}