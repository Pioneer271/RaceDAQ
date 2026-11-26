package com.example.racedaq_cfg.bridge

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import android.location.LocationManager
import android.location.provider.ProviderProperties
import android.os.Build
import android.os.SystemClock
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Calendar
import java.util.TimeZone

data class ParsedFix(
    val fixType: Int,      // 2 = 2D, 3 = 3D
    val satellites: Int,
    val speedKmh: Float
)

class MockLocationInjector(context: Context) {

    private val locationManager =
        context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private var baseElapsedNanos: Long = 0L
    private var baseItowMs: Long = -1L
    private val nmeaBuffer = StringBuilder()

    @SuppressLint("WrongConstant")
    fun start() {
        stop() // На случай если провайдер остался от предыдущей сессии

        locationManager.addTestProvider(
            LocationManager.GPS_PROVIDER,
            /* requiresNetwork = */ false,
            /* requiresSatellite = */ true,
            /* requiresCell = */ false,
            /* hasMonetaryCost = */ false,
            /* supportsAltitude = */ true,
            /* supportsSpeed = */ true,
            /* supportsBearing = */ true,
            ProviderProperties.POWER_USAGE_LOW,
            ProviderProperties.ACCURACY_FINE
        )
        locationManager.setTestProviderEnabled(LocationManager.GPS_PROVIDER, true)
        baseItowMs = -1L
        nmeaBuffer.clear()
    }

    fun stop() {
        try {
            locationManager.setTestProviderEnabled(LocationManager.GPS_PROVIDER, false)
            locationManager.removeTestProvider(LocationManager.GPS_PROVIDER)
        } catch (_: Exception) {
        }
    }

    /**
     * Универсальная точка входа для входящих байтов от BLE:
     * автоматически распознаёт 88-байтный пакет RaceBox Mini и NMEA-поток RaceChrono DIY.
     */
    fun pushIncomingBleBytes(data: ByteArray): ParsedFix? {
        if (data.isEmpty()) return null

        // 1. Бинарный пакет RaceBox Mini (UBX: 0xB5 0x62 0xFF 0x01, 88 байт)
        if (data.size == 88 && data[0] == 0xB5.toByte() && data[1] == 0x62.toByte()) {
            return pushRaceBoxPacket(data)
        }

        // 2. Текстовый поток RaceChrono DIY (NMEA + $RC3)
        val chunk = String(data, Charsets.US_ASCII)
        nmeaBuffer.append(chunk)
        if (nmeaBuffer.length > 4096) {
            nmeaBuffer.clear()
            return null
        }

        var lastFix: ParsedFix? = null
        var newlineIdx: Int
        while (nmeaBuffer.indexOf("\n").also { newlineIdx = it } >= 0) {
            val line = nmeaBuffer.substring(0, newlineIdx).trim()
            nmeaBuffer.delete(0, newlineIdx + 1)

            if (line.startsWith("\$GNRMC") || line.startsWith("\$GPRMC")) {
                parseAndInjectRmc(line)?.let { lastFix = it }
            }
        }
        return lastFix
    }

    fun pushRaceBoxPacket(packet: ByteArray): ParsedFix? {
        if (packet.size != 88) return null
        if (packet[0] != 0xB5.toByte() || packet[1] != 0x62.toByte()) return null
        if (packet[2] != 0xFF.toByte() || packet[3] != 0x01.toByte()) return null
        if (!verifyUbxChecksum(packet)) return null

        val buf = ByteBuffer.wrap(packet).order(ByteOrder.LITTLE_ENDIAN)

        val iTOW = buf.getInt(6).toLong() and 0xFFFFFFFFL
        val year = buf.getShort(10).toInt() and 0xFFFF
        val month = buf.get(12).toInt() and 0xFF
        val day = buf.get(13).toInt() and 0xFF
        val hour = buf.get(14).toInt() and 0xFF
        val min = buf.get(15).toInt() and 0xFF
        val sec = buf.get(16).toInt() and 0xFF
        val nano = buf.getInt(22)

        val fixStatus = buf.get(26).toInt() and 0xFF
        val fixFlags = buf.get(27).toInt() and 0xFF
        val numSV = buf.get(29).toInt() and 0xFF
        val latLonFlags = buf.get(72).toInt() and 0xFF

        // Проверяем валидность фиксации (2D/3D Fix + gnssFixOK)
        if (fixStatus < 2 || (fixFlags and 0x01) == 0 || (latLonFlags and 0x01) != 0) {
            return null
        }

        val lon = buf.getInt(30) / 1e7
        val lat = buf.getInt(34) / 1e7
        val mslAltMeters = buf.getInt(42) / 1000.0
        val hAccMeters = (buf.getInt(46).toLong() and 0xFFFFFFFFL) / 1000f
        val vAccMeters = (buf.getInt(50).toLong() and 0xFFFFFFFFL) / 1000f
        val speedMps = buf.getInt(54) / 1000f
        val headingDeg = (buf.getInt(58) / 1e5f).let { if (it < 0f) it + 360f else it }
        val sAccMps = (buf.getInt(62).toLong() and 0xFFFFFFFFL) / 1000f
        val headAccDeg = (buf.getInt(66).toLong() and 0xFFFFFFFFL) / 1e5f

        val nowElapsed = SystemClock.elapsedRealtimeNanos()
        if (baseItowMs < 0L || iTOW < baseItowMs || (iTOW - baseItowMs) > 5000L) {
            baseItowMs = iTOW
            baseElapsedNanos = nowElapsed
        }
        val monotonicNanos = baseElapsedNanos + (iTOW - baseItowMs) * 1_000_000L

        injectLocation(
            lat = lat,
            lon = lon,
            alt = mslAltMeters,
            speedMps = speedMps,
            bearingDeg = headingDeg,
            hAcc = hAccMeters,
            vAcc = vAccMeters,
            sAcc = sAccMps,
            bAcc = headAccDeg,
            utcMillis = buildUtcMillis(year, month, day, hour, min, sec, nano),
            elapsedNanos = monotonicNanos
        )

        return ParsedFix(
            fixType = fixStatus,
            satellites = numSV,
            speedKmh = speedMps * 3.6f
        )
    }

    private fun parseAndInjectRmc(nmeaLine: String): ParsedFix? {
        if (!verifyNmeaChecksum(nmeaLine)) return null
        val body = nmeaLine.substringBefore('*')
        val parts = body.split(',')
        if (parts.size < 10) return null

        // parts[2]: A = активная фиксация, V = нет фиксации
        if (parts[2] != "A") return null

        val lat = parseNmeaCoord(parts[3], parts[4]) ?: return null
        val lon = parseNmeaCoord(parts[5], parts[6]) ?: return null
        val speedKnots = parts[7].toFloatOrNull() ?: 0f
        val speedMps = speedKnots * 0.514444f
        val courseDeg = parts[8].toFloatOrNull() ?: 0f

        injectLocation(
            lat = lat,
            lon = lon,
            alt = 0.0,
            speedMps = speedMps,
            bearingDeg = courseDeg,
            hAcc = 1.5f,
            vAcc = 2.0f,
            sAcc = 0.2f,
            bAcc = 2.0f,
            utcMillis = System.currentTimeMillis(),
            elapsedNanos = SystemClock.elapsedRealtimeNanos()
        )

        return ParsedFix(
            fixType = 3,
            satellites = 12,
            speedKmh = speedMps * 3.6f
        )
    }
    /**
     * Парсинг стандартного 20-байтного GPS-пакета RaceChrono DIY (характеристика 0x0003).
     */
    fun pushRaceChronoDiyGpsPacket(data: ByteArray): ParsedFix? {
        if (data.size < 20) return null

        val buf = ByteBuffer.wrap(data).order(ByteOrder.BIG_ENDIAN)
        val fixAndSats = data[3].toInt() and 0xFF
        val fixQuality = (fixAndSats shr 6) and 0x03
        val sats = fixAndSats and 0x3F

        // fixQuality == 0 означает отсутствие координат
        if (fixQuality == 0 || sats == 0) return null

        val lat = buf.getInt(4) / 1e7
        val lon = buf.getInt(8) / 1e7

        val rawAlt = buf.getShort(12).toInt() and 0xFFFF
        val altMeters = if ((rawAlt and 0x8000) != 0) {
            ((rawAlt and 0x7FFF) - 500).toDouble()
        } else {
            (rawAlt / 10.0) - 500.0
        }

        val rawSpeed = buf.getShort(14).toInt() and 0xFFFF
        val speedKmh = if ((rawSpeed and 0x8000) != 0) {
            (rawSpeed and 0x7FFF) / 10f
        } else {
            rawSpeed / 100f
        }
        val speedMps = speedKmh / 3.6f

        val bearingDeg = (buf.getShort(16).toInt() and 0xFFFF) / 100f
        val hdop = (data[18].toInt() and 0xFF) / 10f
        val vdop = (data[19].toInt() and 0xFF) / 10f

        injectLocation(
            lat = lat,
            lon = lon,
            alt = altMeters,
            speedMps = speedMps,
            bearingDeg = bearingDeg,
            hAcc = (hdop * 1.5f).coerceAtLeast(0.8f),
            vAcc = (vdop * 1.5f).coerceAtLeast(1.0f),
            sAcc = 0.2f,
            bAcc = 2.0f,
            utcMillis = System.currentTimeMillis(),
            elapsedNanos = SystemClock.elapsedRealtimeNanos()
        )

        return ParsedFix(
            fixType = if (fixQuality >= 2) 3 else 2,
            satellites = sats,
            speedKmh = speedKmh
        )
    }

    private fun injectLocation(
        lat: Double,
        lon: Double,
        alt: Double,
        speedMps: Float,
        bearingDeg: Float,
        hAcc: Float,
        vAcc: Float,
        sAcc: Float,
        bAcc: Float,
        utcMillis: Long,
        elapsedNanos: Long
    ) {
        val loc = Location(LocationManager.GPS_PROVIDER).apply {
            latitude = lat
            longitude = lon
            altitude = alt
            speed = speedMps
            bearing = bearingDeg
            accuracy = hAcc.coerceAtLeast(0.5f)
            time = utcMillis
            elapsedRealtimeNanos = elapsedNanos

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                verticalAccuracyMeters = vAcc
                speedAccuracyMetersPerSecond = sAcc
                bearingAccuracyDegrees = bAcc
            }
        }
        locationManager.setTestProviderLocation(LocationManager.GPS_PROVIDER, loc)
    }

    private fun parseNmeaCoord(raw: String, dir: String): Double? {
        val value = raw.toDoubleOrNull() ?: return null
        val degrees = (value / 100.0).toInt()
        val minutes = value - degrees * 100.0
        var decimal = degrees + minutes / 60.0
        if (dir == "S" || dir == "W") decimal = -decimal
        return decimal
    }

    private fun verifyUbxChecksum(data: ByteArray): Boolean {
        var a = 0
        var b = 0
        for (i in 2 until 86) {
            a = (a + (data[i].toInt() and 0xFF)) and 0xFF
            b = (b + a) and 0xFF
        }
        return a == (data[86].toInt() and 0xFF) && b == (data[87].toInt() and 0xFF)
    }

    private fun verifyNmeaChecksum(line: String): Boolean {
        val starIdx = line.indexOf('*')
        if (starIdx < 2 || starIdx + 3 > line.length) return false
        var cs = 0
        for (i in 1 until starIdx) {
            cs = cs xor line[i].code
        }
        val expected = line.substring(starIdx + 1, starIdx + 3).toIntOrNull(16) ?: return false
        return cs == expected
    }

    private fun buildUtcMillis(
        y: Int, m: Int, d: Int, h: Int, min: Int, s: Int, nano: Int
    ): Long {
        val cal = Calendar.getInstance(TimeZone.getTimeZone("UTC")).apply {
            set(Calendar.YEAR, y)
            set(Calendar.MONTH, (m - 1).coerceIn(0, 11))
            set(Calendar.DAY_OF_MONTH, d)
            set(Calendar.HOUR_OF_DAY, h)
            set(Calendar.MINUTE, min)
            set(Calendar.SECOND, s)
            set(Calendar.MILLISECOND, (nano / 1_000_000).coerceIn(0, 999))
        }
        return cal.timeInMillis
    }
}