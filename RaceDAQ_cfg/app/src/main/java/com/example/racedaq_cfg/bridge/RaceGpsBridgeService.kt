package com.example.racedaq_cfg.bridge

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import androidx.core.app.NotificationCompat

class RaceGpsBridgeService : Service() {

    private lateinit var injector: MockLocationInjector
    private var wakeLock: PowerManager.WakeLock? = null

    private var packetCounter = 0
    private var lastRateCalcMs = 0L
    private var stoppedDueToError = false

    override fun onCreate() {
        super.onCreate()
        activeInstance = this
        injector = MockLocationInjector(this)
        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (intent?.action == ACTION_STOP) {
            stopSelf()
            return START_NOT_STICKY
        }

        stoppedDueToError = false

        try {
            val notification = buildNotification("Ожидание данных от RaceDAQ...")
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                startForeground(
                    NOTIFICATION_ID,
                    notification,
                    ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE or
                            ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION
                )
            } else {
                startForeground(NOTIFICATION_ID, notification)
            }

            injector.start()
            acquireWakeLock()
            BridgeStateHolder.update {
                it.copy(
                    isRunning = true,
                    errorMessage = null,
                    showMockSetupDialog = false
                )
            }
        } catch (e: Exception) {
            stoppedDueToError = true
            val isMockSetupNeeded = e is SecurityException || e is IllegalArgumentException
            val msg = if (isMockSetupNeeded) {
                "Выберите RaceDAQ_cfg для фиктивных местоположений в меню разработчика"
            } else {
                "Ошибка запуска моста: ${e.localizedMessage ?: e.javaClass.simpleName}"
            }
            BridgeStateHolder.update {
                it.copy(
                    isRunning = false,
                    errorMessage = msg,
                    showMockSetupDialog = isMockSetupNeeded
                )
            }
            stopSelf()
        }

        return START_STICKY
    }

    fun handleIncomingPacket(sourceProtocol: BridgeProtocol, packet: ByteArray) {
        val currentProtocol = BridgeStateHolder.state.value.protocol
        if (sourceProtocol != currentProtocol) return

        val telemetry = when (currentProtocol) {
            BridgeProtocol.RACEBOX -> injector.pushRaceBoxPacket(packet)
            BridgeProtocol.RACECHRONO_DIY -> injector.pushRaceChronoDiyGpsPacket(packet)
        } ?: return

        packetCounter++
        val now = System.currentTimeMillis()
        if (lastRateCalcMs == 0L) lastRateCalcMs = now

        var currentHz = BridgeStateHolder.state.value.updateRateHz
        if (now - lastRateCalcMs >= 1000L) {
            currentHz = packetCounter * 1000f / (now - lastRateCalcMs)
            packetCounter = 0
            lastRateCalcMs = now

            val statusText = "${"%.1f".format(currentHz)} Гц | Спутников: ${telemetry.satellites} | ${"%.0f".format(telemetry.speedKmh)} км/ч"
            val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            nm.notify(NOTIFICATION_ID, buildNotification(statusText))
        }

        BridgeStateHolder.update {
            it.copy(
                isBleConnected = true,
                fixType = telemetry.fixType,
                satellites = telemetry.satellites,
                speedKmh = telemetry.speedKmh,
                updateRateHz = currentHz
            )
        }
    }

    override fun onDestroy() {
        activeInstance = null
        injector.stop()
        releaseWakeLock()
        if (!stoppedDueToError) {
            BridgeStateHolder.resetRunningState()
        }
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    private fun acquireWakeLock() {
        val pm = getSystemService(Context.POWER_SERVICE) as PowerManager
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "RaceDAQ::GpsBridgeWakeLock").apply {
            acquire(4 * 60 * 60 * 1000L) // максимум 4 часа на сессию
        }
    }

    private fun releaseWakeLock() {
        try {
            if (wakeLock?.isHeld == true) wakeLock?.release()
        } catch (_: Exception) {}
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                CHANNEL_ID,
                "RaceDAQ GPS Bridge",
                NotificationManager.IMPORTANCE_LOW
            )
            val nm = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            nm.createNotificationChannel(channel)
        }
    }

    private fun buildNotification(text: String): Notification {
        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("RaceDAQ GPS трансляция (Mock GPS)")
            .setContentText(text)
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setOngoing(true)
            .build()
    }

    companion object {
        const val ACTION_STOP = "com.racedaq.bridge.STOP"
        private const val CHANNEL_ID = "racedaq_gps_bridge"
        private const val NOTIFICATION_ID = 1042

        @Volatile
        private var activeInstance: RaceGpsBridgeService? = null

        fun onRaceBoxPacket(packet: ByteArray) {
            activeInstance?.handleIncomingPacket(BridgeProtocol.RACEBOX, packet)
        }

        fun onDiyGpsPacket(packet: ByteArray) {
            activeInstance?.handleIncomingPacket(BridgeProtocol.RACECHRONO_DIY, packet)
        }
    }
}