package com.example.racedaq_cfg

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.ParcelUuid
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import java.util.UUID
import com.example.racedaq_cfg.bridge.RaceGpsBridgeService

@SuppressLint("MissingPermission")
class RaceDaqBleManager(private val context: Context) {

    companion object {
        val SERVICE_UUID: UUID     = UUID.fromString("00001ff8-0000-1000-8000-00805f9b34fb")
        val CHAR_CAN_MAIN: UUID    = UUID.fromString("00000001-0000-1000-8000-00805f9b34fb")
        val CHAR_GPS_MAIN: UUID    = UUID.fromString("00000003-0000-1000-8000-00805f9b34fb")
        val CHAR_IO_CTRL: UUID     = UUID.fromString("00000006-0000-1000-8000-00805f9b34fb")
        val CHAR_GPS_RATE: UUID    = UUID.fromString("00000007-0000-1000-8000-00805f9b34fb")
        val CHAR_SYS_CONFIG: UUID  = UUID.fromString("00000008-0000-1000-8000-00805f9b34fb")
        val CHAR_DEVICE_INFO: UUID = UUID.fromString("00000009-0000-1000-8000-00805f9b34fb")
        val CCCD_UUID: UUID        = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        val NUS_SERVICE_UUID: UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
        val NUS_TX_UUID: UUID      = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
    }


    private val btAdapter: BluetoothAdapter? =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    private val mainHandler = Handler(Looper.getMainLooper())

    private var gatt: BluetoothGatt? = null
    private var lastDevice: BluetoothDevice? = null
    private var isScanning = false
    private var isConnecting = false
    private val notifyQueue = ArrayDeque<BluetoothGattCharacteristic>()

    private val _telemetry = MutableStateFlow(TelemetryState())
    val telemetry = _telemetry.asStateFlow()

    private val _config = MutableStateFlow(SystemConfig())
    val config = _config.asStateFlow()

    private val _gpsRateHz = MutableStateFlow(10)
    val gpsRateHz = _gpsRateHz.asStateFlow()

    private val _deviceInfo = MutableStateFlow(DeviceInfoPacket())
    val deviceInfo = _deviceInfo.asStateFlow()

    private val reconnectRunnable = Runnable { startAutoConnect() }
    private val directConnectTimeoutRunnable = Runnable {
        if (isConnecting && !_telemetry.value.connected) {
            isConnecting = false
            closeGattSafe()
            startBleScan()
        }
    }

    fun startAutoConnect() {
        val adapter = btAdapter ?: return
        if (!adapter.isEnabled) return

        // Не трогаем активное или устанавливаемое соединение
        if (_telemetry.value.connected || isConnecting || isScanning) return

        mainHandler.removeCallbacks(reconnectRunnable)

        // Если устройство уже находили ранее — подключаемся напрямую без сканирования (работает и в фоне)
        val knownDevice = lastDevice
        if (knownDevice != null) {
            connectToDevice(knownDevice, withScanFallback = true)
        } else {
            startBleScan()
        }
    }

    private fun startBleScan() {
        val scanner = btAdapter?.bluetoothLeScanner ?: return
        if (isScanning || isConnecting || _telemetry.value.connected) return

        closeGattSafe()

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        isScanning = true
        runCatching {
            scanner.startScan(null, settings, scanCallback)
        }.onFailure {
            isScanning = false
        }
    }

    private fun stopScanSafe() {
        if (!isScanning) return
        isScanning = false
        runCatching { btAdapter?.bluetoothLeScanner?.stopScan(scanCallback) }
    }

    private fun connectToDevice(device: BluetoothDevice, withScanFallback: Boolean = false) {
        if (isConnecting || _telemetry.value.connected) return
        stopScanSafe()
        closeGattSafe()

        isConnecting = true
        lastDevice = device

        if (withScanFallback) {
            mainHandler.removeCallbacks(directConnectTimeoutRunnable)
            mainHandler.postDelayed(directConnectTimeoutRunnable, 4000L)
        }

        gatt = device.connectGatt(
            context,
            false,
            gattCallback,
            BluetoothDevice.TRANSPORT_LE
        )
    }

    private fun closeGattSafe() {
        gatt?.let {
            runCatching { it.disconnect() }
            runCatching { it.close() }
        }
        gatt = null
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            if (!isScanning || isConnecting || _telemetry.value.connected) return
            val name = result.device.name ?: result.scanRecord?.deviceName ?: ""
            val hasUuid = result.scanRecord?.serviceUuids?.contains(ParcelUuid(SERVICE_UUID)) == true

            if (name.contains("RACEDAQ", ignoreCase = true) ||
                name.contains("RaceBox", ignoreCase = true) ||
                hasUuid
            ) {
                connectToDevice(result.device, withScanFallback = false)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            isScanning = false
            mainHandler.removeCallbacks(reconnectRunnable)
            mainHandler.postDelayed(reconnectRunnable, 2000L)
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED && status == BluetoothGatt.GATT_SUCCESS) {
                mainHandler.removeCallbacks(directConnectTimeoutRunnable)
                mainHandler.removeCallbacks(reconnectRunnable)
                isConnecting = false
                lastDevice = g.device
                _telemetry.update { it.copy(connected = true) }
                g.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED || status != BluetoothGatt.GATT_SUCCESS) {
                mainHandler.removeCallbacks(directConnectTimeoutRunnable)
                isConnecting = false
                _telemetry.update { it.copy(connected = false) }
                runCatching { g.close() }
                if (gatt == g) gatt = null

                mainHandler.removeCallbacks(reconnectRunnable)
                mainHandler.postDelayed(reconnectRunnable, 800L)
            }
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) return
            notifyQueue.clear()
            g.getService(SERVICE_UUID)?.let { svc ->
                svc.getCharacteristic(CHAR_CAN_MAIN)?.let { notifyQueue.add(it) }
                svc.getCharacteristic(CHAR_GPS_MAIN)?.let { notifyQueue.add(it) }
            }
            g.getService(NUS_SERVICE_UUID)?.getCharacteristic(NUS_TX_UUID)?.let {
                notifyQueue.add(it)
            }
            enableNextNotification(g)
        }

        override fun onDescriptorWrite(g: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            if (notifyQueue.isNotEmpty()) {
                enableNextNotification(g)
            } else {
                g.getService(SERVICE_UUID)?.getCharacteristic(CHAR_SYS_CONFIG)?.let {
                    g.readCharacteristic(it)
                }
            }
        }

        // Android 13+ (API 33+)
        override fun onCharacteristicRead(
            g: BluetoothGatt,
            c: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int
        ) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                handleCharacteristicRead(g, c.uuid, value)
            }
        }

        // Android 12 и ниже
        @Deprecated("Deprecated in Java")
        override fun onCharacteristicRead(
            g: BluetoothGatt,
            c: BluetoothGattCharacteristic,
            status: Int
        ) {
            if (Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU && status == BluetoothGatt.GATT_SUCCESS) {
                @Suppress("DEPRECATION")
                handleCharacteristicRead(g, c.uuid, c.value ?: return)
            }
        }

        // Android 13+ (API 33+)
        override fun onCharacteristicChanged(
            g: BluetoothGatt,
            c: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            handleCharacteristicChanged(c.uuid, value)
        }

        // Android 12 и ниже
        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            if (Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU) {
                @Suppress("DEPRECATION")
                handleCharacteristicChanged(c.uuid, c.value ?: return)
            }
        }
    }

    private fun handleCharacteristicRead(g: BluetoothGatt, uuid: UUID, value: ByteArray) {
        when (uuid) {
            CHAR_SYS_CONFIG -> {
                SystemConfig.fromBytes(value)?.let { _config.value = it }
                g.getService(SERVICE_UUID)?.getCharacteristic(CHAR_GPS_RATE)?.let {
                    g.readCharacteristic(it)
                }
            }
            CHAR_GPS_RATE -> {
                if (value.isNotEmpty()) _gpsRateHz.value = value[0].toInt() and 0xFF
                g.getService(SERVICE_UUID)?.getCharacteristic(CHAR_DEVICE_INFO)?.let {
                    g.readCharacteristic(it)
                }
            }
            CHAR_DEVICE_INFO -> {
                DeviceInfoPacket.fromBytes(value)?.let { _deviceInfo.value = it }
            }
        }
    }

    private fun handleCharacteristicChanged(uuid: UUID, value: ByteArray) {
        when (uuid) {
            CHAR_CAN_MAIN -> _telemetry.update { TelemetryParser.parseCanPacket(value, it) }
            CHAR_GPS_MAIN -> {
                _telemetry.update { TelemetryParser.parseGpsPacket(value, it) }
                RaceGpsBridgeService.onDiyGpsPacket(value)
            }
            NUS_TX_UUID -> RaceGpsBridgeService.onRaceBoxPacket(value)
        }
    }

    private fun enableNextNotification(g: BluetoothGatt) {
        val chr = notifyQueue.removeFirstOrNull() ?: return
        g.setCharacteristicNotification(chr, true)
        val desc = chr.getDescriptor(CCCD_UUID) ?: return
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            g.writeDescriptor(desc, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
        } else {
            @Suppress("DEPRECATION")
            desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            @Suppress("DEPRECATION")
            g.writeDescriptor(desc)
        }
    }

    private fun writeCharBytes(chr: BluetoothGattCharacteristic, bytes: ByteArray) {
        val g = gatt ?: return
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            g.writeCharacteristic(chr, bytes, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
        } else {
            @Suppress("DEPRECATION")
            chr.value = bytes
            chr.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            @Suppress("DEPRECATION")
            g.writeCharacteristic(chr)
        }
    }

    fun toggleDout(channel: Int, enable: Boolean) {
        val chr = gatt?.getService(SERVICE_UUID)?.getCharacteristic(CHAR_IO_CTRL) ?: return
        writeCharBytes(chr, byteArrayOf(channel.toByte(), if (enable) 1 else 0))
    }

    fun setGpsRate(hz: Int) {
        val chr = gatt?.getService(SERVICE_UUID)?.getCharacteristic(CHAR_GPS_RATE) ?: return
        writeCharBytes(chr, byteArrayOf(hz.coerceIn(1, 25).toByte()))
        _gpsRateHz.value = hz
    }

    fun writeSystemConfig(newConfig: SystemConfig) {
        val chr = gatt?.getService(SERVICE_UUID)?.getCharacteristic(CHAR_SYS_CONFIG) ?: return
        writeCharBytes(chr, newConfig.toBytes())
        _config.value = newConfig
    }
}