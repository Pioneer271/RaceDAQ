package com.example.racedaq_cfg

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.ParcelUuid

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import java.util.UUID

@SuppressLint("MissingPermission")
class RaceDaqBleManager(private val context: Context) {

    companion object {
        val SERVICE_UUID: UUID    = UUID.fromString("00001ff8-0000-1000-8000-00805f9b34fb")
        val CHAR_CAN_MAIN: UUID   = UUID.fromString("00000001-0000-1000-8000-00805f9b34fb")
        val CHAR_GPS_MAIN: UUID   = UUID.fromString("00000003-0000-1000-8000-00805f9b34fb")
        val CHAR_IO_CTRL: UUID    = UUID.fromString("00000006-0000-1000-8000-00805f9b34fb")
        val CHAR_GPS_RATE: UUID   = UUID.fromString("00000007-0000-1000-8000-00805f9b34fb")
        val CHAR_SYS_CONFIG: UUID = UUID.fromString("00000008-0000-1000-8000-00805f9b34fb")
        val CCCD_UUID: UUID       = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
    }

    private val btAdapter: BluetoothAdapter? =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    private var gatt: BluetoothGatt? = null
    private val notifyQueue = ArrayDeque<BluetoothGattCharacteristic>()

    private val _telemetry = MutableStateFlow(TelemetryState())
    val telemetry = _telemetry.asStateFlow()

    private val _config = MutableStateFlow(SystemConfig())
    val config = _config.asStateFlow()

    private val _gpsRateHz = MutableStateFlow(10)
    val gpsRateHz = _gpsRateHz.asStateFlow()

    fun startAutoConnect() {
        val scanner = btAdapter?.bluetoothLeScanner ?: return
        val filter = ScanFilter.Builder().setServiceUuid(ParcelUuid(SERVICE_UUID)).build()
        val settings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()
        scanner.startScan(listOf(filter), settings, scanCallback)
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            btAdapter?.bluetoothLeScanner?.stopScan(this)
            gatt = result.device.connectGatt(context, true, gattCallback, BluetoothDevice.TRANSPORT_LE)
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                _telemetry.update { it.copy(connected = true) }
                g.requestMtu(247)
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                _telemetry.update { it.copy(connected = false) }
            }
        }

        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) {
            g.discoverServices()
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            val svc = g.getService(SERVICE_UUID) ?: return
            notifyQueue.clear()
            svc.getCharacteristic(CHAR_CAN_MAIN)?.let { notifyQueue.add(it) }
            svc.getCharacteristic(CHAR_GPS_MAIN)?.let { notifyQueue.add(it) }
            enableNextNotification(g)
        }

        override fun onDescriptorWrite(g: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) {
            if (notifyQueue.isNotEmpty()) {
                enableNextNotification(g)
            } else {
                // После подписки на потоки читаем текущие настройки из nRF52832
                g.getService(SERVICE_UUID)?.getCharacteristic(CHAR_SYS_CONFIG)?.let { g.readCharacteristic(it) }
            }
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicRead(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) return
            when (c.uuid) {
                CHAR_SYS_CONFIG -> {
                    SystemConfig.fromBytes(c.value)?.let { _config.value = it }
                    g.getService(SERVICE_UUID)?.getCharacteristic(CHAR_GPS_RATE)?.let { g.readCharacteristic(it) }
                }
                CHAR_GPS_RATE -> {
                    if (c.value.isNotEmpty()) _gpsRateHz.value = c.value[0].toInt() and 0xFF
                }
            }
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            when (c.uuid) {
                CHAR_CAN_MAIN -> _telemetry.update { TelemetryParser.parseCanPacket(c.value, it) }
                CHAR_GPS_MAIN -> _telemetry.update { TelemetryParser.parseGpsPacket(c.value, it) }
            }
        }
    }

    private fun enableNextNotification(g: BluetoothGatt) {
        val chr = notifyQueue.removeFirstOrNull() ?: return
        g.setCharacteristicNotification(chr, true)
        val desc = chr.getDescriptor(CCCD_UUID) ?: return
        desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        g.writeDescriptor(desc)
    }

    fun toggleDout(channel: Int, enable: Boolean) {
        val chr = gatt?.getService(SERVICE_UUID)?.getCharacteristic(CHAR_IO_CTRL) ?: return
        chr.value = byteArrayOf(channel.toByte(), if (enable) 1 else 0)
        gatt?.writeCharacteristic(chr)
    }

    fun setGpsRate(hz: Int) {
        val chr = gatt?.getService(SERVICE_UUID)?.getCharacteristic(CHAR_GPS_RATE) ?: return
        chr.value = byteArrayOf(hz.coerceIn(1, 25).toByte())
        gatt?.writeCharacteristic(chr)
        _gpsRateHz.value = hz
    }

    fun writeSystemConfig(newConfig: SystemConfig) {
        val chr = gatt?.getService(SERVICE_UUID)?.getCharacteristic(CHAR_SYS_CONFIG) ?: return
        chr.value = newConfig.toBytes()
        gatt?.writeCharacteristic(chr)
        _config.value = newConfig
    }
}