package com.example.racedaq_cfg.bridge

import android.content.Context
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update

enum class BridgeProtocol(val id: Int, val title: String) {
    RACEBOX(0, "RaceBox Mini"),
    RACECHRONO_DIY(1, "RaceChrono DIY");

    companion object {
        fun fromId(id: Int): BridgeProtocol = entries.firstOrNull { it.id == id } ?: RACEBOX
    }
}

data class GpsBridgeUiState(
    val isRunning: Boolean = false,
    val isBleConnected: Boolean = false,
    val protocol: BridgeProtocol = BridgeProtocol.RACEBOX,
    val fixType: Int = 0,
    val satellites: Int = 0,
    val speedKmh: Float = 0f,
    val updateRateHz: Float = 0f,
    val errorMessage: String? = null,
    val showMockSetupDialog: Boolean = false // <-- ДОБАВЛЕНО
)

object BridgeStateHolder {
    private const val PREFS_NAME = "racedaq_bridge_prefs"
    private const val KEY_PROTOCOL = "bridge_protocol"

    private val _state = MutableStateFlow(GpsBridgeUiState())
    val state = _state.asStateFlow()

    fun init(context: Context) {
        val savedId = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
            .getInt(KEY_PROTOCOL, BridgeProtocol.RACEBOX.id)
        _state.update { it.copy(protocol = BridgeProtocol.fromId(savedId)) }
    }

    fun setProtocol(context: Context, protocol: BridgeProtocol) {
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)
            .edit()
            .putInt(KEY_PROTOCOL, protocol.id)
            .apply()
        _state.update { it.copy(protocol = protocol, updateRateHz = 0f) }
    }

    fun update(block: (GpsBridgeUiState) -> GpsBridgeUiState) {
        _state.update(block)
    }

    fun resetRunningState() {
        _state.update {
            it.copy(
                isRunning = false,
                showMockSetupDialog = false,
                fixType = 0,
                satellites = 0,
                speedKmh = 0f,
                updateRateHz = 0f,
                errorMessage = null
            )
        }
    }
}