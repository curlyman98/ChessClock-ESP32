package com.example.chess_clock_app.viewmodels

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider.AndroidViewModelFactory.Companion.APPLICATION_KEY
import androidx.lifecycle.viewModelScope
import androidx.lifecycle.viewmodel.initializer
import androidx.lifecycle.viewmodel.viewModelFactory
import com.example.chess_clock_app.ChessClockApplication
import com.example.chess_clock_app.R
import com.example.chess_clock_app.models.Device
import com.example.chess_clock_app.repositories.DeviceRepository
import com.example.chess_clock_app.utils.InputLimits
import com.example.chess_clock_app.utils.UiMessage
import com.example.chess_clock_app.utils.cleanInput
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch
import java.io.IOException

class PairingViewModel(private val repository: DeviceRepository) : ViewModel() {

    // Remembers the latest list, so after a rotation the UI gets it instantly
    val savedDevices: StateFlow<List<Device>> = repository.savedDevices
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), emptyList())

    val connectedDevice: StateFlow<Device?> = repository.connectedDevice

    // Every clock the current or last search found
    private val foundDevices = MutableStateFlow<List<Device>>(emptyList())

    // Found clocks that are not paired yet; a clock moves to the paired list once paired
    val nearbyDevices: StateFlow<List<Device>> = combine(foundDevices, repository.savedDevices) { found, saved ->
        found.filter { device -> saved.none { it.mac == device.mac } }
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), emptyList())

    private val _isSearching = MutableStateFlow(false)
    val isSearching: StateFlow<Boolean> = _isSearching.asStateFlow()

    // Toasts for the screen; waits until the screen is visible to show them
    private val _messages = Channel<UiMessage>(Channel.BUFFERED)
    val messages: Flow<UiMessage> = _messages.receiveAsFlow()

    private var searchJob: Job? = null

    // Pairing or connecting; only one at a time
    private var busyJob: Job? = null

    fun startSearch() {
        if (_isSearching.value) return
        _isSearching.value = true
        foundDevices.value = emptyList()

        searchJob = viewModelScope.launch {
            try {
                repository.scan().collect { device ->
                    // NAME_CHANGED can report the same clock again
                    if (foundDevices.value.none { it.mac == device.mac }) {
                        foundDevices.value += device
                    }
                }
            } finally {
                // Also when Android ends the search by itself
                _isSearching.value = false
            }
        }
    }

    fun stopSearch() {
        searchJob?.cancel()
    }

    // Connects right away once paired
    fun pair(device: Device) {
        if (busyJob?.isActive == true) return
        stopSearch()

        busyJob = viewModelScope.launch {
            _messages.send(UiMessage(R.string.pairing_with, device.name))

            if (repository.pair(device)) {
                connectNow(device)
            } else {
                _messages.send(UiMessage(R.string.pairing_failed, device.name))
            }
        }
    }

    fun connect(device: Device) {
        if (busyJob?.isActive == true) return
        stopSearch()

        busyJob = viewModelScope.launch { connectNow(device) }
    }

    private suspend fun connectNow(device: Device) {
        _messages.send(UiMessage(R.string.connecting_to, device.name))

        val message = try {
            val games = repository.connect(device)
            UiMessage(R.string.connected_games, device.name, games)
        } catch (e: IOException) {
            Log.w(TAG, "Connecting to ${device.mac} failed", e)
            UiMessage(R.string.connect_failed, device.name)
        }
        _messages.send(message)
    }

    fun disconnect() {
        val device = connectedDevice.value ?: return
        repository.disconnect()
        _messages.trySend(UiMessage(R.string.disconnected_from, device.name))
    }

    fun saveDevice(device: Device) {
        viewModelScope.launch { repository.saveDevice(device) }
    }

    // An empty name keeps the old one
    fun renameDevice(device: Device, newName: String) {
        val name = newName.cleanInput(InputLimits.DEVICE_NAME).ifEmpty { device.name }
        viewModelScope.launch { repository.updateDevice(device.copy(name = name)) }
    }

    fun forgetDevice(device: Device) {
        viewModelScope.launch { repository.forgetDevice(device) }
    }

    companion object {
        private const val TAG = "PairingViewModel"

        val Factory = viewModelFactory {
            initializer {
                val app = this[APPLICATION_KEY] as ChessClockApplication
                PairingViewModel(app.deviceRepository)
            }
        }
    }
}
