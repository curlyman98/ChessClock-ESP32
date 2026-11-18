package com.example.chess_clock_app.viewmodels

import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider.AndroidViewModelFactory.Companion.APPLICATION_KEY
import androidx.lifecycle.viewModelScope
import androidx.lifecycle.viewmodel.initializer
import androidx.lifecycle.viewmodel.viewModelFactory
import com.example.chess_clock_app.ChessClockApplication
import com.example.chess_clock_app.models.Device
import com.example.chess_clock_app.models.Game
import com.example.chess_clock_app.repositories.DeviceRepository
import com.example.chess_clock_app.repositories.GameRepository
import com.example.chess_clock_app.repositories.SyncRepository
import com.example.chess_clock_app.utils.InputLimits
import com.example.chess_clock_app.utils.UiMessage
import com.example.chess_clock_app.utils.cleanInput
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.flatMapLatest
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

// Everything the device screen needs to draw itself, in one object
data class DeviceUiState(
    val isLoading: Boolean = true,
    // The device is not in the database (anymore), e.g. right after "Forget"
    val notFound: Boolean = false,
    val device: Device? = null,
    val games: List<Game> = emptyList(),
)

class DeviceViewModel(
    private val deviceRepository: DeviceRepository,
    gameRepository: GameRepository,
    private val syncRepository: SyncRepository,
) : ViewModel() {

    // MAC of the device shown; null until loadDevice is called
    private val mac = MutableStateFlow<String?>(null)

    // Sync only works while this clock is the connected one
    val isConnected: StateFlow<Boolean> = combine(mac, deviceRepository.connectedDevice) { mac, connected ->
        mac != null && connected?.mac == mac
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), false)

    val isSyncing: StateFlow<Boolean> = syncRepository.isSyncing

    // "Games imported: 2" etc.
    val messages: Flow<UiMessage> = syncRepository.results.map { it.toMessage() }

    fun sync() = syncRepository.start()

    fun cancelSync() = syncRepository.cancel()

    // Re-runs whenever the device or its games change in the database
    @OptIn(ExperimentalCoroutinesApi::class)
    val uiState: StateFlow<DeviceUiState> = mac
        .filterNotNull()
        .flatMapLatest { mac ->
            combine(
                deviceRepository.observeDevice(mac),
                gameRepository.gamesFromDevice(mac)
            ) { device, games ->
                DeviceUiState(
                    isLoading = false,
                    notFound = device == null,
                    device = device,
                    games = games,
                )
            }
        }
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), DeviceUiState())

    fun loadDevice(mac: String) {
        this.mac.value = mac
    }

    // An empty name keeps the old one
    fun rename(newName: String) {
        val device = uiState.value.device ?: return
        val name = newName.cleanInput(InputLimits.DEVICE_NAME).ifEmpty { device.name }
        viewModelScope.launch { deviceRepository.updateDevice(device.copy(name = name)) }
    }

    // Removes the device from the app; its games are kept.
    // Returns the database delete, so the caller can wait for it before closing the screen
    fun forgetDevice(): Job = viewModelScope.launch {
        val device = uiState.value.device ?: return@launch
        deviceRepository.forgetDevice(device)
    }

    companion object {
        val Factory = viewModelFactory {
            initializer {
                val app = this[APPLICATION_KEY] as ChessClockApplication
                DeviceViewModel(app.deviceRepository, app.gameRepository, app.syncRepository)
            }
        }
    }
}
