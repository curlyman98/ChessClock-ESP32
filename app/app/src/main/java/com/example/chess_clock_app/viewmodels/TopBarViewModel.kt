package com.example.chess_clock_app.viewmodels

import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider.AndroidViewModelFactory.Companion.APPLICATION_KEY
import androidx.lifecycle.viewmodel.initializer
import androidx.lifecycle.viewmodel.viewModelFactory
import com.example.chess_clock_app.ChessClockApplication
import com.example.chess_clock_app.models.Device
import com.example.chess_clock_app.repositories.DeviceRepository
import kotlinx.coroutines.flow.StateFlow

// Used by BaseActivity for the shared top bar, so every screen gets it
class TopBarViewModel(deviceRepository: DeviceRepository) : ViewModel() {

    // Green light while not null, red otherwise; tapping the light shows its name
    val connectedDevice: StateFlow<Device?> = deviceRepository.connectedDevice

    companion object {
        val Factory = viewModelFactory {
            initializer {
                val app = this[APPLICATION_KEY] as ChessClockApplication
                TopBarViewModel(app.deviceRepository)
            }
        }
    }
}
