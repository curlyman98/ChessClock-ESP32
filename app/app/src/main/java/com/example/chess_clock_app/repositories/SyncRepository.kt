package com.example.chess_clock_app.repositories

import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.receiveAsFlow
import kotlinx.coroutines.launch
import java.io.IOException

// What a sync did, for the message afterwards
data class SyncResult(
    val imported: Int = 0,
    val failed: Int = 0,
    // The connection broke off; the games saved until then stay saved
    val connectionLost: Boolean = false,
)

// Syncing is started from two screens (stored games and the device screen). It runs here instead
// of in a ViewModel, so both screens show the same sync and it keeps going after leaving the screen
class SyncRepository(
    private val gameRepository: GameRepository,
    private val deviceRepository: DeviceRepository,
) {

    companion object {
        private const val TAG = "SyncRepository"
    }

    // Lives as long as the app
    private val scope = CoroutineScope(SupervisorJob())

    private val _isSyncing = MutableStateFlow(false)
    val isSyncing: StateFlow<Boolean> = _isSyncing.asStateFlow()

    // One result per sync; shown by the screen that is visible when the sync ends
    private val _results = Channel<SyncResult>(Channel.BUFFERED)
    val results: Flow<SyncResult> = _results.receiveAsFlow()

    private var syncJob: Job? = null

    // Copies the games from the connected clock into the app
    fun start() {
        if (_isSyncing.value) return
        val device = deviceRepository.connectedDevice.value ?: return
        _isSyncing.value = true

        syncJob = scope.launch {
            val result = try {
                val result = gameRepository.syncFromClock(device.mac)
                deviceRepository.markSynced(device.mac)
                refreshGamesOnClock(device.mac)
                result
            } catch (e: IOException) {
                Log.w(TAG, "Sync with ${device.mac} failed", e)
                SyncResult(connectionLost = true)
            } finally {
                _isSyncing.value = false
            }
            _results.send(result)
        }
    }

    // Also closes the connection, a half read answer would confuse the next request
    fun cancel() {
        syncJob?.cancel()
    }

    // For the device screen. The sync itself already worked, so a failure here is not reported
    private suspend fun refreshGamesOnClock(mac: String) {
        try {
            deviceRepository.refreshGamesOnClock(mac)
        } catch (e: IOException) {
            Log.w(TAG, "Could not ask $mac for its number of games", e)
        }
    }
}
