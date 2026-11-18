package com.example.chess_clock_app.viewmodels

import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider.AndroidViewModelFactory.Companion.APPLICATION_KEY
import androidx.lifecycle.viewModelScope
import androidx.lifecycle.viewmodel.initializer
import androidx.lifecycle.viewmodel.viewModelFactory
import com.example.chess_clock_app.ChessClockApplication
import com.example.chess_clock_app.models.Game
import com.example.chess_clock_app.repositories.DeviceRepository
import com.example.chess_clock_app.repositories.GameRepository
import com.example.chess_clock_app.repositories.SyncRepository
import com.example.chess_clock_app.utils.UiMessage
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.flatMapLatest
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

enum class GameSortOrder { CREATED_AT, NAME }

class GameListViewModel(
    private val repository: GameRepository,
    deviceRepository: DeviceRepository,
    private val syncRepository: SyncRepository,
) : ViewModel() {

    // Sync only works while a chess clock is connected
    val isConnected: StateFlow<Boolean> = deviceRepository.connectedDevice
        .map { it != null }
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), false)

    private val _sortOrder = MutableStateFlow(GameSortOrder.CREATED_AT)
    val sortOrder: StateFlow<GameSortOrder> = _sortOrder.asStateFlow()

    // Switches to the matching query whenever the sort order changes
    @OptIn(ExperimentalCoroutinesApi::class)
    val games: StateFlow<List<Game>> = _sortOrder
        .flatMapLatest { order ->
            when (order) {
                GameSortOrder.CREATED_AT -> repository.gamesByCreatedAt
                GameSortOrder.NAME -> repository.gamesByName
            }
        }
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), emptyList())

    fun setSortOrder(order: GameSortOrder) {
        _sortOrder.value = order
    }

    // Date <-> name
    fun toggleSortOrder() {
        setSortOrder(
            if (_sortOrder.value == GameSortOrder.CREATED_AT) GameSortOrder.NAME else GameSortOrder.CREATED_AT
        )
    }

    fun addGame(game: Game) {
        viewModelScope.launch { repository.addGame(game) }
    }

    fun deleteGame(game: Game) {
        viewModelScope.launch { repository.deleteGame(game) }
    }

    val isSyncing: StateFlow<Boolean> = syncRepository.isSyncing

    // "Games imported: 2" etc.
    val messages: Flow<UiMessage> = syncRepository.results.map { it.toMessage() }

    fun sync() = syncRepository.start()

    fun cancelSync() = syncRepository.cancel()

    companion object {
        val Factory = viewModelFactory {
            initializer {
                val app = this[APPLICATION_KEY] as ChessClockApplication
                GameListViewModel(app.gameRepository, app.deviceRepository, app.syncRepository)
            }
        }
    }
}
