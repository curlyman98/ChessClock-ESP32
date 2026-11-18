package com.example.chess_clock_app.viewmodels

import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider.AndroidViewModelFactory.Companion.APPLICATION_KEY
import androidx.lifecycle.viewModelScope
import androidx.lifecycle.viewmodel.initializer
import androidx.lifecycle.viewmodel.viewModelFactory
import com.example.chess_clock_app.ChessClockApplication
import com.example.chess_clock_app.chess.Move
import com.example.chess_clock_app.chess.Pgn
import com.example.chess_clock_app.chess.PgnGame
import com.example.chess_clock_app.chess.PieceType
import com.example.chess_clock_app.chess.Position
import com.example.chess_clock_app.chess.Side
import com.example.chess_clock_app.models.Game
import com.example.chess_clock_app.models.GameDefaults
import com.example.chess_clock_app.repositories.GameRepository
import com.example.chess_clock_app.utils.InputLimits
import com.example.chess_clock_app.utils.cleanInput
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

// One half-move in the move list: the move (null = only a clock time so far) and its clock time
data class MoveCell(val san: String?, val clock: String?, val isCurrent: Boolean)

// One line of the move list: "1.  e4  4:58   e5  4:55"
data class MoveRow(val number: Int, val white: MoveCell?, val black: MoveCell?)

// The PGN header values edited in the "Game details" popup (the players are edited through the pills).
// Unknown values are "?" as in PGN. createdAt is also the PGN [Date]: set to "now" when the game is
// created or imported, changed only through the date picker.
data class GameDetails(
    val name: String = "",
    val site: String = "?",
    val createdAt: Long = 0L,
    val round: String = "?",
    val result: String = "*",
)

// Everything the screen needs to draw itself, in one object
data class GameEditUiState(
    val isLoading: Boolean = true,
    val notFound: Boolean = false,
    val details: GameDetails = GameDetails(),
    val white: String = "",
    val black: String = "",
    val position: Position = Position.start(),
    val lastMove: Move? = null,
    // How many half-moves into the game the board currently is (0 = start position)
    val currentPly: Int = 0,
    val moveCount: Int = 0,
    val moveRows: List<MoveRow> = emptyList(),
    // The row of the move currently on the board, so the list can scroll to it
    val currentRow: Int = 0,
    val hasUnsavedChanges: Boolean = false,
    // Created with "Add game" and not in the database yet
    val isNewGame: Boolean = false,
) {
    val canGoBack: Boolean get() = currentPly > 0
    val canGoForward: Boolean get() = currentPly < moveCount
    // A new game can be saved even without changes; an existing one only when something changed
    val canSave: Boolean get() = hasUnsavedChanges || isNewGame
}

class GameEditViewModel(private val repository: GameRepository) : ViewModel() {

    // The parts of a game the user can change. Comparing the current version with
    // the saved one tells whether there are unsaved changes.
    private data class Edits(
        val details: GameDetails,
        val white: String,
        val black: String,
        val sans: List<String>,
    )

    private val _uiState = MutableStateFlow(GameEditUiState())
    val uiState: StateFlow<GameEditUiState> = _uiState.asStateFlow()

    // key = 0 means the game is not in the database yet
    private var game: Game? = null
    private var pgn = PgnGame()

    private var details = GameDetails()
    private var white = ""
    private var black = ""

    // positions[i] is the board after i half-moves, so positions[0] is the start position
    private val positions = mutableListOf(Position.start())
    private val moves = mutableListOf<Move>()
    private val sans = mutableListOf<String>()
    private var currentPly = 0

    // What is in the database (for a new game: the defaults it started with)
    private var saved: Edits? = null
    private var saveJob: Job? = null

    fun loadGame(key: Int) {
        // Already loaded
        if (game?.key == key) return
        viewModelScope.launch {
            val loaded = repository.getGame(key)
            if (loaded == null) {
                _uiState.value = GameEditUiState(isLoading = false, notFound = true)
                return@launch
            }
            start(loaded)
        }
    }

    // "Add game": a fresh game that only exists here until it is saved
    fun startNewGame() {
        if (game != null) return
        start(GameDefaults.newGame())
    }

    // Returns false if the move is not legal in the position currently shown
    fun makeMove(from: Int, to: Int, promotion: PieceType? = null): Boolean {
        val position = positions[currentPly]
        val move = position.legalMoves()
            .firstOrNull { it.from == from && it.to == to && it.promotion == promotion }
            ?: return false

        if (currentPly < moves.size) {
            // Same move as the one already recorded: just step forward, keep the rest
            if (moves[currentPly] == move) {
                currentPly++
                publish()
                return true
            }
            // A different move from an earlier position replaces everything after it
            moves.subList(currentPly, moves.size).clear()
            sans.subList(currentPly, sans.size).clear()
            positions.subList(currentPly + 1, positions.size).clear()
        }

        sans += position.toSan(move)
        moves += move
        positions += position.play(move)
        currentPly++
        publish()
        return true
    }

    fun previousMove() {
        if (currentPly > 0) {
            currentPly--
            publish()
        }
    }

    fun nextMove() {
        if (currentPly < moves.size) {
            currentPly++
            publish()
        }
    }

    // From the "Game details" popup. An empty name keeps the old one; other empty fields become "?"
    fun updateDetails(newDetails: GameDetails) {
        details = GameDetails(
            name = newDetails.name.cleanInput(InputLimits.GAME_NAME).ifEmpty { details.name },
            site = newDetails.site.cleanInput(InputLimits.SITE).ifEmpty { "?" },
            createdAt = newDetails.createdAt,
            round = newDetails.round.cleanInput(InputLimits.ROUND).ifEmpty { "?" },
            result = newDetails.result,
        )
        publish()
    }

    // An empty name goes back to the default
    fun setPlayerName(side: Side, newName: String) {
        val playerName = newName.cleanInput(InputLimits.PLAYER_NAME).ifEmpty { GameDefaults.UNKNOWN_PLAYER }
        if (side == Side.WHITE) white = playerName else black = playerName
        publish()
    }

    // Returns the database write, so the caller can wait for it (e.g. before leaving the screen)
    fun save(): Job {
        val current = game ?: return Job().apply { complete() }
        val edits = currentEdits()
        pgn = pgn.copy(
            sans = edits.sans,
            result = details.result,
            tags = pgn.tags + mapOf(
                "Event" to details.name,
                "Site" to details.site,
                // Always follows createdAt, so the two can't drift apart
                "Date" to Pgn.formatDate(details.createdAt),
                "Round" to details.round,
                "White" to white,
                "Black" to black,
            ),
        )
        game = current.copy(name = details.name, createdAt = details.createdAt, pgn = Pgn.write(pgn))
        saved = edits
        publish()

        val previousSave = saveJob
        val job = viewModelScope.launch {
            // One save at a time, so a new game is inserted only once even if Save is tapped twice
            previousSave?.join()
            val latest = game ?: return@launch
            if (latest.key == 0) {
                val key = repository.addGame(latest).toInt()
                game = game?.copy(key = key)
                publish()
            } else {
                repository.updateGame(latest)
            }
        }
        saveJob = job
        return job
    }

    fun discardChanges() {
        val edits = saved ?: return
        details = edits.details
        white = edits.white
        black = edits.black
        replay(edits.sans)
        publish()
    }

    // Returns the database delete, so the caller can wait for it before closing the screen
    fun deleteGame(): Job = viewModelScope.launch {
        // A save that is still running could otherwise insert the game again after the delete
        saveJob?.join()
        val current = game ?: return@launch
        // key 0: never saved, so there is nothing in the database to delete
        if (current.key != 0) repository.deleteGame(current)
    }

    private fun start(loaded: Game) {
        game = loaded
        // Games without the standard header get it here; it is written on the next save
        pgn = GameDefaults.withDefaultTags(Pgn.parse(loaded.pgn), loaded.name, loaded.createdAt)
        details = GameDetails(
            name = loaded.name,
            site = pgn.tags["Site"] ?: "?",
            // A different [Date] in the PGN is replaced on the next save
            createdAt = loaded.createdAt,
            round = pgn.tags["Round"] ?: "?",
            result = pgn.result,
        )
        white = pgn.tags["White"] ?: GameDefaults.UNKNOWN_PLAYER
        black = pgn.tags["Black"] ?: GameDefaults.UNKNOWN_PLAYER
        replay(pgn.sans)
        saved = currentEdits()
        publish()
    }

    // Rebuilds the move list from SAN strings, starting at the start position
    private fun replay(sansToPlay: List<String>) {
        positions.subList(1, positions.size).clear()
        moves.clear()
        sans.clear()
        currentPly = 0
        for (san in sansToPlay) {
            val position = positions.last()
            // Stop at the first move that isn't legal; everything after it can't be shown either
            val move = position.moveFromSan(san) ?: break
            sans += position.toSan(move)
            moves += move
            positions += position.play(move)
        }
    }

    private fun currentEdits() = Edits(details, white, black, sans.toList())

    // Two half-moves per row; a row exists as soon as there is a move or a clock time for it
    private fun moveRows(): List<MoveRow> {
        val halfMoves = maxOf(sans.size, pgn.clocks.size)
        return (0 until (halfMoves + 1) / 2).map { row ->
            MoveRow(number = row + 1, white = moveCell(row * 2), black = moveCell(row * 2 + 1))
        }
    }

    private fun moveCell(ply: Int): MoveCell? {
        val san = sans.getOrNull(ply)
        val clock = pgn.clocks.getOrNull(ply)
        if (san == null && clock == null) return null
        return MoveCell(san, clock, isCurrent = ply == currentPly - 1)
    }

    private fun publish() {
        val current = game ?: return
        _uiState.value = GameEditUiState(
            isLoading = false,
            details = details,
            white = white,
            black = black,
            position = positions[currentPly],
            lastMove = moves.getOrNull(currentPly - 1),
            currentPly = currentPly,
            moveCount = moves.size,
            moveRows = moveRows(),
            currentRow = if (currentPly == 0) 0 else (currentPly - 1) / 2,
            hasUnsavedChanges = currentEdits() != saved,
            isNewGame = current.key == 0,
        )
    }

    companion object {
        val Factory = viewModelFactory {
            initializer {
                val app = this[APPLICATION_KEY] as ChessClockApplication
                GameEditViewModel(app.gameRepository)
            }
        }
    }
}
