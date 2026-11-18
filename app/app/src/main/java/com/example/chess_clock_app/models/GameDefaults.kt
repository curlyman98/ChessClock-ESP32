package com.example.chess_clock_app.models

import com.example.chess_clock_app.chess.Pgn
import com.example.chess_clock_app.chess.PgnGame
import com.example.chess_clock_app.utils.formatDateTime
import java.util.Locale

// What a game looks like before the user changes anything. Used for games created with
// "Add game" and, later, for games synced from the ESP32.
//
// These values are stored in the database and the PGN, so they live in code instead of
// strings.xml: a saved game must not depend on the language the phone is set to.
object GameDefaults {

    const val UNKNOWN_PLAYER = "Unknown GM"

    fun name(createdAt: Long) = "Game Created on ${formatDateTime(createdAt)}"

    // The id keeps games from the same sync apart
    fun syncedName(espGameId: Int, createdAt: Long) = "Game $espGameId Synced on ${formatDateTime(createdAt)}"

    //turns 10ms into empty moves
    // null if the data is not a list of numbers
    fun fromClock(data: String, mac: String, espGameId: Int, createdAt: Long): Game? {
        val ticks = data.split(" ").filter { it.isNotEmpty() }.map { it.toIntOrNull() ?: return null }

        val name = syncedName(espGameId, createdAt)
        val pgn = withDefaultTags(PgnGame(clocks = ticks.map(::formatClock)), name, createdAt)
        return Game(
            pgn = Pgn.write(pgn),
            name = name,
            espMac = mac,
            espGameId = espGameId,
            createdAt = createdAt,
        )
    }

    // 29850 ticks of 10ms -> "0:04:58", the format of {[%clk ...]}
    private fun formatClock(ticks: Int): String {
        val seconds = ticks / 100
        return String.format(Locale.ROOT, "%d:%02d:%02d", seconds / 3600, seconds / 60 % 60, seconds % 60)
    }

    // Not saved yet: key = 0 until it is inserted into the database
    fun newGame(createdAt: Long = System.currentTimeMillis()): Game {
        val name = name(createdAt)
        val pgn = withDefaultTags(PgnGame(), name, createdAt)
        return Game(name = name, createdAt = createdAt, pgn = Pgn.write(pgn))
    }

    // Adds any of the standard PGN header tags that are missing; existing values are kept
    fun withDefaultTags(pgn: PgnGame, name: String, createdAt: Long): PgnGame {
        val defaults = linkedMapOf(
            "Event" to name,
            "Site" to "?",
            "Date" to Pgn.formatDate(createdAt),
            "Round" to "?",
            "White" to UNKNOWN_PLAYER,
            "Black" to UNKNOWN_PLAYER,
            "Result" to pgn.result,
        )
        return pgn.copy(tags = defaults + pgn.tags)
    }
}
