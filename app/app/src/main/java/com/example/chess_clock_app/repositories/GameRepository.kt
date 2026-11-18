package com.example.chess_clock_app.repositories

import com.example.chess_clock_app.bluetooth.ClockConnection
import com.example.chess_clock_app.dao.GameDao
import com.example.chess_clock_app.models.Game
import com.example.chess_clock_app.models.GameDefaults
import kotlinx.coroutines.flow.Flow

// How often a game is asked for again when it arrives broken
private const val DOWNLOAD_TRIES = 3

// Single place the app gets game data from, also the games synced from the ESP32
class GameRepository(
    private val gameDao: GameDao,
    private val connection: ClockConnection,
) {

    val gamesByCreatedAt: Flow<List<Game>> = gameDao.getGameOrderedByCreatedAt()

    val gamesByName: Flow<List<Game>> = gameDao.getGameOrderedByName()

    fun gamesFromDevice(mac: String): Flow<List<Game>> = gameDao.getGamesFromDevice(mac)

    suspend fun getGame(key: Int): Game? = gameDao.getById(key)

    suspend fun addGame(game: Game): Long = gameDao.insertGame(game)

    suspend fun updateGame(game: Game) = gameDao.update(game)

    suspend fun deleteGame(game: Game) = gameDao.deleteGame(game)

    // Copies every game from the connected clock into the database. A game is deleted on the
    // clock only after it is saved here, so a lost connection never loses a game.
    // Throws IOException if the connection fails; the games saved until then stay saved
    suspend fun syncFromClock(mac: String): SyncResult {
        var imported = 0
        var failed = 0

        for (clockGame in connection.listGames()) {
            // Saved by an earlier sync whose DEL got lost: only delete it on the clock
            if (gameDao.getFromClock(mac, clockGame.id) == null) {
                val game = downloadGame(clockGame.id)?.let { data ->
                    GameDefaults.fromClock(data, mac, clockGame.id, System.currentTimeMillis())
                }
                // Stays on the clock, maybe the next sync can read it
                if (game == null) {
                    failed++
                    continue
                }
                gameDao.insertGame(game)
                imported++
            }
            connection.deleteGame(clockGame.id)
        }
        return SyncResult(imported, failed)
    }

    // Asks again when the data does not match its CRC; null if it never does
    private suspend fun downloadGame(id: Int): String? {
        repeat(DOWNLOAD_TRIES) {
            connection.getGame(id)?.let { return it }
        }
        return null
    }
}
