package com.example.chess_clock_app.dao

import androidx.room.Dao
import androidx.room.Delete
import androidx.room.Insert
import androidx.room.Query
import androidx.room.Update
import com.example.chess_clock_app.models.Game
import kotlinx.coroutines.flow.Flow

@Dao
interface GameDao {

    // Returns the key Room generated for the new row
    @Insert
    suspend fun insertGame(game: Game): Long

    @Delete
    suspend fun deleteGame(game: Game)

    @Update
    suspend fun update(game: Game)

    @Query("SELECT * FROM games ORDER BY createdAt ASC")
    fun getGameOrderedByCreatedAt(): Flow<List<Game>>

    @Query("SELECT * FROM games ORDER BY name COLLATE NOCASE ASC")
    fun getGameOrderedByName(): Flow<List<Game>>

    @Query("SELECT * FROM games WHERE espMac = :mac ORDER BY createdAt ASC")
    fun getGamesFromDevice(mac: String): Flow<List<Game>>

    @Query("SELECT * FROM games WHERE key = :key LIMIT 1")
    suspend fun getById(key: Int): Game?

    @Query("SELECT * FROM games WHERE espMac = :mac AND espGameId = :espGameId LIMIT 1")
    suspend fun getFromClock(mac: String, espGameId: Int): Game?

}

