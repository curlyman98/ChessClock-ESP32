package com.example.chess_clock_app.dao

import androidx.room.Dao
import androidx.room.Delete
import androidx.room.Insert
import androidx.room.OnConflictStrategy
import androidx.room.Query
import androidx.room.Update
import com.example.chess_clock_app.models.Device
import kotlinx.coroutines.flow.Flow

@Dao
interface DeviceDao {

    // REPLACE: pairing an already known MAC again overwrites it instead of crashing
    @Insert(onConflict = OnConflictStrategy.REPLACE)
    suspend fun insertDevice(device: Device)

    @Update
    suspend fun updateDevice(device: Device)

    @Delete
    suspend fun deleteDevice(device: Device)

    @Query("SELECT * FROM devices ORDER BY name ASC")
    fun getAllDevices(): Flow<List<Device>>

    @Query("SELECT * FROM devices WHERE mac = :mac LIMIT 1")
    suspend fun getByMac(mac: String): Device?

    @Query("SELECT * FROM devices WHERE mac = :mac LIMIT 1")
    fun observeByMac(mac: String): Flow<Device?>

    @Query("UPDATE devices SET lastConnectedAt = :time WHERE mac = :mac")
    suspend fun setLastConnected(mac: String, time: Long)

    @Query("UPDATE devices SET lastSyncedAt = :time WHERE mac = :mac")
    suspend fun setLastSynced(mac: String, time: Long)

    @Query("UPDATE devices SET gamesOnClock = :games WHERE mac = :mac")
    suspend fun setGamesOnClock(mac: String, games: Int)

}
