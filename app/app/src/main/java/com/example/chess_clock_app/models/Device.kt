package com.example.chess_clock_app.models
import androidx.room.Entity
import androidx.room.PrimaryKey

@Entity(tableName = "devices")

data class Device(
    @PrimaryKey
    val mac: String,
    val name: String,
    // In milliseconds since 1970; null = never
    val lastConnectedAt: Long? = null,
    val lastSyncedAt: Long? = null,

    val gamesOnClock: Int? = null,
)