package com.example.chess_clock_app.models
import androidx.room.Entity
import androidx.room.PrimaryKey

@Entity(tableName = "games")

data class Game(
    val pgn: String = "",
    val name: String = "",

    val espMac: String = "",
    val espGameId: Int? = null,

    // milliseconds since 1970
    val createdAt: Long = 0L,
    @PrimaryKey(autoGenerate = true)
    val key: Int = 0
)
