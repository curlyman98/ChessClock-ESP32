package com.example.chess_clock_app.databases
import androidx.room.Database
import androidx.room.RoomDatabase
import com.example.chess_clock_app.dao.GameDao
import com.example.chess_clock_app.dao.DeviceDao
import com.example.chess_clock_app.models.Game
import com.example.chess_clock_app.models.Device

@Database(
    entities = [Game::class,
               Device::class],
    version = 5,
    exportSchema = false
)
abstract class ChessClockDatabase : RoomDatabase() {
    abstract fun gameDao(): GameDao
    abstract fun deviceDao(): DeviceDao

}