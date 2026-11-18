package com.example.chess_clock_app

import android.app.Application
import androidx.room.Room
import androidx.room.RoomDatabase
import androidx.sqlite.db.SupportSQLiteDatabase
import com.example.chess_clock_app.bluetooth.ClockConnection
import com.example.chess_clock_app.bluetooth.ClockScanner
import com.example.chess_clock_app.databases.ChessClockDatabase
import com.example.chess_clock_app.repositories.DeviceRepository
import com.example.chess_clock_app.repositories.GameRepository
import com.example.chess_clock_app.repositories.SyncRepository

// shared by the whole app.
class ChessClockApplication : Application() {

    // `by lazy`: built on first use, only once, thread-safe
    val database by lazy {
        Room.databaseBuilder(this, ChessClockDatabase::class.java, "chess_clock_db")
            //.addCallback(seedTestData) // TODO: remove once games come from the ESP32

            //TODO Remove before final version, this formats the db upon structure change
            .fallbackToDestructiveMigration(dropAllTables = true)

            .build()
    }
    val clockConnection by lazy { ClockConnection(this) }

    val deviceRepository by lazy { DeviceRepository(database.deviceDao(), ClockScanner(this), clockConnection) }
    val gameRepository by lazy { GameRepository(database.gameDao(), clockConnection) }
    val syncRepository by lazy { SyncRepository(gameRepository, deviceRepository) }

    // Runs when the database is first created on the device, and again when it is wiped
    // because the version changed. To re-seed otherwise, uninstall the app or clear its storage.
    private val seedTestData = object : RoomDatabase.Callback() {
        // Room calls onDestructiveMigration after dropping the tables but before creating
        // them again, so the seeding waits for onOpen
        private var reseed = false

        override fun onCreate(db: SupportSQLiteDatabase) {
            insertTestData(db)
        }

        override fun onDestructiveMigration(db: SupportSQLiteDatabase) {
            reseed = true
        }

        override fun onOpen(db: SupportSQLiteDatabase) {
            if (reseed) {
                reseed = false
                insertTestData(db)
            }
        }

        private fun insertTestData(db: SupportSQLiteDatabase) {
            db.execSQL(
                """
                INSERT INTO games (name, pgn, espMac, createdAt) VALUES
                    ('Grenke R9', '{[%clk 0:04:58]} {[%clk 0:04:55]} {[%clk 0:04:51]} {[%clk 0:04:47]} {[%clk 0:04:40]} {[%clk 0:04:31]}', 'AA:AA:AA:AA:AA:01', 1790872200000),
                    ('Random with Kriss', '[Event "Random with Kriss"]
                [Site "?"]
                [Date "2026.10.05"]
                [Round "?"]
                [White "Kriss"]
                [Black "Unknown GM"]
                [Result "*"]

                1. d4 {[%clk 0:09:57]} d5 {[%clk 0:09:52]} 2. c4 {[%clk 0:09:41]} e6 {[%clk 0:09:30]} {[%clk 0:09:12]} {[%clk 0:09:01]} *', 'AA:AA:AA:AA:AA:01', 1791224100000),
                    ('Bib session', '1. c4 e5 2. Nc3 Nf6', 'AA:AA:AA:AA:AA:02', 1791633600000)
                """.trimIndent()
            )
            db.execSQL(
                """
                INSERT INTO devices (mac, name, lastConnectedAt, lastSyncedAt) VALUES
                    ('AA:AA:AA:AA:AA:01', 'Chess Clubs Clock', 1791633600000, 1791224100000),
                    ('AA:AA:AA:AA:AA:02', 'Tournament Clock', NULL, NULL)
                """.trimIndent()
            )
        }
    }
}
