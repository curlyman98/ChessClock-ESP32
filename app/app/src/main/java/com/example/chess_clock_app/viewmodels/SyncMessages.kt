package com.example.chess_clock_app.viewmodels

import com.example.chess_clock_app.R
import com.example.chess_clock_app.repositories.SyncResult
import com.example.chess_clock_app.utils.UiMessage

// The toast after a sync; used by every screen with a sync button
fun SyncResult.toMessage(): UiMessage = when {
    connectionLost -> UiMessage(R.string.sync_failed)
    failed == 0 -> UiMessage(R.string.sync_done, imported)
    else -> UiMessage(R.string.sync_done_with_errors, imported, failed)
}
