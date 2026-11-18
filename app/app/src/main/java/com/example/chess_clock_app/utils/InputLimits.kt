package com.example.chess_clock_app.utils

import android.text.InputFilter
import android.widget.EditText
import com.google.android.material.textfield.TextInputLayout

// Longest text allowed per field. Long names squeeze the move list and the game list.
object InputLimits {
    const val GAME_NAME = 60
    const val PLAYER_NAME = 40
    const val SITE = 60
    const val ROUND = 10
    const val DEVICE_NAME = 30
}

// Used for every text that gets saved, also text that doesn't come from a text field (e.g. synced games).
// Newlines become spaces: a newline inside a PGN tag would break the [Tag "..."] line.
// "  Grenke\n R9 " -> "Grenke R9"
fun String.cleanInput(maxLength: Int): String =
    replace(Regex("\\s+"), " ").trim().take(maxLength).trim()

// Typing stops at maxLength
fun EditText.limitLength(maxLength: Int) {
    filters = arrayOf(InputFilter.LengthFilter(maxLength))
}

// Same as above, plus a "23/60" counter under the field
fun TextInputLayout.limitLength(maxLength: Int) {
    isCounterEnabled = true
    counterMaxLength = maxLength
    editText?.limitLength(maxLength)
}
