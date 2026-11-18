package com.example.chess_clock_app.utils

import android.content.Context
import android.widget.Toast
import androidx.annotation.StringRes

// A toast a ViewModel wants to show. Only the string id, because a ViewModel has no Context
class UiMessage(@StringRes val text: Int, vararg val args: Any)

fun Context.showMessage(message: UiMessage) {
    Toast.makeText(this, getString(message.text, *message.args), Toast.LENGTH_SHORT).show()
}
