package com.example.chess_clock_app.utils

import android.view.View

// Disabled buttons look the same as enabled ones by default, so also fade them
fun View.setEnabledLook(enabled: Boolean) {
    isEnabled = enabled
    alpha = if (enabled) 1f else 0.4f
}
