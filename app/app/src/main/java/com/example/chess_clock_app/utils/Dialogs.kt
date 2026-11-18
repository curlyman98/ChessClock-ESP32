package com.example.chess_clock_app.utils

import android.view.LayoutInflater
import android.view.WindowManager
import android.widget.EditText
import androidx.annotation.StringRes
import androidx.appcompat.app.AppCompatActivity
import com.example.chess_clock_app.R
import com.google.android.material.dialog.MaterialAlertDialogBuilder

// Popup with a text field, filled with the current value; typing stops at maxLength
fun AppCompatActivity.showEditDialog(
    @StringRes title: Int,
    currentValue: String,
    maxLength: Int,
    onDone: (String) -> Unit
) {
    // Inflate with the popup's context, so the field uses the popup colors, not the screen's
    val builder = MaterialAlertDialogBuilder(this)
    val dialogView = LayoutInflater.from(builder.context).inflate(R.layout.dialog_edit_text, null)
    val editText = dialogView.findViewById<EditText>(R.id.editText)
    editText.limitLength(maxLength)
    editText.setText(currentValue)
    editText.selectAll()

    val dialog = builder
        .setTitle(title)
        .setView(dialogView)
        .setPositiveButton(R.string.ok) { _, _ -> onDone(editText.text.toString()) }
        .setNegativeButton(R.string.cancel, null)
        .create()

    // Open the keyboard right away
    dialog.window?.setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_VISIBLE)
    editText.requestFocus()
    dialog.show()
}
