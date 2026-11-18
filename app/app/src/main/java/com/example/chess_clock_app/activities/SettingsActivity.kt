package com.example.chess_clock_app.activities

import android.os.Bundle
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.chess_clock_app.R
import com.example.chess_clock_app.adapters.GameAdapter
import com.example.chess_clock_app.models.Game
import android.widget.Toast
import android.view.View
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity

class SettingsActivity : BaseActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Insert pairing-specific UI into activity_base.xml.
        setActivityContent(R.layout.content_settings)
    }

}