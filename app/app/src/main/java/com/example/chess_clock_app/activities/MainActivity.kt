package com.example.chess_clock_app.activities

import android.os.Bundle
import com.example.chess_clock_app.R

class MainActivity : BaseActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setActivityContent(R.layout.content_main)
    }
}