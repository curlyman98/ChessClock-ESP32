package com.example.chess_clock_app.activities

import android.content.Intent
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.widget.Button
import android.widget.FrameLayout
import android.widget.ImageButton
import android.widget.Toast
import androidx.activity.viewModels
import androidx.annotation.LayoutRes
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.GravityCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.updatePadding
import androidx.drawerlayout.widget.DrawerLayout
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.example.chess_clock_app.R
import com.example.chess_clock_app.viewmodels.TopBarViewModel
import kotlinx.coroutines.launch

abstract class BaseActivity : AppCompatActivity() {

    protected lateinit var drawerLayout: DrawerLayout

    private val topBarViewModel: TopBarViewModel by viewModels { TopBarViewModel.Factory }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Load the shared drawer and top bar.
        super.setContentView(R.layout.activity_base)

        drawerLayout = findViewById(R.id.drawerLayout)

        val menuButton =
            findViewById<ImageButton>(R.id.menuButton)

        menuButton.setOnClickListener {
            drawerLayout.openDrawer(GravityCompat.START)
        }

        setupDrawerNavigation()

        // The app draws behind the system navigation bar, so keep every screen's content above it
        val contentContainer =
            findViewById<FrameLayout>(R.id.contentContainer)

        ViewCompat.setOnApplyWindowInsetsListener(contentContainer) { view, insets ->
            view.updatePadding(
                bottom = insets.getInsets(WindowInsetsCompat.Type.navigationBars()).bottom
            )
            insets
        }

        // Connection light: green while a chess clock is connected
        val connectionIndicator =
            findViewById<View>(R.id.connectionIndicator)

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                topBarViewModel.connectedDevice.collect { device ->
                    connectionIndicator.setBackgroundResource(
                        if (device != null) R.drawable.circle_green else R.drawable.circle_red
                    )
                }
            }
        }

        // Tapping the light tells which clock is connected
        val connectionIndicatorButton =
            findViewById<View>(R.id.connectionIndicatorButton)

        connectionIndicatorButton.setOnClickListener {
            val device = topBarViewModel.connectedDevice.value
            val message =
                if (device != null) getString(R.string.connected_to, device.name)
                else getString(R.string.no_device_connected)

            Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
        }
    }

    protected fun setActivityContent(
        @LayoutRes layoutResource: Int
    ) {
        val contentContainer =
            findViewById<FrameLayout>(R.id.contentContainer)

        LayoutInflater.from(this).inflate(
            layoutResource,
            contentContainer,
            true
        )
    }

    // Runs before the drawer opens another screen
    protected open fun confirmLeave(proceed: () -> Unit) = proceed()

    private fun setupDrawerNavigation() {

        //Navigate to pairing
        val pairingButton =
            findViewById<Button>(R.id.pairingButtonDrawer)

        pairingButton.setOnClickListener {
            drawerLayout.closeDrawer(GravityCompat.START)

            if (this !is PairingActivity) {
                confirmLeave {
                    val intent =
                        Intent(this, PairingActivity::class.java)

                    startActivity(intent)
                }
            }
        }
        //Navigate to games/sync

        val gamesButton =
            findViewById<Button>(R.id.gamesButtonDrawer)

        gamesButton.setOnClickListener {
            drawerLayout.closeDrawer(GravityCompat.START)

            if (this !is GameActivity) {
                confirmLeave {
                    val intent =
                        Intent(this, GameActivity::class.java)

                    startActivity(intent)
                }
            }
        }

        val settingsButton =
            findViewById<Button>(R.id.settingsButtonDrawer)

        settingsButton.setOnClickListener {
            drawerLayout.closeDrawer(GravityCompat.START)

            if (this !is SettingsActivity) {
                confirmLeave {
                    val intent =
                        Intent(this, SettingsActivity::class.java)

                    startActivity(intent)
                }
            }
        }

        // Stored Games and Settings navigation will be added
        // when those Activities exist.
    }
}