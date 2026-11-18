package com.example.chess_clock_app.activities

import android.content.ClipData
import android.content.ClipboardManager
import android.content.Intent
import android.os.Build
import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.ImageButton
import android.widget.ProgressBar
import android.widget.TextView
import android.widget.Toast
import androidx.activity.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.chess_clock_app.R
import com.example.chess_clock_app.adapters.GameAdapter
import com.example.chess_clock_app.utils.InputLimits
import com.example.chess_clock_app.utils.showEditDialog
import com.example.chess_clock_app.utils.formatDateTime
import com.example.chess_clock_app.utils.setEnabledLook
import com.example.chess_clock_app.utils.showMessage
import com.example.chess_clock_app.viewmodels.DeviceViewModel
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import kotlinx.coroutines.launch

// Details of one paired chess clock.
// TODO: not essential, add later:
//  - Connection status with a Connect / Disconnect button (needs the BT code)
//  - Toggle: sync automatically when connected
//  - Toggle: delete games from the clock after a successful sync
//  - Send the phone's date and time to the clock (the ESP32 has no real-time clock)
//  - Clock settings sent to the device: default time control + increment, display brightness
//  - Advanced: calibration values (alpha, threshold, offset)
//  - Firmware version, if the ESP reports it

class DeviceActivity : BaseActivity() {

    companion object {
        // MAC of the device to open
        const val EXTRA_DEVICE = "com.example.chess_clock_app.extra.DEVICE"
    }

    private val viewModel: DeviceViewModel by viewModels { DeviceViewModel.Factory }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setActivityContent(R.layout.content_device)

        viewModel.loadDevice(intent.getStringExtra(EXTRA_DEVICE) ?: "")

        val titleText = findViewById<TextView>(R.id.deviceTitle)
        val renameButton = findViewById<ImageButton>(R.id.renameDeviceButton)
        val forgetButton = findViewById<ImageButton>(R.id.forgetDeviceButton)
        val macText = findViewById<TextView>(R.id.deviceMac)
        val lastConnectedText = findViewById<TextView>(R.id.lastConnected)
        val lastSyncedText = findViewById<TextView>(R.id.lastSynced)
        val gamesOnClockText = findViewById<TextView>(R.id.gamesOnClock)
        val gamesRecyclerView = findViewById<RecyclerView>(R.id.deviceGamesRecyclerView)
        val noGamesText = findViewById<TextView>(R.id.noDeviceGamesText)

        gamesRecyclerView.layoutManager = LinearLayoutManager(this)

        val syncButton = findViewById<Button>(R.id.syncButton)
        val syncingIndicator = findViewById<ProgressBar>(R.id.syncingIndicator)

        syncButton.setOnClickListener {
            if (viewModel.isSyncing.value) viewModel.cancelSync() else viewModel.sync()
        }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.isConnected.collect { connected -> syncButton.setEnabledLook(connected) }
            }
        }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.isSyncing.collect { syncing ->
                    syncingIndicator.visibility = if (syncing) View.VISIBLE else View.GONE
                    syncButton.text = getString(if (syncing) R.string.cancel_sync else R.string.sync)
                }
            }
        }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.messages.collect { showMessage(it) }
            }
        }

        renameButton.setOnClickListener {
            val device = viewModel.uiState.value.device ?: return@setOnClickListener
            showEditDialog(R.string.rename_device, device.name, InputLimits.DEVICE_NAME) { newName ->
                viewModel.rename(newName)
            }
        }
        forgetButton.setOnClickListener { confirmForget() }
        macText.setOnClickListener { copyMac(macText.text.toString()) }

        // Re-runs every time the device or its games change; only while the screen is visible
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.uiState.collect { state ->
                    // Forgotten (or never existed): nothing left to show
                    if (state.notFound) {
                        finish()
                        return@collect
                    }
                    val device = state.device ?: return@collect

                    titleText.text = device.name
                    macText.text = device.mac
                    lastConnectedText.text = getString(R.string.last_connected, formatTime(device.lastConnectedAt))
                    lastSyncedText.text = getString(R.string.last_synced, formatTime(device.lastSyncedAt))
                    // null = never connected, so the app has not asked yet
                    gamesOnClockText.text = getString(
                        R.string.games_on_clock,
                        device.gamesOnClock?.toString() ?: getString(R.string.unknown)
                    )

                    gamesRecyclerView.adapter = GameAdapter(
                        games = state.games,
                        onGameClicked = { game ->
                            val intent = Intent(this@DeviceActivity, GameEditActivity::class.java)
                            intent.putExtra(GameEditActivity.EXTRA_GAME, game.key)
                            startActivity(intent)
                        },
                        onGameSettingsClicked = {}
                    )
                    noGamesText.visibility = if (state.games.isEmpty()) View.VISIBLE else View.GONE
                }
            }
        }
    }

    // null = never happened
    private fun formatTime(millis: Long?): String =
        millis?.let(::formatDateTime) ?: getString(R.string.never)

    private fun copyMac(mac: String) {
        val clipboard = getSystemService(ClipboardManager::class.java)
        clipboard.setPrimaryClip(ClipData.newPlainText(getString(R.string.mac_address), mac))
        // Android 13+ shows its own "Copied" popup
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.TIRAMISU) {
            Toast.makeText(this, R.string.mac_copied, Toast.LENGTH_SHORT).show()
        }
    }

    // Asks first; the games from this device stay in the app
    private fun confirmForget() {
        val device = viewModel.uiState.value.device ?: return
        MaterialAlertDialogBuilder(this)
            .setTitle(R.string.forget_device)
            .setMessage(getString(R.string.forget_device_message, device.name))
            .setPositiveButton(R.string.forget) { _, _ ->
                // Wait until the device is deleted, then close the screen
                lifecycleScope.launch {
                    viewModel.forgetDevice().join()
                    finish()
                }
            }
            .setNegativeButton(R.string.cancel, null)
            .show()
    }
}
