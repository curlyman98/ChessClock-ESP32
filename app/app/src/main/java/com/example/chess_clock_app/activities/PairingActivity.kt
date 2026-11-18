package com.example.chess_clock_app.activities

import android.bluetooth.BluetoothAdapter
import android.content.Intent
import android.os.Bundle
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.chess_clock_app.R
import com.example.chess_clock_app.adapters.DeviceAdapter
import com.example.chess_clock_app.bluetooth.BLUETOOTH_PERMISSIONS
import com.example.chess_clock_app.bluetooth.bluetoothAdapter
import com.example.chess_clock_app.bluetooth.hasBluetoothPermissions
import com.example.chess_clock_app.models.Device
import android.widget.Toast
import android.view.View
import android.widget.Button
import android.widget.ProgressBar
import android.widget.TextView
import com.example.chess_clock_app.utils.showMessage
import com.example.chess_clock_app.viewmodels.PairingViewModel
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.launch

class PairingActivity : BaseActivity() {

    private val viewModel: PairingViewModel by viewModels { PairingViewModel.Factory }

    // What to do once the user allowed Bluetooth; null = nothing waiting
    private var afterPermission: (() -> Unit)? = null

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {
            val action = afterPermission
            afterPermission = null

            if (hasBluetoothPermissions()) {
                withBluetooth { action?.invoke() }
            } else {
                Toast.makeText(this, R.string.bluetooth_permission_needed, Toast.LENGTH_LONG).show()
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Insert pairing-specific UI into activity_base.xml.
        setActivityContent(R.layout.content_pairing)

        val recyclerView =
            findViewById<RecyclerView>(R.id.devicesRecyclerView)

        val noPairedDevicesText =
            findViewById<TextView>(R.id.noPairedDevicesText)

        recyclerView.layoutManager =
            LinearLayoutManager(this)

        // Re-runs every time the devices table changes; only while the screen is visible
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                // Also redraws when a clock connects or disconnects, for its colour
                combine(viewModel.savedDevices, viewModel.connectedDevice, ::Pair).collect { (devices, connected) ->
                    showDevices(recyclerView, devices, connected?.mac)
                    noPairedDevicesText.visibility = if (devices.isEmpty()) View.VISIBLE else View.GONE
                }
            }
        }
        val pairButton =
            findViewById<Button>(R.id.pairButton)

        val nearbyRecyclerView =
            findViewById<RecyclerView>(R.id.nearbyDevicesRecyclerView)
        val nearbyDevicesTitle = findViewById<TextView>(R.id.nearbyDevicesTitle)

        val scanningIndicator =
            findViewById<ProgressBar>(R.id.scanningIndicator)

        nearbyRecyclerView.layoutManager =
            LinearLayoutManager(this)

        // The found clocks stay listed after the search ends, so there is time to tap one
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                combine(viewModel.isSearching, viewModel.nearbyDevices, ::Pair).collect { (isSearching, devices) ->
                    showNearbyDevices(nearbyRecyclerView, devices)

                    val listVisibility = if (isSearching || devices.isNotEmpty()) View.VISIBLE else View.GONE
                    nearbyRecyclerView.visibility = listVisibility
                    nearbyDevicesTitle.visibility = listVisibility

                    scanningIndicator.visibility = if (isSearching) View.VISIBLE else View.GONE
                    pairButton.text = getString(if (isSearching) R.string.stop_search else R.string.pair_new_device)
                }
            }
        }

        pairButton.setOnClickListener {
            if (viewModel.isSearching.value) viewModel.stopSearch()
            else withBluetooth { viewModel.startSearch() }
        }

        // "Connecting to ...", "Paired with ..." etc.
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                viewModel.messages.collect { showMessage(it) }
            }
        }
    }

    // Runs the action once the permissions are granted and Bluetooth is on
    private fun withBluetooth(action: () -> Unit) {
        val adapter = bluetoothAdapter()

        when {
            adapter == null ->
                Toast.makeText(this, R.string.no_bluetooth, Toast.LENGTH_SHORT).show()

            !hasBluetoothPermissions() -> {
                afterPermission = action
                permissionLauncher.launch(BLUETOOTH_PERMISSIONS)
            }

            // Android asks the user to turn it on; tap again afterwards
            !adapter.isEnabled ->
                startActivity(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))

            else -> action()
        }
    }

    private fun showDevices(recyclerView: RecyclerView, devices: List<Device>, connectedMac: String?) {
        recyclerView.adapter = DeviceAdapter(
            devices = devices,

            // Tapping the connected clock disconnects it
            onDeviceClicked = { device ->
                if (viewModel.connectedDevice.value?.mac == device.mac) {
                    viewModel.disconnect()
                } else {
                    withBluetooth { viewModel.connect(device) }
                }
            },

            // Opens the device screen
            onDeviceSettingsClicked = { device ->
                val intent = Intent(this, DeviceActivity::class.java)

                intent.putExtra(DeviceActivity.EXTRA_DEVICE, device.mac)

                startActivity(intent)
            },

            connectedMac = connectedMac
        )
    }

    private fun showNearbyDevices(recyclerView: RecyclerView, devices: List<Device>) {
        recyclerView.adapter = DeviceAdapter(
            devices = devices,

            onDeviceClicked = { device ->
                withBluetooth { viewModel.pair(device) }
            }
            // No onDeviceSettingsClicked: nearby devices have no edit button
        )
    }
}
