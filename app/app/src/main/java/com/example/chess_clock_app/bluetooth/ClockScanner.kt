package com.example.chess_clock_app.bluetooth

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import androidx.core.content.ContextCompat
import androidx.core.content.IntentCompat
import com.example.chess_clock_app.models.Device
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.withTimeoutOrNull

// Finds chess clocks nearby and pairs with them.
// Callers check the Bluetooth permissions first (see BluetoothPermissions.kt)
@SuppressLint("MissingPermission")
class ClockScanner(private val context: Context) {

    companion object {
        // The clock calls itself "ChessClock-XXXX", other Bluetooth devices are ignored
        const val CLOCK_NAME_PREFIX = "ChessClock-"

        // Enough time for the user to answer Android's pairing dialog
        private const val PAIR_TIMEOUT_MS = 60_000L
    }

    private val adapter: BluetoothAdapter? = context.bluetoothAdapter()

    // Emits every chess clock found; completes when Android ends the search (after about 12 seconds)
    fun scan(): Flow<Device> = callbackFlow {
        // A FINISHED left over from an earlier search must not end this one
        var started = false

        val receiver = object : BroadcastReceiver() {
            override fun onReceive(context: Context, intent: Intent) {
                when (intent.action) {
                    BluetoothAdapter.ACTION_DISCOVERY_STARTED -> started = true

                    BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> if (started) close()

                    // FOUND, or NAME_CHANGED when the name arrives later than the device
                    else -> {
                        val device = intent.bluetoothDevice() ?: return
                        val name = intent.getStringExtra(BluetoothDevice.EXTRA_NAME) ?: device.name ?: return

                        if (name.startsWith(CLOCK_NAME_PREFIX)) {
                            trySend(Device(mac = device.address, name = name))
                        }
                    }
                }
            }
        }

        val filter = IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothDevice.ACTION_NAME_CHANGED)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        }
        // EXPORTED: these broadcasts come from the Bluetooth app, not from Android itself,
        // and a NOT_EXPORTED receiver silently never gets them
        ContextCompat.registerReceiver(context, receiver, filter, ContextCompat.RECEIVER_EXPORTED)

        // Clocks the phone is already paired with (e.g. in the phone's Bluetooth settings) are shown
        // right away; the search may not find them, e.g. while another app is connected to the clock
        adapter?.bondedDevices
            ?.filter { it.name?.startsWith(CLOCK_NAME_PREFIX) == true }
            ?.forEach { trySend(Device(mac = it.address, name = it.name)) }

        // false = Bluetooth is off
        if (adapter?.startDiscovery() != true) close()

        awaitClose {
            adapter?.cancelDiscovery()
            context.unregisterReceiver(receiver)
        }
    }

    // Asks Android to pair; the phone may show a confirmation dialog.
    // Returns false if pairing failed or the user cancelled
    suspend fun pair(mac: String): Boolean {
        val adapter = adapter ?: return false
        val device = adapter.getRemoteDevice(mac)

        if (device.bondState == BluetoothDevice.BOND_BONDED) return true

        // Pairing while searching often fails
        adapter.cancelDiscovery()

        val bondStates = callbackFlow {
            val receiver = object : BroadcastReceiver() {
                override fun onReceive(context: Context, intent: Intent) {
                    if (intent.bluetoothDevice()?.address != mac) return
                    trySend(intent.getIntExtra(BluetoothDevice.EXTRA_BOND_STATE, BluetoothDevice.BOND_NONE))
                }
            }
            ContextCompat.registerReceiver(
                context,
                receiver,
                IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED),
                // Sent by the Bluetooth app, see scan()
                ContextCompat.RECEIVER_EXPORTED
            )

            // Registered first, so no state change is missed
            if (!device.createBond()) trySend(BluetoothDevice.BOND_NONE)

            awaitClose { context.unregisterReceiver(receiver) }
        }

        // BONDING comes first, then BONDED or NONE (failed / cancelled)
        val result = withTimeoutOrNull(PAIR_TIMEOUT_MS) {
            bondStates.first { it != BluetoothDevice.BOND_BONDING }
        }
        return result == BluetoothDevice.BOND_BONDED
    }

    private fun Intent.bluetoothDevice(): BluetoothDevice? =
        IntentCompat.getParcelableExtra(this, BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
}
