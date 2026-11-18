package com.example.chess_clock_app.bluetooth

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.Context
import android.content.pm.PackageManager
import android.os.Build
import androidx.core.content.ContextCompat

// The runtime permissions Bluetooth needs; they changed in Android 12
val BLUETOOTH_PERMISSIONS: Array<String> =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
    } else {
        // Android 11 and older only find nearby devices with the location permission
        arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
    }

fun Context.hasBluetoothPermissions(): Boolean =
    BLUETOOTH_PERMISSIONS.all {
        ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
    }

// null = the phone has no Bluetooth at all
fun Context.bluetoothAdapter(): BluetoothAdapter? =
    getSystemService(BluetoothManager::class.java)?.adapter
