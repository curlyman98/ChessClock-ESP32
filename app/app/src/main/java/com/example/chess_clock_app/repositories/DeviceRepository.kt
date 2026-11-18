package com.example.chess_clock_app.repositories

import com.example.chess_clock_app.bluetooth.ClockConnection
import com.example.chess_clock_app.bluetooth.ClockScanner
import com.example.chess_clock_app.dao.DeviceDao
import com.example.chess_clock_app.models.Device
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.StateFlow

// Single place the app gets device data from, saved ones and the Bluetooth side
class DeviceRepository(
    private val deviceDao: DeviceDao,
    private val scanner: ClockScanner,
    private val connection: ClockConnection,
) {

    val savedDevices: Flow<List<Device>> = deviceDao.getAllDevices()

    // The chess clock the app is talking to; null = none connected
    val connectedDevice: StateFlow<Device?> = connection.connectedDevice

    suspend fun getDevice(mac: String): Device? = deviceDao.getByMac(mac)

    // Emits again whenever the device changes (e.g. renamed); null once it is forgotten
    fun observeDevice(mac: String): Flow<Device?> = deviceDao.observeByMac(mac)

    suspend fun saveDevice(device: Device) = deviceDao.insertDevice(device)

    suspend fun updateDevice(device: Device) = deviceDao.updateDevice(device)

    suspend fun forgetDevice(device: Device) {
        if (connectedDevice.value?.mac == device.mac) connection.disconnect()
        deviceDao.deleteDevice(device)
    }

    // Chess clocks nearby; completes when the search ends
    fun scan(): Flow<Device> = scanner.scan()

    // Pairs with Android and saves the device. false = pairing failed or was cancelled
    suspend fun pair(device: Device): Boolean {
        if (!scanner.pair(device.mac)) return false
        deviceDao.insertDevice(device)
        return true
    }

    // Returns how many games are waiting on the clock. Throws IOException if it fails
    suspend fun connect(device: Device): Int {
        val games = connection.connect(device)
        deviceDao.setLastConnected(device.mac, System.currentTimeMillis())
        deviceDao.setGamesOnClock(device.mac, games)
        return games
    }

    // Asks the connected clock again, e.g. after a sync deleted games on it
    suspend fun refreshGamesOnClock(mac: String) = deviceDao.setGamesOnClock(mac, connection.hello())

    fun disconnect() = connection.disconnect()

    suspend fun markSynced(mac: String) = deviceDao.setLastSynced(mac, System.currentTimeMillis())
}
