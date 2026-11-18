package com.example.chess_clock_app.bluetooth

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.util.Log
import com.example.chess_clock_app.models.Device
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeout
import java.io.ByteArrayOutputStream
import java.io.IOException
import java.util.UUID
import java.util.zip.CRC32

// A game waiting on the clock, as announced by GAME (in LIST) and DATA (in GET)
data class ClockGame(val id: Int, val length: Int, val crc: Long)

// Talks to the chess clock over Bluetooth serial (SPP). The app always asks, the clock only answers.
// The protocol is described at the top of firmware/bt_pairing_test/bt_pairing_test.ino.
// Every failure is an IOException, and closes the connection.
// Callers check the Bluetooth permissions first (see BluetoothPermissions.kt)
@SuppressLint("MissingPermission")
class ClockConnection(context: Context) {

    companion object {
        private const val TAG = "ClockConnection"

        // Standard UUID for Bluetooth serial; the ESP32 BluetoothSerial library uses it too
        private val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

        private const val PROTOCOL_VERSION = 1
        private const val ANSWER_TIMEOUT_MS = 5_000L

        // Anything longer is a broken answer, not a real one
        private const val MAX_LINE_LENGTH = 64
        private const val MAX_GAME_LENGTH = 64 * 1024

        private const val NEWLINE = '\n'.code.toByte()
        private const val CARRIAGE_RETURN = '\r'.code.toByte()
    }

    private val adapter: BluetoothAdapter? = context.bluetoothAdapter()

    // Runs the reader; lives as long as the app
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    // One request at a time, otherwise two answers could mix
    private val mutex = Mutex()

    @Volatile
    private var socket: BluetoothSocket? = null

    // Everything the clock sends, byte by byte; filled by the reader
    @Volatile
    private var incoming = Channel<Byte>(Channel.UNLIMITED)

    private val _connectedDevice = MutableStateFlow<Device?>(null)

    // The clock the app is talking to; null = none
    val connectedDevice: StateFlow<Device?> = _connectedDevice.asStateFlow()

    // Opens the connection and checks that the clock speaks the same protocol.
    // Returns how many games are waiting on the clock
    suspend fun connect(device: Device): Int = withContext(Dispatchers.IO) {
        disconnect()

        val adapter = adapter ?: throw IOException("This phone has no Bluetooth")

        // A running search makes connecting slow or fail
        adapter.cancelDiscovery()

        val newSocket = adapter.getRemoteDevice(device.mac).createRfcommSocketToServiceRecord(SPP_UUID)
        try {
            // Blocks until connected, or throws after a few seconds
            newSocket.connect()
        } catch (e: IOException) {
            newSocket.close()
            throw e
        }

        incoming = Channel(Channel.UNLIMITED)
        socket = newSocket
        startReading(newSocket, incoming)

        // If HELLO fails, request() already disconnected
        val games = hello()
        _connectedDevice.value = device
        Log.d(TAG, "Connected to ${device.name} (${device.mac}), $games games on the clock")
        games
    }

    fun disconnect() {
        val oldSocket = socket
        socket = null
        _connectedDevice.value = null
        try {
            // Also ends the reader
            oldSocket?.close()
        } catch (e: IOException) {
            // Already closed
        }
    }

    // HELLO -> HELLO <protocol version> <number of games>
    suspend fun hello(): Int = request("HELLO") {
        val parts = parseAnswer(readLine(), "HELLO", 3)
        val version = parts[1].toNumber()
        if (version != PROTOCOL_VERSION) {
            throw IOException("Clock speaks protocol $version, the app $PROTOCOL_VERSION")
        }
        parts[2].toNumber()
    }

    // LIST -> one GAME line per game, then END
    suspend fun listGames(): List<ClockGame> = request("LIST") {
        val games = mutableListOf<ClockGame>()
        var line = readLine()
        while (line != "END") {
            games += parseGame(line, "GAME")
            line = readLine()
        }
        games
    }

    // GET -> DATA header, exactly <length> bytes, \n, END <id>.
    // Returns null if the data does not match its CRC, so the caller can ask again
    suspend fun getGame(id: Int): String? = request("GET $id") {
        val header = parseGame(readLine(), "DATA")
        if (header.id != id) throw IOException("Asked for game $id, got ${header.id}")

        val data = ByteArray(header.length) { readByte() }
        if (readByte() != NEWLINE) throw IOException("No newline after the data of game $id")
        expectLine("END $id")

        val crc = CRC32().apply { update(data) }.value
        if (crc == header.crc) {
            String(data, Charsets.UTF_8)
        } else {
            Log.w(TAG, "Game $id: CRC %08X, expected %08X".format(crc, header.crc))
            null
        }
    }

    // DEL -> OK <id>
    suspend fun deleteGame(id: Int) = request("DEL $id") {
        expectLine("OK $id")
    }

    //-------------------------------------------------------------------------------------

    // Sends one command and reads its answer
    private suspend fun <T> request(command: String, readAnswer: suspend () -> T): T = mutex.withLock {
        val currentSocket = socket ?: throw IOException("Not connected")
        try {
            withContext(Dispatchers.IO) {
                Log.d(TAG, "-> $command")
                currentSocket.outputStream.write("$command\n".toByteArray())
                currentSocket.outputStream.flush()
                try {
                    withTimeout(ANSWER_TIMEOUT_MS) { readAnswer() }
                } catch (e: TimeoutCancellationException) {
                    throw IOException("No answer to $command")
                }
            }
        } catch (e: Exception) {
            // Also on cancel: the rest of this answer would end up in front of the next one,
            // so start over with a new connection
            Log.w(TAG, "$command failed", e)
            disconnect()
            throw e
        }
    }

    // Copies everything the clock sends into the channel, until the connection closes
    private fun startReading(socket: BluetoothSocket, channel: Channel<Byte>) = scope.launch {
        val buffer = ByteArray(256)
        try {
            while (true) {
                val count = socket.inputStream.read(buffer)
                if (count < 0) break
                for (i in 0 until count) channel.send(buffer[i])
            }
        } catch (e: IOException) {
            // Closed: disconnect() was called, or the clock was turned off / is out of range
        }
        channel.close()

        // Lost by itself, not through disconnect(): tell the UI
        if (this@ClockConnection.socket === socket) {
            Log.d(TAG, "Connection lost")
            disconnect()
        }
    }

    private suspend fun readByte(): Byte =
        incoming.receiveCatching().getOrNull() ?: throw IOException("Connection lost")

    // One line from the clock, without the \r\n. An ERR answer becomes an IOException
    private suspend fun readLine(): String {
        val bytes = ByteArrayOutputStream()
        while (true) {
            val byte = readByte()
            if (byte == NEWLINE) break
            if (byte != CARRIAGE_RETURN) bytes.write(byte.toInt())
            if (bytes.size() > MAX_LINE_LENGTH) throw IOException("Line too long")
        }
        val line = String(bytes.toByteArray(), Charsets.UTF_8)
        Log.d(TAG, "<- $line")

        if (line.startsWith("ERR")) throw IOException("Clock answered: $line")
        return line
    }

    private suspend fun expectLine(expected: String) {
        val line = readLine()
        if (line != expected) throw IOException("Expected \"$expected\", got \"$line\"")
    }

    // "HELLO 1 2" -> [HELLO, 1, 2]; throws if the first word or the number of parts is wrong
    private fun parseAnswer(line: String, word: String, parts: Int): List<String> {
        val split = line.split(" ")
        if (split.size != parts || split[0] != word) throw IOException("Expected $word, got \"$line\"")
        return split
    }

    // GAME/DATA <id> <length> <crc32 in hex>
    private fun parseGame(line: String, word: String): ClockGame {
        val parts = parseAnswer(line, word, 4)
        val length = parts[2].toNumber()
        if (length !in 0..MAX_GAME_LENGTH) throw IOException("Game too long: $length bytes")

        val crc = parts[3].toLongOrNull(16) ?: throw IOException("Not a CRC: ${parts[3]}")
        return ClockGame(id = parts[1].toNumber(), length = length, crc = crc)
    }

    private fun String.toNumber(): Int = toIntOrNull() ?: throw IOException("Not a number: $this")
}
