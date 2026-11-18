package com.example.chess_clock_app.chess

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Test

class ChessRulesTest {

    // Counts all move sequences of the given length; known values catch almost any rules bug
    private fun perft(position: Position, depth: Int): Long {
        if (depth == 0) return 1
        return position.legalMoves().sumOf { perft(position.play(it), depth - 1) }
    }

    // Plays the moves and returns the SAN the engine generates for each
    private fun playSans(vararg sans: String): List<String> {
        var position = Position.start()
        return sans.map { san ->
            val move = position.moveFromSan(san)
            assertNotNull("Illegal move: $san", move)
            position.toSan(move!!).also { position = position.play(move) }
        }
    }

    @Test
    fun perftFromStartPosition() {
        val start = Position.start()
        assertEquals(20L, perft(start, 1))
        assertEquals(400L, perft(start, 2))
        assertEquals(8902L, perft(start, 3))
        assertEquals(197281L, perft(start, 4))
    }

    @Test
    fun castling() {
        assertEquals("O-O", playSans("e4", "e5", "Nf3", "Nc6", "Bc4", "Nf6", "O-O").last())
    }

    @Test
    fun enPassant() {
        assertEquals("exd6", playSans("e4", "a6", "e5", "d5", "exd6").last())
    }

    @Test
    fun promotion() {
        assertEquals("hxg8=Q", playSans("h4", "g5", "hxg5", "h6", "gxh6", "Nf6", "h7", "Ng8", "hxg8=Q").last())
    }

    @Test
    fun checkmateSuffix() {
        assertEquals("Qxf7#", playSans("e4", "e5", "Bc4", "Nc6", "Qh5", "Nf6", "Qxf7").last())
    }

    @Test
    fun disambiguation() {
        assertEquals("Nbd2", playSans("d4", "d5", "Nf3", "Nf6", "Nbd2").last())
    }

    @Test
    fun pgnKeepsClocksAndMoves() {
        val imported = Pgn.parse("{[%clk 0:04:58]} {[%clk 0:04:55]} {[%clk 0:04:51]}")
        assertEquals(listOf("0:04:58", "0:04:55", "0:04:51"), imported.clocks)
        assertEquals(emptyList<String>(), imported.sans)

        val written = Pgn.write(imported.copy(sans = listOf("e4", "e5")))
        assertEquals("1. e4 {[%clk 0:04:58]} e5 {[%clk 0:04:55]} {[%clk 0:04:51]} *", written)

        val reparsed = Pgn.parse(written)
        assertEquals(listOf("e4", "e5"), reparsed.sans)
        assertEquals(imported.clocks, reparsed.clocks)
    }
}
