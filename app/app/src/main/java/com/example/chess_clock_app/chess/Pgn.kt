package com.example.chess_clock_app.chess

import java.time.Instant
import java.time.LocalDate
import java.time.ZoneId
import java.time.format.DateTimeFormatter
import java.time.format.DateTimeParseException

data class PgnGame(
    // Header tags like [White "Magnus"], in the order they are written
    val tags: Map<String, String> = emptyMap(),
    // One entry per half-move, in SAN: e4, e5, Nf3, ...
    val sans: List<String> = emptyList(),
    // One entry per half-move: the remaining clock time from {[%clk 0:04:58]}
    val clocks: List<String> = emptyList(),
    val result: String = "*",
)

// TODO format according to esp format
object Pgn {


    private val TAG_LINE = Regex("""^\s*\[(\w+)\s+"((?:[^"\\]|\\.)*)"\]\s*$""")
    private val COMMENT = Regex("""\{([^\}]*)\}""")
    private val CLOCK = Regex("""\[%clk\s+([0-9:.]+)\]""")
    private val MOVE_NUMBER = Regex("""^\d+\.+""")
    private val RESULTS = setOf("1-0", "0-1", "1/2-1/2", "*")

    // The date format PGN uses in the [Date] tag
    private val DATE = DateTimeFormatter.ofPattern("yyyy.MM.dd")

    fun formatDate(millis: Long): String =
        DATE.format(Instant.ofEpochMilli(millis).atZone(ZoneId.systemDefault()))

    fun formatDate(date: LocalDate): String = DATE.format(date)

    // "2026.10.14" -> the date; null for unknown dates like "????.??.??"
    fun parseDate(tag: String): LocalDate? = try {
        LocalDate.parse(tag, DATE)
    } catch (e: DateTimeParseException) {
        null
    }

    fun parse(text: String): PgnGame {
        val tags = linkedMapOf<String, String>()
        val movetextLines = mutableListOf<String>()
        for (line in text.lines()) {
            val tag = TAG_LINE.matchEntire(line)
            if (tag != null) {
                tags[tag.groupValues[1]] = unescape(tag.groupValues[2])
            } else {
                movetextLines += line
            }
        }
        val movetext = movetextLines.joinToString(" ")

        // Pull the clock times out of the comments, then drop the comments
        val clocks = mutableListOf<String>()
        val withoutComments = COMMENT.replace(movetext) { comment ->
            CLOCK.find(comment.groupValues[1])?.let { clocks += it.groupValues[1] }
            " "
        }

        val sans = mutableListOf<String>()
        var result = "*"
        for (rawToken in withoutComments.split(Regex("""\s+"""))) {
            // Handles "1." and "1..." as well as "1.e4" written without a space
            val token = rawToken.replace(MOVE_NUMBER, "")
            when {
                token.isEmpty() -> {}
                token in RESULTS -> result = token
                token.startsWith("$") -> {} // annotation like $1, not a move
                else -> sans += token
            }
        }

        return PgnGame(tags, sans, clocks, result)
    }

    fun write(game: PgnGame): String {
        val parts = mutableListOf<String>()
        val halfMoves = maxOf(game.sans.size, game.clocks.size)

        for (ply in 0 until halfMoves) {
            val san = game.sans.getOrNull(ply)
            if (san != null) {
                if (ply % 2 == 0) parts += "${ply / 2 + 1}."
                parts += san
            }
            game.clocks.getOrNull(ply)?.let { parts += "{[%clk $it]}" }
        }
        parts += game.result

        val movetext = parts.joinToString(" ")
        if (game.tags.isEmpty()) return movetext

        // The [Result] tag must always match the result at the end of the moves
        val tags = game.tags + ("Result" to game.result)
        val header = tags.entries.joinToString("\n") { (name, value) -> "[$name \"${escape(value)}\"]" }
        return header + "\n\n" + movetext
    }

    // Inside a tag value, " and \ are written as \" and \\
    private fun escape(value: String) = value.replace("\\", "\\\\").replace("\"", "\\\"")
    private fun unescape(value: String) = value.replace("\\\"", "\"").replace("\\\\", "\\")
}
