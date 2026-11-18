package com.example.chess_clock_app.utils

import java.time.Instant
import java.time.LocalDate
import java.time.ZoneId
import java.time.format.DateTimeFormatter

private val DATE_TIME = DateTimeFormatter.ofPattern("d MMM yyyy, HH:mm")
private val DATE = DateTimeFormatter.ofPattern("d MMM yyyy")

// 1760450700000 -> "14 Oct 2026, 16:05" in the phone's time zone
fun formatDateTime(millis: Long): String =
    DATE_TIME.format(Instant.ofEpochMilli(millis).atZone(ZoneId.systemDefault()))

// 2026-10-14 -> "14 Oct 2026"
fun formatDate(date: LocalDate): String = DATE.format(date)

// 1760450700000 -> 2026-10-14 in the phone's time zone
fun toLocalDate(millis: Long): LocalDate =
    Instant.ofEpochMilli(millis).atZone(ZoneId.systemDefault()).toLocalDate()

// Moves millis to another day and keeps the time of day: (14 Oct 2026, 16:05) + 5 Oct -> 5 Oct 2026, 16:05
fun withDate(millis: Long, date: LocalDate): Long =
    Instant.ofEpochMilli(millis).atZone(ZoneId.systemDefault()).with(date).toInstant().toEpochMilli()
