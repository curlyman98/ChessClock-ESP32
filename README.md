# Chess Clock

I made an open source chess clock, and you can build one too: the parts, 3D-printed case and wiring are on [Instructables](https://www.instructables.com/Chess-Clock-Open-Source-3D-Printed-and-Powered-by-/). This repository has the firmware for the ESP32 and the Android app.

You can set any time for each player, an increment before and after a bonus turn, a bonus that is added at that turn, and choose between Fischer increment and delay. The clock records every game (the time left after each move) and an Android app downloads the games over Bluetooth, so you can enter the moves on a board afterwards and keep the games as PGN.

**About AI use:** I used AI (Claude) for the final additions to the sensor filtering (the move detection described in [section 6](#6-how-the-move-detection-and-calibration-work)) and for this README.

**What is in this repository**

| Path | What it is |
|---|---|
| `ChessClock_18_11_2026.ino` | Current firmware for the clock |
| `Older Firmware/` | Earlier firmware versions |
| `Android App/app-debug.apk` | Ready-built Android app |
| `app/` | Source code of the Android app |

**Contents**

1. [How to use the clock](#1-how-to-use-the-clock)
2. [How to use the app](#2-how-to-use-the-app)
3. [Flashing the firmware](#3-flashing-the-firmware)
4. [Installing the app](#4-installing-the-app)
5. [Known limitations](#5-known-limitations)
6. [How the move detection and calibration work](#6-how-the-move-detection-and-calibration-work)
7. [How the clock and the app talk to each other](#7-how-the-clock-and-the-app-talk-to-each-other)
8. [How the app works](#8-how-the-app-works)
9. [Firmware reference](#9-firmware-reference)

---

# 1. How to use the clock

## Buttons

| Button | What it does |
|---|---|
| **+ / −** | Change a value. For times and setup values you can hold the button to repeat, it gets faster the longer you hold |
| **Play** | Start, pause and resume a game |
| **OK** | Open the setup, confirm a value and go to the next one |
| **Battery** | Hold to see the battery. Combined with other buttons: brightness, calibration, sync |
| **On/Off** | Hold for 1.5 s to turn the clock off (a running game is saved first). Doesn't work in the calibration menu |

The lever is how the players switch turns: when you press your side down, your clock stops and your opponent's clock starts.

In menus, one display shows the name of the setting and the other shows its value.

## Quick start

1. Turn the clock on.
2. Set the time with **+** and **−**. Both players get the same time. The steps grow with the time: 30 s steps up to 10 min, then 1 min, then 5 min from 20 min, then 30 min from 1 h. The new time is saved when you start the game.
3. The player who does **not** move first (Black) presses their side of the lever down.
4. Press **Play**. White's clock starts.
5. After each move, press your side of the lever.
6. **Play** pauses the game. In pause, **Play** resumes and **OK** ends the game early and saves it. Don't move the lever during a pause: after resuming, that counts as a move.
7. When a player runs out of time, their display shows **FLAG**. Press **Play** to go back to the start screen.

Every game is saved on the clock automatically: when a flag falls, when you end a paused game with OK, or when you turn the clock off during a game. A game without moves is not saved, and only the first 400 half-moves are recorded. The clock keeps up to 50 games. When it is full it shows **FuLL** for 2 s at the end of the game, and **that game is lost**, so [sync](#download-games-sync) before you reach 50.

If the clock shows **CALI** on both displays for a moment when you start a game, it has never been calibrated or the lever is in neither of the learned positions. The game still starts, but you should [calibrate](#calibration) it. A clock that was never calibrated doesn't react to the lever at all.

On the first start the clock is set to 5 min per player with a 3 s Fischer increment.

## Quick setup (short press on OK)

Use this for increments and bonus time. The values apply to both players: when you leave, player 2 gets a copy of player 1's settings, including the time, so a handicap from the full setup is gone.

| Display | Setting | Unit |
|---|---|---|
| `inc ` | Increment per move, before the bonus turn | seconds |
| `Bon ` | Bonus time added once at the bonus turn | minutes |
| `inc2` | Increment per move from the bonus turn on (starts at the value of `inc`) | seconds |
| `turn` | The bonus turn | move number |
| `tYPE` | `inc ` = Fischer increment, `dELy` = delay. **+** or **−** switches | – |

**OK** goes to the next setting. After `tYPE` everything is saved and you are back on the start screen.

**Fischer**: the increment is added to your time after your move.
**Delay**: after your opponent moves, your clock waits for the increment before it starts counting down. Unused delay is not added to your time.

Example, classical 90 min / 40 moves + 30 min, with 30 s from move 1: set 1:30 on the start screen, then `inc` 30, `Bon` 30, `inc2` 30, `turn` 40, `tYPE` inc.

## Full setup (hold OK for 1 s)

Hold OK until the displays go blank, then release. Here you can give the players different times (handicap games) and set hours, minutes and seconds separately. Going past 59 minutes or seconds carries over into the next hour or minute.

| Display | Setting |
|---|---|
| `P1_h`, `P1nn`, `P1_s` | Player 1 time: hours, minutes, seconds |
| `P1in` | Player 1 increment before the bonus turn (seconds) |
| `P1bh`, `P1bn`, `P1bs` | Player 1 bonus: hours, minutes, seconds |
| `P1bi` | Player 1 increment from the bonus turn on (seconds) |
| `turn` | Bonus turn (the same for both players) |
| `P2_h` … `P2bi` | The same for player 2. They start as a copy of player 1 |
| `tYPE` | Fischer (`inc `) or delay (`dELy`) |

**Hold OK for 1.2 s** at any point to save and leave. If you leave while you are still on player 1's settings, player 2 gets a copy of them.

## Battery, brightness, calibration and sync

On the start screen, during a game, in pause and on the FLAG screen, hold the **Battery** button. One display shows the battery in percent. While you hold it:

- **+ / −** changes the brightness (1–7). It is not saved and is back to 1 after every start
- **OK** opens the calibration menu (only from the start screen)
- **Play** starts sync mode for the app (only from the start screen)

On the start screen, a Battery hold also undoes time changes made with + / − that were not used for a game yet.

## Calibration

Calibrate the clock once after building it, and again if you change the magnet or the sensor or the clock shows **CALI** when you start a game.

Hold **Battery** and press **OK**. The calibration menu goes through these screens, **OK** goes to the next one:

1. **`CALI`**: press **Play** to start the automatic calibration:
   - `8888` appears on player 1's display. Within 5 s press player 1's side of the lever down and hold it there. The clock measures for 20 s.
   - `8888` appears on player 2's display. Within 5 s press player 2's side down and hold it for 20 s.
   - `Snr` and a number are shown for 3 s. This is the signal-to-noise ratio of the sensor, the higher the better (see [below](#calibration-in-detail)).
   - You are back on `CALI`. Press **OK** to continue.
2. **`READ`**: the live lever position, roughly 0 to 100. It should be about **100** with player 1's side pressed and about **0** with player 2's side pressed. Rock the lever to check.
3. **`P1`**: the sensor value learned for player 1's side (raw value of the built-in sensor, or mV with an [external sensor](#using-an-external-hall-sensor)). **+ / −** change it by 5.
4. **`P2`**: the same for player 2's side.
5. **`ALPH`**: the filter strength × 100 (5–100, default 25). Lower is smoother but slower, higher reacts faster but is more sensitive to noise.
6. **`5AP`** (swap): if you pressed the wrong sides during the calibration, press **Play** to swap them. Press **OK** to **save** and go back to the start screen.

The calibration is only saved at the last step, and that is also the only way out of the menu.

## Turning off

Hold **On/Off** for 1.5 s (not possible in the calibration menu). The displays go blank when the hold time is reached, then the clock turns off when you release it. A running or paused game is saved first.

## What the displays show

| Time left | Format | Example |
|---|---|---|
| 1 h or more | hours : minutes | `1:30` |
| 11 to 59 min | hours : minutes | `0:45` |
| under 11 min | minutes : seconds | `9:59` |
| under 1 min | seconds | `:42` |

While a clock is running, the colon blinks in the first two formats.

---

# 2. How to use the app

The Android app downloads the games from the clock, lets you enter the moves on a board next to the recorded clock times, and keeps every game as PGN on your phone. To install it, see [Installing the app](#4-installing-the-app).

## Finding your way around

- The **menu button** (top left) opens the side menu: **Stored Games**, **Pairing** and **Settings**.
- The **light** in the top right is **green** while a clock is connected and **red** when none is. Tap it to see which clock is connected.

## Pair the clock (first time only)

1. On the clock, in the start screen: hold **Battery** and press **Play**. The clock shows **`SYnc`** and the number of games stored on it. Bluetooth is only on in this mode.
2. In the app, open **Pairing** from the menu and tap **Add new device**. The app searches for about 12 seconds. Tap **Stop Search** to stop earlier.
   - The first time, Android asks for the Bluetooth permission (*Nearby devices*, or *Location* on Android 11 and older). The app can't find the clock without it.
   - If Bluetooth is off, Android asks to turn it on. Tap **Add new device** again afterwards.
3. Your clock appears under **Nearby devices** as **`ChessClock-XXXX`**. The last four characters come from the clock's MAC address, so two clocks almost never have the same name. Other Bluetooth devices are not shown.
4. Tap the clock. Android may show a pairing dialog, confirm it. The app pairs, connects right away and shows *"Connected to ChessClock-XXXX, games on the clock: N"*.
5. The clock moves to **Paired Devices** and the light turns green.

A clock that is already paired in the phone's Bluetooth settings shows up right away when you search.

## Connect and disconnect

On the **Pairing** screen, under **Paired Devices**:

- **Tap a clock** to connect to it. The clock has to be in sync mode (**Battery** + **Play**).
- **Tap the connected clock** (it is highlighted) to disconnect.
- **Tap the pencil** next to a clock to open its device screen.

When you leave sync mode on the clock or turn it off, the app notices that the connection is gone and the light turns red.

## Download games (sync)

1. Put the clock in sync mode (**Battery** + **Play**) and connect to it in the app.
2. Tap **Sync**, either on **Stored Games** or on the clock's device screen. The button is greyed out while no clock is connected.
3. A spinner shows while the games are downloaded. When it is done you see *"Games imported: N"*.
4. On the clock, press **OK** or **Play** to leave sync mode. Bluetooth turns off again.

What you should know about syncing:

- **Synced games are deleted from the clock.** Each game is deleted on the clock only after it is saved in the app, so a broken connection never loses a game. This is also how you make room when the clock shows **FuLL**.
- A game that arrives broken is downloaded again, up to 3 tries in total. If it still doesn't arrive complete, it stays on the clock for the next sync and the message says *"Games imported: N, could not be read: M"*.
- If anything goes wrong (lost connection, no answer within 5 s, an error from the clock), you see *"Sync failed, the connection to the clock was lost"* and the app disconnects. The games that were already imported are kept.
- The sync keeps running if you leave the screen, and the result is shown on the next Stored Games or device screen you open. While it runs, the Sync button reads **Cancel Pairing**: tapping it cancels the sync (not the pairing) and usually disconnects the clock.

A synced game is named *"Game &lt;id&gt; Synced on &lt;date and time&gt;"*, with both players set to *Unknown GM*. The id is the game's number on the clock, which is never reused, so the numbers can have gaps. You can change all of that afterwards.

## Device screen

Open it with the pencil next to a paired clock. It shows:

- the clock's **name**. Tap the pencil to **rename** it (up to 30 characters)
- the **MAC address**. Tap it to copy it
- **Last connected** and **Last synced**
- **Games on the clock** (*Unknown* until the app has connected once)
- a **Sync** button, which works while this clock is the connected one
- **Games from this device**: every game imported from this clock. Tap one to open it

The bin icon **forgets** the clock (after asking): it is disconnected and removed from the app, but its games stay. Android still has it paired, so it shows up again right away the next time you search.

## Stored Games

**Stored Games** lists every game in the app.

- The **sort button** switches between by date, oldest first (calendar icon), and by name (A–Z icon).
- **Add game** opens a new game without clock times, for example to enter a game that was played without the clock. It is only stored when you tap **Save**.
- **Sync** downloads the games from the connected clock (see above).
- Tap a game to open it.

## Entering the moves of a game

The clock only records times, not moves. A synced game opens with the starting position and a move list that already has the clock time of every move. The moves are shown as **…** until you enter them.

| Row | White | | Black | |
|---|---|---|---|---|
| 1. | … | 4:58 | … | 4:55 |
| 2. | … | 4:51 | … | 4:49 |

To enter a move, **tap a piece** of the side to move, then **tap one of the marked squares**. Only legal moves are possible, including castling and en passant. When a pawn reaches the last rank, the app asks which piece to promote to. Each move takes the next clock time in the list, so play the moves in the order they were played.

- **Previous move** / **Next move** step through the game. The current move is highlighted in the list, and the last move is marked on the board.
- If you go back and play the **same** move that is already recorded, the app just steps forward. If you play a **different** move, every move after it is removed and you continue from there.
- Tap **Save** to keep your changes (moves, names and details). If you leave a game with unsaved changes, the app asks: **Save**, **Discard** or **Cancel**.

## Game details

- **Tap the white or black player's name** to change it (up to 40 characters). An empty name goes back to *Unknown GM*.
- **The pencil** opens **Game details**:
  - **Name** of the game (also the PGN `Event`), up to 60 characters
  - **Site**, up to 60 characters
  - **Date**, with a date picker. This is also the date used for sorting
  - **Round**, up to 10 characters
  - **Result**: 1-0, ½-½, 0-1 or Unknown
- **The bin** deletes the game permanently (after asking).

An empty Site or Round is saved as `?`, which means "unknown" in PGN, and an empty name keeps the old one. Remember to tap **Save** after changing details.

---

# 3. Flashing the firmware

The current firmware is `ChessClock_18_11_2026.ino`. Older versions are in `Older Firmware/`.

- **Board**: ESP32 (classic Bluetooth is needed, so ESP32-S2/S3/C3 won't work)
- **Core**: arduino-esp32 **2.0.x**. The timer API (`timerBegin` / `timerAlarmWrite`) changed in 3.x and `hallRead()` (the built-in Hall sensor) was removed there, so the sketch won't compile.
- **Library**: `TM1637Display` (by Avishay Orpaz). `Preferences`, `LittleFS` and `BluetoothSerial` come with the core.
- **Partition scheme**: one with a SPIFFS/LittleFS partition, the games are stored there. If the sketch is too big for the default scheme, use *Huge APP (3MB No OTA/1MB SPIFFS)*.

The data partition is formatted automatically on the first start.

## Pins

| Function | GPIO |
|---|---|
| Player 1 display CLK / DIO | 21 / 22 |
| Player 2 display CLK / DIO | 25 / 26 |
| Built-in Hall sensor (inside the ESP32, leave these pins unconnected) | 36 / 39 |
| External Hall sensor, only if you [use one](#using-an-external-hall-sensor) (analog) | 34 |
| Battery voltage (analog) | 13 |
| Power latch (output, HIGH keeps the clock on) | 23 |
| Button + | 4 |
| Button − | 16 |
| Button Play | 17 |
| Button Battery | 5 |
| Button OK | 18 |
| Button On/Off | 27 |

The button pins have no internal pull resistors. +, −, Play, Battery and OK are active-high and need pull-down resistors. On/Off is active-low and needs a pull-up. The battery pin reads 0 % at about 1.3 V and 100 % at about 2.1 V, so the battery voltage has to go through a voltage divider. In the code, `display_2` is player 1's display and `display_1` is player 2's.

## Using an external Hall sensor

The firmware uses the Hall sensor that is built into the ESP32, so the lever magnet has to move right over the ESP32 chip (under the metal shield of the module). The built-in sensor is much less sensitive than a separate one, so if the SNR after the [calibration](#calibration) stays low, use an external analog Hall sensor like the SS49E on GPIO 34:

1. Connect the sensor output to GPIO 34 and place it at the magnet.
2. In `read_hall_burst()`, comment out `samples[i] = hallRead();` and uncomment the line below it, `samples[i] = analogReadMilliVolts(hall);`. Nothing else in the code has to change, GPIO 34 is already set up in `setup()`.
3. Flash the firmware and [calibrate](#calibration) again. The saved levels are in the units of the old sensor and don't fit the new one (usually the clock shows **CALI** until you do).

To go back to the built-in sensor, swap the two lines again and recalibrate.

---

# 4. Installing the app

The folder `Android App/` contains a ready-built app: `app-debug.apk`.

- **Needs**: Android 8.0 or newer, with Bluetooth.
- **Install**: copy `app-debug.apk` to the phone and open it. Android asks you to allow installing apps from that source (the app is not in the Play Store). The app is called **Chess_Clock_App** on the home screen.
- **Permissions**: *Nearby devices* on Android 12 and newer, *Location* on Android 11 and older (older Android versions only find Bluetooth devices with it, and usually the Location setting has to be switched on too). The app asks for them the first time you search for or connect to a clock.

## Building the app from source

The source is in `app/` (Kotlin, Gradle). Open the `app/` folder (not the repository root) in a recent Android Studio and press Run, or build from the command line:

```
cd app
./gradlew assembleDebug     # Windows: gradlew.bat assembleDebug
```

The APK ends up in `app/app/build/outputs/apk/debug/`. You need a JDK 17 or newer to start Gradle (Gradle downloads the JDK 25 it builds with by itself) and the Android SDK with platform 37. `./gradlew test` runs the unit tests for the chess rules.

Read the last point of [Known limitations](#5-known-limitations) before you install your own build over an existing one.

---

# 5. Known limitations

- The battery is on GPIO13 (ADC2) and can't be read while Bluetooth is on. It should move to GPIO32–39 on the next board.
- The time settings of a game are stored on the clock but not sent to the app yet.
- A full clock (50 games) drops the game that was just played instead of warning before it starts.
- All button interrupts are attached twice in `setup()` (`setup_buttons()` and `enable_button_interrupts()`). Harmless, the second one wins.
- The app has no export or share for the PGN yet, and the **Settings** screen is still empty.
- The app is a development build: when an update changes the structure of its database, the database is wiped, **including the stored games**. An APK built on another computer is signed with a different debug key and only installs after uninstalling the old app, which also deletes the games. Since synced games are deleted from the clock, the app holds the only copy.

---

# 6. How the move detection and calibration work

The lever carries a magnet, and the ESP32's built-in Hall sensor (or an [external one](#using-an-external-hall-sensor)) measures its field. The value changes depending on which side of the lever is pressed down. Everything below is about turning that noisy value into a reliable "the player moved". The values are raw numbers with the built-in sensor (they can be negative) and millivolts with an external one. The method is the same for both.

## Reading the sensor

The sensor is read in its own FreeRTOS task (`hall_task`) every few milliseconds (the reads plus a 2 ms pause), independent of the menus and displays, so a busy display or a held button can never make the clock miss a move.

Every sample is:

1. **16 quick reads** of the sensor.
2. **Sorted**, the 3 highest and 3 lowest are dropped (spikes from the power supply end up there), the remaining 10 are averaged.
3. An **exponential moving average** on top: `filtered = alpha × new + (1 − alpha) × filtered`. `alpha` is the `ALPH` setting.

## From sensor value to lever position

Calibration learns two numbers: `level_p1`, the sensor value with player 1's side pressed, and `level_p2`, the value with player 2's side pressed. Every sample is turned into a position:

```
pos = (filtered − level_p2) / (level_p1 − level_p2)
```

So `pos` is about **0** with player 2's side pressed and about **1** with player 1's side pressed, no matter how strong the magnet is or which way round it is mounted. The `READ` screen shows `pos × 100`.

## Deciding that a player moved

- **Hysteresis**: the lever has to cross **0.65** to count as player 1's side and **0.35** to count as player 2's side. Anything in between keeps the current side, so a lever that is only half pressed doesn't flicker between players.
- **Confirmation**: the new side has to hold for **3 samples in a row** (a few ms), so a single noisy sample can't switch.
- **Lockout**: after a switch nothing can switch for **250 ms**, so a lever that bounces back doesn't count as a second move.

## Drift tracking

The sensor output drifts slowly with temperature and supply voltage. The drift is mostly a common offset, so both levels move together:

- **During a game**, while the lever rests (within 0.2 of its level), both levels follow the measured value very slowly (a time constant of a few seconds). A real move is far faster and is not affected.
- **At the start of a game** the clock takes 50 samples. If the lever is clearly on one side, the drift since the calibration is taken out of both levels at once. These corrections are kept only until the clock is turned off. If it is on neither side, the calibration no longer fits and the clock shows **CALI**.

The side that is pressed when you press Play decides whose clock runs. For the first 250 ms after Play no switch is possible.

## Calibration in detail

For each side the automatic calibration:

1. waits 5 s so you have time to press the lever,
2. warms up the filter with 50 samples,
3. measures for 20 s in the sensor task with the same filter as in a game, and keeps the **average** (the new level) and the **peak-to-peak noise** (max − min).

At the end:

```
SNR = |level_p1 − level_p2| / max(noise_p1, noise_p2)
```

The SNR is only shown for information, nothing depends on it. The switch thresholds are 0.35 of the separation away from each resting level, so the noise has to stay well below that: the higher the SNR the safer. A low single-digit SNR means the magnet is too weak or too far from the sensor, or the supply is noisy.

The levels, `alpha` and the SNR are saved in flash (NVS) and loaded at every start.

---

# 7. How the clock and the app talk to each other

## Recording games

During a game, every move records the **time left of the player who just moved, after the increment and bonus**, in 10 ms units (up to 400 half-moves per game). The time settings at the start of the game are kept too.

A game is saved to LittleFS as two files:

- `/g/<id>`: the move times separated by spaces, e.g. `17970 17955 17910 …`
- `/g/<id>.tc`: the time settings: `<F|D> <bonus_turn> <who started 1|2>` and, for each player, `<time> <increment_start> <bonus> <increment_after_bonus>`, all times in 10 ms units

Game ids come from a counter in NVS and are never reused, even after a game is deleted.

## Bluetooth

The clock uses classic Bluetooth serial (SPP, UUID `00001101-0000-1000-8000-00805F9B34FB`) and is named `ChessClock-XXXX`. Bluetooth is only on in sync mode, so it adds no noise to the sensor during a game. While it is on, the battery can't be read (the battery pin is on ADC2, which the ESP32 can't use together with Bluetooth).

## Protocol (version 1)

Text lines ending with `\n`, the app asks and the clock answers. Commands are case-sensitive, `\r` is ignored and a command can be at most 23 characters.

| App sends | Clock answers |
|---|---|
| `HELLO` | `HELLO <protocol version> <number of games>` |
| `LIST` | one `GAME <id> <length> <crc32>` per game, then `END` |
| `GET <id>` | `DATA <id> <length> <crc32>`, then `<length>` bytes of the game file, then `\nEND <id>`, or `ERR NO_GAME <id>` |
| `DEL <id>` | `OK <id>` (also if it was already deleted), or `ERR NO_GAME <id>` if the id never existed |
| anything else | `ERR UNKNOWN` |
| a line that is too long | `ERR TOO_LONG` |

`<crc32>` is a standard CRC-32 as 8 hex digits (the app checks it with `java.util.zip.CRC32`), so the app can check that a game arrived complete before it deletes it from the clock. The `.tc` settings file is not sent yet, that needs protocol version 2.

## A sync, step by step

1. **Connect**: the app opens the SPP connection and sends `HELLO`. If the clock answers with a different protocol version, the app refuses to talk to it (you only see *"Could not connect"*). The number of games is shown in the app.
2. **`LIST`**: the app gets the id, length and CRC of every game on the clock.
3. For each game:
   - If the app already has a game with this clock's MAC and this id (an earlier sync saved it, but its `DEL` got lost), it skips the download.
   - Otherwise **`GET <id>`**: the app reads exactly `<length>` bytes and compares the CRC. If it doesn't match, it asks again, up to 3 tries in total. A game that never arrives complete stays on the clock.
   - The game is **saved in the app first**, then **`DEL <id>`** removes it from the clock.
4. The app saves the sync time and asks `HELLO` again to update the number of games on the clock.

Because the clock never reuses an id, "this MAC + this id" always means the same game, so a game is never imported twice.

The app sends one command at a time and waits up to 5 s for each complete answer (for `LIST` and `GET` that includes every line and byte up to `END`). Every error closes the connection, so a half-read answer can never mix with the next one.

---

# 8. How the app works

The app is written in Kotlin (Android 8.0+, Room database, ViewModels and coroutines).

## Screens

| Screen | What it does |
|---|---|
| Start | Welcome text |
| Pairing | Search, pair, connect and disconnect clocks |
| Device | One clock: rename, forget, copy MAC, last connected / synced, games on the clock, sync, its games |
| Stored Games | All games, sort, add, sync |
| Game | Board, move list with clock times, player names, game details, save, delete |
| Settings | Empty for now |

Every screen shares the side menu and the connection light in the top bar.

## Finding and pairing clocks

The search uses Android's classic Bluetooth discovery and only shows devices whose name starts with `ChessClock-`. Clocks that the phone is already paired with are shown right away, because the search may not find a clock that is busy. A running search is stopped before pairing or connecting, because both often fail while it runs. Pairing waits up to 60 s, enough time to answer Android's pairing dialog. After pairing, the app connects right away.

## Where things are stored

Everything lives in a local database on the phone, with two tables:

- **devices**: MAC (the key), name, last connected, last synced, number of games on the clock
- **games**: an id, the game as PGN, its name, the date it was created or imported, and, for synced games, the clock's MAC and the game id on the clock (empty for games added by hand)

Forgetting a clock only deletes its row in **devices**. Its games stay.

## From clock times to PGN

A synced game arrives as a list of numbers, the time left after each half-move in 10 ms units (see [Recording games](#recording-games)). The app turns each number into a clock time like `0:04:58` and writes a PGN with the standard tags:

```
[Event "Game 12 Synced on 18 Nov 2026, 14:30"]
[Site "?"]
[Date "2026.11.18"]
[Round "?"]
[White "Unknown GM"]
[Black "Unknown GM"]
[Result "*"]
```

The clock times are kept as `{[%clk 0:04:58]}` comments, the usual PGN way to store clock times, which Lichess and other chess programs can read once the moves are entered. When you enter moves on the board, the app writes them in SAN (`e4`, `Nf3`, `O-O`, …) and puts each move next to its clock time.

The move generator in the app knows all the rules: castling (not out of, through or into check), en passant, promotion, and it only allows moves that don't leave the own king in check. Moves that are read from a PGN are checked the same way, and the game stops at the first move that isn't legal.

Text you type is cleaned before it is saved: line breaks, tabs and repeated spaces become a single space (a line break inside a PGN tag would break it) and the length is limited (game name and site 60, player 40, round 10, clock name 30 characters).

---

# 9. Firmware reference

## How the firmware is organised

| Part | Runs | Does |
|---|---|---|
| `Timer0_ISR` | every 10 ms | counts the running player's time (or the delay) down |
| `hall_task` | every few ms, core 1, priority 2 | reads the sensor, switches players, records moves |
| `loop()` | priority 1 | buttons, menus, displays, saving, Bluetooth |

The task has a higher priority than `loop()`, so it runs on time even while `loop()` is busy. Every shared value has one writer at a time. Only `time_ms` and `delay_left` are written by both the ISR and the task, so they are changed under the spinlock `time_mux`. `loop()` tells the task what to do with `hall_mode`, the task tells `loop()` about a move with `switch_count`.

The CPU runs at 80 MHz.

## Structs and enums

### `chess_player`
- **`volatile int time_ms`**: remaining time in 10 ms units (the name is historical)
- **`int increment_start`**: increment before the bonus turn
- **`int bonus_1`**: bonus added once at the bonus turn
- **`int increment_1`**: increment from the bonus turn on
- **`int current_turn`**: the player's move number

### `game_state`
`READY`, `SETUP`, `SMALL_SETUP`, `RUNNING`, `PAUSED`, `CALIBRATION`, `GAME_ENDED`, `SYNC`

### `time_modes`
`FISCHER` (increment after the move) or `DELAY` (the clock waits before counting down). `time_mode` itself is an `int`

### `setup_steps`
`STEP_1` … `STEP_20`, the current step in the setup and calibration menus

### `hall_modes`
- `HALL_IDLE`: the task does nothing, `loop()` owns the filter and levels
- `HALL_GAME`: sample, track drift, switch players
- `HALL_MEASURE`: sample and collect min / max / sum for calibration

### `TYPE`
Step sizes for `plus_minus_one()` in 10 ms units: `h` (1 h), `m` (1 min), `s` (1 s), `turn` (1)

## Global variables

### Game
- **`player_1`, `player_2`**: the players' settings and times
- **`current_player`**: the player whose time is running (the pointer is volatile, the ISR uses it)
- **`player_flag`**: the side the lever is on, true = player 1's side pressed (player 2's clock runs)
- **`bonus_turn`**: the move at which the bonus is added and the increment changes
- **`time_mode`**: `FISCHER` or `DELAY`
- **`delay_left`**: delay mode, ticks left before the running time counts down
- **`state`, `last_state`, `step_flag`**: state machine and the current menu step
- **`times_loaded_flag`**: the saved times are loaded once when entering `READY`. `resetFlags()` clears it, so they are loaded again after a Battery hold

### Move detection
- **`alpha`**: EMA factor (saved, 0.05–1.00)
- **`filtered`**: filtered sensor value (raw built-in sensor value, or mV with an external sensor)
- **`pos`**: lever position, 0 = player 2's side, 1 = player 1's side
- **`level_p1`, `level_p2`**: sensor value with each side pressed (saved)
- **`snr`**: separation / noise from the last calibration (saved)
- **`pos_to_p1`, `pos_to_p2`**: switch thresholds 0.65 / 0.35
- **`confirm_samples`, `lockout_ms`**: 3 samples, 250 ms
- **`rest_band`, `drift_beta`**: drift tracking, 0.2 and 0.0004
- **`hall_mode`**: what the task does
- **`switch_count`**: +1 per move, `loop()` compares it to redraw the displays
- **`measure_min`, `measure_max`, `measure_sum`, `measure_count`**: calibration statistics

### Recording and Bluetooth
- **`half_moves[400]`, `move_count`**: times recorded in the current game
- **`game_settings`**: time settings at the start of the game
- **`bt_name`**: `ChessClock-XXXX`
- **`bt_on`**: Bluetooth is running
- **`line`, `line_length`, `line_too_long`**: the incoming command

### Buttons and display
- **`plus_flag`, `minus_flag`, `play_flag`, `bat_flag`, `ok_flag`, `onoff_flag`**: set by the button interrupts
- **`last_plus` … `last_onoff`, `debounce_ms`**: debounce, in CPU cycles (150 ms)
- **`brightness`**: 1–7
- **`last_drawn_time`, `last_drawn_player`, `last_drawn_switch`**: what the displays show, so they are only rewritten when something changed
- **`prefs`**: NVS storage

## Functions

### Interrupts
- **`Timer0_ISR()`**: every 10 ms, counts `delay_left` down first, then the running player's time
- **`plus_ISR()`, `minus_ISR()`, `play_ISR()`, `bat_ISR()`, `ok_ISR()`, `onoff_ISR()`**: debounced, set their flag
- **`setup_buttons()`, `enable_button_interrupts()`, `disable_button_interrupts()`**: set the pin modes and attach / detach the button interrupts (`disable_` is not used)

### Input and settings
- **`long_press(button, ms, level)`**: true if the button stays at `level` for `ms`. Clears the displays when the time is reached and waits for the release
- **`resetFlags()`**: clears the button flags (except `onoff_flag`) and `times_loaded_flag`. Called on every state change and at the end of `read_battery()`
- **`read_battery()`**: battery in percent while the button is held, brightness with + / −, and from `READY`: OK → calibration, Play → sync. Skipped while Bluetooth is on
- **`set_brightness()`**: + / − change the brightness of both displays
- **`simple_plus()`, `simple_minus()`**: change both players' time, the step depends on the time, holding repeats faster (250 ms down to 100 ms). Minus stops at 30 s
- **`equalise_normalise()`**: copies player 1's time to player 2 and rounds both down to 10 s
- **`plus_minus_one(value, type)`**: + / − change a value by one `type` step, with repeat while held
- **`time_mode_step()`**: the `tYPE` menu step, toggles Fischer / delay

### Display
- **`display_time(time, blink, which)`**: shows a time as H:MM, 0:MM, M:SS or :SS (see [What the displays show](#what-the-displays-show)), optionally with a blinking colon. `which` = true draws on `display_1` (player 2)
- **`refresh_displays()`**: shows both players' times
- **`charToMask(c)`**: character → 7-segment pattern
- **`showWord(text, display)`**: shows 4 characters on one display

### Move detection
- **`read_hall_burst()`**: 16 reads of the built-in sensor (`hallRead()`), drops 3 at each end, returns the average. The commented line reads an external sensor instead
- **`hall_sample()`**: one burst + EMA + `pos`. False while not calibrated
- **`anchor_levels()`**: at game start, takes the drift since the last game out of both levels. False if the lever is on neither side
- **`warmupFilter(samples)`**: fills the filter and anchors the levels. False if the lever wasn't found
- **`check_player()`**: drift tracking, lockout, hysteresis and confirmation. True when the lever moved to the other side
- **`turn_increment(player)`**: the increment for the player's current move
- **`player_change()`**: adds bonus and increment (Fischer) or sets the delay, counts the move, switches the running player under `time_mux`, then records the move and counts up `switch_count`
- **`record_move(time)`**: stores the time left after the move
- **`hall_task()`**: the sensor task (burst, then a 2 ms sleep), runs `check_player()` / `player_change()` or `measure_step()` depending on `hall_mode`
- **`stop_hall()`**: sets `HALL_IDLE` and waits 5 ms, longer than one task step

### Calibration
- **`measure_step()`**: one calibration sample, updates min / max / sum
- **`measure_side(level, noise)`**: 20 s on one side, returns the average and the peak-to-peak noise
- **`calibrate()`**: both sides, then calculates and shows the SNR

### Saving
- **`save_player(key, player)`**, **`save_int_if_changed()`**, **`save_float_if_changed()`**: write to NVS only if the value changed, to save flash wear
- **`save_settings_if_changed()`**: both players, `bonus_turn` and `time_mode`
- **`start_game_record()`**: clears the move list and stores the time settings of the new game
- **`count_games()`**: number of games in `/g`
- **`file_name(file)`**: `/g/12` → `12`
- **`save_game()`**: writes `/g/<id>` and `/g/<id>.tc`, or shows `FuLL` and drops the game if 50 games are stored. Does nothing without moves
- **`power_off()`**: stops the task, saves the game, releases the power latch

### Bluetooth
- **`file_crc(file)`**: CRC-32 of a file
- **`send_header(type, id, file)`**: `GAME` / `DATA <id> <length> <crc32>`
- **`handle_command(command)`**: `HELLO`, `LIST`, `GET`, `DEL`
- **`read_commands()`**: collects incoming bytes into lines and handles them

### State handlers
- **`ready_handler()`**: start screen. Loads the saved times, + / − change the time, Play starts a game (warm-up, side detection, recording), OK opens the setup, Battery, power off
- **`running_handler()`**: draws the running time when the shown second changes and the mover's final time after a switch, detects the flag, pause, Battery, power off
- **`small_setup_handler()`**: quick setup, or full setup if OK is held for 1 s
- **`setup_handler()`**: full setup for both players, hold OK for 1.2 s to save and leave
- **`paused_handler()`**: Play resumes (with a new warm-up), OK ends and saves the game
- **`game_end_handler()`**: `FLAG` on the losing side, Play goes back to `READY`
- **`calibration_handler()`**: the calibration menu (`CALI`, `READ`, `P1`, `P2`, `ALPH`, `5AP`)
- **`sync_handler()`**: starts Bluetooth, answers the app, shows `SYnc` and the number of games, OK or Play turns Bluetooth off and goes back to `READY`

### Arduino
- **`setup()`**: power latch, displays, ADC, buttons, loads the settings and calibration, mounts LittleFS, sets the Bluetooth name, 80 MHz, the 10 ms timer and starts the sensor task
- **`loop()`**: resets the flags on a state change and runs the handler of the current state

