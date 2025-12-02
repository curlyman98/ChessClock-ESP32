# Chess Clock

I made an open source chess clock and you can make one as well with the instructions on https://www.instructables.com/Chess-Clock-Open-Source-3D-Printed-and-Powered-by-/

Currently you can set any time for any player and also any bonus and any increment before and after the bonus turn. I want to eventualy extend the functionality to save the turn times and then exort them to a phone or PC using Bluetooth


This document describes every function, global variable, and struct used in the code.
I only used AI for the README file.

---

# Structs

## `chess_player`
Represents one player's full time-control configuration and current game status.

Fields:
- **`int time_ms`**  
  Current remaining time in milliseconds.

- **`int increment_start`**  
  Increment applied before the first move.

- **`int bonus_1`**  
  Additional bonus time for early moves.

- **`int increment_1`**  
  Increment applied after `bonus_turn` moves are completed.

- **`int bonus_turn`**  
  Turn number where the time-control switches from bonus to increment.

---

# Global Variables

## Player / Game State
- **`chess_player player_1`**  
  Saved + editable configuration for player 1.

- **`chess_player player_2`**  
  Saved + editable configuration for player 2.

- **`chess_player* current_player`**  
  Pointer to current active player's struct.

- **`int turn`**  
  Current turn counter used to determine when to switch increments/bonuses.

- **`bool player_flag`**  
  Indicates which side the Hall sensor believes is active.

- **`bool update_flag`**  
  Set whenever a player switch event requires processing.

- **`int last_state`**  
  Previous state, used for detecting transitions.

- **`int state`**  
  Main state machine variable.

- **`bool times_loaded_flag`**  
  Ensures time configuration is loaded once when entering READY.

---

## Buttons (flags only)
Each flag becomes **true** when the corresponding ISR fires:
- **`volatile bool plus_flag`**  
- **`volatile bool minus_flag`**  
- **`volatile bool play_flag`**  
- **`volatile bool bat_flag`**  
- **`volatile bool ok_flag`**  
- **`volatile bool onoff_flag`**

---

## Display / ADC / Power
- **`int brightness`**  
  Display brightness level (1–7).

- **`int batt_percent`**  
  Battery percentage based on ADC reading.

- **`int filtered`**  
  Filtered Hall sensor value.

- **`int big_mid`**, **`int small_mid`**  
  Threshold calibration values for Hall sensor switching.

- **`int threshhold`**  
  Current switching threshold between players.

- **`Preferences pref`**  
  NVS storage object.

---

# Functions

## Interrupt Handlers (ISR)

### `Timer0_ISR()`
Decrements the active player's time each millisecond (or tick) and sets `update_flag` to request a player switch.

### `plus_ISR()`
Debounced interrupt for the “plus” button; sets `plus_flag`.

### `minus_ISR()`
Debounced interrupt for the “minus” button; sets `minus_flag`.

### `play_ISR()`
Debounced interrupt for the “play/pause” button; sets `play_flag`.

### `bat_ISR()`
Debounced interrupt for battery-check button; sets `bat_flag`.

### `ok_ISR()`
Debounced interrupt for OK button; sets `ok_flag`.

### `onoff_ISR()`
Debounced interrupt for power button; sets `onoff_flag`.

---

## Initialization / Interrupt Control

### `setup_buttons()`
Configures button pins and attaches interrupt handlers.

### `disable_button_interrupts()`
Detaches all button interrupts so no button events are processed.

### `enable_button_interrupts()`
Re-attaches all button interrupts, restoring normal input handling.

---

## Arduino Core

### `setup()`
Initializes GPIOs, ADC, power pin, displays, interrupts, stored preferences, CPU frequency, timer configuration, and loads calibration parameters.

### `loop()`
Runs the main state machine. On every iteration, detects state changes, clears flags if needed, and jumps to the appropriate state handler.

---

## User Input and Helpers

### `long_press(button, ms, pull)`
Samples a button for `ms` milliseconds to determine if it is held continuously. Clears the display while waiting.

### `resetFlags()`
Clears button flags such as `plus_flag`, `minus_flag`, `play_flag`, etc. Also resets `times_loaded_flag`.

---

## Battery / Brightness

### `read_battery()`
Reads ADC repeatedly, calculates battery percentage (1–100), temporarily displays it, allows brightness changes, and enables entering calibration via OK.

### `set_brightness()`
Uses plus/minus flags to change brightness (1–7) and updates displays.

---

## Time Editing

### `simple_plus()`
Increases player 1's time using tiered increments (3s base, larger steps at higher times), then mirrors the value to player 2 and normalizes to whole seconds.

### `simple_minus()`
Decreases time with tiered steps; enforces a minimum of 3000 ms; mirrors and normalizes.

### `equalise_normalise()`
Copies player 1's time to player 2 and rounds both down to the nearest whole second.

### `plus_minus_one(int* value)`
Adds or subtracts 100 ms from `*value` using the plus/minus flags.

---

## Display

### `display_time(time_ms, bling, which)`
Formats and writes time to a TM1637 display in HH:MM, MM:SS, or SS format. Supports blinking separator dots.

### `refresh_displays()`
Rewrites both player displays with their current formatted time.

### `charToMask(char)`
Converts a character into the 7-segment bitmask needed for TM1637 output.

### `showWord(text, display)`
Displays a 4-character word on the left or right display using `charToMask`.

---

## Sensor / Player Switching

### `warmupFilter(samples)`
Reads the Hall sensor multiple times to stabilize the filtered value before starting a game.

### `check_player()`
Reads the Hall sensor, applies an exponential filter, detects transitions across threshold values, and updates the `player_flag` and switching threshold.

### `player_change()`
Handles the actual switch between players: applies increment/bonus, updates time, advances turn count, swaps `current_player`, and refreshes the display.

---

## Calibration

### `calibrate()`
Automatic two-stage calibration of the Hall sensor: samples min/max values, computes center points for small/big magnet distances, and sets new thresholds.

---

## Storage

### `save_player(key, player)`
Reads the stored struct from NVS; if different, writes the updated struct to NVS.

### `save_players_if_changed()`
Saves both player configurations if any difference from stored data is detected.

---

# State Handlers

### `ready_handler()`
Main idle mode: loads time settings once, updates displays, processes button-based edits, starts the game, enters battery display, manages power-off, or enters mini-setup.

### `running_handler()`
Active game mode: enables the timer, processes sensor input, handles player switching, checks for time expiration, manages pause and battery display.

### `small_setup_handler()`
Short setup for increment_start, bonus_1, increment_1, and bonus_turn. Saves changes and returns to READY.

### `setup_handler()`
Full configuration menu: sets hours/minutes/seconds for both players, increments, bonuses, and turn settings. Can copy P1 to P2.

### `paused_handler()`
Timer disabled. Resume on play, exit to READY on OK, supports battery and power-off.

### `game_end_handler()`
Displays “FLAG” on the losing side, waits for reset or power-off.

### `calibration_handler()`
Interactive UI menu for calibration: shows messages, allows running `calibrate()`, shows live Hall values, allows manual threshold tuning, saves to NVS.


