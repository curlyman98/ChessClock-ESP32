/* Chess clock - task version
    The hall sensor is read in its own FreeRTOS task (hall_task), every 2 ms, and the task also
    switches the players. loop() only does the buttons, menus and displays, so a delay() or a
    held button can't make the clock miss a move. chess_clock/ is the same clock reading the
    sensor from loop().

    Who talks to whom
        - timer ISR (10 ms): counts the running time down
        - hall_task (core 1, priority 2, interrupts loop()): reads the sensor, switches players
        - loop() (priority 1): UI, saving, Bluetooth
    Every shared value has one writer at a time. Only time_ms / delay_left have two writers
    (ISR and task), so they are changed under time_mux. loop() tells the task what to do with
    hall_mode, the task tells loop() about a move with switch_count.

    Needs the arduino-esp32 core 2.0.x (timerBegin / timerAlarmWrite were changed in 3.x).

    Turn detection (external SS49E on the hall pin)
        - every 2 ms: 16 reads, the 3 highest and 3 lowest are dropped, the rest averaged, EMA on top
        - calibration learns the average value with each side pressed (level_p1 / level_p2)
        - pos = where the value is between the two levels, 0 = player 2 side, 1 = player 1 side
        - switch at pos > 0.65 / pos < 0.35, 3 samples in a row, then 250 ms lockout
        - drift tracking: while the lever rests, both levels follow slowly

    Games are saved to LittleFS (/g/<id>) and sent to the app in sync mode (battery + play).
    Protocol: see bt_pairing_test.ino, unchanged.

    TO DO
    - On/off interrupt is attached twice (RISING in setup_buttons, FALLING in enable_button_interrupts)
    - Battery is on GPIO13 = ADC2, it can't be read while BT is on. Move it to GPIO32-39 on the next board
    - Send the time settings (/g/<id>.tc) to the app, needs protocol version 2
*/
#include <TM1637Display.h>
#include <Preferences.h>
#include <LittleFS.h>
#include "BluetoothSerial.h"
#include "esp_rom_crc.h"

BluetoothSerial BT;
Preferences prefs;
//-------------  Tuning  ------------

float alpha = 0.25;              // EMA on top of the burst average, saved, 0.05 - 1.00
volatile float filtered = 0;     // mV
volatile float pos = 0;          // 0 = player 2 side pressed, 1 = player 1 side pressed
float level_p1 = 0;              // mV with player 1's side pressed (player_flag true), from calibration
float level_p2 = 0;              // mV with player 2's side pressed (player_flag false)
float snr = 0;                   // separation / noise, from calibration

#define HALL_BURST 16            // reads per sample
#define HALL_TRIM 3              // dropped at each end, spikes from the supply end up there
const unsigned long hall_period_ms = 2;
const float pos_to_p1 = 0.65;    // hysteresis as a part of the separation, works for any magnet
const float pos_to_p2 = 0.35;
const int confirm_samples = 3;   // new side has to hold for 3 samples (6 ms)
const unsigned long lockout_ms = 250; // no second switch from a slow press or lever bounce
const float rest_band = 0.2;     // lever counts as resting this close to a level
const float drift_beta = 0.0004; // about 5 s time constant at 2 ms per sample

int confirm_count = 0;
unsigned long last_switch = 0;

// loop() -> task. IDLE: the task does nothing, loop() owns the filter and the levels
enum hall_modes {
    HALL_IDLE,
    HALL_GAME,     // sample, track drift, switch players
    HALL_MEASURE   // sample and collect min / max / sum for calibration
};
volatile hall_modes hall_mode = HALL_IDLE;

// task -> loop(). +1 per move. A counter can't lose a move and nobody has to reset it, unlike a flag
volatile uint32_t switch_count = 0;

// Calibration, written by the task in HALL_MEASURE, read by loop() after it is IDLE again
volatile float measure_min = 1e6f;
volatile float measure_max = -1e6f;
volatile double measure_sum = 0;
volatile long measure_count = 0;
//------------------------------------------------------------------------------------
//Variables - time_ms 10ms

struct chess_player {
  volatile int time_ms = 30000;
  int increment_start = 300;
  
  int bonus_1 = 0;
  int increment_1 = 0; // not implemented yet have to extend player change
  
  int current_turn = 1;
};

enum game_state {
	READY,
	SETUP,
	SMALL_SETUP,
	RUNNING,
	PAUSED,
	CALIBRATION,
	GAME_ENDED,
	SYNC
};

// Fischer: increment added after the move. Delay: the clock waits <increment> before it counts down
enum time_modes {
    FISCHER,
    DELAY
};

enum setup_steps {
    STEP_1, STEP_2, STEP_3, STEP_4, STEP_5,
    STEP_6, STEP_7, STEP_8, STEP_9, STEP_10,
    STEP_11, STEP_12, STEP_13, STEP_14, STEP_15,
    STEP_16, STEP_17, STEP_18, STEP_19, STEP_20
};

enum TYPE {
    h = 360000,
    m = 6000,
    s = 100,
    turn = 1
};

int print_time = 0;
int hours = 0;
int mins = 0;
int sec = 0;
int ms = 0;
uint8_t blank = 0x00;

//State handlers

game_state state = READY;
game_state last_state = GAME_ENDED;
setup_steps step_flag = STEP_1;
unsigned long now = millis();

int bonus_turn = 40;
int increment = 0;
int time_mode = FISCHER;
chess_player player_1;
chess_player player_2;
volatile bool player_flag = true; // 0 - player_1    1 - player_2
// The pointer itself changes while the timer ISR uses it, so the pointer is volatile, not the player
chess_player* volatile current_player = &player_2;
volatile int delay_left = 0; // delay mode: ticks left before the current player's time counts down

// The timer ISR and player_change() both change time_ms. Without the lock a tick or an increment can get lost
portMUX_TYPE time_mux = portMUX_INITIALIZER_UNLOCKED;

//Recording games
#define MAX_HALF_MOVES 400
#define MAX_GAMES 50
int half_moves[MAX_HALF_MOVES]; // time left of the player who just moved, after the increment
volatile int move_count = 0;    // written by the task, read by loop() after the task is IDLE
char game_settings[96];         // time settings at the start of the game, saved next to it as /g/<id>.tc

//Bluetooth
#define PROTOCOL_VERSION 1
char bt_name[16];
bool bt_on = false;
char line[24];                  // incoming command, collected until \n
int line_length = 0;
bool line_too_long = false;

//Display setup
#define CLK_1  21 // ESP32 pin GPIO22 connected to CLK_1
#define DIO_1  22 // ESP32 pin GPIO23 connected to DIO_1

#define CLK_2  25 // ESP32 pin GPIO22 connected to CLK_2
#define DIO_2  26 // ESP32 pin GPIO23 connected to DIO_2

// Creates display object

TM1637Display display_2 = TM1637Display(CLK_1, DIO_1);
TM1637Display display_1 = TM1637Display(CLK_2, DIO_2);


//------------------------------------------------------------------------------------

//Battery
#define battery 13 //adc input

//Power on pin
#define power 23 //output
#define hall 34

//Buttons
#define button_plus 4//inputs
#define button_minus 16
#define button_play 17
#define button_bat 5
#define button_ok 18
#define button_onoff 27 

// Flags

volatile bool update_flag = false;
volatile bool play_flag = false;
volatile bool minus_flag = false;
volatile bool plus_flag = false;
volatile bool ok_flag = false;
volatile bool bat_flag = false;
volatile bool onoff_flag = false;
bool times_loaded_flag = false;

// Debounce variables
volatile uint32_t last_plus = 0;
volatile uint32_t last_minus = 0;
volatile uint32_t last_play = 0;
volatile uint32_t last_bat = 0;
volatile uint32_t last_ok = 0;
volatile uint32_t last_onoff = 0;
const uint32_t debounce_ms = 150*80*1000;

//Screen refresh
unsigned long last_screen_update = 0;
const unsigned long screen_update = 20;
int last_drawn_time = -1;                 // RUNNING: what the running display shows now
chess_player* last_drawn_player = NULL;
uint32_t last_drawn_switch = 0;           // switch_count when the displays were last drawn

int8_t brightness = 1;

//----------------------------------------------------------------------------

// ISR functions

// Time Interupt 10ms or 1 ms

hw_timer_t *Timer0_Cfg = NULL;
void IRAM_ATTR Timer0_ISR()
{
    portENTER_CRITICAL_ISR(&time_mux);
    if (delay_left > 0) {
        delay_left--;                  // delay mode: the time only counts once the delay is used up
    } else {
        (current_player->time_ms)--;   //Decrements the time reserv of the current player
    }
    portEXIT_CRITICAL_ISR(&time_mux);
	update_flag = true;
}

//Interupts for buttons

void IRAM_ATTR plus_ISR() {
    uint32_t  now = ESP.getCycleCount();
    if (now - last_plus > debounce_ms) {
        plus_flag = true;
        last_plus = now;
    }
}

void IRAM_ATTR minus_ISR() {
    uint32_t  now = ESP.getCycleCount();
    if (now - last_minus > debounce_ms) {
        minus_flag = true;
        last_minus = now;
    }
}

void IRAM_ATTR play_ISR() {
    uint32_t  now = ESP.getCycleCount();
    if (now - last_play > debounce_ms) {
        play_flag = true;
        last_play = now;
    }
}

void IRAM_ATTR bat_ISR() {
    uint32_t  now = ESP.getCycleCount();
    if (now - last_bat > debounce_ms) {
        bat_flag = true;
        last_bat = now;
    }
}

void IRAM_ATTR ok_ISR() {
    uint32_t now = ESP.getCycleCount();
    if (now - last_ok > (debounce_ms+100) ) {
        ok_flag = true;
        last_ok = now;
    }
}

void IRAM_ATTR onoff_ISR() {
    uint32_t now = ESP.getCycleCount();
    if (now - last_onoff > debounce_ms) {
        onoff_flag = true;
        last_onoff = now;
    }
}

// Setup function for IRS
void setup_buttons() {
    pinMode(button_plus, INPUT);   attachInterrupt(digitalPinToInterrupt(button_plus), plus_ISR, RISING);
    pinMode(button_minus, INPUT);  attachInterrupt(digitalPinToInterrupt(button_minus), minus_ISR, RISING);
    pinMode(button_play, INPUT);   attachInterrupt(digitalPinToInterrupt(button_play), play_ISR, RISING);
    pinMode(button_bat, INPUT);    attachInterrupt(digitalPinToInterrupt(button_bat), bat_ISR, RISING);
    pinMode(button_ok, INPUT);     attachInterrupt(digitalPinToInterrupt(button_ok), ok_ISR, RISING);
    pinMode(button_onoff, INPUT);  attachInterrupt(digitalPinToInterrupt(button_onoff), onoff_ISR, RISING);
}

// Functions to disable/enable interrupts
void disable_button_interrupts() {
	  detachInterrupt(button_plus);
	  detachInterrupt(button_minus);
	  detachInterrupt(button_play);
	  detachInterrupt(button_bat);
    detachInterrupt(button_ok);
    detachInterrupt(button_onoff);
}

// enables interrupts
void enable_button_interrupts() {
    attachInterrupt(digitalPinToInterrupt(button_plus), plus_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(button_minus), minus_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(button_play), play_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(button_bat), bat_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(button_ok), ok_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(button_onoff), onoff_ISR, FALLING);
}



//Functions
//-------------------------------------------------------------------------------------

//Long press detection
bool long_press(int button, int ms, int pull_up_or_down) {
	const unsigned long hold_time = ms; // ms
    unsigned long start = millis();

    while (millis() - start < hold_time) {
        if (digitalRead(button) != pull_up_or_down) {
            // Button released before 200ms
            return false;
        }
        delay(1); // small delay to avoid busy-waiting
    }
    // Button held for full ms
    display_1.clear();
    display_2.clear();
    while(digitalRead(button)==pull_up_or_down){
      delay(1);
    }   
    return true;
}

//Reading Battery

void read_battery() {
  
    const int ADC_MIN = 2420;  // = 1.3 V
    const int ADC_MAX = 3910;  // = 2.1 V
    bool button_held = true;

    if (bt_on) return; // GPIO13 is on ADC2, which can't be read while Bluetooth is on

    while(button_held){
      
      int sum = 0;
      int cycles = 10;
      
      for (int i = 0; i < cycles; i++) {
			sum += analogRead(battery);
			delay(2);
		  }
      int raw = sum / cycles;

      // Linear mapping (integer math)
      int percent = (raw - ADC_MIN) * 100 / (ADC_MAX - ADC_MIN);

      if (percent < 1)   percent = 1;
      if (percent > 100) percent = 100;


      now = millis();
      if (now - last_screen_update > 200) {
          display_2.clear();
          display_1.showNumberDec(percent, false);
          last_screen_update = now;
      }

      
      delay(10);

      set_brightness();
	  if(state == READY){
			if (ok_flag) {
				ok_flag = false;
				state = CALIBRATION;
				step_flag = STEP_1;
				button_held = false;
			}
			if (play_flag) { // battery + play -> sync with the app
				play_flag = false;
				state = SYNC;
				button_held = false;
			}
	  }
      
	  
	  
      if(digitalRead(button_bat) == LOW) button_held = false;
      //Serial.print(raw);
    }
    resetFlags();
}


//Brightnes adjustment

void set_brightness(){
	if(plus_flag) {
		brightness++;
		plus_flag = false;
	}
	if(minus_flag){
		brightness--;
		minus_flag = false;
	}

	if(brightness > 7) brightness = 7;
	if(brightness < 1) brightness = 1;

	display_1.setBrightness(brightness);
	display_2.setBrightness(brightness);
  }

//Time manipulation

void simple_plus(){

    int wait_time = 250;

	while( digitalRead(button_plus) ){

        
	player_1.time_ms = player_1.time_ms + 3000;

    if(player_1.time_ms > 360000){ // if above 1h add 30 min total
        player_1.time_ms = player_1.time_ms + 150000;
    }
    if(player_1.time_ms > 120000){ // if above 20min add 5 min total
        player_1.time_ms = player_1.time_ms + 24000;  
    }
    if(player_1.time_ms > 60000){ // if above 10min add 1 min total
		player_1.time_ms = player_1.time_ms + 3000;
	}

    
    equalise_normalise();

    refresh_displays();

    delay(wait_time);
    if(wait_time > 100) wait_time= wait_time - 50;


    }
}

void simple_minus(){
	int wait_time = 250;

	while( digitalRead(button_minus) ){

        player_1.time_ms = player_1.time_ms - 3000;
	
        if(player_1.time_ms > 60000){ //if above 10min add 1 min total
            player_1.time_ms = player_1.time_ms - 3000;
        }
        
        if(player_1.time_ms > 120000){ // if above 25min subtract 5 min total
            player_1.time_ms = player_1.time_ms - 24000;	
        }
        
        if(player_1.time_ms > 360000){ // if above 1:30 h subtract 30 min total
            player_1.time_ms = player_1.time_ms - 150000;
            
            if(player_1.time_ms < 360000) {
                player_1.time_ms = player_1.time_ms - 150000;
            }
        }
        if(player_1.time_ms < 3000) {
		    player_1.time_ms = 3000;
	    }

        equalise_normalise();
        refresh_displays();
	    
        delay(wait_time);
        if(wait_time > 100) wait_time= wait_time - 50;


        
        }
    }
	




void equalise_normalise(){
	
	player_2.time_ms = player_1.time_ms;
	player_1.time_ms = (player_1.time_ms / 1000)*1000;
	player_2.time_ms = (player_2.time_ms / 1000)*1000;
}

void plus_minus_one(volatile int& value, TYPE type) { // value and type(h,m,s,turn)
     // TO BE - passing the unit and implement holding
    int bonus = type;
    int wait_time = 300;

    if (plus_flag) {

        do {

            value+=bonus; 

            display_1.showNumberDec(value/type, false);

            delay(wait_time);
            if(wait_time > 100) wait_time= wait_time - 50;

        }while(digitalRead(button_plus));

        plus_flag = false;
    }

    if (minus_flag) {

        do {

            if (value >= bonus) value-=bonus;      // decrement to 0 at most

            display_1.showNumberDec(value/type, false);
            delay(wait_time);
            if(wait_time > 100) wait_time= wait_time - 50;

        }while(digitalRead(button_minus));

        minus_flag = false;
    }

}

void resetFlags(){
	times_loaded_flag = false;
	play_flag   = false;
    plus_flag   = false;
    minus_flag  = false;
    ok_flag     = false;
    bat_flag    = false;
};


//-------------------------------------------------------------------------------------

// Display time

void display_time(int player_time,bool bling, bool which_screen) {
  
  hours = (player_time / 360000)%100;
  mins = ((player_time - hours*360000) / 6000)%100;
  sec = (player_time / 100) % 60; // hours were not subtracted before, so the blink was off above 1 h
  ms = player_time % 100;
  bool blink_state = sec%2;
  uint8_t dots = 0b00000000;
  print_time = 0;

  if(hours>0){ // more than hour
    print_time = hours*100 + mins;
    if(bling){
    dots = blink_state ? 0b11111111 : 0b00000000;
    } else dots = 0b11111111;
    
    if (which_screen){
      
      display_1.showNumberDecEx(print_time, dots,false, 4, 0);
    }else{
      
      display_2.showNumberDecEx(print_time, dots,false, 4, 0);
    } 
           
  }else if(mins>10){ // more than hour
		print_time = mins;
		if(bling){
		dots = blink_state ? 0b11111111 : 0b00000000;
		} else dots = 0b11111111;
    
		if (which_screen){
      display_1.setSegments(&blank, 1, 0);   
			display_1.showNumberDecEx(print_time, dots,true, 3, 1);           
		}else{
      display_2.setSegments(&blank, 1, 0);
			display_2.showNumberDecEx(print_time, dots,true, 3, 1);
     
		} 
           
  }else if(mins > 0){ // more than a min 
		print_time = mins*100 + sec;
		dots = 0b11111111;
    
		if (which_screen){
      
		display_1.showNumberDecEx(print_time, dots,false, 4, 0);
		}else{
      
			display_2.showNumberDecEx(print_time, dots,false, 4, 0);
		}
       
  }else {
		print_time = sec;  // 2-digit seconds
		dots =0b11111111;
		if (which_screen){
      
			display_1.showNumberDecEx(print_time, dots,false, 4, 0);
		}else{
      
			display_2.showNumberDecEx(print_time, dots,false, 4, 0);
		}
  }
}

void refresh_displays(){
	//display_1.clear();
	display_time(player_1.time_ms, false, false); 
	//display_2.clear();            
	display_time(player_2.time_ms, false, true);
}

uint8_t charToMask(char c) {
	
    switch(c) {
        case '0': return 0b00111111;
        case '1': return 0b00000110;
        case '2': return 0b01011011;
        case '3': return 0b01001111;
        case '4': return 0b01100110;
        case '5': return 0b01101101;
        case '6': return 0b01111101;
        case '7': return 0b00000111;
        case '8': return 0b01111111;
        case '9': return 0b01101111;

        case 'd': return 0b01011110;
        case 'y': return 0b01101110;
        case 'Y': return 0b01101110;
        case 'S': return 0b01101101;

        case 'A': return 0b01110111;
        case 'B': return 0b01111100;
        case 'C': return 0b00111001;
        case 'F': return 0b01110001;
        case 'L': return 0b00111000;
        case 'G': return 0b00111101;
		    case 'R': return 0b01010000;
		    case 'E': return 0b01111001;
		    case 'D': return 0b00111111;
		
        case 'i': return 0b00000100;
        case 'n': return 0b01010100;
        case 'c': return 0b01011000;
        case 't': return 0b01111000;
        case 'u': return 0b00011100;
        case 'r': return 0b01010000;
        case 'o': return 0b01011100;

        case '_': return 0b00001000;
		    case '*': return 0b00000001;

        case ' ': return 0b00000000;
		    case 'I': return 0b00000110;  
		    case '-': return 0b01000000;  
        case 'H': return 0b01110110; 
        case 'h': return 0b01110100; 
        case 's': return 0b01101101; 
        case 'b': return 0b01111100; 
        case 'P': return 0b01110011;
    }
    return 0;
}

void showWord(const char *w, bool display) {
    uint8_t segs[4];
    for (int i = 0; i < 4; i++)
        segs[i] = charToMask(w[i]);
	
	if (display){
		display_2.setSegments(segs);
	}else {
		display_1.setSegments(segs);
	}
}

//CheckPlayer -----------------------------------------------------------------------

// 16 reads, sorted, the 3 highest and 3 lowest dropped, the rest averaged.
// Spikes from the supply land at the ends and are thrown away
// Uses the ESP32's built-in Hall sensor (raw values, can be negative, GPIO36 and 39 must stay free).
// For an external sensor (SS49E) on the hall pin, use the commented line instead (see the README)
float read_hall_burst() {
    int samples[HALL_BURST];
    for (int i = 0; i < HALL_BURST; i++) {
        samples[i] = hallRead();
        // samples[i] = analogReadMilliVolts(hall); // external Hall sensor, in mV
    }

    // insertion sort, fast enough for 16 values
    for (int i = 1; i < HALL_BURST; i++) {
        int value = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > value) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = value;
    }

    int32_t sum = 0;
    for (int i = HALL_TRIM; i < HALL_BURST - HALL_TRIM; i++) {
        sum += samples[i];
    }
    return (float)sum / (HALL_BURST - 2 * HALL_TRIM);
}

// One sample: burst, EMA and pos. False while not calibrated (both levels the same)
bool hall_sample() {
    filtered = alpha * read_hall_burst() + (1 - alpha) * filtered;

    float separation = level_p1 - level_p2;
    if (fabsf(separation) < 1) return false;

    pos = (filtered - level_p2) / separation;
    return true;
}

// The lever rests at the start of a game. If it is clearly on one side,
// the drift since the last game is taken out of both levels.
// False if it is on neither side: the calibration doesn't fit any more
bool anchor_levels() {
    float shift = 0;

    if (fabsf(pos) < pos_to_p2) {
        shift = filtered - level_p2;
    } else if (fabsf(1 - pos) < 1 - pos_to_p1) {
        shift = filtered - level_p1;
    } else {
        return false;
    }
    level_p1 += shift;
    level_p2 += shift;
    hall_sample();
    return true;
}

// True if the lever was found on one of the two sides
bool warmupFilter(int samples = 50) {
    filtered = read_hall_burst(); // start from a real value, not from 0
    for (int i = 0; i < samples; i++) {
        hall_sample();
        delay(hall_period_ms);
    }
    bool lever_found = anchor_levels();
    confirm_count = 0;
    return lever_found;
}

// Runs in the task every 2 ms. True when the lever moved to the other side
bool check_player(){
    if (!hall_sample()) return false;

    // Drift tracking: while the lever rests, its level follows slowly.
    // The other level moves by the same amount, drift of the SS49E is mostly a common offset
    float rest_level = player_flag ? level_p1 : level_p2;
    float rest_pos = player_flag ? 1 : 0;
    if (fabsf(pos - rest_pos) < rest_band) {
        float shift = drift_beta * (filtered - rest_level);
        level_p1 += shift;
        level_p2 += shift;
    }

    // Lever bounce right after a switch must not count as a second move
    if (millis() - last_switch < lockout_ms) {
        confirm_count = 0;
        return false;
    }

    bool other_side = player_flag ? (pos < pos_to_p2) : (pos > pos_to_p1);
    confirm_count = other_side ? confirm_count + 1 : 0;

    if (confirm_count >= confirm_samples) {
        confirm_count = 0;
        last_switch = millis();
        player_flag = !player_flag;
        return true;
    }
    return false;
}


//Player Change --------------------------------------------------------------------

int turn_increment(chess_player* player) {
    return (player->current_turn < bonus_turn) ? player->increment_start : player->increment_1;
}

// Value first, then the count, so loop() never sees a count with an empty slot
void record_move(int time_after_move) {
    if (move_count < MAX_HALF_MOVES) {
        half_moves[move_count] = time_after_move;
        move_count = move_count + 1;
    }
}

void player_change(){
    chess_player* mover = current_player;
    chess_player* next;

    if (player_flag == false && mover == &player_2) {
        next = &player_1;
    } else if (player_flag == true && mover == &player_1) {
        next = &player_2;
    } else {
        return;
    }

    // One step under the lock, so the timer ISR never sees half a switch
    portENTER_CRITICAL(&time_mux);
    increment = turn_increment(mover);

    if (mover->current_turn == bonus_turn) {
        mover->time_ms = mover->time_ms + mover->bonus_1;
    }
    if (time_mode == FISCHER) {
        mover->time_ms = mover->time_ms + increment;
    }
    mover->current_turn++;
    current_player = next;
    delay_left = (time_mode == DELAY) ? turn_increment(next) : 0;
    int time_after_move = mover->time_ms;
    portEXIT_CRITICAL(&time_mux);

    record_move(time_after_move);
    switch_count = switch_count + 1; // loop() redraws both displays
}

// Calibration sample: same filter as in a game, plus min / max / sum for measure_side()
void measure_step() {
    hall_sample();
    float value = filtered;
    if (value < measure_min) measure_min = value;
    if (value > measure_max) measure_max = value;
    measure_sum = measure_sum + value;
    measure_count = measure_count + 1;
}

void hall_task(void* parameter) {
    for (;;) {
        if (hall_mode == HALL_GAME) {
            if (check_player()) player_change();
        } else if (hall_mode == HALL_MEASURE) {
            measure_step();
        }
        vTaskDelay(pdMS_TO_TICKS(hall_period_ms));
    }
}

// Handshake: after this the task is asleep in IDLE and loop() owns the filter, levels and moves.
// 5 ms is more than one task step (about 1 ms burst + 2 ms sleep)
void stop_hall() {
    hall_mode = HALL_IDLE;
    delay(5);
}


// Calibration ------------------------------------------------------------------------

// One side for 20 s: average of the filtered value, and the noise as peak to peak
void measure_side(float& level, float& noise) {
    const int warmupSamples = 50;
    const unsigned long phaseMillis = 20000;
    filtered = read_hall_burst();
    for (int i = 0; i < warmupSamples; i++) {
        filtered = alpha * read_hall_burst() + (1 - alpha) * filtered;
        delay(hall_period_ms);
    }

    // The task collects, loop() only waits
    measure_min = 1e6f;
    measure_max = -1e6f;
    measure_sum = 0;
    measure_count = 0;
    hall_mode = HALL_MEASURE;
    delay(phaseMillis);
    stop_hall();

    if (measure_count == 0) return; // task didn't run, keep the old level
    level = measure_sum / measure_count;
    noise = measure_max - measure_min;
}

void calibrate() {
    float noise_p1, noise_p2;

    // 8888 on player 1's display: press player 1's side
    display_1.clear(); display_2.clear();
    showWord("8888", true);
    delay(5000);
    measure_side(level_p1, noise_p1);

    // 8888 on player 2's display: press player 2's side
    display_1.clear(); display_2.clear();
    showWord("8888", false);
    delay(5000);
    measure_side(level_p2, noise_p2);

    float noise = noise_p1 > noise_p2 ? noise_p1 : noise_p2;
    if (noise < 1) noise = 1; // a perfectly quiet reading would divide by 0
    snr = fabsf(level_p1 - level_p2) / noise;

    // Only for information: separation / peak-to-peak noise, the higher the better
    display_1.clear(); display_2.clear();
    showWord("Snr ", true);
    display_1.showNumberDec(snr > 9999 ? 9999 : (int)snr, false); // 4 digits
    delay(3000);
}

//Saving player times ------------------------------------------


void save_player(const char* key, chess_player& player) {
    chess_player stored;
    size_t n = prefs.getBytes(key, &stored, sizeof(stored));

    // Compare current struct with stored struct
    if (n != sizeof(stored) || memcmp(&player, &stored, sizeof(player)) != 0) {
        prefs.putBytes(key, &player, sizeof(player));
		    //Serial.print("Values Saved");
    } 
}

void save_int_if_changed(const char* key, int value) {
    if (prefs.getInt(key, INT_MIN) != value) {
        prefs.putInt(key, value);
    }
}

void save_float_if_changed(const char* key, float value) {
    float old = prefs.getFloat(key, NAN);
    if (isnan(old) || fabsf(old - value) > 0.0001f) {
        prefs.putFloat(key, value);
    }
}

// bonus_turn and time_mode are not in the player struct, so they were lost on every restart
void save_settings_if_changed(){
	save_player("player_1", player_1);
	save_player("player_2", player_2);
	save_int_if_changed("bonus_turn", bonus_turn);
	save_int_if_changed("time_mode", time_mode);
}

//Saving games ------------------------------------------------------------------------

// The settings of this game, kept for the app: <mode F/D> <bonus_turn> <white 1/2>
// then per player <time> <increment_start> <bonus_1> <increment_1>, times in 10 ms ticks
void start_game_record() {
    move_count = 0;
    snprintf(game_settings, sizeof(game_settings), "%c %d %d %d %d %d %d %d %d %d %d",
             time_mode == DELAY ? 'D' : 'F', bonus_turn, current_player == &player_1 ? 1 : 2,
             player_1.time_ms, player_1.increment_start, player_1.bonus_1, player_1.increment_1,
             player_2.time_ms, player_2.increment_start, player_2.bonus_1, player_2.increment_1);
}

// "/g/12" -> 12, the file name is the game id
const char* file_name(File& file) {
    const char* name = file.name();
    const char* slash = strrchr(name, '/');
    return slash ? slash + 1 : name;
}

// /g/<id>.tc are the settings next to a game, only /g/<id> counts
int count_games() {
    int count = 0;
    File dir = LittleFS.open("/g");
    if (!dir) return 0;

    File file = dir.openNextFile();
    while (file) {
        if (strchr(file_name(file), '.') == NULL) count++;
        file = dir.openNextFile();
    }
    return count;
}

// One write per game, LittleFS spreads the writes over the flash
void save_game() {
    if (move_count == 0) return;

    if (count_games() >= MAX_GAMES) {
        // Full: the old games are kept, nothing is overwritten. Sync to make room
        display_1.clear(); display_2.clear();
        showWord("FuLL", true);
        showWord("FuLL", false);
        delay(2000);
        move_count = 0;
        return;
    }

    // The counter is saved first, so an id is never used twice even if the write below fails
    int id = prefs.getInt("next_id", 1);
    prefs.putInt("next_id", id + 1);

    char path[16];
    snprintf(path, sizeof(path), "/g/%d", id);
    File file = LittleFS.open(path, FILE_WRITE);
    if (file) {
        for (int i = 0; i < move_count; i++) {
            if (i > 0) file.print(' ');
            file.print(half_moves[i]);
        }
        file.close();
    }

    snprintf(path, sizeof(path), "/g/%d.tc", id);
    file = LittleFS.open(path, FILE_WRITE);
    if (file) {
        file.print(game_settings);
        file.close();
    }

    move_count = 0;
}

// Saves a running game first, so turning the clock off doesn't lose it
void power_off() {
    stop_hall();
    save_game();
    digitalWrite(power, LOW);
}

//Sync with the app (protocol: see bt_pairing_test.ino) -----------------------------

uint32_t file_crc(File& file) {
    uint8_t buffer[64];
    uint32_t crc = 0;

    file.seek(0);
    while (file.available()) {
        int n = file.read(buffer, sizeof(buffer));
        crc = esp_rom_crc32_le(crc, buffer, n);
    }
    file.seek(0);
    return crc;
}

// GAME/DATA <id> <length> <crc32>
void send_header(const char* type, int id, File& file) {
    BT.printf("%s %d %u %08lX\n", type, id, (unsigned)file.size(), (unsigned long)file_crc(file));
}

void handle_command(const char* command) {
  char path[16];

  if (strcmp(command, "HELLO") == 0) {
    BT.printf("HELLO %d %d\n", PROTOCOL_VERSION, count_games());

  } else if (strcmp(command, "LIST") == 0) {
    File dir = LittleFS.open("/g");
    File file = dir ? dir.openNextFile() : File();
    while (file) {
      const char* name = file_name(file);
      if (strchr(name, '.') == NULL) send_header("GAME", atoi(name), file);
      file = dir.openNextFile();
    }
    BT.print("END\n");

  } else if (strncmp(command, "GET ", 4) == 0) {
    int id = atoi(command + 4);
    snprintf(path, sizeof(path), "/g/%d", id);

    if (!LittleFS.exists(path)) {
      BT.printf("ERR NO_GAME %d\n", id);
      return;
    }
    File file = LittleFS.open(path, FILE_READ);
    send_header("DATA", id, file);

    uint8_t buffer[64];
    while (file.available()) {
      int n = file.read(buffer, sizeof(buffer));
      BT.write(buffer, n);
    }
    file.close();
    BT.printf("\nEND %d\n", id);

  } else if (strncmp(command, "DEL ", 4) == 0) {
    int id = atoi(command + 4);

    // Ids are never reused, so an id below next_id existed once: OK even if it is already deleted
    if (id < 1 || id >= prefs.getInt("next_id", 1)) {
      BT.printf("ERR NO_GAME %d\n", id);
      return;
    }
    snprintf(path, sizeof(path), "/g/%d", id);
    if (LittleFS.exists(path)) LittleFS.remove(path);
    snprintf(path, sizeof(path), "/g/%d.tc", id);
    if (LittleFS.exists(path)) LittleFS.remove(path);
    BT.printf("OK %d\n", id);

  } else {
    BT.print("ERR UNKNOWN\n");
  }
}

void read_commands() {
  while (BT.available()) {
    char c = BT.read();

    if (c == '\r') continue;

    if (c == '\n') {
      line[line_length] = '\0';

      if (line_too_long) {
        BT.print("ERR TOO_LONG\n");
      } else if (line_length > 0) {
        handle_command(line);
      }
      line_length = 0;
      line_too_long = false;

    } else if (line_length < (int)sizeof(line) - 1) {
      line[line_length++] = c;
    } else {
      line_too_long = true;   // ignore the rest until \n
    }
  }
}

// ------------------------------------------------------------------------------------

//State handlers

void ready_handler() {
	
	
	if (!times_loaded_flag) { // Loads players at start and after gameend to Ready transition
		prefs.getBytes("player_1", &player_1, sizeof(player_1));
		prefs.getBytes("player_2", &player_2, sizeof(player_2));
		times_loaded_flag = true;
	}
	
	unsigned long now = millis();
	if (now - last_screen_update > 20) {
		// Display both player times       
		refresh_displays();
		last_screen_update = now;
	}

	// Handle flags
	if (play_flag) {              				
		save_settings_if_changed(); // Saves times when starting the game if they are changed				
		state = RUNNING; // move to running state

        refresh_displays();

        // CALI only when it is needed: never calibrated, or the lever is on neither side,
        // so the levels don't fit any more (sensor or magnet moved, too much drift)
        bool lever_found = warmupFilter();
        if (fabsf(level_p1 - level_p2) < 1 || !lever_found) {
            showWord("CALI", true);
            showWord("CALI", false);
            delay(1500);
            refresh_displays();
        }
        player_flag = pos > 0.5; // the side pressed now decides whose clock runs
        current_player = (player_flag) ? &player_2 : &player_1;

        start_game_record();
        delay_left = (time_mode == DELAY) ? turn_increment(current_player) : 0;
        last_switch = millis();
        last_drawn_player = NULL; // redraw on the next pass
        last_drawn_switch = switch_count;

        hall_mode = HALL_GAME;    // from here the task owns the filter, levels and moves
        timerAlarmEnable(Timer0_Cfg);
		play_flag = false;
	}
	
	if (plus_flag) {
		simple_plus();
		plus_flag = false;               
	}
	
	if (minus_flag) {
		simple_minus();
		minus_flag = false;
	}
	
	//Displays battery while the button is pressed, allows changing brightness with + - while holding bat and entering calibration with ok
	if (bat_flag) {
		read_battery(); 
		bat_flag = false;
	}

	if (onoff_flag) {
		if(long_press(button_onoff, 1500, LOW)){
			power_off();	
			//Serial.println("Power OFF");
		}
		
		onoff_flag = false;
	   
	}

	if (ok_flag) {
		ok_flag = false;
		state = SMALL_SETUP;
		step_flag = STEP_1;
	}
	
}
void running_handler() {
    delay(1);

	// One copy under the lock, so both times and the running player belong together
	portENTER_CRITICAL(&time_mux);
	int time_1 = player_1.time_ms;
	int time_2 = player_2.time_ms;
	chess_player* running = current_player;
	portEXIT_CRITICAL(&time_mux);
	int time_left = (running == &player_1) ? time_1 : time_2;
	uint32_t switches = switch_count;

	// The task switched: draw the final time of the player who just moved
	if (switches != last_drawn_switch) {
		if (running == &player_1) {
			display_time(time_2, false, true);
		} else {
			display_time(time_1, false, false);
		}
		last_drawn_switch = switches;
	}

	// Display current player, only when the shown second changes.
	// TM1637 writes are slow and every write is a current spike on the supply
	if (time_left / 100 != last_drawn_time / 100 || running != last_drawn_player) {
		display_time(time_left, true, running == &player_2);
		last_drawn_time = time_left;
		last_drawn_player = running;
	}

	// Check for timeout
	if (time_left < 1) {
        timerAlarmDisable(Timer0_Cfg);
        stop_hall();
        save_game();
		state = GAME_ENDED;
	}
	
	//Displays battery while the button is pressed, allows changing brightness with + - while holding bat
	if (bat_flag) {
		read_battery();
		refresh_displays();
		bat_flag = false;
	}
	
	// Pause game
	if (play_flag) {  
		play_flag = false;
        timerAlarmDisable(Timer0_Cfg);
        stop_hall();
		refresh_displays();
		state = PAUSED; 
	}
    if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			power_off();			
		}
	}
}
void small_setup_handler() {
	
	if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			power_off();			
		}
	}
	
	switch (step_flag) {

		case STEP_1: {
      delay(20);
			// Check for long press to enter SETUP
		  if (long_press(button_ok, 1000, HIGH)) {
         display_1.clear();
         display_2.clear();
         while(digitalRead(button_ok)){
          delay(1);
         }
				 state = SETUP;
			} else {
				step_flag = STEP_2;
			}
			break;
		}

		case STEP_2: {
			// Edit increment_start
			now = millis();
			if (now - last_screen_update > 20) {

				showWord("inc ", true);
				sec = player_1.increment_start / 100;
				display_1.showNumberDecEx(sec, 0b00000000, false, 4, 0);

				last_screen_update = now;
			}

			plus_minus_one(player_1.increment_start, s);

			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_3;
                player_1.increment_1 = player_1.increment_start;
			}
			break;
		}

		case STEP_3: {
			// Edit bonus_1
			now = millis();
			if (now - last_screen_update > 20) {

				showWord("Bon ", true);

				mins = player_1.bonus_1 / 6000;			

                
				display_1.showNumberDecEx(mins, 0b11111111,true, 3, 1);

				last_screen_update = now;
			}

			plus_minus_one(player_1.bonus_1,m);

			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_4;
			}
			break;
		}

		case STEP_4: {
			// Edit increment_1
			now = millis();
			if (now - last_screen_update > 20) {

				showWord("inc2", true);
				sec = player_1.increment_1 / 100;
				display_1.showNumberDecEx(sec, 0b00000000, false, 4, 0);

				last_screen_update = now;
			}

			plus_minus_one(player_1.increment_1, s);
			
			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_5;
			}
			break;
		}

		case STEP_5: {
			// Edit bonus_turn
			now = millis();
			if (now - last_screen_update > 20) {

				showWord("turn", true);
				display_1.showNumberDecEx(bonus_turn, 0b00000000, false, 4, 0);

				last_screen_update = now;
			}
			plus_minus_one(bonus_turn,turn);

			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_6;
			}
			break;
		}

		case STEP_6: {
			time_mode_step();

			if (ok_flag) {
				ok_flag = false;
                player_2 = player_1;
				save_settings_if_changed(); // Saves settings if they are changed
				state = READY;
			}
			break;
		}
	}

}

// Fischer or delay, for both players. + or - switches between them
void time_mode_step() {
	now = millis();
	if (now - last_screen_update > 20) {
		showWord("tYPE", true);
		showWord(time_mode == DELAY ? "dELy" : "inc ", false);
		last_screen_update = now;
	}

	if (plus_flag || minus_flag) {
		plus_flag = false;
		minus_flag = false;
		time_mode = (time_mode == DELAY) ? FISCHER : DELAY;
	}
}

void setup_handler() {
	
	
	if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			power_off();			
		}
	}
	if (long_press(button_ok, 1200, HIGH)) {
    
    if( step_flag < 9){
      player_2 = player_1;
    }

    display_1.clear();
    display_2.clear();
    step_flag = STEP_19;
	}
	
    switch (step_flag) {

        case STEP_1: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1_h", true);

                hours = player_1.time_ms / 360000;
                display_1.showNumberDec(hours, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.time_ms ,h);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_2;
            }
            break;
        }

        case STEP_2: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1nn", true);

                mins = (player_1.time_ms % 360000) / 6000;
                display_1.showNumberDec(mins, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.time_ms,m);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_3;
            }
            break;
        }

        case STEP_3: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1_s", true);

                sec = (player_1.time_ms % 6000) / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.time_ms,s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_4;
            }
            break;
        }

        case STEP_4: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1in", true);

                sec = player_1.increment_start / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.increment_start, s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_5;
            }
            break;
        }

        case STEP_5: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1bh", true);

                hours = player_1.bonus_1 / 360000;
                display_1.showNumberDec(hours, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.bonus_1,h);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_6;
            }
            break;
        }

        case STEP_6: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1bn", true);

                mins = (player_1.bonus_1 % 360000) / 6000;
                display_1.showNumberDec(mins, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.bonus_1,m);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_7;
            }
            break;
        }

        case STEP_7: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1bs", true);

                sec = (player_1.bonus_1 % 6000) / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.bonus_1,s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_8;
            }
            break;
        }

        case STEP_8: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P1bi", true);

                sec = player_1.increment_1 / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_1.increment_1, s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_9;
            }
            break;
        }

        case STEP_9: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("turn", true);
                display_1.showNumberDec(bonus_turn, false);

                last_screen_update = now;
            }

            plus_minus_one(bonus_turn,turn);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_10;
				player_2 = player_1;
            }
            break;
        }

        case STEP_10: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2_h", true);

                hours = player_2.time_ms / 360000;
                display_1.showNumberDec(hours, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.time_ms,h);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_11;
            }
            break;
        }

        case STEP_11: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2nn", true);

                mins = (player_2.time_ms % 360000) / 6000;
                display_1.showNumberDec(mins, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.time_ms, m);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_12;
            }
            break;
        }

        case STEP_12: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2_s", true);

                sec = (player_2.time_ms % 6000) / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.time_ms, s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_13;
            }
            break;
        }

        case STEP_13: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2in", true);

                sec = player_2.increment_start / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.increment_start, s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_14;
            }
            break;
        }


        case STEP_14: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2bh", true);

                hours = player_2.bonus_1 / 360000;
                display_1.showNumberDec(hours, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.bonus_1,h);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_15;
            }
            break;
        }

        case STEP_15: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2bn", true);

                mins = (player_2.bonus_1 % 360000) / 6000;
                display_1.showNumberDec(mins, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.bonus_1, m);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_16;
            }
            break;
        }


        case STEP_16: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2bs", true);

                sec = (player_2.bonus_1 % 6000) / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.bonus_1, s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_17;
            }
            break;
        }

        case STEP_17: {
            now = millis();
            if (now - last_screen_update > 20) {

                showWord("P2bi", true);

                sec = player_2.increment_1 / 100;
                display_1.showNumberDec(sec, false);

                last_screen_update = now;
            }

            plus_minus_one(player_2.increment_1, s);

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_18;
            }
            break;
        }
        case STEP_18: {
            time_mode_step();

            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_19;
            }
            break;
        }
        case STEP_19: { // exit
            save_settings_if_changed();
            state = READY;
        }
    }
}

	
	

void paused_handler() {
	
	
	if (play_flag) { // Continue Game
        warmupFilter();
        last_drawn_player = NULL; // redraw on the next pass
        last_drawn_switch = switch_count;
        hall_mode = HALL_GAME;
        timerAlarmEnable(Timer0_Cfg);
		play_flag = false;
		state = RUNNING;
	}
	if (ok_flag) { // game is stopped, keep what was played
		ok_flag = false;
		save_game();
		state = READY;
	}
	if (bat_flag) {
		read_battery();
    refresh_displays(); 
		bat_flag = false;
	}
	if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			power_off();			
		}
	}
}
void game_end_handler() {
	
	showWord("FLAG", !player_flag);
	// Optionally display end state
	
	if (play_flag) { // reset
		play_flag = false;
		state = READY;
	}
	if (bat_flag) {
		read_battery(); 
		bat_flag = false;
	}
	if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			power_off();			
		}
	}
}
void calibration_handler(){
	
	switch (step_flag) {
		case STEP_1:{
      
			display_1.clear();
			showWord("CALI", true);
			
			if (play_flag){
				play_flag = false;
				calibrate ();
        play_flag = false;
			}
			
			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_2;
				display_1.clear();
				display_2.clear();
			}
      delay(100);
      break;
		}
		case STEP_2:{
			// Live position: about 0 with player 2's side pressed, about 100 with player 1's
			showWord("READ", true);
			hall_mode = HALL_MEASURE; // the task samples, loop() only shows pos
			display_1.showNumberDec((int)(pos * 100), false);
			
			if (ok_flag) {
				ok_flag = false;
				stop_hall();
				step_flag = STEP_3;
				display_1.clear();
				display_2.clear();
			}
			delay(100);
			break;
		}
		
		case STEP_3:{
			// Nudge the level of player 1's side, in mV
			display_1.showNumberDecEx((int)level_p1, 0b00000000, false, 4, 0);
			showWord("P1  ", true);
			
			if(plus_flag){
				plus_flag = false;
				level_p1 = level_p1 + 5.0;
			}
			if(minus_flag){
				minus_flag = false;
				level_p1 = level_p1 - 5.0;
			}
			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_4;
				display_1.clear();
				display_2.clear();
			}
			delay(100);
      break;
		}
		
		case STEP_4:{
			// Nudge the level of player 2's side, in mV
			display_1.showNumberDecEx((int)level_p2, 0b00000000, false, 4, 0);
			showWord("P2  ", true);
			
			if(plus_flag){
				plus_flag = false;
				level_p2 = level_p2 + 5.0;
			}
			if(minus_flag){
				minus_flag = false;
				level_p2 = level_p2 - 5.0;
			}
            if (ok_flag) {
                ok_flag = false;
                step_flag = STEP_5;
                display_1.clear();
                display_2.clear();
            }
			
			delay(100);
      break;
		}	
    case STEP_5:{
        display_1.showNumberDecEx((int)roundf(alpha*100), 0b00000000, false, 4, 0);
        showWord("ALPH", true);
        if (ok_flag) {
            ok_flag = false;
            step_flag = STEP_6;
            display_1.clear();
            display_2.clear();
        }
        if(plus_flag){
            plus_flag = false;
            alpha += 0.01;
        }
        if(minus_flag){
            minus_flag = false;
            alpha-= 0.01;
        }
        // Under 0.05 the filter barely follows the lever, over 1 it is not a filter any more
        if (alpha < 0.05) alpha = 0.05;
        if (alpha > 1.00) alpha = 1.00;

        break;
    }
    case STEP_6:{
      // Swap the sides, if the wrong side was pressed during calibration
      display_1.showNumberDecEx((int)level_p1, 0b00000000, false, 4, 0);
      showWord("5AP ", true);

      if(play_flag){
        float holder = level_p1;
        level_p1 = level_p2;
        level_p2 = holder;
        play_flag = false;
        
      }
      
      if (ok_flag) {
          ok_flag = false;
          save_float_if_changed("level_p1", level_p1);
          save_float_if_changed("level_p2", level_p2);
          save_float_if_changed("alpha", alpha);
          save_float_if_changed("snr", snr);
          state = READY;
      }
      
      break;
    }
    
	}
}

// Bluetooth is only on in this state, so it adds no noise during a game
void sync_handler() {
	if (!bt_on) {
		BT.begin(bt_name);
		bt_on = true;
		line_length = 0;
		line_too_long = false;
		display_1.clear();
		display_2.clear();
	}

	// A half received command is thrown away when the app disconnects
	if (!BT.hasClient()) {
		line_length = 0;
		line_too_long = false;
	}
	read_commands();

	now = millis();
	if (now - last_screen_update > 500) {
		showWord("SYnc", true);
		display_1.showNumberDec(count_games(), false);
		last_screen_update = now;
	}

	if (ok_flag || play_flag) { // leave sync
		ok_flag = false;
		play_flag = false;
		BT.end();
		bt_on = false;
		state = READY;
	}
	if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			power_off();
		}
	}
	delay(10);
}



void setup() 
{

//test
  //Serial.begin(115200);
  
  pinMode(power, OUTPUT);
  digitalWrite(power, HIGH);
  
  //Display Setup
  display_1.clear();
  display_1.setBrightness(brightness);
  display_2.clear();
  display_2.setBrightness(brightness);

  //GPIO SetUp
  pinMode(button_plus, INPUT);
  pinMode(button_minus, INPUT);
  pinMode(button_play, INPUT);
  pinMode(button_ok, INPUT);
  pinMode(button_bat, INPUT);
  pinMode(button_onoff, INPUT);
  
  //Battery setup
  pinMode(battery, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(battery,ADC_6db);

  //Hall setup
  pinMode(hall, INPUT);
  analogSetPinAttenuation(hall, ADC_11db);

  setup_buttons();  // attaches interrupts
  enable_button_interrupts();
  //Setup of save
  prefs.begin("chess", false);
  
  level_p1   = prefs.getFloat("level_p1", 0);
  level_p2   = prefs.getFloat("level_p2", 0);
  alpha      = prefs.getFloat("alpha", alpha);
  snr        = prefs.getFloat("snr", 0);
  bonus_turn = prefs.getInt("bonus_turn", bonus_turn);
  time_mode  = prefs.getInt("time_mode", FISCHER);

  // Games: /g/<id> and /g/<id>.tc. Formats the partition the first time
  LittleFS.begin(true);
  if (!LittleFS.exists("/g")) LittleFS.mkdir("/g");

  // Last 2 bytes of the MAC, so two clocks don't show up with the same name
  uint64_t mac = ESP.getEfuseMac();
  snprintf(bt_name, sizeof(bt_name), "ChessClock-%02X%02X",
           (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
  	
  //Frequency
  setCpuFrequencyMhz(80);

  //Time Interupt every 10ms
  
   Timer0_Cfg = timerBegin(0, 80, true);
   timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
   timerAlarmWrite(Timer0_Cfg, 10000, true); 
   //timerAlarmEnable(Timer0_Cfg);

   // Priority 2 is above loop() (1), so the task runs on time even while loop() is busy
   xTaskCreatePinnedToCore(hall_task, "hall", 4096, NULL, 2, NULL, 1);

   delay(1000);
}


void loop() {
	
	now = millis();
	
	if (state != last_state) { // Resets Flags upon State change
    resetFlags();       
    last_state = state; 
	}
	
    switch (state) {

        case READY: {
			ready_handler();           			
            break;
        }
		case SMALL_SETUP: {
			small_setup_handler();
			break;
		}
		
		case SETUP: {
			setup_handler();
			break;
		}
		case CALIBRATION: {
			calibration_handler();
			break;
		}
        case RUNNING: {
			running_handler();           			
            break;
        }
		case PAUSED: {
			paused_handler();
			break;
			}
		
        case GAME_ENDED: {
            game_end_handler();
            break;
        }
		case SYNC: {
			sync_handler();
			break;
		}
		
    } // switch
} // loop
