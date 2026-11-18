#include <TM1637Display.h>
#include <Preferences.h>
#include "BluetoothSerial.h"

BluetoothSerial BT;
Preferences prefs;
//-------------  Tuning  ------------

const float alpha = 0.3;
float threshhold = 0;
float filtered = threshhold;
float small_mid = threshhold;
float big_mid = threshhold;

//------------------------------------------------------------------------------------
//Variables - time_ms 10ms

struct chess_player {
  volatile int time_ms = 30000;
  int increment_start = 300;
  
  int bonus_1 = 0;
  int increment_1 = 0; // not implemented yet have to extend player change
  
  int turn = 1;
};

enum game_state {
	READY,
	SETUP,
	SMALL_SETUP,
	RUNNING,
	PAUSED,
	CALIBRATION,
	GAME_ENDED
};

enum setup_steps {
    STEP_1, STEP_2, STEP_3, STEP_4, STEP_5,
    STEP_6, STEP_7, STEP_8, STEP_9, STEP_10,
    STEP_11, STEP_12, STEP_13, STEP_14, STEP_15,
    STEP_16, STEP_17, STEP_18, STEP_19, STEP_20
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
chess_player player_1;
chess_player player_2;
bool player_flag = true; // 0 - player_1    1 - player_2
chess_player* current_player = &player_2;

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
unsigned long last_plus = 0;
unsigned long last_minus = 0;
unsigned long last_play = 0;
unsigned long last_bat = 0;
unsigned long last_ok = 0;
unsigned long last_onoff = 0;
const unsigned long debounce_ms = 150;

//Screen refresh
unsigned long last_screen_update = 0;
const unsigned long screen_update = 20;

int8_t brightness = 1;

//----------------------------------------------------------------------------

// ISR functions

// Time Interupt 10ms or 1 ms

hw_timer_t *Timer0_Cfg = NULL;
void IRAM_ATTR Timer0_ISR()
{
    (current_player->time_ms)--; //Decrements the time reserv of the current player
	  update_flag = true;
}

//Interupts for buttons

void IRAM_ATTR plus_ISR() {
    unsigned long now = millis();
    if (now - last_plus > debounce_ms) {
        plus_flag = true;
        last_plus = now;
    }
}

void IRAM_ATTR minus_ISR() {
    unsigned long now = millis();
    if (now - last_minus > debounce_ms) {
        minus_flag = true;
        last_minus = now;
    }
}

void IRAM_ATTR play_ISR() {
    unsigned long now = millis();
    if (now - last_play > debounce_ms) {
        play_flag = true;
        last_play = now;
    }
}

void IRAM_ATTR bat_ISR() {
    unsigned long now = millis();
    if (now - last_bat > debounce_ms) {
        bat_flag = true;
        last_bat = now;
    }
}

void IRAM_ATTR ok_ISR() {
    unsigned long now = millis();
    if (now - last_ok > (debounce_ms+100) ) {
        ok_flag = true;
        last_ok = now;
    }
}

void IRAM_ATTR onoff_ISR() {
    unsigned long now = millis();
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
    // Button held for full 200ms
    return true;
}

//Reading Battery

void read_battery() {
  
    const int ADC_MIN = 2420;  // = 1.3 V
    const int ADC_MAX = 3910;  // = 2.1 V
    bool button_held = true;
    
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

      
      display_2.clear();
      display_1.showNumberDec(percent, false);
      
      delay(10);

      set_brightness();
	  if(state == READY){
			if (ok_flag) {
				ok_flag = false;
				state = CALIBRATION;
				step_flag = STEP_1;
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
	
}

void simple_minus(){
	
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
}

void equalise_normalise(){
	
	player_2.time_ms = player_1.time_ms;
	player_1.time_ms = (player_1.time_ms / 1000)*1000;
	player_2.time_ms = (player_2.time_ms / 1000)*1000;
}

void plus_minus_one(int* value) {

    if (plus_flag) {
        (*value)=(*value)+100;   // upper bound
        plus_flag = false;
    }

    if (minus_flag) {
        if (*value >= 100) (*value)=(*value)-100;;      // lower bound
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
  sec = ((player_time - mins*6000)/100)%100;
  ms = (player_time - sec*100)%100;
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

void warmupFilter(int samples = 50) {
    for (int i = 0; i < samples; i++) {
        float r = hallRead();
        filtered = alpha * r + (1 - alpha) * filtered;
        delay(1);     // or whatever your sampling delay is
    }
}

void check_player(){
	float reading = hallRead();
	filtered = alpha * reading + (1 - alpha) * filtered;
	
	if( !player_flag ){
		  if(filtered > threshhold){
			player_flag = true;
			threshhold = small_mid;
		  }
	} else {
		  if(filtered < threshhold){
			player_flag = false;
			threshhold = big_mid;
		  }     
	}
	
	//Serial.println(filtered);
}


//Player Change --------------------------------------------------------------------

void player_change(){
				
	if( player_flag == false && current_player == &player_2){
		
		increment = (current_player->turn < bonus_turn) ? current_player->increment_start : current_player->increment_1;
		
		if(current_player->turn == bonus_turn) {
		  current_player->time_ms = current_player->time_ms + current_player->bonus_1;
		}
		
		current_player->time_ms = current_player->time_ms + increment;
		display_time(current_player->time_ms, false, true);
		current_player->turn++;
		current_player = &player_1;	
		
	}else if( player_flag == true && current_player == &player_1){
		increment = (current_player->turn < bonus_turn) ? current_player->increment_start : current_player->increment_1;
		
		if(current_player->turn == bonus_turn) {
		  current_player->time_ms = current_player->time_ms + current_player->bonus_1;
		}
		
		current_player->time_ms = current_player->time_ms + increment;
		display_time(current_player->time_ms, false, false);
		current_player->turn++;
		current_player = &player_2;	
	}
}


void calibrate() {
    const int warmupSamples = 50;
    const unsigned long phaseMillis = 20000;
    float min_a = 1e6f, max_a = -1e6f;  // Max values in the opposite direction
    float min_b = 1e6f, max_b = -1e6f;
    unsigned long start;
    int offset = 2;

    // show indicator, wait
    display_1.clear(); display_2.clear();
    showWord("8888", true);
    delay(5000);

    // PHASE A: collect samples without changing state
    start = millis();
    // warm up filter (do not record)
    for (int i=0; i<warmupSamples && millis()-start < phaseMillis; ++i) {
        float r = hallRead();
        filtered = alpha * r + (1 - alpha) * filtered;
        delay(2);
    }
    while (millis() - start < phaseMillis) {
        float r = hallRead();
        filtered = alpha * r + (1 - alpha) * filtered;
        min_a = filtered < min_a ? filtered : min_a;
        max_a = filtered > max_a ? filtered : max_a;
        delay(2);
    }
    display_1.clear(); display_2.clear();
    showWord("8888", false);
    delay(5000);

    // PHASE B
    start = millis();
    // warm up filter
    for (int i=0; i<warmupSamples && millis()-start < phaseMillis; ++i) {
        float r = hallRead();
        filtered = alpha * r + (1 - alpha) * filtered;
        delay(1);
    }
    while (millis() - start < phaseMillis) {
        float r = hallRead();
        filtered = alpha * r + (1 - alpha) * filtered;
        min_b = filtered < min_b ? filtered : min_b;
        max_b = filtered > max_b ? filtered : max_b;
        delay(1);
    }

    // compute mids properly
    float mid_a = min_a + (max_a - min_a) / 2.0f;
    float mid_b = min_b + (max_b - min_b) / 2.0f;
    if (max_a > max_b) {
        //small_mid = mid_b;
        //big_mid   = mid_a;
        big_mid = min_a + offset;
        small_mid = max_b - offset;
    } else {
        //small_mid = mid_a;
        //big_mid   = mid_b;
        big_mid = max_b - offset;
        small_mid = min_a + offset;
    }
    threshhold = big_mid;

    /*
    Serial.print("min_a "); Serial.println(min_a);
    Serial.print("max_a "); Serial.println(max_a);
    Serial.print("min_b "); Serial.println(min_b);
    Serial.print("max_b "); Serial.println(max_b);
    Serial.print("MID_A "); Serial.println(mid_a);
    Serial.print("MID_B "); Serial.println(mid_b);
    Serial.print("small_mid "); Serial.println(small_mid);
    Serial.print("big_mid "); Serial.println(big_mid);
    */
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

void save_players_if_changed(){
	save_player("player_1", player_1);
	save_player("player_2", player_2);
}

// ------------------------------------------------------------------------------------

//State handlers

void ready_handler() {
	enable_button_interrupts();
	
	if (!times_loaded_flag) { // Loads players at start and after gameend-> Ready
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
		save_players_if_changed(); // Saves times when starting the game if they are changed				
		state = RUNNING; // move to running state
		refresh_displays();
    for ( int i = 1; i <100; i++){
      check_player();
    }
		play_flag = false;
	}
	
	if (plus_flag) {
		equalise_normalise();
		simple_plus();
		plus_flag = false;               
	}
	
	if (minus_flag) {
		equalise_normalise();
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
			digitalWrite(power, LOW);	
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
	
	timerAlarmEnable(Timer0_Cfg);
	check_player();

	if (update_flag) {
		player_change();
		update_flag = false;
	}

	// Display current player
	display_time(current_player->time_ms, true, player_flag);

	// Check for timeout
	if (current_player->time_ms < 1) {
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
		refresh_displays();
		state = PAUSED; 
	}
}
void small_setup_handler() {
	
	if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			digitalWrite(power, LOW);			
		}
	}
	
	switch (step_flag) {

		case STEP_1: {
      delay(20);
			// Check for long press to enter SETUP
		  if (long_press(button_ok, 200, HIGH)) {
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

			plus_minus_one(&player_1.increment_start);

			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_3;
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
			
			if (plus_flag) { 
				player_1.bonus_1=player_1.bonus_1+6000; 
				plus_flag = false; 
			} 
			
			if (minus_flag) { 
				if (player_1.bonus_1 >= 6000 ){
					player_1.bonus_1=player_1.bonus_1-6000;
				}
				minus_flag = false; 
			}
			
			plus_minus_one(&player_1.bonus_1);

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

			plus_minus_one(&player_1.increment_1);
			
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
			
			if (plus_flag) { 
				(bonus_turn)++; 
				plus_flag = false; 
			} 
			
			if (minus_flag) { 
				if (bonus_turn > 1 ){
					(bonus_turn)--;
				}
				minus_flag = false; 
			}
			

			if (ok_flag) {
				ok_flag = false;
        player_2 = player_1;
				save_players_if_changed(); // Saves settings if they are changed
				state = READY;
			}
			break;
		}
	}	
	
}

void setup_handler() {
	
	
	if (onoff_flag) { // POWER OFF
		onoff_flag = false;
		if(long_press(button_onoff, 1500, LOW)){
			digitalWrite(power, LOW);			
		}
	}
	if (long_press(button_ok, 1200, HIGH)) {
    display_1.clear();
    display_2.clear();
		state = READY;
		ok_flag = false;
		return;      // EXIT SETUP IMMEDIATELY
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

            if (plus_flag) {
                player_1.time_ms += 360000;
                plus_flag = false;
            }

            if (minus_flag && player_1.time_ms >= 360000) {
                player_1.time_ms -= 360000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_1.time_ms += 6000;
                plus_flag = false;
            }

            if (minus_flag && player_1.time_ms >= 6000) {
                player_1.time_ms -= 6000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_1.time_ms += 100;
                plus_flag = false;
            }

            if (minus_flag && player_1.time_ms >= 100) {
                player_1.time_ms -= 100;
                minus_flag = false;
            }

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

            plus_minus_one(&player_1.increment_start);

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

            if (plus_flag) {
                player_1.bonus_1 += 360000;
                plus_flag = false;
            }

            if (minus_flag && player_1.bonus_1 >= 360000) {
                player_1.bonus_1 -= 360000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_1.bonus_1 += 6000;
                plus_flag = false;
            }

            if (minus_flag && player_1.bonus_1 >= 6000) {
                player_1.bonus_1 -= 6000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_1.bonus_1 += 100;
                plus_flag = false;
            }

            if (minus_flag && player_1.bonus_1 >= 100) {
                player_1.bonus_1 -= 100;
                minus_flag = false;
            }

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

            plus_minus_one(&player_1.increment_1);

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

            if (plus_flag) {
                bonus_turn++;
                plus_flag = false;
            }

            if (minus_flag && bonus_turn > 1) {
                bonus_turn--;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_2.time_ms += 360000;
                plus_flag = false;
            }

            if (minus_flag && player_2.time_ms >= 360000) {
                player_2.time_ms -= 360000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_2.time_ms += 6000;
                plus_flag = false;
            }

            if (minus_flag && player_2.time_ms >= 6000) {
                player_2.time_ms -= 6000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_2.time_ms += 100;
                plus_flag = false;
            }

            if (minus_flag && player_2.time_ms >= 100) {
                player_2.time_ms -= 100;
                minus_flag = false;
            }

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

            plus_minus_one(&player_2.increment_start);

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

            if (plus_flag) {
                player_2.bonus_1 += 360000;
                plus_flag = false;
            }

            if (minus_flag && player_2.bonus_1 >= 360000) {
                player_2.bonus_1 -= 360000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_2.bonus_1 += 6000;
                plus_flag = false;
            }

            if (minus_flag && player_2.bonus_1 >= 6000) {
                player_2.bonus_1 -= 6000;
                minus_flag = false;
            }

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

            if (plus_flag) {
                player_2.bonus_1 += 100;
                plus_flag = false;
            }

            if (minus_flag && player_2.bonus_1 >= 100) {
                player_2.bonus_1 -= 100;
                minus_flag = false;
            }

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

            plus_minus_one(&player_2.increment_1);

            if (ok_flag) {
                ok_flag = false;
                save_players_if_changed();
                state = READY;
            }
            break;
        }

    }
}

	
	

void paused_handler() {
	
	timerAlarmDisable(Timer0_Cfg);
	if (play_flag) { // Continue Game
		play_flag = false;
		state = RUNNING; 
	}
	if (ok_flag) {
		ok_flag = false;
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
			digitalWrite(power, LOW);			
		}
	}
}
void game_end_handler() {
	timerAlarmDisable(Timer0_Cfg);
	
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
			digitalWrite(power, LOW);			
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
			showWord("READ", true);
			display_1.showNumberDecEx(filtered*10, 0b00000000, false, 4, 0);
			
			check_player();
			
			if (ok_flag) {
				ok_flag = false;
				step_flag = STEP_3;
				display_1.clear();
				display_2.clear();
			}
			delay(100);
			break;
		}
		
		case STEP_3:{
			
			display_1.showNumberDecEx(small_mid*10, 0b00000000, false, 4, 0);
			showWord("____", true);
			
			if(plus_flag){
				plus_flag = false;
				small_mid = small_mid + 0.1;
			}
			if(minus_flag){
				minus_flag = false;
				small_mid = small_mid - 0.1;
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
			
			display_1.showNumberDecEx(big_mid*10, 0b00000000, false, 4, 0);
			showWord("****", true);
			
			if(plus_flag){
				plus_flag = false;
				big_mid = big_mid + 0.1;
			}
			if(minus_flag){
				minus_flag = false;
				big_mid = big_mid - 0.1;
			}
			if (ok_flag) {
				ok_flag = false;
				float old_small = prefs.getFloat("small_mid", NAN);
        float old_big   = prefs.getFloat("big_mid",   NAN);

        const float eps = 0.0001f;

        if (isnan(old_small) || fabsf(old_small - small_mid) > eps) {
            prefs.putFloat("small_mid", small_mid);
        }
        
        if (isnan(old_big) || fabsf(old_big - big_mid) > eps) {
            prefs.putFloat("big_mid", big_mid);
        }
				state = READY;
			}
			delay(100);
      break;
		}	
	}
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

  setup_buttons();  // attaches interrupts
  //Setup of save
  prefs.begin("chess", false);
  
  small_mid  = prefs.getFloat("small_mid", threshhold);
  big_mid    = prefs.getFloat("big_mid", threshhold);
  	
  //Frequency
  setCpuFrequencyMhz(80);

  //Time Interupt every 10ms
  
   Timer0_Cfg = timerBegin(0, 80, true);
   timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
   timerAlarmWrite(Timer0_Cfg, 10000, true); 
   //timerAlarmEnable(Timer0_Cfg);

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
		
    } // switch
} // loop
