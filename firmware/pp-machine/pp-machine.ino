/*
 * Vaporiser System for delivering flavours
 * 
 * (c) 2017 by LatexDoc (https://fetlife.com/users/1444096)
 * 
 * Functions:
 * - fixed time-slots of open valve to provide flavour to the system
 *     0 .. 10 Minutes run-time:  5s on -> 55s off
 *    10 .. 20 Minutes run-time:  7s on -> 53s off
 *    20 .. 30 Minutes run-time: 10s on -> 50s off
 *    > 30 Minutes run-time:     20s on -> 40s off
 * 
 * - manual flavour delivery on button-press (Pin 8)
 * - switch between automatic/manual mode (Pin 4)
 * - start of timer in automatic mode (Pin 5)
 * - stop of timer in automatic mode (Pin 6)
 * - pause of automatic mode (Pin 7)
 * - output of valve status through LED (Pin 9)
 *    LED off: valve is shut / timer if off 
 *    LED on:  valve open
 *    LED blink: valve is shut / timer paused
 * - status of power supply through LED
 * - LED to display the status of the device (heartbeat) (Pin 3)
 * 
 * Description of the hardware: 
 * - output-pin is always HIGH-Logic: HIGH || 1 == HIGH (3.3V) on the Pin
 *    Pin X -> 4.7kOhm -> LED_A -> LED_K -> GND
 *    Pin X -> relais-coil -> GND
 * 
 * 
 */

 /* To-Do: 
  *
  */

#include <avr/pgmspace.h>

// declaration of constants
const byte out_ventil = 2;        // output - valve: Pin2
const byte out_hb = 3;            // output heartbeat: Pin3
const byte out_led_pcb = 13;      // output - LED on Arduino-PCB
const byte out_led_status = 9;    // output - LED to display the valve status: Pin9
const byte in_auto_man = 4;       // input - switch (SPDT) for altering between manual (HIGH) and timer mode (LOW): Pin4
const byte in_start = 5;          // input - push-button for start in timer mode or manual opening of the valve: Pin5
const byte in_stop = 6;           // input - push-button for stop in timer mode: Pin6
const byte in_pause = 7;          // input - push-button for pause in timer mode: Pin7
const byte in_abgabe = 8;         // input - push-button for immediate valve opening: Pin8 !!! this function is currentlty not in use !!! use start-button to override valve status to open

// hard-coded array of the timer-mode: x-axis: seconds 0..59; y-axis: minutes 0..29; 0=valve open; 1=valve closed
// the very last bit represents the valve status after the time > 30 minutes! 
const unsigned int time_output_values[1800] PROGMEM = 
 {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, // 1 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, // 2 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 3 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, // 4 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, // 5 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, // 6 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, // 7 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, // 8 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, // 9 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, //10 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, //11 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1, //12 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, //13 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1, //14 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, //15 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, //16 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1, //17 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, //18 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1, //19 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, //20 minute   
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1, //21 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, //22 minute 
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1, //23 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1, //24 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, //25 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1, //26 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1, //27 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, //28 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1, //29 minute
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1  //30 minute
  };

// declaration of the global variables
byte out_ventil_state = HIGH;     // init of the valve output: HIGH (valve shut)
byte out_hb_state = HIGH;         // init of the HB-LED to on
byte out_led_pcb_state = LOW;     // init of the PCB-LED to off
byte out_led_state = LOW;         // init of status-LED to off
int hb_interval = 1000;           // set HB-interval to 1000ms
int led_interval = 500;           // set LED update interval to 500ms
int timer_interval = 1000;        // set timer update interval to 1000ms
byte in_auto_man_state = 0;       // init timer mode input to LOW; HIGH == timer mode on; LOW == timer mode off / manual mode
byte in_start_state = 0;          // init input values 
byte in_stop_state = 0;           // init input values
byte in_pause_state = 0;          // init input values
byte in_abgabe_state = 0;         // init input values

byte abgabe_taster = 0;           // buffer for immediate valve opening

unsigned long lastDebounceTime_in_start = 0;   // last time read a "1" on pin "start" 
unsigned long lastDebounceTime_in_stop = 0;    // last time read a "1" on pin "stop"
unsigned long lastDebounceTime_in_pause = 0;   // last time read a "1" on pin "pasue"
unsigned long lastDebounceTime_in_abgabe = 0;  // last time read a "1" on pin "immediate valve opening or "sample""

byte debounceDelay = 50;         // Debounce-Zeit in ms; attention: passoible values only 8 bit!

unsigned long hb_previousMillis = 0;      // to save the absolute time in ms of the last HB-LED-update
unsigned long led_previousMillis = 0;     // to save the absolute time in ms of the last status-LED-update
unsigned long timer_previousMillis = 0;   // to save the absolute time in ms of the last timer-update
unsigned long sample_currentMillis = 0;   // to save the absolute time in ms of the last sample-update
unsigned int hb = 0;             // helper variable for HB-function
unsigned int boot_millis = 0;    // to save the time in ms at start of system 
unsigned int operation_mode = 0; // recent mode of operation : 0 = off; 1 = running; 2 = pause; 3 = manual
unsigned long sekunden = 0;      // counter for seconds

void setup() {
  // init of hardware pins
  pinMode(out_ventil, OUTPUT);      
  pinMode(out_hb, OUTPUT);
  pinMode(out_led_pcb, OUTPUT);
  pinMode(out_led_status, OUTPUT);
  pinMode(in_auto_man, INPUT);  
  pinMode(in_start, INPUT);  
  pinMode(in_stop, INPUT);  
  pinMode(in_pause, INPUT);  
  pinMode(in_abgabe, INPUT);    
  // UART configuration
  Serial.begin(115200);
  // save time of ms-counter
  boot_millis = millis();
}

void loop() {

  int i=0;
  char mychar;

  // start of HB-function; toggle HB-LED every 1000ms to show that the system is still running and not dead
  unsigned long hb_currentMillis = millis(); 
  if(hb_currentMillis - hb_previousMillis > hb_interval) {
    Serial.print("\n\Status:");
    Serial.print("\nMode of timer: ");
    Serial.print(in_auto_man_state);
    Serial.print("\nMode of push-button Start: ");
    Serial.print(in_start_state);
    Serial.print("\nMode of push-button Stop: ");
    Serial.print(in_stop_state);
    Serial.print("\nMode of push-button Pause: ");
    Serial.print(in_pause_state);
    Serial.print("\nBMode of Operation: ");
    Serial.print(operation_mode);
    Serial.print("\nTimer - elapsed seconds: ");
    Serial.print(sekunden);    
    Serial.print("\n****Status of Valve****: ");
    Serial.print(out_ventil_state);    
    Serial.print("\nTimer target state: ");

    mychar =  pgm_read_byte_near(time_output_values + sekunden); 
     if (mychar == 0)
      Serial.print("0");
     if (mychar == 1)
      Serial.print("1");
        
    // save the last time since the HB-LED toggled
    hb_previousMillis = hb_currentMillis;   
    // toggle of the HB-LED
    if (hb == LOW)
      hb = HIGH;
    else
      hb = LOW;
    // set the state of HB-LED in hardware
   digitalWrite(out_hb, hb);
   digitalWrite(out_led_pcb, hb);
  } // end of HB-LED function

  // read the input switch and push-buttons and debounce them
  // read timer mode switch Pin4
  // input of timer mode operation: auto (LOW), manual (HIGH)
  in_auto_man_state = digitalRead(in_auto_man);
  if (in_auto_man_state == LOW)
   operation_mode = 3;  // set mode of operation to manual 
  if ((in_auto_man_state == HIGH) && (operation_mode == 3))
   operation_mode = 0;
//  else
//   operation_mode = 0;  // Setzen des Betriebsmodus "Zeitautomatik Stop"    
  // read input of push-button 
  // read recent value of input push-button start
  in_start_state = digitalRead(in_start);  
  // debounce
  if ( (millis() - lastDebounceTime_in_start) > debounceDelay) {
    // if button was pressed -> save state
    if ((in_start_state == 1) && (operation_mode < 3)) {
     operation_mode = 1; // timer mode  -> "start"     
     lastDebounceTime_in_start = millis(); // set recent debounce time
    } 
  }
  // read recent state of push-button stop
  in_stop_state = digitalRead(in_stop);  
  // debounce
  if ( (millis() - lastDebounceTime_in_stop) > debounceDelay) {
    // if button was pressed -> save state
    if ((in_stop_state == 1) && (operation_mode < 3)) {
     operation_mode = 0; // timer mode -> "stop"
     sekunden = 0;  // reset timer to 0
     lastDebounceTime_in_stop = millis(); // set recent debounce time
    } 
  }
  // read recent state of push-button "Pause"
  in_pause_state = digitalRead(in_pause);  
  // deounce
  if ( (millis() - lastDebounceTime_in_pause) > debounceDelay) {
    // if button was pressed -> save state
    if ((in_pause_state == 1) && (operation_mode == 1)) {
     operation_mode = 2; // timer mode -> "pause"
     lastDebounceTime_in_pause = millis(); // set recent debounce time
    } 
  }
  // read recent state of push-button "sample"
  in_abgabe_state = digitalRead(in_abgabe);  
  abgabe_taster = 0;  // set valve to shut
  // debounce
  if ( (millis() - lastDebounceTime_in_abgabe) > debounceDelay) {
    // if button was pressed -> save state
    if (in_abgabe_state == HIGH) {
     abgabe_taster = 1;  // set valve state to open
     lastDebounceTime_in_abgabe = millis(); // set debounce time
    } 
  }

  // set the status-LED
  // output-state of delivery (Pin 9)
  //    LED off: valve shut / timer in stop mode or off
  //    LED on:  valve open in timer mode or manual override and valve open
  //    LED blink: in timre mode: valve shut / timer paused

  if (operation_mode != 2)
   out_led_state = !(out_ventil_state); //out_ventil_state;
  
/*
  if ((operation_mode == 0) && (out_ventil_state == HIGH))  // Betriebsmodus: Manuell; Ventil geschlossen
   out_led_state = LOW;

  if ((operation_mode == 0) && (out_ventil_state == LOW)) // Betriebsmodus: Manuell; Ventil offen
   out_led_state = HIGH;

  if ((operation_mode == 1) && (out_ventil_state == HIGH))  // Betriebsmodus: Zeitautomatik; Ventil geschlossen
   out_led_state = LOW;

  if ((operation_mode == 1) && (out_ventil_state == LOW)) // Betriebsmodus: Zeitautomatik; Ventil offen
   out_led_state = HIGH;
*/

  
  if (operation_mode == 2) { // enter this function when in timer-mode and paused
   // toggle status LED (500ms)
   unsigned long led_currentMillis = millis(); 
   if(led_currentMillis - led_previousMillis > led_interval) {
     // save the last time the LED toggled
     led_previousMillis = led_currentMillis;   
     // toggle status LED
     if (out_led_state == LOW)
       out_led_state = HIGH;
     else
       out_led_state = LOW;
     // set LED status in hardware
   } 
  }
  digitalWrite(out_led_status, out_led_state);
  // end of LED function

  //timing of timer mode
  //     0 .. 10 minutes in run-time:  5s on -> 55s off
  //    10 .. 20 minutes in run-time:  7s on -> 53s off
  //    20 .. 30 minutes in run-time: 10s on -> 50s off
  //    > 30 minutes in run-time:     20s on -> 40s off

  // count seconds only when in timer mode 
  unsigned long timer_currentMillis = millis();
  if(timer_currentMillis - timer_previousMillis > timer_interval)
   if ((operation_mode == 1) && (sekunden < 1800)){
    timer_previousMillis = timer_currentMillis;       
    sekunden++;  // increase by one second
   }
   
  if ((operation_mode == 3) || (operation_mode == 0))
    out_ventil_state = HIGH; // prepare output and set to 0

  if (abgabe_taster == 1)
    out_ventil_state = LOW; // prepare output and set to 1

  // manual flavour delivery in operational mode 3 or 1
  if ((in_start_state == 1) && ((operation_mode == 3) || (operation_mode == 1)))
   out_ventil_state = LOW;
  if ((in_start_state == 0) && ((operation_mode == 3) || (operation_mode == 1)))
   out_ventil_state = HIGH;

  if ((in_start_state == 0) && (operation_mode == 1) || (operation_mode == 2)){ // set values during timer mode 
     mychar =  pgm_read_byte_near(time_output_values + sekunden); 
     if (mychar == 0)
      out_ventil_state = HIGH;
     if (mychar == 1)
      out_ventil_state = LOW;
  }
  //if ((in_start_state == 1) && (operation_mode = 3))
   
  // set putput state of the valve in hardware
  digitalWrite(out_ventil, out_ventil_state);

}
