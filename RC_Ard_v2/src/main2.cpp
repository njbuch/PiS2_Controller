/*
  This is the first example of a sketch that tries to support a handshake
  between the Rpi3 and the Arduino on the sleepy pi 2.
  
  when running the rpi3 command: "i2cget -y 1 0x07"
  when running the rpi3 command: "i2cset -y 1 0x07 0x11" 
 
 TODO: Merge with https://github.com/SpellFoundry/SleepyPi2/blob/master/examples/ButtonOnOff3/ButtonOnOff3.ino
 
 */

#include "Arduino.h"
#include "Fsm.h"
#include "SleepyPi2.h"
#include <Time.h>
#include <LowPower.h>
#include <PCF8523.h>
#include <Wire.h>

#define CIRCULAR_BUFFER_XS
#include <CircularBuffer.h>

static char c = '0';
// Pin 13 has an LED connected on most Arduino boards. 
int led_pin = 13;
int button_pin = 3;
int messagesignal_pin = 7;
int number;
int QuickReply = 255;

volatile bool buttonPressed = false;
int CmdToSend = -1;

// Intersystem codes
#define MSG_OK 11
#define MSG_READY 22
#define MSG_TODO_TRIGGER 50;
#define MSG_TODO_REPORT 51;
#define MSG_TODO_SHUTDOWN 52

//Events & message-codes
#define CONTR_GPIO_TRIGGER 70
#define CONTR_GPIO_BUTTON 71
#define CONTR_GPIO_BUTTON2 72
#define CONTR_TIMEOUT 99
#define CONTR_I2C_TRIGACK_RCV 51
#define CONTR_I2C_TRIGDONE_RCV 52
#define CONTR_I2C_REPACK_RCV 60
#define CONTR_I2C_REPDONE_RCV 61

// I2C FSM Events
#define CMD_SEND_MESSAGE 80
#define CMD_REQUEST 81
#define CMD_RDY_RECEIVED 82
#define CMD_OK_RECEIVED 83


#pragma region ControlFunctions
/////////////////////////////////////
// Enter/exit functions
bool PiOn_flag = false;
unsigned long PiOn_time = 0;
void turnOnPi()
{
  float pi_current;
  Serial.println(F("func:turnOnPi"));
  if (!PiOn_flag) {
    Serial.println(F("Now switching on power to RPi!\n"));
    SleepyPi.enableExtPower(true);
    SleepyPi.enablePiPower(true);
    PiOn_time = millis();
    PiOn_flag = true;
  } else {
    pi_current = SleepyPi.rpiCurrent();
    // Assuming that something is rotten if there is a power-consumption on less than 10mA
    if (pi_current > 10) {
      Serial.println(F("RPi should already be switched on but does not use any power, trying to switch on!"));
      SleepyPi.enableExtPower(true);
      SleepyPi.enablePiPower(true);
      PiOn_time = millis();
      PiOn_flag = true;
    }
  }
}

void SignalNOMessageToPi()
{
  Serial.println(F("func:SignalNOMessageToPi"));
  digitalWrite(messagesignal_pin, LOW);  
}

void shutDownPi()
{
  Serial.println(F("func:shutdownPi"));
  //Use the SleepyPi2 lib to switch off
  SleepyPi.enableExtPower(false);
  SleepyPi.enablePiPower(false);
  PiOn_flag = false;
  SignalNOMessageToPi();
}


int timeswithlowpower = 0;
void verifyPiOnState() 
{
  /* unsigned long pi_current = 1000*SleepyPi.rpiCurrent();
  current_readings.push(pi_current);
  unsigned long avg = 0;
	
  /* for (unsigned int i = 0; i < current_readings.size(); i++) {
			avg += current_readings[i] / current_readings.size();
	} 
  //Serial.print("The average power consumpt seen over a period is:");
  //Serial.print(avg);
  
  if (avg < 130000 && current_readings.size() > 18 && PiOn_flag == true) {
    //Serial.println("Now cutting power off, and setting state to off. Period.");
    shutDownPi();
  } */
}

void SignalMessageToPi()
{
  Serial.println(F("func:SignalMessageToPi"));
  digitalWrite(messagesignal_pin, HIGH);  
}

void errorMessage()
{
  Serial.println(F("In a bad error-state. RPI not responding."));
}

void GPIO_Trigger()
{
  Serial.println(F("func:GPIO_Trigger"));
}

#pragma endregion


#pragma region States
// Primary controller states
State state_contr_waiting(1, NULL, NULL, NULL);
State state_contr_waitfortriggerack(2, NULL, NULL, NULL);
State state_contr_rpinotworking(3, NULL, NULL, NULL);
State state_contr_waitforshutdown(4, NULL, NULL, NULL);
Fsm fsm(&state_contr_waiting);

// I2C protocol communication states
State state_i2c_onning(10, NULL, NULL, NULL);
State state_i2c_readying(11, NULL, NULL, NULL);
State state_i2c_confirming(12, NULL, NULL, NULL);
State state_i2c_commanding(13, NULL, NULL, NULL);
State state_i2c_waiting_pioff(14, NULL, NULL, NULL);
Fsm fsmi2c(&state_i2c_waiting_pioff);

#pragma endregion

#pragma region I2CTransitionFunctions
////////////////////////////////
// Transition functions

void I2C_TrigAck_Rcv()
{
  Serial.println(F("func:I2C_TrigAck_Rcv"));
}

void I2C_RepAck_Rcv()
{
  Serial.println(F("func:I2C_RepAck_Rcv"));
}

void I2C_RepDone_Rcv()
{
  Serial.println(F("func:I2C_RepDone_Rcv"));
}

////////////////////////
// I2C Transition functions
void i2c_waiting_to_onning()
{
   Serial.println(F("\n\nTransition:i2c_waiting_to_onning.\n"));
   SignalMessageToPi();
   QuickReply = MSG_READY;
   Serial.println(F("MessagePin set to high. Hoping for a i2c read request.\n"));
}

void i2c_onning_to_readying()
{
   //Wire.write(MSG_READY);
   Serial.println(F("\n\nTransition:i2c_onning_to_readying. Now sent MSG_READY!\n"));
}

void i2c_readying_to_confirming()
{
   Serial.println(F("\n\nTransition:i2c_readying_to_confirming.\n"));
   QuickReply = CmdToSend;
   digitalWrite(messagesignal_pin, LOW);  
   Serial.println(F("MessagePin set to LOW. Hoping for a i2c read request again.\n"));
}

void i2c_confirming_to_commanding()
{
   // Wire.write(CmdToSend);
   Serial.println(F("\n\nTransition:i2c_confirming_to_commanding.\n"));
   Serial.print(F("\nNow sent:"));
   Serial.print(CmdToSend);
}

void i2c_commanding_to_waiting()
{
   Serial.println(F("\n\nTransition:i2c_commanding_to_waiting.\n"));
   fsm.trigger( CONTR_I2C_TRIGDONE_RCV);
}

#pragma endregion I2CTransitionFunctions

#pragma region ContrTransitionFunctions
////////////////////////
// Controller transition functions
void contr_waiting_to_waitfortriggerack()
{
   Serial.println(F("\n\nTranisition:contr_waiting_to_waitfortriggerack.\n"));
   turnOnPi();
   CmdToSend = MSG_TODO_TRIGGER;
   fsmi2c.trigger(CMD_SEND_MESSAGE);
}

int triggerackwaits = 0;
void contr_waitfortriggerack_to_waitfortriggerack()
{
   Serial.println(F("\n\nTranisition:contr_waitfortriggerack_to_waitfortriggerack.\n"));
   if (triggerackwaits++ > 10) {
    Serial.print(F("\nKept waiting for RPi too long. Returning to wait."));
    triggerackwaits = 0;
    fsm.trigger(CONTR_TIMEOUT);
  }
}

void contr_waitfortriggerack_to_rpinotworking()
{
   Serial.println(F("\n\nTranisition:contr_waitfortriggerack_to_rpinotworking.\n"));
}

void contr_rpinotworking_to_waiting()
{
   Serial.println(F("\n\nTranisition:contr_rpinotworking_to_waiting.... LETS TRY AGAIN :) \n"));
}

void contr_waitfortriggerack_to_waitforsoundplaying()
{
   Serial.println(F("\n\nTranisition:contr_waitfortriggerack_to_waitforsoundplaying.\n"));
}

void contr_waitfortriggerack_to_waiting()
{
   Serial.println(F("\n\nTranisition:contr_waitfortriggerack_to_waiting.\n"));
}

void contr_waitforshutdown_to_waiting()
{
   Serial.println(F("\n\nTranisition:contr_waitforshutdown_to_waiting.\n"));
   shutDownPi();
}

void contr_waiting_to_waitforshutdown()
{
  Serial.println(F("\n\nTranisition:contr_waiting_to_waitforshutdown.\n"));
  CmdToSend = MSG_TODO_SHUTDOWN;
  fsmi2c.trigger(CMD_SEND_MESSAGE);
}

#pragma endregion ContrTransitionFunctions

#pragma region Transitions
//////////////////////////////////////////////////////////
// Declaring and defining TWO state machines
// One for the primary states of the controller and one for I2C comms with RPi3
// States  State(void (*on_enter)(), void (*on_state)(), void (*on_exit)());

void setupTransitions() {
  // First the core controller state transitions
  fsm.add_transition(&state_contr_waiting, &state_contr_waitfortriggerack, CONTR_GPIO_TRIGGER, &contr_waiting_to_waitfortriggerack);
  // fsm.add_timed_transition(&state_contr_waitfortriggerack, &state_contr_waitfortriggerack, 3000, &contr_waitfortriggerack_to_waitfortriggerack);
  fsm.add_timed_transition(&state_contr_waitfortriggerack, &state_contr_rpinotworking, 120000, &contr_waitfortriggerack_to_rpinotworking); // Max 2 minute sounds!
  fsm.add_timed_transition(&state_contr_rpinotworking, &state_contr_waiting, 10000, &contr_rpinotworking_to_waiting); // self timed
  fsm.add_transition(&state_contr_waitfortriggerack, &state_contr_rpinotworking, CONTR_TIMEOUT, &contr_waitfortriggerack_to_rpinotworking); //forced
  fsm.add_transition(&state_contr_waitfortriggerack, &state_contr_waiting, CONTR_I2C_TRIGDONE_RCV, &contr_waitfortriggerack_to_waiting); 
  fsm.add_transition(&state_contr_waiting, &state_contr_waitforshutdown, CONTR_GPIO_BUTTON, &contr_waiting_to_waitforshutdown);
  fsm.add_timed_transition(&state_contr_waitforshutdown, &state_contr_waiting, 20000, &contr_waitforshutdown_to_waiting);

  // Secondly the i2c protocol transitions
  fsmi2c.add_transition(&state_i2c_waiting_pioff, &state_i2c_onning, CMD_SEND_MESSAGE, &i2c_waiting_to_onning);
  fsmi2c.add_transition(&state_i2c_onning, &state_i2c_readying, CMD_REQUEST, &i2c_onning_to_readying);
  fsmi2c.add_transition(&state_i2c_readying, &state_i2c_confirming, CMD_RDY_RECEIVED, &i2c_readying_to_confirming);
  fsmi2c.add_transition(&state_i2c_confirming, &state_i2c_commanding, CMD_REQUEST, &i2c_confirming_to_commanding);
  fsmi2c.add_transition(&state_i2c_commanding, &state_i2c_waiting_pioff, CMD_OK_RECEIVED, &i2c_commanding_to_waiting);
  fsmi2c.add_timed_transition(&state_i2c_commanding, &state_i2c_waiting_pioff, 20000, NULL);
  fsmi2c.add_timed_transition(&state_i2c_confirming, &state_i2c_waiting_pioff, 20000, NULL);
  fsmi2c.add_timed_transition(&state_i2c_readying, &state_i2c_waiting_pioff, 20000, NULL);
  fsmi2c.add_timed_transition(&state_i2c_onning, &state_i2c_waiting_pioff, 20000, NULL);
}

#pragma endregion Transitions

#pragma region I2CCore
//////////////////////////
// I2C core functions

uint8_t fromking = 0;
void requestEvent() 
{
  // Very important that the first command in this function is just sending a message, anything else will result
  // in problems with timing
  Wire.write(QuickReply);
  // Wire.write((uint8_t)22);
  // delay(500);
  // Serial.print(F("requestEvent triggerede by I2C comms...:");
  // Serial.println(F(QuickReply);
  // fsmi2c.trigger(CMD_REQUEST); // Please note this can trigger two different state changes!
  fromking = CMD_REQUEST;
}

void receiveEvent(int numBytes) 
{
  Serial.print(F("receiveEvent:"));
  Serial.print(numBytes);
  while (1 < Wire.available()) { // loop through all but the last
     int code = Wire.read(); // receive byte as a character
     Serial.print(code);         // print the character
     }
  Serial.print(F(":"));
  int inByte = Wire.read();    // receive byte as an integer
  Serial.println(inByte);
  switch (inByte) {
    case MSG_READY:
      fsmi2c.trigger(CMD_RDY_RECEIVED);
      break;
    case MSG_OK:
      Serial.print(F("OK received, now relax!"));
      fsmi2c.trigger(CMD_OK_RECEIVED);
      break;
    default:
      break;
 }
  
}
#pragma endregion



// A handler for the Button interrupt.

byte buttonPresses = 0;                // how many times the button has been pressed 
unsigned long lastpress_time = 0;
unsigned long last_interrupt_time = 0;
void button_isr()
{
  // first we have the debounce
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
    last_interrupt_time = interrupt_time;
    // then we have the buttonpress
    buttonPresses++;
    if (buttonPresses == 4) buttonPresses = 0;         // rollover every fourth press
    lastpress_time = millis();  
    Serial.print(F("Press:"));
    Serial.println(buttonPresses);
  }
}

// void add_timed_transition(State* state_from, State* state_to, unsigned long interval, void (*on_transition)());
// void add_transition(State* state_from, State* state_to, int event, void (*on_transition)());

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);           // start serial for output
  Serial.println(F("Setup beginning..."));
  delay(1000);
  
  setupTransitions();

  // Allow wake up triggered by button press
  attachInterrupt(1, button_isr, LOW);       // button pin
  
  // initialize the digital pin as an output.
  pinMode(led_pin, OUTPUT);  
  pinMode(messagesignal_pin, OUTPUT);   
  
  // Setup the i2c communication with the RPI3 with two event functions
  Wire.begin(7);                // join i2c bus with address #7 
  Wire.onRequest(requestEvent); // register event 
  Wire.onReceive(receiveEvent);



  digitalWrite(messagesignal_pin, LOW);  
  const char compile_date[] = __DATE__ " " __TIME__;
  Serial.println(compile_date);
    // Switching on
  PiOn_flag = true;
  SleepyPi.enablePiPower(true);  
  SleepyPi.enableExtPower(true);
  
  Serial.println(F("\n\n---------------\nSetup complete. MessagePin set to low. Now waiting."));


}

// the loop routine runs over and over again forever:
int c2 = 0;
bool toggle = true;
float  pi_current; 
unsigned long verifytime = 0;
void loop() {
  fsm.run_machine();
  fsmi2c.run_machine();
  
  // button-press handler
  if ((buttonPresses > 0) && ((millis() - lastpress_time) > 1000)) {
    switch (buttonPresses) {
      case 1:
        Serial.println(F("1 button press, fire!"));
        fsm.trigger(CONTR_GPIO_TRIGGER);
        // statements
        break;
      case 2:
        // statements
        Serial.println(F("2 button press, fire!"));
        fsm.trigger(CONTR_GPIO_BUTTON);
        break;
      case 3:
        // statements
        Serial.println(F("3 button press, fire!"));
        break;
      default:
        // statements
        Serial.println(F("Weird button press, fire!"));
    }
    buttonPresses = 0;
  }

  // stupid hack! Something about the i2c interrupt cannot call fsm!??
  if (fromking != 0) {
    fsmi2c.trigger(CMD_REQUEST);
    fromking = 0;
    Serial.println(F("I2cRequest executed!"));
  }


  if (millis() - verifytime >= 500) {
		verifytime = millis();
		verifyPiOnState();
	}  
  
  if (c++ > 300) {
    toggle = !toggle;
    digitalWrite(led_pin, toggle);   // turn the LED on (HIGH is the voltage level)
    c = 0;
  }
  if (c2++ > 20000) {
    Serial.print(F("State controller: "));
    Serial.print(fsm.current_state_name());
    Serial.print(F("     State i2c: "));
    Serial.print(fsmi2c.current_state_name());
    pi_current = SleepyPi.rpiCurrent();
    Serial.print(F("  Data: "));
    Serial.print(pi_current);
    Serial.print(F(" mA  "));
    Serial.print(PiOn_flag);
    Serial.print(F(" -> since "));
    Serial.print(PiOn_time);
    Serial.println(F(".  "));
    // Serial.print(c);
    // Wire.beginTransmission(7); // transmit to device #
    // Wire.write("c is ");        // sends five bytes
    // Wire.write(c);              // sends one byte
    // Wire.endTransmission();    // stop transmitting
    c2 = 0;
  } 
  
}

