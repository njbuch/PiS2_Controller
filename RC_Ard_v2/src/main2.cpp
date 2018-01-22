/*
  This is the rfirst example of a sketch that tries to support a handshake
  between the Rpi3 and the arduino on the sleepy pi 2.
  
  when running the rpi3 command: "i2cget -y 1 0x07"
  when running the rpi3 command: "i2cset -y 1 0x07 0x11" 
 
 TODO: Merge with https://github.com/SpellFoundry/SleepyPi2/blob/master/examples/ButtonOnOff3/ButtonOnOff3.ino
 TODO: Establish a protocol / statemachine that adresses the commands needed 
 TODO: Convert existing mechanics control to https://github.com/jonblack/arduino-fsm
 
 */
 
#include <Wire.h> 
#include "Arduino.h"
#include "Fsm.h"

static char c = '0';


//Events & message-codes
#define I2C_TRIGGER 50
#define I2C_TRIGACK_RCV 51
#define I2C_TRIGDONE_RCV 52

#define I2C_REPACK_RCV 60
#define I2C_REPDONE_RCV 61

#define GPIO_TRIGGER 70
#define GPIO_BUTTON 71
#define GPIO_BUTTON2 72

/////////////////////////////////////
// Enter/exit functions
void turnOnPiSendI2CTrigger()
{
  Serial.println("func:turnOnSendI2CTrigger");
}

void sendI2CTrigger()
{
  Serial.println("func:sendI2CTrigger");
}



void shutDownPi()
{
  Serial.println("func:shutdownPi");
}


////////////////////////////////////
// States  State(void (*on_enter)(), void (*on_state)(), void (*on_exit)());
State state_waiting(NULL, NULL, NULL);
State state_waitfortriggerack(&turnOnPiSendI2CTrigger, NULL, NULL);
State state_waitforsoundplaying(NULL, NULL, &shutDownPi);

Fsm fsm(&state_waiting);

////////////////////////////////
// Transition functions
void GPIO_Trigger()
{
  Serial.println("func:GPIO_Trigger");

}

void I2C_TrigAck_Rcv()
{
  Serial.println("func:I2C_TrigAck_Rcv");
}

void I2C_RepAck_Rcv()
{
  Serial.println("func:I2C_RepAck_Rcv");
}

void I2C_RepDone_Rcv()
{
  Serial.println("func:I2C_RepDone_Rcv");
}

// this function is registered as an event, see setup() 
void requestEvent() 
{
  Serial.print("requestEvent!");
  Wire.write("Hej");
 if (c > 'z')
   c = '0';
}

void receiveEvent(int numBytes) 
{
  Serial.print("receiveEvent:");
  Serial.print(numBytes);
  while (1 < Wire.available()) { // loop through all but the last
     int code = Wire.read(); // receive byte as a character
     Serial.print(code);         // print the character
     }
  Serial.print(":");
 int inByte = Wire.read();    // receive byte as an integer
 Serial.println(inByte);
 switch (inByte) {
 case I2C_TRIGACK_RCV:
   fsm.trigger(I2C_TRIGACK_RCV);
   break;
 case I2C_TRIGDONE_RCV:
   fsm.trigger(I2C_TRIGDONE_RCV);
   break;
 default:
   break;
 }
  
}


// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
volatile bool  buttonPressed = false;

// A handler for the Button interrupt.
void button_isr()
{
    fsm.trigger(GPIO_TRIGGER);
}

// void add_timed_transition(State* state_from, State* state_to, unsigned long interval, void (*on_transition)());
// void add_transition(State* state_from, State* state_to, int event, void (*on_transition)());

// the setup routine runs once when you press reset:
void setup() {

  fsm.add_transition(&state_waiting, &state_waitfortriggerack,
                     GPIO_TRIGGER, NULL);
  fsm.add_timed_transition(&state_waitfortriggerack, &state_waiting, 3000, &sendI2CTrigger);

  fsm.add_transition(&state_waitfortriggerack, &state_waitforsoundplaying, I2C_TRIGACK_RCV, NULL);
  fsm.add_transition(&state_waitforsoundplaying, &state_waiting, I2C_TRIGDONE_RCV, NULL);
  
  // Allow wake up triggered by button press
  attachInterrupt(1, button_isr, LOW);       // button pin
  
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
  
  // Setup the i2c communication with the RPI3 with two event functions
  Wire.begin(7);                // join i2c bus with address #7 
  Wire.onRequest(requestEvent); // register event 
  Wire.onReceive(receiveEvent);
 
  Serial.begin(9600);           // start serial for output
}

// the loop routine runs over and over again forever:
int c2 = 0;
void loop() {
  fsm.run_machine();
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(2000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(200);               // wait for a second
  Serial.print("Hi:");
  Serial.print(c);
  if (c2++ > 10) {
    Serial.print("Now writing...");
    Serial.print(c);
    Wire.beginTransmission(7); // transmit to device #
    Wire.write("c is ");        // sends five bytes
    Wire.write(c);              // sends one byte
    Wire.endTransmission();    // stop transmitting
    c++;
    c2 = 0;
  }
}

