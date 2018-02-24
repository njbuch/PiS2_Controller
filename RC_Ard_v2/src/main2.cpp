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
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led_pin = 13;
int button_pin = 3;
int messagesignal_pin = 7;
int number;

volatile bool  buttonPressed = false;


//Events & message-codes
#define I2C_TRIGGER 50
#define I2C_TRIGACK_RCV 51
#define I2C_TRIGDONE_RCV 52

#define I2C_REPACK_RCV 60
#define I2C_REPDONE_RCV 61

#define GPIO_TRIGGER 70
#define GPIO_BUTTON 71
#define GPIO_BUTTON2 72

#define ARD_MESSAGE 80
#define MSG_REQUEST 81

#define TIMEOUT 99

/////////////////////////////////////
// Enter/exit functions
void turnOnPi()
{
  Serial.println("func:turnOnPi");
}


void shutDownPi()
{
  Serial.println("func:shutdownPi");
}

void SignalMessageToPi()
{
  Serial.println("func:SignalMessageToPi");
  digitalWrite(messagesignal_pin, HIGH);  
}

void errorMessage()
{
  Serial.println("In a bad error-state. RPI not responding.");
}


////////////////////////////////////
// States  State(void (*on_enter)(), void (*on_state)(), void (*on_exit)());
State state_waiting(NULL, NULL, &turnOnPi);
Fsm fsm(&state_waiting);
State state_waiting_pioff(NULL, NULL, &turnOnPi);
Fsm fsmi2c(&state_waiting_pioff);

char *ArdMessage = "Empty";
void sendI2CTrigger()
{
  static int sendTriggerCount = 0;

  Serial.print("func:sendI2CTrigger->");
  Serial.println(sendTriggerCount);

  ArdMessage = "Trigger";
  fsmi2c.trigger(ARD_MESSAGE);

  sendTriggerCount++;
  if (sendTriggerCount > 10) {
    sendTriggerCount = 0;
    fsm.trigger(TIMEOUT);
  }
}


State state_waitfortriggerack(&sendI2CTrigger, NULL, NULL);
State state_waitforsoundplaying(NULL, NULL, &shutDownPi);
State state_rpinotworking(&errorMessage, NULL, NULL);




State state_waitingforaction(&SignalMessageToPi, NULL, NULL);
State state_waitingformessage(NULL, NULL, NULL);




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
  Wire.write(11);
  // Serial.print("requestEvent!");
  // byte output[] = {0x01,0x02,0x03,0x04};  // This is just some sample data for testing
  // Wire.write(output, 4);
  // Wire.write(ArdMessage);
  // delay(200);
  Serial.print("Setting Signal Pin LOW");
  digitalWrite(messagesignal_pin, LOW);  
  fsmi2c.trigger(MSG_REQUEST);
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
  fsm.add_timed_transition(&state_waitfortriggerack, &state_waitfortriggerack, 3000, NULL);

  fsm.add_transition(&state_waitfortriggerack, &state_rpinotworking, TIMEOUT, NULL);
  fsm.add_transition(&state_waitfortriggerack, &state_waitforsoundplaying, I2C_TRIGACK_RCV, NULL);
  fsm.add_transition(&state_waitforsoundplaying, &state_waiting, I2C_TRIGDONE_RCV, NULL);
  

  fsmi2c.add_transition(&state_waiting_pioff, &state_waitingforaction, ARD_MESSAGE, NULL);
  fsmi2c.add_transition(&state_waitingforaction, &state_waitingformessage, MSG_REQUEST, NULL);
  fsmi2c.add_transition(&state_waitingformessage, &state_waitingforaction, ARD_MESSAGE, NULL);

  // Allow wake up triggered by button press
  attachInterrupt(1, button_isr, LOW);       // button pin
  
  // initialize the digital pin as an output.
  pinMode(led_pin, OUTPUT);  
  pinMode(messagesignal_pin, OUTPUT);   
  
  // Setup the i2c communication with the RPI3 with two event functions
  Wire.begin(7);                // join i2c bus with address #7 
  Wire.onRequest(requestEvent); // register event 
  Wire.onReceive(receiveEvent);
 
  Serial.begin(9600);           // start serial for output

  Serial.println("func:SignalNOMessageToPi");
  digitalWrite(messagesignal_pin, LOW);  

}

// the loop routine runs over and over again forever:
int c2 = 0;
void loop() {
  fsm.run_machine();
  fsmi2c.run_machine();
  digitalWrite(led_pin, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for a second
  digitalWrite(led_pin, LOW);    // turn the LED off by making the voltage LOW
  delay(200);               // wait for a second

  if (c2++ > 10) {
    Serial.print(".");
    Serial.print(c);
    // Wire.beginTransmission(7); // transmit to device #
    // Wire.write("c is ");        // sends five bytes
    // Wire.write(c);              // sends one byte
    // Wire.endTransmission();    // stop transmitting
    c++;
    c2 = 0;
  }
}

