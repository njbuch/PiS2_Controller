/*
  This is the first example of a sketch that tries to support a handshake
  between the Rpi3 and the Arduino on the sleepy pi 2.
  
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

volatile bool buttonPressed = false;


//Events & message-codes
#define CMD_TRIGGER 50
#define I2C_TRIGACK_RCV 51
#define I2C_TRIGDONE_RCV 52

#define I2C_REPACK_RCV 60
#define I2C_REPDONE_RCV 61

#define GPIO_TRIGGER 70
#define GPIO_BUTTON 71
#define GPIO_BUTTON2 72

// I2C FSM Events
#define CMD_SEND_MESSAGE 80
#define CMD_MSG_REQUEST 81
#define CMD_RDY_RECEIVED 82
#define CMD_CMD_REQUESTED 83
#define CMD_OK_RECEIVED 84

#define TIMEOUT 99

/////////////////////////////////////
// Enter/exit functions
void turnOnPi()
{
  Serial.println("func:turnOnPi");
  //TODO Use the SleepyPi2 lib to switch on
}


void shutDownPi()
{
  Serial.println("func:shutdownPi");
  //TODO Use the SleepyPi2 lib to switch off
}

void SignalMessageToPi()
{
  Serial.println("func:SignalMessageToPi");
  digitalWrite(messagesignal_pin, HIGH);  
}

void SignalNOMessageToPi()
{
  Serial.println("func:SignalNOMessageToPi");
  digitalWrite(messagesignal_pin, LOW);  
}

void errorMessage()
{
  Serial.println("In a bad error-state. RPI not responding.");
}


//////////////////////////////////////////////////////////
// Declaring and defining TWO state machines
// One for the primary states of the controller and one for I2C comms with RPi3
// States  State(void (*on_enter)(), void (*on_state)(), void (*on_exit)());
State state_contr_waiting(NULL, NULL, NULL);
Fsm fsm(&state_contr_waiting);

State state_i2c_waiting_pioff(NULL, NULL, &turnOnPi);
Fsm fsmi2c(&state_i2c_waiting_pioff);

char *ArdMessage = "Empty";

int CmdToSend = 0;

void sendI2CTrigger()
{
  static int sendTriggerCount = 0;

  Serial.print("func:sendI2CTrigger->");
  Serial.println(sendTriggerCount);

  CmdToSend = CMD_TRIGGER;
  fsmi2c.trigger(CMD_SEND_MESSAGE);

  sendTriggerCount++;
  if (sendTriggerCount > 10) {
    sendTriggerCount = 0;
    fsm.trigger(TIMEOUT);
  }
}


// Primary controller states
State state_contr_waitfortriggerack(NULL, NULL, NULL);
State state_contr_waitforsoundplaying(NULL, NULL, NULL);
State state_contr_rpinotworking(&errorMessage, NULL, NULL);


// I2C protocol communication states
State state_i2c_onning(NULL, NULL, NULL);
State state_i2c_readying(NULL, NULL, NULL);
State state_i2c_confirming(NULL, NULL, NULL);
State state_i2c_commanding(NULL, NULL, NULL);


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
  //Wire.write(11);
  Serial.print("requestEvent!");
  // byte output[] = {0x01,0x02,0x03,0x04};  // This is just some sample data for testing
  // Wire.write(output, 4);
  // Wire.write(ArdMessage);
  // delay(200);
  // Serial.print("Setting Signal Pin LOW");
  // digitalWrite(messagesignal_pin, LOW);  
  // fsmi2c.trigger(MSG_REQUEST);
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

void i2c_waiting_to_onning()
{

}

void i2c_onning_to_readying()
{

}

void i2c_readying_to_confirming()
{

}

void i2c_confirming_to_commanding()
{

}

void i2c_commanding_to_waiting()
{

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

  // First the core controller state transitions
  fsm.add_transition(&state_contr_waiting, &state_contr_waitfortriggerack, GPIO_TRIGGER, NULL);
  fsm.add_timed_transition(&state_contr_waitfortriggerack, &state_contr_waitfortriggerack, 3000, NULL);

  fsm.add_transition(&state_contr_waitfortriggerack, &state_contr_rpinotworking, TIMEOUT, NULL);
  fsm.add_transition(&state_contr_waitfortriggerack, &state_contr_waitforsoundplaying, I2C_TRIGACK_RCV, NULL);
  fsm.add_transition(&state_contr_waitforsoundplaying, &state_contr_waiting, I2C_TRIGDONE_RCV, NULL);
  
  // Secondly the i2c protocol transitions
  fsmi2c.add_transition(&state_i2c_waiting_pioff, &state_i2c_onning, CMD_SEND_MESSAGE, &i2c_waiting_to_onning);
  fsmi2c.add_transition(&state_i2c_onning, &state_i2c_readying, CMD_MSG_REQUEST, &i2c_onning_to_readying);
  fsmi2c.add_transition(&state_i2c_readying, &state_i2c_confirming, CMD_RDY_RECEIVED, &i2c_readying_to_confirming);
  fsmi2c.add_transition(&state_i2c_confirming, &state_i2c_commanding, CMD_CMD_REQUESTED, &i2c_confirming_to_commanding);
  fsmi2c.add_transition(&state_i2c_commanding, &state_i2c_waiting_pioff, CMD_OK_RECEIVED, &i2c_commanding_to_waiting;
  // TODO add time transitions for timeouts!

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

  digitalWrite(messagesignal_pin, LOW);  
  Serial.println("\n\n---------------\nSetup complete. MessagePin set to low. Now waiting.");

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

