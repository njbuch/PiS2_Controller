/*
  This is the rfirst example of a sketch that tries to support a handshake
  between the Rpi3 and the arduino on the sleepy pi 2.
  
  What works:
  Can be deployed and starts blinking the LED
  
  when running the rpi3 command: "i2cget -y 1 0x07" it returns an increasing number to the rpi3
  when running the rpi3 command: "i2cset -y 1 0x07 0x11" it prints 17 to the serial interface monitor in Arduino IDE
 
 TODO: Merge with https://github.com/SpellFoundry/SleepyPi2/blob/master/examples/ButtonOnOff3/ButtonOnOff3.ino
 TODO: Establish a protocol / statemachine that adresses the commands needed 
 TODO: Convert existing mechanics control to https://github.com/jonblack/arduino-fsm
 
 */
 
#include <Wire.h> 
#include "Arduino.h"
 
static char c = '0';

// function that executes whenever data is received from master 
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
     char c = Wire.read(); // receive byte as a character
     Serial.print(c);         // print the character
     }
 int x = Wire.read();    // receive byte as an integer
 Serial.println(x);
  
}


// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
void setup() {                
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

