#!/usr/bin/env python
#have to run 'sudo apt-get install python-smbus'
#in Terminal to install smbus
import smbus
import time
import os

import RPi.GPIO as GPIO

slaveAddress = 0x07
messagePIN = 25


GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.IN)
# GPIO.output(25, GPIO.HIGH)

# This is the address we setup in the Arduino Program
address = 0x07

def writeNumber(value):
    bus.write_byte(address, value)
    # bus.write_byte_data(address, 0, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number

def readMessageFromArduino():
    global smsMessage
    data_received_from_Arduino = i2c.read_i2c_block_data(slaveAddress, 0,15)
    for i in range(len(data_received_from_Arduino)):
        smsMessage += chr(data_received_from_Arduino[i])

    print(smsMessage.encode('utf-8'))
    data_received_from_Arduino =""
    smsMessage = ""

# display system info
print os.uname()



i2c = smbus.SMBus(1)
bus = i2c

def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted

# send welcome message at start-up
#bytesToSend = ConvertStringToBytes("Hello Uno")
#bus.write_i2c_block_data(i2c_address, i2c_cmd, bytesToSend)

while True:
    if (GPIO.input(25)):
        print ("Sleepy Pi requesting comms on pin 25")
        number = readNumber()
        print "Number:", number
        if number == 11:
            writeNumber(11)
            print "OK, code 11, confirmed. Waiting order"
    time.sleep(1)
    print "Waiting.."
print "slut"

