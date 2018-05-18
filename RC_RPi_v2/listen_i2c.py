#!/usr/bin/env python
#have to run 'sudo apt-get install python-smbus'
#in Terminal to install smbus
import smbus
import time
import os

import RPi.GPIO as GPIO

from time import sleep
from transitions import Machine
from transitions.extensions.states import add_state_features, Timeout

# Set up logging; The basic log level will be DEBUG
import logging
logging.basicConfig(level=logging.DEBUG)
# Set transitions' log level to INFO; DEBUG messages will be omitted
# logging.getLogger('transitions').setLevel(logging.INFO)


## Message codes, should be verified and the same on the Arduino side
MSG_OK = 11
MSG_READY = 22

CMD_TODO_TRIGGER = 50;
CMD_TODO_REPORT = 51;
CMD_TODO_SHUTDOWN = 52

## The timeout is the waiting time until there is a timeout for a reply on the i2c bus
WAIT = 600 #TODO set this timeout to 5



""" Use the decorator to add extensions """
@add_state_features(Timeout)
class CustomStateMachine(Machine):
    pass


class RPI_Controller(object):
    def __init__(self):
        self.todo_command = 0;

    def on_enter_waking(self):
        print("Transition: on_enter_waking.")
        data = readNumber()
        print("Data received on request:", data)
        if data == MSG_READY:
            self.rdy_received()
        if data != MSG_READY:
            self.not_ready()
        return

    def on_enter_state2(self):
        print("Transition: on_enter_state2.")
        writeNumber(MSG_READY)
        print("Er vi her?")
        while True:
            print("Maerkelig linie")
            if (not GPIO.input(25)):
               print("PIN 25 was low and now proceeding!")
               self.pinlow_received()
               print("Efter pinlow")
               break
            time.sleep(0.1)
        return

    def on_enter_state3(self):
        print("Transition: on_enter_state3.")

        # send_data(self.received_data)
        print("Now reading the command!")
        self.todo_command = readNumber()
        print("Command was:", self.todo_command)
        self.cmd_received()
        return

    def on_enter_state4(self):
        print("The state 4 here", self.todo_command)
        if self.todo_command == CMD_TODO_TRIGGER:
            print("Here we will play the SOUND AND BLINK THE LIGHTS -------------------")
            print("Now waiting 5 secs till finished!")
            time.sleep(5)

        if self.todo_command == CMD_TODO_REPORT:
            print("Here we will do some reporting! -------------------")
            time.sleep(5)

        if self.todo_command == CMD_TODO_SHUTDOWN:
	    print("Shutting down!")
            os.system("sudo shutdown -h now")

        writeNumber(MSG_OK) ## this is the confirmation of all went ok!
        self.done()
        return

    def on_enter_sleep(self):
        print("Now sleeping....")
        return

    def error_transition(self):
        print("Moving to sleep state because of error")

states = [{'name': 'sleep'},
          {'name': 'waking', 'timeout': WAIT, 'on_timeout': 'not_ready'},
          {'name': 'state2', 'timeout': WAIT, 'on_timeout': 'timeout1'},
          {'name': 'state3', 'timeout': WAIT, 'on_timeout': 'timeout2'},
          {'name': 'state4'}
          ]

#                Name, From, To
transitions = [['pin_high', 'sleep', 'waking'],
               ['not_ready', 'waking', 'waking'],
               ['rdy_received', 'waking', 'state2'],
               ['pinlow_received', 'state2', 'state3'],
               ['cmd_received', 'state3', 'state4'],
               ['done', 'state4', 'sleep'],
               ['timeout1', 'state2', 'sleep'],
               ['timeout2', 'state3', 'sleep']]

controller = RPI_Controller()
machine = CustomStateMachine(model=controller, states=states, transitions=transitions, initial='sleep')
machine.add_transition('error', source='*', dest='sleep', before='error_transition')


slaveAddress = 0x07
messagePIN = 25

GPIO.setmode(GPIO.BCM)
GPIO.setup(messagePIN, GPIO.IN)
# syntax GPIO.output(25, GPIO.HIGH)

def writeNumber(value):
    i2c.write_byte(slaveAddress, value)
    # bus.write_byte_data(address, 0, value)
    return -1

def readNumber18():
    print("Read number.... v18")
    number = 0;
    readsx = 1;
    print("Now reading")
    number=i2c.read_byte(slaveAddress)
    return number

def readNumber():
    # try up to 3 times on a failure
    success = False
    caught_exception = None
    for _ in range(200):
        try:
            print("Reading at:", slaveAddress)
            number=i2c.read_byte(slaveAddress)
            # if we get here, we succeeded, so break out of the loop
            success = True
            break
        except:
            print "error!"
            # wait a second for the retry
            time.sleep(0.1)

    if not success:
        print "Failed after X retries!"
	number = -1
    print("Number was", number)
    return number

def readNumber17():
    print("Read number.... v17")
    number = -1;
    readsx = 1;
    while True:
        try:
            print("Now reading")
            number=i2c.read_byte(slaveAddress)
            if not number == 0:
                break
        except:
            readsx += 1
            print("Exception!", readsx)
            pass
        print("try catch done")
        if not number == 0:
            print("Number received from the other side:", number)
            break
        if readsx > 10:
            print("Too many attempts failing.")
            break
        time.sleep(0.05)
        print("Weird number", number)
    print("Number was", number)
    return number

def readMessageFromArduino():
    global smsMessage
    data_received_from_Arduino = i2c.read_i2c_block_data(slaveAddress, 0,15)
    for i in range(len(data_received_from_Arduino)):
        smsMessage += chr(data_received_from_Arduino[i])

    print(smsMessage.encode('utf-8'))
    data_received_from_Arduino =""
    smsMessage = ""

i2c = smbus.SMBus(1)

def ConvertStringToBytes(src):
    converted = []
    for b in src:
        converted.append(ord(b))
    return converted

# send welcome message at start-up
#bytesToSend = ConvertStringToBytes("Hello Uno")
#bus.write_i2c_block_data(i2c_address, i2c_cmd, bytesToSend)

while True:
    if (GPIO.input(messagePIN)):
        print ("Sleepy Pi requesting comms on pin 25")
        controller.pin_high()
        print("EXIT")
#        number = readNumber()
#        print "Number:", number
#        if number == 11:
#            writeNumber(11)
#            print "OK, code 11, confirmed. Waiting order"
    time.sleep(1)
    print "Waiting.."

print "You will never see this! THE END"

