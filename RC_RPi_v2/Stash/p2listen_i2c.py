#!/usr/bin/python

from smbus import SMBus
import time
bus = SMBus(1)
slaveAddress = 0x07
data_received_from_Arduino = ""
data_to_send_to_Arduino = "Hello Uno"

def StringToBytes(val):
    retVal = []
    for c in val:
            retVal.append(ord(c))
    return retVal


while(1):
    try:
        x = bus.read_byte(slaveAddress)

        if (x == 1):
            print(x)
            bus.write_byte(slaveAddress,1)
            time.sleep(0.2)
            data_received_from_Arduino = bus.read_i2c_block_data(slaveAddress, 0,12)
            print(data_received_from_Arduino)
            bus.write_byte(slaveAddress,3)
            print(StringToBytes(data_to_send_to_Arduino))
            bus.write_i2c_block_data(slaveAddress, 0x00,StringToBytes(data_to_send_to_Arduino))
print "test"

