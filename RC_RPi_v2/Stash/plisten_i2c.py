#!/usr/bin/python
#
# Simple script to receive and display n raw bytes from 
# another I2C slave device connected to GPIO I2C pin.
# Check http://abyz.co.uk/rpi/pigpio/python.html for more help


import pigpio

#-------------------------#
bus = 1 #i2c bus number 0 or 1 in RPI 
addr = 0x07 #i2c slave address from 0x03 to 0x77
data_size = 2 #number of bytes to read
#-------------------------#

pi = pigpio.pi() #init pigpio

handle = pi.i2c_open(bus,addr,0) #get i2c handle

print "Handle:",handle #print i2c handle

#read <data_size> bytes from i2c <handle> and store:  
#count = number of returned bytes
#data = array of byte
(count, data) = pi.i2c_read_device(handle, data_size)

#print count
print "Count:",count

#print single bytes
for i in range(count):
	print ( "Byte{0}: {1}".format(i+1, data[i]) )

pi.i2c_close(handle); #close i2c device association with handle
pi.stop() #stop connection with raspberry GPIO
