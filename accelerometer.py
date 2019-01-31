#!/usr/bin/python

import smbus
import math
import time



class Accelerometer(object):
    
    def __init__(self):
        # Power management registers
        self.power_mgmt_1 = 0x6b
        self.power_mgmt_2 = 0x6c
         
        self.bus = smbus.SMBus(0) #bus = smbus.SMBus(0) # or bus = smbus.SMBus(1) for Revision 2 boards #smbus.SMBus(0) b/c using i2c0
        self.address = 0x68       # This is the address value read via the i2cdetect command

    def read_byte(self, adr):
        return self.bus.read_byte_data(self.address, adr)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        rTime = time.time() #to get time of reading
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val, rTime


    def readACC(self):
        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)
        
        #it may be necessary to add a slight delay here, actually itll be awake after calibration

##        print ("accelerometer data")
##        print ("------------------")

        #accel_xout = self.read_word_2c(0x3b)
        #accel_yout = self.read_word_2c(0x3d)
        accel_zout, readTime = self.read_word_2c(0x3f)

        #accel_xout_scaled = accel_xout / 16384.0
        #accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = -1*(((accel_zout / 16384.0)*9.8) - (2*9.8))

        #print ("accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled)
        #print ("accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled)
#        print ("accel_zout: ", accel_zout_scaled, "  -->  ", readTime)

        return accel_zout_scaled, readTime
##
##acc = Accelerometer()
##tm
##for i in range(0,25):
##    press, time = acc.readACC()
##    time.sleep(0.5)

