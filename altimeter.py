from smbus import SMBus
import time


class Altimeter(object):
    
    def __init__(self):
        
        # Special Chars
        self.deg = u'\N{DEGREE SIGN}'

        # I2C Constants
        self.ADDR = 0x60
        self.CTRL_REG1 = 0x26
        PT_DATA_CFG = 0x13
        self.bus = SMBus(1)
        
        # Set oversample rate to 128
        setting = self.bus.read_byte_data(self.ADDR, self.CTRL_REG1)
        newSetting = setting | 0x38
        self.bus.write_byte_data(self.ADDR, self.CTRL_REG1, newSetting)

        # Enable event flags
        self.bus.write_byte_data(self.ADDR, PT_DATA_CFG, 0x07)
    
    def readALT(self):
        
        # Toggel One Shot
        setting = self.bus.read_byte_data(self.ADDR, self.CTRL_REG1)
        if (setting & 0x02) == 0:
            self.bus.write_byte_data(self.ADDR, self.CTRL_REG1, (setting | 0x02))

        # Read sensor data
        #print ("Waiting for data...")
        status = self.bus.read_byte_data(self.ADDR,0x00)
        while (status & 0x08) == 0:
            #print bin(status)
            status = self.bus.read_byte_data(self.ADDR,0x00)
            time.sleep(0.003)

        #print ("Reading sensor data...")
        p_data = self.bus.read_i2c_block_data(self.ADDR,0x01,3)
        t_data = self.bus.read_i2c_block_data(self.ADDR,0x04,2)
        status = self.bus.read_byte_data(self.ADDR,0x00)
        
        altReadTime = time.time()

        p_msb = p_data[0]
        p_csb = p_data[1]
        p_lsb = p_data[2]
        t_msb = t_data[0]
        t_lsb = t_data[1]

        pressure = (p_msb << 10) | (p_csb << 2) | (p_lsb >> 6)
        p_decimal = ((p_lsb & 0x30) >> 4)/4.0
        
        pressure = pressure + p_decimal

        kelvin = (t_msb + (t_lsb >> 4)/16.0) + 273.15

##        print (str(pressure+p_decimal)+" Pa")
##        print (str(kelvin)+self.deg)
        
        #print (str(celsius)+deg+"C")
        
        return altReadTime, pressure, kelvin

##
##alt = Altimeter()
##for i in range(0,15):
##    
##    alt.readALT()
#print (press)
#print (temp)


