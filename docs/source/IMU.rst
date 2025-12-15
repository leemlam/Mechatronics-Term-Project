IMU
====

Imports and Init::

    from pyb import Pin, Timer, ADC, I2C
    from struct import unpack

    Y_OFFSET = int(1.45*25.4) #radius from IMU to Romi Center [mm]
    X_OFFSET= int(1.8*25.4)
    TAU=3.14159265*2

    CALIB_STAT_REG=0x35
    CALIB_COEFF_REG=0x55
    EULER_REG=0x1A
    OP_MODE_REG=0x3D
    MAG_RAD_REG=0x69
    ACC_RAD_REG=0x67
    MAG_X_OFFSET_REG= 0x5b
    MAG_y_OFFSET_REG= 0x5d
    GYR_X_OFFSET_REG= 0x61
    GYR_y_OFFSET_REG= 0x63
    GYR_X_REG=0x14
    GYR_Y_REG=0x16
    GYR_Z_REG=0x18
    UNIT_SEL_REG=0x3B

    class IMU:
        def __init__(self):
            self.address = 0x28 # Address of IMU in hex
            self.timeout = 5000 # Read/Write timeout in ms
            self.i2c=I2C(1, I2C.CONTROLLER) # create bus 1
            self.i2c.init(I2C.CONTROLLER) # Initialize as controller
            self.buffer = bytearray(0 for i in range(6)) # Allocate 6 bytes to read


            #print(self.i2c.scan())
            if self.i2c.is_ready(self.address):
                print(f"IMU Responded to address {self.address}")
            else:
                print(f"IMU not responding to address {self.address}")

            #set units and offset
            self.i2c.mem_write(X_OFFSET, self.address, MAG_X_OFFSET_REG, timeout=self.timeout, addr_size=8)
            self.i2c.mem_write(X_OFFSET, self.address, GYR_X_OFFSET_REG, timeout=self.timeout, addr_size=8)
            self.i2c.mem_write(Y_OFFSET, self.address, MAG_y_OFFSET_REG, timeout=self.timeout, addr_size=8)
            self.i2c.mem_write(Y_OFFSET, self.address, GYR_y_OFFSET_REG, timeout=self.timeout, addr_size=8)
            units=self.i2c.mem_read(2, self.address, UNIT_SEL_REG, timeout=self.timeout, addr_size=8)
            unit_mask=0b00000110
            self.i2c.mem_write(units[0] | unit_mask, self.address, MAG_RAD_REG, timeout=self.timeout, addr_size=8) #bitwise or to set mag and gyro to radians

            self.heading_offset=0
            self.set_heading(0)

Maintenance and individual measurement methods::

        def changemode(self, mode):
            # ---- Convert mode setting to register value ------
            self.i2c.mem_write(mode, self.address, OP_MODE_REG, timeout=self.timeout, addr_size=8)

        def cal_stat(self): # Reads calibration status byte
            stat = self.i2c.mem_read(1, self.address, CALIB_STAT_REG, timeout=self.timeout, addr_size=8)
            
            self.mag = stat[0] & 0b11
            self.acc = (stat[0] >> 2) & 0b11
            self.gyr = (stat[0] >> 4) & 0b11
            self.sys = (stat[0] >> 6) & 0b11

            return self.mag, self.acc, self.gyr, self.sys
        
        def cal_coeff(self):
            coeffs = self.i2c.mem_read(18, self.address, CALIB_COEFF_REG, timeout=self.timeout, addr_size=8)
            self.acc_x, self.acc_y, self.acc_z, self.mag_x, self.mag_y, self.mag_z, self.gyr_x, self.gyr_y, self.gyr_z = unpack("<hhhhhhhhh", coeffs)
            return self.acc_x, self.acc_y, self.acc_z, self.mag_x, self.mag_y, self.mag_z, self.gyr_x, self.gyr_y, self.gyr_z

        def set_cal_consts(self, data): #
            self.i2c.mem_write(bytearray(data), self.address, CALIB_COEFF_REG, timeout=self.timeout, addr_size=8)
            

        def euler(self): #gets euler heading data
            self.eul = unpack("<h", self.i2c.mem_read(2, self.address, EULER_REG, timeout=self.timeout, addr_size=8))
            return self.eul

        def omega(self):
            self.psidot=unpack("<h", self.i2c.mem_read(2, self.address, GYR_Z_REG, timeout=self.timeout, addr_size=8))[0]/900#convert to radians
            return self.psidot

        
        def heading(self):
        head=tuple(TAU-x/900+self.heading_offset for x in self.euler()) #convert to radians
        return head[0]
        
        def set_heading(self, new_heading):
            old_heading= self.heading()
            self.heading_offset+=new_heading-old_heading
