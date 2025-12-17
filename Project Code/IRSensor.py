from pyb import Pin, Timer, ADC 
import array
samples = 10
spacing = 4 #mm

class IRSensor:
    def __init__(self, omega_req):
        self.channels=[ADC(Pin("PC1", mode=Pin.ANALOG)),
                       ADC(Pin("PC0", mode=Pin.ANALOG)),
                       ADC(Pin("PC3", mode=Pin.ANALOG)),
                       ADC(Pin("PC2", mode=Pin.ANALOG)),
                       ADC(Pin("PB0", mode=Pin.ANALOG)),
                       ADC(Pin("PB1", mode=Pin.ANALOG)),
                       ADC(Pin("PA5", mode=Pin.ANALOG)),
                       ADC(Pin("PA6", mode=Pin.ANALOG)),
                       ADC(Pin("PA7", mode=Pin.ANALOG)),
                       ADC(Pin("PC4", mode=Pin.ANALOG)),
                       ADC(Pin("PC5", mode=Pin.ANALOG))]
        self.num_chan=len(self.channels)
        self.raw_vals=[array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples))),
                      array.array('H', (0 for i in range(samples)))]
        self.norm_vals = [0.0 for i in range(self.num_chan)]
        self.omega_req = omega_req
        self.sample_flg = 0
        self.tim=Timer(15, freq=1000)
        self.black_cal=[3460.3,3192.7,3295.1,3134.9,2442.8,2336.0,2034.7,2531.2,1555.3,1919.8,2838.7]
        self.white_cal=[1806.9,1514.5,1787.5,1505.5,1124.1,625.9,483.7,804.1,332.0,664.9,1461.6]
        self.centroid=0
        self.white_average = 0
        self.black_average = 2000

    def run(self):
        while True:
            if self.sample_flg >=1:
                self.update_vals
            yield
    
    # IMPLEMENT SAVING CALIBRATED DATA TO A SEPARATE FILE ---------------------------

    def calibratewhite(self):
        ADC.read_timed_multi(self.channels,self.raw_vals, self.tim)

        self.white_cal = [sum(vals)/samples for vals in self.raw_vals]
        self.white_average = sum(self.white_cal) / self.num_chan

        # Calibrate white and black
        print(f"White Cal Values: {self.white_cal}")
        return
    
    def calibrateblack(self):
        ADC.read_timed_multi(self.channels,self.raw_vals, self.tim)

        self.black_cal = [sum(vals)/samples for vals in self.raw_vals]
        self.black_average = sum(self.black_cal) / self.num_chan

        # Calibrate white and black
        print(f"Black Cal Values: {self.black_cal}")
        return
    
    def update_vals(self):
        ADC.read_timed_multi(self.channels,self.raw_vals, self.tim)

        self.avg_vals = [sum(vals)/samples for vals in self.raw_vals]
        self.average = sum(self.avg_vals) / self.num_chan
        self.average = (self.average - self.white_average) / (self.black_average - self.white_average)

        #interpolate with raw vals, white cal, and black cal
        for i in range(self.num_chan):
            self.norm_vals[i] = (self.avg_vals[i] - self.white_cal[i]) / (self.black_cal[i]-self.white_cal[i])

        #find centroid and weight
        total = 0
        weighted = 0
        for i in range(self.num_chan):
            total+=self.norm_vals[i]
            weighted+=self.norm_vals[i]*(i-((self.num_chan-1)/2))*(spacing)
        self.centroid = -weighted / total
        return self.centroid, self.average