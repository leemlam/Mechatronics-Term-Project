import IRSensor
import PID
from math import atan

S_WAIT=0
S_CALW=1
S_CALB=2
S_READ=3

sensor_radius = 75 #closest distance from center of romi to sensor mm

class Line_Sensor:

    def __init__(self, shares):
        self.omega_req, self.ir_cal_flg, self.follow_flg, self.psidot_kP, self.psidot_kI = shares
        self.state=S_WAIT
        self.cal_timer = 1000
        self.sensor = IRSensor.IRSensor(self.omega_req)
        self.ndebug = 20
        self.omega_controller = PID.PID(0, self.psidot_kP.get(), 0, 0, 0)
        self.transition_to_wait=True

    def run(self, shares):
        while True:
            
            if self.ir_cal_flg.get() == 1:
                self.state = S_CALW
                print("White Calibration State")
            elif self.ir_cal_flg.get() == 2:
                self.state = S_CALB
                print("Black Calibration State")
            elif self.follow_flg.get()==1:
                self.state=S_READ
            else:
                self.state = S_WAIT
            
            if self.state == S_WAIT: #----------------State 0--------------------
                if self.transition_to_wait:
                    #self.omega_req.put(0)
                    self.transition_to_wait=False

            elif self.state == S_CALW: #----------------State 1--------------------
                self.sensor.calibratewhite()
                print("White Calibrated")
                self.ir_cal_flg.put(0)
            
            elif self.state == S_CALB: #----------------State 2--------------------
                self.sensor.calibrateblack()
                print("Black Calibrated")
                self.ir_cal_flg.put(0)

            elif self.state == S_READ:#----------------State 3--------------------
                self.sensor_reading, self.average = self.sensor.update_vals()
                self.angle = atan(self.sensor_reading / sensor_radius)
               
                self.omega_controller.update_params(self.psidot_kP.get(), self.psidot_kI.get(),0)
                
                if self.average >= 0.15: # 0.2 for circle, 0.1 for big sheet
                    self.omega_req.put(self.omega_controller.output(self.sensor_reading))
                else:
                    self.omega_req.put(0)

                if self.ndebug <= 0:
                    print(f"Centroid: {self.sensor_reading}, Angle: {self.angle}, Average: {self.average}")
                    print(f"Omega req: {self.omega_req.get()}")
                    self.ndebug = 20
                self.ndebug -= 1
                self.transition_to_wait=True
            yield
