import motor, PID, encoder
from pyb import Timer, Pin, ADC
from time import ticks_us

S_WAIT=0
S_CLOSEDLOOP=1

TW = 141#/25.4 #trackwidth in mm
r= 35#/25.4    #wheel radius in mm
class Mobility: 
    def __init__(self, shares, mot_L, mot_R, enc_L, enc_R):
        self.v_req, self.omega_req, self.q_pos_L, self.q_vel_L,self.q_pos_R, self.q_vel_R, self.run_flg, self.log_flg, self.t, self.motor_vel_kP, self.motor_vel_kI, self.motor_vel_kD, self.motor_vel_kFF, self.q_PID_e, self.q_PID_i, self.q_PID_d, self.volts_L, self.volts_R = shares
        self.motor_left, self.motor_right, self.encoder_left, self.encoder_right = mot_L, mot_R, enc_L, enc_R


        #initialize left and right velocity control PIDs
        self.left_velocity=PID.PID(0,self.motor_vel_kP.get(),self.motor_vel_kI.get(),self.motor_vel_kD.get(),self.motor_vel_kFF.get()+80)
        self.right_velocity=PID.PID(0,self.motor_vel_kP.get(),self.motor_vel_kI.get(),self.motor_vel_kD.get(),self.motor_vel_kFF.get())
        
        self.motor_left.enable()
        self.motor_right.enable()
        self.state=S_CLOSEDLOOP
        
        self.Vbat_pin = Pin('PA2', mode=Pin.ANALOG, value=0)

        self.Vnom = 9.6 # Nominal battery pack voltage
        self.Vbat = self.Vnom # Initialize battery voltage
        self.D = 0 # Battery droop gain
        self.bat_notified = 1
        self.loop_num = 20
        self.paused=True

        return

    def run(self, shares):
        while True:
            if self.run_flg.get()>=1:
                self.state=S_CLOSEDLOOP
            else:
                self.state=S_WAIT
                self.motor_left.disable()
                self.motor_right.disable()
                self.paused = True


            if self.log_flg.get()==1:  #--------------------------- Logging -----------------------------------
                #log data into queues
                #print(f"Left Encoder Pos: {self.encoder_left.get_position()}")
                if not self.q_pos_L.full() and not self.q_vel_L.full() and not self.t.full():
                    err,integral,deriv = self.left_velocity.performance()
                    self.q_pos_L.put(self.encoder_left.get_position())
                    self.q_vel_L.put(int(10_000_000*self.encoder_left.get_velocity())) # Convert to integer ticks per sec *10
                    self.q_pos_R.put(self.encoder_right.get_position())
                    self.q_vel_R.put(int(10_000_000*self.encoder_right.get_velocity())) # Convert to integer ticks per sec *10
                    self.t.put(ticks_us())
                    '''self.q_PID_e.put(int(err*1000))
                    self.q_PID_i.put(int(integral*1000))
                    self.q_PID_d.put(int(deriv*1000))'''
                    self.q_PID_e.put(err)
                    self.q_PID_i.put(integral)
                    self.q_PID_d.put(deriv)
                if self.t.full():
                    #remove last line from queue and replace with finality marker
                    self.t.get()
                    self.q_pos_L.get()
                    self.q_pos_R.get()
                    self.q_vel_L.get()
                    self.q_vel_R.get()

                    self.t.put(0)
                    self.q_pos_L.put(-1)
                    self.q_pos_R.put(-1)
                    self.q_vel_L.put(-1)
                    self.q_vel_R.put(-1)

                    self.log_flg.put(0)
                    print("Filled Buffer")
              #don't log data

            if self.state==S_WAIT:  #-------------------------STATE 1---------------------------
                # Motors to brake mode
                self.motor_left.set_effort(0)
                self.motor_right.set_effort(0)
            elif self.state==S_CLOSEDLOOP:   #-------------------------STATE 2-----------------------
                # Kinematics  
                # Free speed should be able to grow up to 21 in/s
                vL_req=self.v_req.get() - (TW/2)*self.omega_req.get() # mm/second
                vR_req=self.v_req.get() + (TW/2)*self.omega_req.get()

                # Current state
                #encoder velocity is in counts per microsecond
                vL_meas=self.encoder_left.get_velocity()*10**6/12*33/3952 *3.142*r*2 # mm/second
                vR_meas=self.encoder_right.get_velocity()*10**6/12*33/3952 *3.142*r*2
                
                # Battery droop compensation
                self.Vbat = ADC(self.Vbat_pin).read()/4096*3.3 / 0.32
                self.Kbat = self.Vnom / self.Vbat
                if self.Kbat<0.95 and self.bat_notified>.95:
                        print("Battery Voltage 95%")
                        self.bat_notified=.95
                elif self.Kbat<0.90 and self.bat_notified>.90:
                        print("Battery Voltage 90%")
                        self.bat_notified=.90
                elif self.Kbat<0.80 and self.bat_notified>.80:
                        print("Battery Voltage 80%")
                        self.bat_notified=.80
                elif self.Kbat<0.70 and self.bat_notified>.70:
                        print("Battery Voltage 70%")
                        self.bat_notified=.70
                elif self.Kbat<0.60 and self.bat_notified>.60:
                        print("Battery Voltage 60%")
                        self.bat_notified=.60

                # Closed loop
                if self.paused:
                    self.motor_left.enable()
                    self.motor_right.enable()
                    self.left_velocity.reset_errors()
                    self.right_velocity.reset_errors()
                    self.paused=False
                effort_L=self.left_velocity.output(vL_meas, vL_req) * self.Kbat
                effort_R=self.right_velocity.output(vR_meas, vR_req) * self.Kbat

                self.volts_L.put(effort_L/100*self.Vnom)
                self.volts_R.put(effort_R/100*self.Vnom)# 

                
                self.motor_left.set_effort(effort_L)
                self.motor_right.set_effort(effort_R)

                self.left_velocity.update_params(self.motor_vel_kP.get(),self.motor_vel_kI.get(),self.motor_vel_kD.get(),self.motor_vel_kFF.get()) # division by 10000 handled in PID
                self.right_velocity.update_params(self.motor_vel_kP.get(),self.motor_vel_kI.get(),self.motor_vel_kD.get(),self.motor_vel_kFF.get())
                '''if self.v_req.get() !=0:
                    print("Mobility success")'''
                if self.loop_num<=0:
                     print(f"VBat: {self.Vbat}, self.Kbat: {self.Kbat}")
                     self.loop_num=20
                self.loop_num-=1
            yield

