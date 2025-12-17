from pyb import UART, Pin

S0_WAIT=0
S1_SEND=1
S2_INTERPRET=2

class Communication:

    def __init__(self, shares):
        self.q_pos_L, self.q_vel_L,self.q_pos_R, self.q_vel_R, self.t, self.run_flg, self.log_flg, self.ir_cal_flg, self.follow_flg, self.v_req, self.omega_req, self.motor_vel_kP, self.motor_vel_kI, self.motor_vel_kD, self.motor_vel_kFF, self.psidot_kP, self.psidot_kI, self.q_PID_e, self.q_PID_i, self.q_PID_d, self.calibrate_flg = shares
        self.state=0
        self.instruction = ""
        self.first_data = True
        self.BT_ser = UART(5, 115200)
        self.arg = 0
        self.read_buf= bytearray() # 10 byte long null array
        self.send_flag = False
        self.usr_but=Pin('PC13', mode=Pin.IN, pull=Pin.PULL_UP, value=1)

    def run(self, shares):
        while True:
            if self.state == 0: #----------------State 0--------------------
                if self.BT_ser.any():
                    #self.instruction = self.BT_ser.read(1).decode()
                    newchar=self.BT_ser.read()
                    self.read_buf.extend(newchar)
                    self.BT_ser.write(newchar)
                    buf_len=len(self.read_buf)
                    print(f"Read buffer length: {buf_len} containing: {self.read_buf.decode()}")
                    try:
                        if b'\n' in self.read_buf or b'\r' in self.read_buf or buf_len>=16:
                            self.state = S2_INTERPRET
                            self.BT_ser.write('\n\r')
                            if b':' in self.read_buf:
                                raw_instruction, raw_arg = self.read_buf.split(b":")
                                #self.arg = int.from_bytes(raw_arg.rstrip(), 'big')   # micropyhton only supports these as positional, rstip removes trailing newline
                                self.arg=int(raw_arg.decode().strip())
                            else:
                                raw_instruction=self.read_buf
                                self.arg=0
                            self.instruction=raw_instruction.decode().strip()
                            print(f"Read user instruction: {self.instruction} and argument: {self.arg}")
                    except Exception as e:
                        print(e)
                        self.read_buf=bytearray()
                if self.send_flag:
                    self.send_data()
            
            elif self.state == 1: #----------------State 1--------------------
                if self.first:
                    print(f"Queue lengths: {self.t.num_in()} {self.q_pos_L.num_in()} {self.q_vel_L.num_in()} {self.q_pos_R.num_in()} {self.q_vel_R.num_in()}")
                    self.BT_ser.write("Time,Left Position,Left Velocity,Right Position,Right Velocity,PID Error*1k,PID Integral*1k,PID Derivative*1k\r\n")
                    self.first = False
                elif self.t.any() or self.q_pos_L.any() or self.q_vel_L.any() or self.q_pos_R.any() or self.q_vel_R.any():
                    self.BT_ser.write(f"{self.t.get()},{self.q_pos_L.get()},{self.q_vel_L.get()},{self.q_pos_R.get()},{self.q_vel_R.get()},{self.q_PID_e.get()},{self.q_PID_i.get()},{self.q_PID_d.get()}\n")
                else:
                    print("End data transfer")
                    self.state=S0_WAIT

            elif self.state == 2:#----------------State 2--------------------
                if self.instruction == "":
                    self.state=S0_WAIT
                else:
                    if self.instruction.lower() == "en": #enable
                        self.run_flg.put(1)
                        self.state=S0_WAIT
                    elif self.instruction.lower() == "s": #close loop stop
                        self.run_flg.put(0)
                        self.v_req.put(0)
                        self.omega_req.put(0)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="d": #send data
                        self.send_flag = True
                        #self.state=S1_SEND
                    elif self.instruction.lower()=="c": #cancel action
                        self.run_flg.put(0)
                        self.send_flag=False
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="l": #log data
                        self.log_flg.put(1)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="sl": # stop logging data
                        self.log_flg.put(0)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="rt": # run test
                        self.log_flg.put(1)
                        self.send_flg=True
                        self.run_flg.put(1)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="r": # Run waypoint path
                        self.run_flg.put(2)
                        self.state=S0_WAIT
                    

                    elif self.instruction.lower()=="kp": #set kP
                        self.motor_vel_kP.put(self.arg)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="ki": #set kI
                        self.motor_vel_kI.put(self.arg)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="kd": #set kD
                        self.motor_vel_kD.put(self.arg)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="kff": #set kFF
                        self.motor_vel_kFF.put(self.arg)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="kpt": #set kP for turning
                        self.psidot_kP.put(self.arg)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="kit": #set kI for turning
                        self.psidot_kI.put(self.arg)
                        self.state=S0_WAIT
                        
                    elif self.instruction.lower()=="setp": #set v_req
                        self.v_req.put(self.arg)
                        self.state=S0_WAIT
                    elif self.instruction.lower()=="omega": #set omega_req
                        self.omega_req.put(self.arg/100)
                        self.state=S0_WAIT

                    elif self.instruction.lower()=="tls": #toggle line steering
                        if self.arg==1:
                            self.follow_flg.put(1)
                        elif self.arg==-1:
                            self.follow_flg.put(0)
                        elif self.follow_flg.get()>=1:
                            self.follow_flg.put(0)
                        else:
                            self.follow_flg.put(1)
                        self.state=S0_WAIT

                    elif self.instruction.lower()=="calirw": #calibrate line sensor with white
                        self.ir_cal_flg.put(1)
                        self.state=S0_WAIT
                           
                    elif self.instruction.lower()=="calirb": #calibrate line sensor with black
                        self.ir_cal_flg.put(2)
                        self.state=S0_WAIT

                    elif self.instruction.lower()=="calimu": #Save IMU calibration
                        self.calibrate_flg.put(self.arg)
                        self.state=S0_WAIT
                    elif self.usr_but.value()==0: #calimu:7 via the user button, hold for a half second
                        self.calibrate_flg.put(7)
                        print("Calimu:7 Via User Button")
                        self.BT_ser.write("Calimu:7 Via User Button")


                    else:
                        print(f"Unknown instruction: {self.instruction}")
                    self.instruction=""
                    self.read_buf=bytearray()
            yield
        
    def send_data(self):
        if self.first_data:
            print(f"Queue lengths: {self.t.num_in()} {self.q_pos_L.num_in()} {self.q_vel_L.num_in()} {self.q_pos_R.num_in()} {self.q_vel_R.num_in()}")
            self.BT_ser.write("Time,Left Position,Left Velocity,Right Position,Right Velocity,PID Error*1k,PID Integral*1k,PID Derivative*1k\r\n")
            self.first_data = False
        elif self.t.any() or self.q_pos_L.any() or self.q_vel_L.any() or self.q_pos_R.any() or self.q_vel_R.any():
            self.BT_ser.write(f"{self.t.get()},{self.q_pos_L.get()},{self.q_vel_L.get()},{self.q_pos_R.get()},{self.q_vel_R.get()},{self.q_PID_e.get()},{self.q_PID_i.get()},{self.q_PID_d.get()}\n")
        else:
            print("End data transfer")
            self.first_data = True
            self.send_flag=False
        return