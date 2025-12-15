Communication Protocol
======================

The communication task handles bluetooth communication bewteen Romi and the user, allowing the user to command Romi, adjust parameters, and receive batched data sets. The task runs at a higher frequency and lower priority than many of the other tasks to allow as much comminication as possible without interfering with other funcitonality. The frequency is limited by the bluetooth throughput speed, ecxeeding the current frequency can lead to missing information.
Imports and Init::

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

Multicharacter User Interface with Numerical Arguments
------------------------------------------------------

Generator Function Sate 0, handles waiting for input and parsing multicharacter inputs from the user. The user enters a short command, optionally followed by a colon and a positive or negative integer, that gets interpreted as an argument.
The bluetooth serial buffer is read into an individual new character, echoed to the user, and appended to a running bytearray buffer. Once the buffer reaches a set length or contains an enter, the buffer is split into a string instruction and integer argument. There is no support for backspace so any unintentional inputs should be followed by random characters to allow the buffer to reset.::

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
                
State 1 handles sending batched data back to the user. It checks that the queues have data to send and sends one line at a time. ::

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

Instruction Interpretation
--------------------------
State 2 interprets the user's instruction and sets relevant flags to facilitate inter-task communication. Often the default argument is zero but it can be specified by the user allowing very easy PID tuning without flashing new code, which would require our team to wipe and reinstall firmware every time. 
This section includes commands to enable and disable motors, specify setpoints for forward velocity and yaw rate, specify  :doc:`PID` constants for the :doc:`motor` PIDs and :doc:`line-sensing` PID, send data to the user, calibrate sensors, enabling and disabling line following, and run planned routines. In particular, the arguments for "calimu" all perform different fuctions in the :doc:`state-estimation` task inlcuding changing the IMU mode, setting the heading offsets, toggling diagnostic output, and resetting the estimated state. 
To start the term project course, Romi must be placed in the starting profile and aligned with the X axis. The user then enters "calimu:7" to reset the heading and set the estimated X and Y coordinates to 100,800 and "r" to make Romi start the course. Once Romi completes the course or if it must be stopped early, the user sends "s" or "c" to disable the motors.

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

 Method to empty queues into the serial stream to send data to the user::
               
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