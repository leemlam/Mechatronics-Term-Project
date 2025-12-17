import IMU
from os import listdir
from ulab import numpy as np
from time import ticks_us, ticks_diff

filelist = listdir()

CONFIG_MODE=0b0000
IMU_MODE=0b1000
NDOF_MODE=0b1100
AMG_MODE= 0b0111
tw = 141 # Romi track width (mm)
rtire = 35 # Romi tire radius (mm)

'''Ad = np.array([[0.2947,0.2885,-0.0165,-0.0000],
               [0.2885,0.2951,-0.0165,-0.0000],
               [0.1120,0.1122,0.0003,0.0000],
               [0.0000,	0.0000,	0.0000,	0.6065]])'''
'''Ad = np.array([[1.0184e-22,	1.0184e-22,	-5.7668e-21,	2.2743e-33],
               [1.0184e-22,	1.0184e-22,	-5.7668e-21,	2.2742e-33],
               [3.5678e-25,	3.5678e-25,	-2.0204e-23,	9.0761e-36],
               [4.1808e-37,	4.1003e-37,	-2.2961e-35,	1.9287e-22]])'''
'''Ad = np.array([[0.6061,	0.0021,	0.0012,	-0.0000],
               [0.0058,	0.5798,	-0.0149,	-0.0000],
               [0.1254,	0.1020,	0.0040,	0.0000],
               [0.0000,	-0.0000,	0.0000,	0.0067]])'''
'''Ad = np.array([[-0.0043,	-0.0043,	0.0331,	0.0000],
               [-0.0043,	-0.0043,	0.0331,	0.0000],
               [-0.0013,	-0.0013,	0.0046,	-0.0000],
               [0.0000,	0.0000,	-0.0000,	0.0000]])
Ad = np.array([[0.2888,	0.2888,	-0.0165,	0.0000],
              [0.2888,	0.2888,	-0.0165, 0.0000],
              [0.0102,	0.0102,	-0.0006,	0.0000],
              [0.0000,	0.0000,	0.0000,	0.6065]])
Ad = np.array([[0.0032,	0.0032,	-0.0168,	0.0000],
               [0.0032,	0.0032,	-0.0168,	0.0000],
               [0.0000,	0.0000,	-0.0001,	0.0000],
               [0.0000,	0.0000,	-0.0000,	0.0067]])'''
#Ad = np.array([[0.288478,0.288478,0.000101,0.000000],[0.288478,0.288478,0.000101,0.000000],[-0.016484,-0.016484,-0.000006,0.000000],[0.000000,0.000000,0.000000,0.606531]]) #p=[-10.000000,-11.000000,-100000.000000,-100001.000000]
#Ad = np.array([[0.288504,0.288504,0.00101 1,0.000000],[0.288504,0.288504,0.001011,0.000000],[-0.016486,-0.016486,-0.000058,0.000000],[-0.000000,-0.000000,0.000000,0.606531]]) #p=[-10.000000,-11.000000,-10000.000000,-10001.000000]
#Ad = np.array([[0.471165,0.338616,0.130913,0.000000],[0.338710,0.471538,0.131439,-0.000000],[-0.019474,-0.019081,0.127806,0.000000],[-0.000000,0.000000,0.000000,0.818731]]) #p=[-10.000000,-11.000000,-100.000000,-101.000000]
#Ad = np.array([[0.471165,0.338616,0.,0.000000],[0.338710,0.471538,0,-0.000000],[-0.019474,-0.019081,0.127806,0.000000],[-0.000000,0.000000,0.000000,0.818731]]) #p=[-10.000000,-11.000000,-100.000000,-101.000000]
#Ad = np.array([[0.402100,0.402058,0.028718,0.000000],[0.402058,0.402104,0.028718,-0.000000],[-0.022975,-0.022975,-0.001596,0.000000],[0.000000,0.000000,0.000000,0.818731]]) #p=[-10.000000,-11.000000,-500.000000,-501.000000]
#Ad = np.array([[0.066457,0.066456,0.000047,0.000000],[0.066441,0.066440,0.000047,0.000000],[-0.345536,-0.345531,-0.000242,-0.000000],[0.000000,0.000000,0.000000,0.135335]]) #p=[-100.000000,-101.000000,-50000.000000,-5001.000000]
#Ad = np.array([[0.40386457,0.40386441,-0.345536,0.000000],[0.40386457,0.40386441,-0.345531,0.000000],[0.000047,0.000047,1,0.000000],[0.000000,0.000000,-0.000000,0.135335]]) #p=[-100.000000,-101.000000,-50000.000000,-5001.000000]
Ad = np.array([[0.41386457,0.41386441,-0.345536,0.000000],[0.41386457,0.41386441,-0.345531,0.000000],[0.000047,0.000047,1,0.000000],[0.000000,0.000000,-0.000000,1]]) #p=[-100.000000,-101.000000,-50000.000000,-5001.000000]
'''Bd = np.array([[1.4138,0.8420,0.0082,0.0082,-0.0000,-1.8038],
               [0.8418,1.4147,0.0082,0.0082,0.0000,1.8026],
               [0.3257,0.3269,0.4998,0.4998,-0.0000,-0.0006],
               [0.0000,-0.0000,-0.0028,0.0028,0.0000,0.0393]])'''
'''Bd = np.array([[0.0348,	0.0290,	0.0000,	0.0000,	-0.0000,	-2.0123],
               [0.0290,	0.0348,	0.0000,	0.0000,	0.0000,	2.0123],
               [0.0001,	0.0001,	0.5000,	0.5000,	-0.0000,	-0.0000],
               [0.0000,	0.0000,	-0.0071,	0.0071,	0.0001,	0.0010]])
Bd = np.array([[2.2880,	0.0035,	-0.0006,	-0.0006,	-0.0000,	-0.0068],
               [0.0137,	2.2455,	0.0074,	0.0074,	0.0000,	0.0853],
               [0.3584,	0.2971,	0.4980,	0.4980,	0.0000,	0.0684],
               [0.0000,	-0.0000,	-0.0070,	0.0070,	0.0001,	0.0099]])
Bd = np.array([[0.3408,	0.2825,	-0.0165,	-0.0165,	-0.0000,	-1.9941],
               [0.2825,	0.3408,	-0.0165, -0.0165,	0.0000,	1.9941],
               [0.0578,	0.0578,	0.4977,	0.4977,	-0.0000,	-0.0000],
               [-0.0000,	-0.0000,	-0.0071,	0.0071,	0.0001,	0.0010]])
Bd = np.array([[1.1489,	1.0908,	0.0083,	0.0083,	-0.0000,	-1.9942],
               [1.0908,	1.1489,	0.0083,	0.0083,	0.0000,	1.9942],
               [0.0386,	0.0386,	0.5003,	0.5003,	-0.0000,	-0.0000],
               [0.0000,	-0.0000,	-0.0028,	0.0028,	0.0000,	0.0393]])
Bd = np.array([[0.2917,	0.2859,	0.0084,	0.0084,	-0.0000,	-2.0123],
               [0.2859,	0.2917,	0.0084,	0.0084,	0.0000,	2.0123],
               [0.0010,	0.0010,	0.5000,	0.5000,	-0.0000,	-0.0000],
               [0.0000,	0.0000,	-0.0070,	0.0070,	0.0001,	0.0099]])'''
#Bd = np.array([[1.119032,1.118450,0.000391,0.000000,1.118450,1.119032],[0.000391,-0.000000,0.008242,0.008242,0.500003,-0.002790],[0.008242,0.008242,0.500003,0.002790,-0.000000,0.000000],[-0.000000,0.000040,-2.014084,2.014084,-0.000000,0.039347]]) #p=[-10.000000,-11.000000,-100000.000000,-100001.000000]
#Bd = np.array([[1.121750,1.115933,0.003910,0.000000,1.115933,1.121750],[0.003910,-0.000000,0.008243,0.008243,0.500029,-0.002790],[0.008243,0.008243,0.500029,0.002790,-0.000000,0.000000],[-0.000000,0.000040,-2.012272,2.012272,-0.000000,0.039347]]) #p=[-10.000000,-11.000000,-10000.000000,-10001.000000]
#Bd = np.array([[0.774956,0.275385,0.106134,0.000000,0.275504,0.775228],[0.106768,-0.000000,0.009737,0.009541,0.436097,-0.001285],[0.009737,0.009541,0.436097,0.001285,-0.000000,0.000000],[-0.000000,0.000018,-1.574558,1.573481,-0.001278,0.018127]]) #p=[-10.000000,-11.000000,-100.000000,-101.000000]
#Bd = np.array([[0.774956,0.275385,0,0.000000,0.275504,0.775228],[0.106768,-0.000000,0,0,0.436097,-0.001285],[0.009737,0.009541,0.436097,0.001285,-0.000000,0.000000],[-0.000000,0.000018,-1.574558,1.573481,-0.001278,0.018127]]) #p=[-10.000000,-11.000000,-100.000000,-101.000000]
#Bd = np.array([[0.581233,0.465117,0.033211,0.000000,0.465117,0.581237],[0.033218,-0.000000,0.011487,0.011487,0.500798,-0.001285],[0.011487,0.011487,0.500798,0.001285,-0.000000,0.000000],[-0.000000,0.000018,-1.973997,1.973988,-0.000003,0.018127]]) #p=[-10.000000,-11.000000,-500.000000,-501.000000]
#Bd = np.array([[0.256138,0.244501,0.000188,0.000000,0.244379,0.256009],[0.000162,0.000000,0.172768,0.172765,0.500121,-0.006132],[0.172768,0.172765,0.500121,0.006132,-0.000000,0.000000],[-0.000000,0.000087,-2.010183,2.010333,0.000009,0.008647]]) #p=[-100.000000,-101.000000,-50000.000000,-5001.000000]
#Bd = np.array([[0.256138,0.244379,0.172768,0.172768,-0.000000,-2.010183],[0.244501,0.256009,0.172765,0.172765,0.000000,2.010333],[0.000188,0.000162,0.500121,0.500121,-0.000000,0.000009],[0.000000,0.000000,-0.006132,0.006132,0.000087,0.008647]]) #p=[-100.000000,-101.000000,-50000.000000,-5001.000000]
Bd = np.array([[0.256138,0.244379,0.172768,0.172768,-0.000000,-2.010183],[0.244501,0.256009,0.172765,0.172765,0.000000,2.010333],[0.000188,0.000162,0.500121,0.500121,-0.000000,0.000009],[0.000000,0.000000,-0.001132,0.001132,0,0.008647]]) #p=[-100.000000,-101.000000,-50000.000000,-5001.000000]
C = np.array([[0,0,1,-tw/2],[0,0,1,tw/2],[0,0,0,1],[-rtire/tw,rtire/tw,0,0]])

class Observer:
    def __init__(self, shares, period, encoder_left, encoder_right):
        (self.calibrate_flg, self.t, self.q_pos_L, self.q_vel_L, self.q_pos_R, self.q_vel_R, self.v_req, self.omega_req, self.volts_L, self.volts_R, self.s_X, self.s_Y, self.s_X_dot, self.s_Y_dot, self.psi) = shares
        self.imu=IMU.IMU()
        self.period=period
        self.encoder_left=encoder_left
        self.encoder_right=encoder_right
        self.X = 100
        self.Y = 800
        self.psi.put(self.imu.heading())

        self.loop_count=4
        self.obs_output_on=True
        #check if calibration profile exists
        # if not profile, perform calibration
        self.calibrate_flg.put(0)
        data=()
        if "IMU_Calibration.txt" in filelist:
            with open ("IMU_Calibration.txt", "r") as file:
                dummy = file.readline() #skip header line
                lines = file.readlines()
            self.imu.changemode(CONFIG_MODE)
            '''for line in data:
                split_line= line.strip()
                data=data+int(split_line)'''
            data=tuple(int(line.strip()) for line in lines)
            #acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,gyr_x,gyr_y,gyr_z=data
            print(f"Read IMU calibration constants as: {data}")
            self.imu.set_cal_consts(data[:9])
            self.calibrate_flg.put(2)
        self.imu.changemode(NDOF_MODE)
        print(f"IMU Calibration Status: {self.imu.cal_stat()}")
        self.last_t=ticks_us()

    def save_data(self,data): # Writes data from txt
        print(filelist)
        acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,gyr_x,gyr_y,gyr_z=data
        '''with open("IMU_Calibration.txt", "w") as file:
            file.write("IMU Calibration Constants")
            #for x in [acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z]:
             #   file.write(str(x)+"\n")
        print("Saved Calibration Data")'''
        print("Saving files currently bricks Romi, stop it")


    def run(self, shares):
        while True:
            now=ticks_us()
            self.dt=ticks_diff(now, self.last_t)
            self.last_t=now
            self.heading = self.imu.heading()#make local variables for this iteration to minimize I2C utilization
            self.omega = self.imu.omega()

            vel_L=self.encoder_left.get_velocity()*10**6/12*33/3952 *3.142*rtire*2
            vel_R=self.encoder_right.get_velocity()*10**6/12*33/3952 *3.142*rtire*2

            pos_L=self.encoder_left.get_position()/12*33/3952 *3.142*rtire*2
            pos_R=self.encoder_right.get_position()/12*33/3952 *3.142*rtire*2

            if self.calibrate_flg.get()==1: # Switch to config mode to read and save calibration settings
                self.imu.changemode(CONFIG_MODE) 
                cal_coefs=self.imu.cal_coeff()
                print(cal_coefs)
                #self.save_data(cal_coefs)
                self.calibrate_flg.put(2)
                self.imu.changemode(NDOF_MODE)

            elif self.calibrate_flg.get()==3: # Switch to AMG (all sensors) mode to read and save calibration settings
                self.imu.changemode(AMG_MODE)

            elif self.calibrate_flg.get()==4:
                print(f"IMU Calibration Status: {self.imu.cal_stat()}")
                print(f"Euler Heading: {self.imu.heading()} radians")
                print(f"Gyro Psi Dot: {self.omega} radians/sec")
                self.calibrate_flg.put(2)

            elif self.calibrate_flg.get()==5: # Set heading to 0
                self.imu.set_heading(0)
                self.calibrate_flg.put(2)

            elif self.calibrate_flg.get()==6: # Toggle observer output
                if self.obs_output_on:
                    self.obs_output_on=False
                else:
                    self.obs_output_on=True
                self.calibrate_flg.put(2)

            elif self.calibrate_flg.get()==7: # Set position to (0,0)
                self.X = 100
                self.Y = 800
                self.x_hat=[0,0,0,0]
                self.u_star=[0,0,0,0,0,0]
                self.y_hat=[0,0,0,0]
                self.encoder_left.zero()
                self.encoder_right.zero()
                print("Position zeroed")
                self.calibrate_flg.put(5)


            elif self.calibrate_flg.get()==-1: # Switch to config mode
                self.imu.changemode(CONFIG_MODE)


            self.x_hat = np.array(((vel_L / rtire), (vel_R / rtire), ((pos_L + pos_R) / 2), (self.heading)))
            self.u_star = np.array(((self.volts_L.get()), (self.volts_R.get()), (pos_L), (pos_R), (self.heading), (self.omega)))
            
            self.y_hat = np.dot(C, self.x_hat)
            self.x_hat = np.dot(Ad, self.x_hat) + np.dot(Bd, self.u_star)

            
            self.V_L = self.x_hat[0] * rtire # Left and right tangential wheel velocities
            self.V_R = self.x_hat[1] * rtire

            self.V = (self.V_L + self.V_R) / 2 # Romi velocity

            self.X_dot = self.V*np.cos(self.heading) # Update absolute velocities
            self.Y_dot = self.V*np.sin(self.heading)

            self.X += self.X_dot *self.dt/1_000_000  # Update positions based on velocities
            self.Y += self.Y_dot *self.dt/1_000_000

            self.s_X.put(self.X)
            self.s_Y.put(self.Y)
            self.s_X_dot.put(self.X_dot)
            self.s_Y_dot.put(self.Y_dot)
            self.psi.put(self.heading)
            
            if self.loop_count<=0 and self.obs_output_on:
                self.loop_count=20
                '''print(f"IMU Calibration Status: {self.imu.cal_stat()}")
                print(f"Euler Heading: {self.imu.heading()} radians")
                print(f"Gyro Psi Dot: {self.imu.omega()} radians/sec")
                print(f"x_hat=|Omega_L={self.x_hat[0]}|  y_hat=|    S_L={self.y_hat[0]}|")
                print(f"      |Omega_R={self.x_hat[1]}|        |    S_R={self.y_hat[1]}|")
                print(f"      |      s={self.x_hat[2]}|        |    Psi={self.y_hat[2]}|")
                print(f"      |    Psi={self.x_hat[3]}|        |Psi_dot={self.y_hat[3]}|")'''
                Ax=np.dot(Ad, self.x_hat)
                Bu=np.dot(Bd, self.u_star)
                print(f"   Ax=|Omega_L={Ax[0]}|   Bu=|Omega_L={Bu[0]}|")
                print(f"      |Omega_R={Ax[1]}|      |Omega_R={Bu[1]}|")
                print(f"      |      s={Ax[2]}|      |      s={Bu[2]}|")
                print(f"      |    Psi={Ax[3]}|      |    Psi={Bu[3]}|")
                print("")
                #print(f"{pos_L},{pos_R},{self.y_hat[0]},{self.y_hat[1]},{self.omega},{self.y_hat[3]}")
                print(f"ENCODER  - Left Vel: {vel_L}, Right Vel: {vel_R}")
                print(f"OBSERVER - Left Vel: {self.V_L}, Right Vel: {self.V_R}, Vel: {self.V}")
                print("")
                print(f"X dot: {self.X_dot}, Y dot: {self.Y_dot}")
                print(f"X pos: {self.X}, Y pos: {self.Y}")
                print(f"left effort: {self.volts_L.get()}, right effort: {self.volts_R.get()}")
                print("")

            else:
                self.loop_count-=1
            yield