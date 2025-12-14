State Estimation
================

Constants and Constants
----------------------------

Imports::

    import IMU
    from os import listdir
    from ulab import numpy as np
    from time import ticks_us, ticks_diff
    filelist = listdir()

IMU Register Constants and Romi Kinematics Constants::

    CONFIG_MODE=0b0000
    IMU_MODE=0b1000
    NDOF_MODE=0b1100
    AMG_MODE= 0b0111
    tw = 141 # Romi track width (mm)
    rtire = 35 # Romi tire radius (mm)

State Estimation Matrices
--------------------------

The state estimation algorithm fuses output from several sensors to estimate Romi's position while making up for each sensors weaknesses. The Ad matrix weights the effects of the current state on the next state and the Bd matrix weights the effects of the systems inputs on the next state.
After using a MATLAB script with pole placement to determine the continuous and discrete matrices before manually tuning the matrix values to match the state estimated wheel velocities to the encoder wheel velocities in a variety of strucutred test cases (forward and reverse at low and high speeds and turning at low and high speeds). ::

    Ad = np.array([[0.41386457,0.41386441,-0.345536,0.000000],
                   [0.41386457,0.41386441,-0.345531,0.000000],
                   [0.000047,0.000047,1,0.000000],
                   [0.000000,0.000000,-0.000000,1]]) 
    Bd = np.array([[0.256138,0.244379,0.172768,0.172768,-0.000000,-2.010183],
                   [0.244501,0.256009,0.172765,0.172765,0.000000,2.010333],
                   [0.000188,0.000162,0.500121,0.500121,-0.000000,0.000009],
                   [0.000000,0.000000,-0.001132,0.001132,0,0.008647]]) 
    C = np.array([[0,0,1,-tw/2],
                  [0,0,1,tw/2],
                  [0,0,0,1],
                  [-rtire/tw,rtire/tw,0,0]])

Observer Class Init and Methods
--------------------------------

Our Romi was unable to write files to local storage so we disabled the IMU calibration saving functionality. We manually copied the calibration constants into a file that Romi does read every initialization.::

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

Generator Function
------------------

Encoder and IMU updates and interpreting user commands.::
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

Build x_hat and u_star before running the state estimation difference equation::

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
            
Diagnostic information for debugging::

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


