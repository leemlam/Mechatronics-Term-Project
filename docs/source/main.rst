::
    """!
    @file basic_tasks.py
        This file contains a demonstration program that runs some tasks, an
        inter-task shared variable, and a queue. The tasks don't really @b do
        anything; the example just shows how these elements are created and run.

    @author JR Ridgely
    @date   2021-Dec-15 JRR Created from the remains of previous example
    @copyright (c) 2015-2021 by JR Ridgely and released under the GNU
        Public License, Version 2. 
    """

    import gc
    import cotask
    import task_share
    import mobility
    import communication
    import line_sensor
    import micropython
    import observer
    import PathPlanner

    from pyb import Timer
    import motor, encoder

    micropython.alloc_emergency_exception_buf(100)

    def encoder_update(tim):
        encoder_left.update()
        encoder_right.update()

    # This code creates a share, a queue, and two tasks, then starts the tasks. The
    # tasks run until somebody presses ENTER, at which time the scheduler stops and
    # printouts show diagnostic information about the tasks, share, and queue.
    if __name__ == "__main__":
        print("Testing ME405 stuff in cotask.py and task_share.py\r\n"
            "Press Ctrl-C to stop and show diagnostics.")

        mobility_freq=20 #Hz

        tim3=Timer(3, freq=30000) # Create timer object for motors
        motor_left=motor.Motor('PC6','D5','D4',tim3,1)
        motor_right=motor.Motor('PC7','D3','D2',tim3,2)

        tim1=Timer(1, prescaler=0, period=65535) # Create timer object for encoders
        tim2=Timer(2, prescaler=0, period=65535) # Create timer object for encoders
        encoder_left=encoder.Encoder(tim2, 'PA0','PA1',3,4) # Create encoder objects
        encoder_right=encoder.Encoder(tim1,'PA8','PA9',3,4)

        tim5=Timer(5, freq=mobility_freq) # Create timer object for encoders
        tim5.callback(encoder_update)

        # Create shares and queues
        q_pos_L = task_share.Queue('l', 100, thread_protect=False, overwrite=False, name="Position") # Position in counts
        q_vel_L = task_share.Queue('h', 100, thread_protect=False, overwrite=False, name="Velocity") # Velocity in counts/sec
        q_pos_R = task_share.Queue('l', 100, thread_protect=False, overwrite=False, name="Position") # Position in counts
        q_vel_R = task_share.Queue('h', 100, thread_protect=False, overwrite=False, name="Velocity") # Velocity in counts/sec
        q_PID_e = task_share.Queue('f', 1, thread_protect=False, overwrite=False, name="PID Error") # PID error
        q_PID_i = task_share.Queue('f', 1, thread_protect=False, overwrite=False, name="PID Integral") # PID error integral
        q_PID_d = task_share.Queue('f', 1, thread_protect=False, overwrite=False, name="PID Derivative") # PID error derivative
        
        t = task_share.Queue('L', 1000, thread_protect=False, overwrite=False, name="Time") # Time in microsec

        v_req=task_share.Share('h',thread_protect=False, name="Requested Velocity") #velocity in inches/kilosecond
        omega_req=task_share.Share('f',thread_protect=False, name="Requested Yaw Rate") #yaw rate in radians/(second)

        motor_vel_kP=task_share.Share('h',thread_protect=False, name="Motor Velocity PID kP") #kP *10000
        motor_vel_kI=task_share.Share('h',thread_protect=False, name="Motor Velocity PID kI") #kI *10000
        motor_vel_kD=task_share.Share('h',thread_protect=False, name="Motor Velocity PID kD") #kD *10000
        motor_vel_kFF=task_share.Share('h',thread_protect=False, name="Motor Velocity PID kFF") #kFF *10000
        psidot_kP=task_share.Share('h',thread_protect=False, name="Yaw Rate PID kP") #kP *10000
        psidot_kI=task_share.Share('h',thread_protect=False, name="Yaw Rate PID kI") #kP *10000
        volts_L=task_share.Share('f',thread_protect=False, name="Left Motor Voltage") # volts
        volts_R=task_share.Share('f',thread_protect=False, name="Right Motor Voltage") # volts

        X=task_share.Share('f',thread_protect=False, name="X-Position") # X-Position in mm
        Y=task_share.Share('f',thread_protect=False, name="Y-Position") # Y-Position in mm
        X_dot=task_share.Share('f',thread_protect=False, name="X-Velocity") # X-Velocity in mm/s
        Y_dot=task_share.Share('f',thread_protect=False, name="Y-Velocity") # Y-Velocity in mm/s
        psi=task_share.Share('f',thread_protect=False, name="Heading") # heading in radians, positive CCW

        '''motor_vel_kP.put(7000)
        motor_vel_kI.put(100)
        motor_vel_kD.put(0)
        motor_vel_kFF.put(1000)
        psidot_kP.put(200)'''
        motor_vel_kP.put(2000)
        motor_vel_kI.put(20)
        motor_vel_kD.put(0)
        motor_vel_kFF.put(500)
        psidot_kP.put(200)
    
        
        log_flg = task_share.Share('H', thread_protect=False, name="Log Flag") # true when the logger should fill queues
        run_flg = task_share.Share('H', thread_protect=False, name="Run Test Flag") # true when the test should start, lower to end the test early    
        ir_cal_flg = task_share.Share('H', thread_protect=False, name="Line Sensor Calibration Flag")
        follow_flg = task_share.Share('H', thread_protect=False, name="Line Sensor Follower Flag") # >= 1 when Romi should steer to a line
        calibrate_flg = task_share.Share('H', thread_protect=False, name="Calibrate Flag") # 0 before calibration, 1 to save calibration data, 2 when calibration data saved

        # Set Logger task in cotask
        #log = logger.Logger((q_pos_L, q_vel_L,q_pos_R, q_vel_R, log_flg,t)) #create logger class object
        #log_per = 1000/50 #1000 [ms] / freq [Hz]
        #logger_task = cotask.Task(log.run, name="Logger", priority=4, period = log_per, profile=True, trace=False, shares=(q_pos_L, q_vel_L,q_pos_R, q_vel_R, log_flg,t))

        # Set Communication task in cotask
        coms=communication.Communication((q_pos_L, q_vel_L,q_pos_R, q_vel_R,t,run_flg, log_flg, ir_cal_flg, follow_flg, v_req, omega_req, motor_vel_kP, motor_vel_kI, motor_vel_kD, motor_vel_kFF, psidot_kP,psidot_kI, q_PID_e, q_PID_i, q_PID_d,calibrate_flg))#create communication object
        coms_task = cotask.Task(coms.run, name= "Communication", priority = 2, period = 20, profile=True, trace=False, shares=(q_pos_L, q_vel_L,q_pos_R, q_vel_R,t,run_flg, log_flg, ir_cal_flg, follow_flg, v_req, omega_req, motor_vel_kP, motor_vel_kI, motor_vel_kD , motor_vel_kFF, psidot_kP,psidot_kI, q_PID_e, q_PID_i, q_PID_d,calibrate_flg))

        #set up mobility task
        drive=mobility.Mobility((v_req, omega_req, q_pos_L, q_vel_L,q_pos_R, q_vel_R, run_flg, log_flg, t, motor_vel_kP, motor_vel_kI, motor_vel_kD, motor_vel_kFF, q_PID_e, q_PID_i, q_PID_d,volts_L, volts_R),motor_left, motor_right, encoder_left, encoder_right)
        drive_task=cotask.Task(drive.run, name= "Mobility", priority = 3, period = 1000//mobility_freq, profile=True, trace=False, shares=(v_req, omega_req, q_pos_L, q_vel_L,q_pos_R, q_vel_R, run_flg, log_flg, t, motor_vel_kP, motor_vel_kI, motor_vel_kD, motor_vel_kFF, q_PID_e, q_PID_i, q_PID_d, volts_L, volts_R))
        
        # Set Line Sensor task in cotask
        lins=line_sensor.Line_Sensor((omega_req, ir_cal_flg,follow_flg, psidot_kP,psidot_kI)) #create line sensor object
        lins_task = cotask.Task(lins.run, name= "Line Sensor", priority = 1, period = 1000//mobility_freq, profile=True, trace=False, shares=(omega_req, ir_cal_flg, follow_flg, psidot_kP,psidot_kI))

        #observer/IMU task
        obs_period=40 #50ms period
        obs=observer.Observer((calibrate_flg, t, q_pos_L, q_vel_L, q_pos_R, q_vel_R, v_req, omega_req, volts_L, volts_R, X, Y, X_dot, Y_dot,psi), obs_period, encoder_left, encoder_right)
        IMU_task=cotask.Task(obs.run, name= "Observer", priority = 5, period = obs_period, profile=True, trace=False, shares=(calibrate_flg, t, q_pos_L, q_vel_L, q_pos_R, q_vel_R, v_req, omega_req, volts_L, volts_R, X, Y, X_dot, Y_dot, psi))

        #Path planner task
        PP=PathPlanner.waypointsPlanner((v_req, omega_req, X, Y, X_dot, Y_dot, psi, run_flg,follow_flg), 1000//mobility_freq)
        PP_task=cotask.Task(PP.run, name= "PathPlanner", priority = 4, period = 1000//mobility_freq, profile=True, trace=False, shares=(v_req, omega_req, X, Y, X_dot, Y_dot, psi, run_flg,follow_flg))

        # Add tasks to list
        #otask.task_list.append(logger_task)
        cotask.task_list.append(coms_task)
        cotask.task_list.append(drive_task)
        cotask.task_list.append(lins_task)
        cotask.task_list.append(IMU_task)
        cotask.task_list.append(PP_task)


        # Run the memory garbage collector to ensure memory is as defragmented as
        # possible before the real-time scheduler is started
        gc.collect()

        # Run the scheduler with the chosen scheduling algorithm. Quit if ^C pressed
        while True:
            try:
                cotask.task_list.pri_sched()
            except KeyboardInterrupt:
                break

        # Print a table of task data and a table of shared information data
        print('\n' + str (cotask.task_list))
        print(task_share.show_all())
        print('')