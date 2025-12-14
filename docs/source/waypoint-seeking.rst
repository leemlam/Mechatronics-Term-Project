Waypoint Seeking
================

Our group implemented a waypoint-seeking algorithm to give our robot the ability to adaptively path toward
locations.

This system allows setting coordinates, a velocity vector, and a path target time in order to calculate
polynomial paths to each objective. 

The results of these paths are shown below:

.. image:: waypoints.png

In this image, the robot path is shown by a gradient between blue and red, with higher velocities at
more red points. The waypoint locations are shown with black circles, and the dashed line indicates the 
location of cups to be pushed. The wall and garage are also shown for reference in black.

This model allowed for faster iteration of paths between tests, showing if velocity targets would cause too
sharp turns, maximum velocities during straights, and whether the waypoints would cause the robot to crash
into obstacles.


Code
----

Initialization of the waypoint class is shown below::

    def __init__(self, shares, time_step):
        (self.v_req, self.omega_req, self.s_X, self.s_Y, self.s_X_dot, self.s_Y_dot, self.psi, self.run_flg, self.follow_flg) = shares
        self.step = 0
        self.waypointtimer = 0
        self.first_flg = 1
        self.V_request = 0
        self.om_req = 0
        self.psi_req=0 #psi is zero along starting direction (+x) and positive CCW
        self.Vx = 0
        self.Vy = 0
        self.time_step=time_step
        self.current_psi=self.psi.get()
        self.state=INS_STATE
        self.t0=ticks_us()/1_000_000

        self.bmp_0=Pin('PC8', mode=Pin.IN, pull=Pin.PULL_UP, value=1)
        self.bmp_1=Pin('PB12', mode=Pin.IN, pull=Pin.PULL_UP, value=1)
        self.bmp_2=Pin('PB2', mode=Pin.IN, pull=Pin.PULL_UP, value=1)
        self.bmp_3=Pin('PH1', mode=Pin.IN, pull=Pin.PULL_UP, value=1)
        self.bmp_4=Pin('PH0', mode=Pin.IN, pull=Pin.PULL_UP, value=1)
        self.bmp_5=Pin('PB7', mode=Pin.IN, pull=Pin.PULL_UP, value=1)
        self.buzzer=Pin('PC9',mode=Pin.OUT_PP, value=1)
        self.wall_step=11
        
        # X-position, Y-position, X-velocity, Y-velocity, waypoint time           
        self.waypoints = [[750,800,200,0,2.5],      # End of start line
                    [950,600,0,-200,2],             # "Line following" complete
                    [950,425,100,-200,1],           # CP1
                    [1100,400,100,200,0.75],        # Smooooth turn
                    [1400,800,300,0,2],             # CP2
                    [1525,500,0,-300,1.5],          # Cone 1
                    [1350,100,-300,0,1.75],         # CP3
                    [800,125,-300,0,2],             # CP4
                    [300,100,-200,0,2],             # Dont hit the garage
                    [200,300,0,200,1],              # CP5
                    [200,450,-20,0,3],              # Wall
                    [200,450,0,0,3],                # Wall 2: wall's revenge
                    [500, 450,100,200,4],           # Cone 2
                    [200,800,-200,0,2],             # Don't hit the wall
                    [100,800,-200,0,2],             # CP6
                    [75,800,0,0,.5]]                # Stop here
        
        return

Run state is shown here::

    def run(self, shares):
        while True:
            if self.run_flg.get() >= 2:
                if self.state==INS_STATE:
                    if self.first_flg == 1:
                        self.t0=ticks_us()/1_000_000
                        self.first_flg = 0
                        self.v_req.put(0)
                        print("==============================================================")
                        print("=========================STARTED==============================")
                        print("==============================================================")
                        self.buzzer.value(0)
                    if self.step==0 or self.step==1:
                        self.follow_flg.put(1)
                    else:
                        self.follow_flg.put(0)
                    self.buzzer.value(1)
                    # Update positions from observer
                    self.X = self.s_X.get()
                    self.Y = self.s_Y.get()
                    self.X_dot = self.s_X_dot.get()
                    self.Y_dot = self.s_Y_dot.get()
                        
                    self.t = ticks_us()/1_000_000 - self.t0
                    
                    self.delta_X = self.waypoints[self.step][0]-self.X
                    self.delta_Y = self.waypoints[self.step][1]-self.Y
                    self.distance = np.sqrt((self.delta_X)**2 + (self.delta_Y)**2)
                    
                    if self.distance <= tw/3 and self.step <= len(self.waypoints)-1: # Increase step if romi is within checkpoint
                        print(f"------------Reached waypoint: {self.step} ------------")
                        self.step += 1
                        self.waypointtimer = 0
                        self.t0=ticks_us()/1_000_000
                        self.t=0
                        if self.step==self.wall_step:
                            self.state=BUMP_STATE
                            yield
                        elif self.step==len(self.waypoints)-1:
                            self.run_flg.put(0)
                            yield
    
                    if self.waypointtimer <= 0: # Calculate new polynomial path at set interval
                        print(f"V req: {self.V_request} - ({self.Vx}, {self.Vy}), Omega Req: {self.om_req}")
                        print(f"Current heading: {self.current_psi%(2*np.pi)}, Desired heading: {self.psi_req}, t:{self.t}, t0:{self.t0}")
                        #print("------New Loop-------")
                        #print(f"Distance to waypoint: {self.distance} - ({self.delta_X}, {self.delta_Y})")
                        
                        q0 = [self.X, self.Y]
                        q0_dot = [self.X_dot, self.Y_dot]
                        
                        '''
                        if self.step==0:
                            q0 = [100, 800]
                            q0_dot = [0, 0]
                        else:
                            q0 = [self.waypoints[self.step-1][0], self.waypoints[self.step-1][1]]
                            q0_dot = [self.waypoints[self.step-1][2], self.waypoints[self.step-1][3]]
                        '''

                        if self.t>= self.waypoints[self.step][4]: # Add time to t1 if path takes too long
                            self.waypoints[self.step][4]+=2 

                        q1 = [self.waypoints[self.step][n] for n in [0,1]]
                        q1_dot = [self.waypoints[self.step][n] for n in [2,3]]

                        qvec = np.array([q0,
                                        q0_dot,
                                        q1,
                                        q1_dot])
                        
                        self.t1 = self.waypoints[self.step][4]

                        tvec = np.array([[1,self.t,self.t**2,self.t**3],
                                        [0,1,2*self.t,3*self.t**2],
                                        [1,self.t1,self.t1**2,self.t1**3],
                                        [0,1,2*self.t1,3*self.t1**2]])
                        '''tvec = np.array([[1,0,0,0],
                                        [0,1,0,0],
                                        [1,self.t1,self.t1**2,self.t1**3],
                                        [0,1,2*self.t1,3*self.t1**2]])'''
                        
                        self.a,self.b,self.c,self.d = np.dot(np.linalg.inv(tvec), qvec)
                        '''ax,ay=100,800#self.a
                        bx,by=0,0#self.b
                        cx,cy=387.5,0#self.c
                        dx,dy=-112.5,0#self.d'''
                        ax,ay=self.a
                        bx,by=self.b
                        cx,cy=self.c
                        dx,dy=self.d
                        #print(f"({ax} + {bx}t + {cx}t^2 + {dx}t^3, {ay} + {by}t + {cy}t^2 + {dy}t^3)")
                        #print(f"({bx} + 2*{cx}t + 3*{dx}t^2, {by} + 2*{cy}t + 3*{dy}t^2)")

                        self.waypointtimer = 10
                    self.t+=self.time_step/1000 #solve for next velocity, not current

                    self.Vx = bx + 2*cx*self.t + 3*dx*self.t**2 # Calculate velocities from polynomial path
                    self.Vy = by + 2*cy*self.t + 3*dy*self.t**2

                    self.V_request = int(np.sqrt(self.Vx**2 + self.Vy**2))
                    self.psi_req = np.arctan2(self.Vy,self.Vx)
                    self.current_psi=self.psi.get()
                    
                    self.difference = self.psi_req-self.current_psi%(2*np.pi) # (-pi, pi) - (0, 2pi) range (-3pi, pi)
                    if self.difference < -np.pi:
                        self.difference += 2*np.pi
                    
                    self.om_req=1000/2 * self.difference / self.time_step
                    #self.om_req=1000/4 * ((self.psi_req-self.current_psi%(2*np.pi)) if (self.psi_req-self.current_psi%(2*np.pi))<np.pi else (self.psi_req-self.current_psi%(2*np.pi))-2*np.pi)/self.time_step

                    self.v_req.put(self.V_request) # Set velocity and yaw rate requests
                    self.omega_req.put(self.om_req)

                    self.waypointtimer -= 1
                elif self.state==FINISH_STATE:

                    pass
                elif self.state==BUMP_STATE:
                    self.v_req.put(30)
                    self.omega_req.put(0)
                    self.buzzer.value(0)
                    print(f"bump: {self.bmp_0.value()}, {self.bmp_1.value()}, {self.bmp_2.value()}, {self.bmp_3.value()}, {self.bmp_4.value()}, {self.bmp_5.value()}")
                    if not (self.bmp_0.value() and self.bmp_1.value() and self.bmp_2.value() and self.bmp_3.value() and self.bmp_4.value() and self.bmp_5.value()):
                        self.state=INS_STATE
                        self.v_req.put(-100)
                        self.buzzer.value(1)
                        self.step+=1
            else:
                self.step=0
                self.first_flg=1
            
            yield