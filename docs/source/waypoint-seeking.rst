Waypoint Seeking
================

.. image:: waypoints.png
 

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

