State Estimation
================

Constants and Initialization
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

State Estimation Matrices::

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