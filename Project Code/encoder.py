from time import ticks_us, ticks_diff   # Use to get dt value in update()
from pyb import Pin, Timer


AR_1= 65535+1 # Auto-reload value for timer (timers 1 and 2 are 16 and 32 bit)
counts_lim = AR_1//2 # Half the auto-reload value for overflow/underflow checking
neg_counts_lim = -AR_1//2 # Half the auto-reload value for overflow/underflow checking

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chB_pin, chA, chB):
        '''Initializes an Encoder object'''
        
        self.channel_A = tim.channel(chA, mode=Timer.ENC_AB, pin=Pin(chA_pin,mode=Pin.OUT_PP, value=0)) # Set up timer in encoder mode
        self.channel_B = tim.channel(chB, mode=Timer.ENC_AB, pin=Pin(chB_pin,mode=Pin.OUT_PP, value=0)) # Set up timer in encoder mode

        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 1     # Amount of time between last two updates in usec
        self.last_t     = ticks_us() # Time of last update call
        self.tim        = tim   # Timer object used for encoder
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        self.delta=self.tim.counter()-self.prev_count # Compute change in count
        #print(self.tim.counter())
        if self.delta < neg_counts_lim:  # Handle overflow
            self.delta += AR_1
        elif self.delta > counts_lim: # Handle underflow
            self.delta -= AR_1

        self.position += self.delta
        self.prev_count = self.tim.counter()

        now=ticks_us()
        self.dt=ticks_diff(now, self.last_t)
        self.last_t=now
        return self.delta
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        return self.delta/self.dt
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0
        self.prev_count = 0
        self.tim.counter(0)
        self.delta = 0