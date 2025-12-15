::
    from pyb import Pin, Timer 

    class Motor:
        '''A motor driver interface encapsulated in a Python class. Works with
        motor drivers using separate PWM and direction inputs such as the DRV8838
        drivers present on the Romi chassis from Pololu.'''
        
        def __init__(self, PWM, DIR, nSLP, timer: Timer, channel: int):
            '''Initializes a Motor object'''
            self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
            self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)
            self.PWM_pin = Pin(PWM, mode=Pin.OUT_PP, value=0)
            self.ch=timer.channel(channel, Timer.PWM, pin=Pin(PWM)) # Create timer channel for motor 
            self.period=timer.period() # Get timer period for motor
            self.ch.pulse_width(0) # Initialize motor to 0% duty cycle
            return
        
        def set_effort(self, effort):
            '''Sets the present effort requested from the motor based on an input value
            between -100 and 100'''
            #if effort <-100 or effort >100:
                #print(f"Effort {effort} out of range")
                #return
            # If effort == 0: will remember the direction it had last but engages brake mode
            if effort > 0: # High for forward, low for reverse
                self.DIR_pin.low()
            elif effort < 0:
                self.DIR_pin.high()
            self.ch.pulse_width_percent(abs(effort)) # Set duty cycle
            return
                
        def enable(self):
            '''Enables the motor driver by taking it out of sleep mode into brake mode'''
            self.nSLP_pin.high()
            self.PWM_pin.low()
            self.ch.pulse_width(0) # Set duty cycle to 0%
            return
                
        def disable(self):
            '''Disables the motor driver by taking it into sleep mode'''
            self.nSLP_pin.low()
            self.PWM_pin.low()
            self.ch.pulse_width(0) # Set duty cycle to 0%
            return