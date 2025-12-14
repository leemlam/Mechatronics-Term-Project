PID Control
=====================

Imports and Init::

   from time import ticks_us, ticks_diff
   INV_SCALAR = 100_000
   class PID:


      def __init__(self, setpoint: float, kP: float, kI: float, kD: float, kFF=0):
         self.kP = kP /INV_SCALAR # Proportional gain
         self.kI = kI /INV_SCALAR# Integral gain
         self.kD = kD /INV_SCALAR # Derivative gain
         self.kFF = kFF /INV_SCALAR # Feed Forward gain
         self.setpoint = setpoint
         self.error = 0
         self.last = ticks_us()
         self.integral = 0
         self.derivative = 0
         self.dt = 1
         self.prev_error = 0
         self.inton = 1 # Switch to trigger integrator on
         self.saturation = 100 # Saturation effort (symmetric)
         return

Method to implement PID::

      def output(self,value: float, setpoint=None): # Allow optional changing setpoint during call
         # Update time variables
         if setpoint is not None:
               self.setpoint=setpoint
         now = ticks_us()
         self.dt = ticks_diff(now,self.last)
         self.last = now

         # Update error variables
         self.prev_error = self.error
         self.error = self.setpoint-value
         self.integral += self.error*self.dt
         self.derivative = (self.error-self.prev_error)/self.dt

         # Compute output via PID control
         output = self.kP*self.error + self.kI*self.integral*self.inton + self.kD*self.derivative + self.kFF*((self.setpoint>0)-(self.setpoint<0))

         # Actuator Saturation with integrator shutoff
         if output > self.saturation:
               output = self.saturation
               #self.inton = 0
         elif output < -self.saturation:
               output = -self.saturation
               #self.inton = 0
         else:
               self.inton = 1
         
         return output

Maintenance and data collection methods::
         
      def change_setpoint(self, setpoint):
         self.setpoint = setpoint
         return
      
      def performance(self):
         return (self.error, self.integral, self.derivative)
      
      def update_params(self, kP: float, kI: float, kD: float, kFF=0):
         self.kP=kP/INV_SCALAR
         self.kI=kI/INV_SCALAR/1000
         self.kD=kD/INV_SCALAR/1000
         self.kFF=kFF/INV_SCALAR

      def reset_errors(self):
         self.error=0
         self.prev_error=0
         self.integral=0
         self.derivative=0
