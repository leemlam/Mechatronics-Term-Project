import encoder
from pyb import Pin, Timer
from time import ticks_us

class Logger:

    def __init__(self, shares):
        self.q_pos_L, self.q_vel_L,self.q_pos_R, self.q_vel_R, self.log_flg, self.t= shares
         
    def encoder_update(self,tim):
            self.encoder_left.update()
            self.encoder_right.update()


    def run(self, shares):
        pass
        
        '''while True:
            if self.log_flg.get()==1:
                #log data into queues
                #print(f"Left Encoder Pos: {self.encoder_left.get_position()}")
                if not self.q_pos_L.full() and not self.q_vel_L.full() and not self.t.full():
                    self.q_pos_L.put(self.encoder_left.get_position())
                    self.q_vel_L.put(int(1_000_000*self.encoder_left.get_velocity())) # Convert to integer ticks per sec
                    self.q_pos_R.put(self.encoder_right.get_position())
                    self.q_vel_R.put(int(1_000_000*self.encoder_right.get_velocity())) # Convert to integer ticks per sec
                    self.t.put(ticks_us())
                if self.t.full():
                    self.log_flg.put(0)
                    print("Filled Buffer")
              #don't log data
            yield'''