#!/usr/bin/python

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=  I M P O R T   #=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
import math, time
from time import sleep

from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal, pyqtSlot

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  F U N C T I O N S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

# adds vectors: x + y
def vector_add(x, y):
    return [x[i] + y[i] for i in range(len(x))]

# subtracts vectors: x - y
def vector_subtract(x, y):
    return [x[i] - y[i] for i in range(len(x))]

def vector_scale(x, a):
    return [x[i] * a for i in range(len(x))]

def vector_dot_product(x, y):
    return sum(x[i] * y[i] for i in range(len(x)))
    
def vector_length(x):
    return math.sqrt(vector_dot_product(x,x))
    
def vector_normalize(x):
    length = vector_length(x)
    return [x[i] / length for i in range(len(x))]
    
def vector_project_onto_plane(x, n):
    n_normalized = vector_normalize(n)
    d = vector_dot_product(x, n) / vector_length(n)
    p = [d * n_normalized[i] for i in range(len(n))]
    return [x[i] - p[i] for i in range(len(x))]
    
def clamp(val_min, value, val_max):
    return max(val_min, min(val_max, value))
    
def map_value_to_scale(value, val_min, val_max, scale_min, scale_max):
    scaled_value = (value - val_min) / (val_max - val_min) * (scale_max - scale_min) + scale_min
    return clamp(min(scale_min, scale_max), scaled_value, max(scale_min, scale_max))
    
def mean(values_list):
    if len(values_list) > 0:
        return sum(values_list) / len(values_list)
    else:
        return 0.0

#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#  C L A S S E S   =#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=
#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=#=

#--- Basic PID controller class
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class PidController():

    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, kp, ki, kd, output_min, output_max, set_point):
    
        # modifiable settings
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.set_point = set_point
        
        # internal variables
        self._integral = 0.0
        self._prev_value = 0.0
        self._prev_error = 0.0
        self._previous_time = time.time()
        self._u = 0.0
        
        # TODO: integral wind-up reset?
        
        
    # M E T H O D S 
    #===========================================================================
    def update(self, current_value):
        # update the times
        # note: this controller works best when this update function is called
        # periodically on a consistent period
        #
        current_time = time.time()
        delta_t = current_time - self._previous_time
        
        if delta_t > 0.0:
        
            # calculate error
            error = self.set_point - current_value
            
            # proportional term
            self._p_value = self.kp * error
            
            # integral term
            self._i_value = self._integral
            
            # derivative term
            #d_value = self.kd * (error - self._prev_error) / delta_t
            self._d_value = -self.kd * (current_value - self._prev_value) / delta_t # slightly better transient response
            
            # output signal
            v = self._p_value + self._i_value + self._d_value
            self._u = max(self.output_min, min(self.output_max, v))
            
            self._integral += self.ki * error * delta_t
            self._integral = max(self.output_min, min(self.output_max, self._integral)) # this might make it slower to converge?
            
            self._prev_value = current_value
            self._prev_error = error
            self._previous_time = current_time
            
            # debug output
            #print('p = {:8.3f}, i = {:8.3f}, d = {:8.3f}, PID = {:8.3f}'.format(self._p_value, self._i_value, self._d_value, u))
        
        return self._u
        


#--- Remote controller state definition
#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-
class KPRemoteControlState(QtCore.QObject):

    # C O N S T R U C T O R 
    #===========================================================================
    def __init__(self, button_state=0, joystick_x=0, joystick_y=0, joystick_z=0, **kwds):
        super(KPRemoteControlState, self).__init__(**kwds)

        self._button_state = button_state
        self.joystick = {}
        self.joystick['x'] = joystick_x
        self.joystick['y'] = joystick_y
        self.joystick['z'] = joystick_z

        # unpack button states
        self.set_button_states(button_state)

    def set_button_states(self, button_state):
        self._button_state      = button_state
        self.btn_switch_red     = (button_state & (1 << 0)) != 0
        self.btn_switch_blue    = (button_state & (1 << 1)) != 0
        self.btn_pushbtn_red    = (button_state & (1 << 2)) != 0
        self.btn_pushbtn_green  = (button_state & (1 << 3)) != 0
        self.btn_rocker_up      = (button_state & (1 << 4)) != 0
        self.btn_rocker_down    = (button_state & (1 << 5)) != 0
        self.btn_joystick_btn   = (button_state & (1 << 6)) != 0


    def get_joystick(self, axis):
        return self.joystick[axis]