import sys
import time
import logging
import traceback

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveInterfaceSimulator(object):
    encoder_cpr = 4096
    connected = False
    engaged = False

    tilt_axis_vel = 0 # units: encoder counts/s
    yaw_axis_vel  = 0
    tilt_axis_pos = 0 # go from 0 up to encoder_cpr-1
    yaw_axis_pos  = 0
    last_time_update = None
    
    
    def __init__(self, logger=None):
        self.logger = logger if logger else default_logger
        
    def update_time(self, curr_time):
        # provided so simulator can update position
        if last_time_update is None:
            last_time_update = curr_time
            return
        
        dt = curr_time - last_time_update
        yaw_axis_pos  = floor(yaw_axis_pos  + yaw_axis_vel *dt) % self.encoder_cpr
        tilt_axis_pos = floor(tilt_axis_pos + tilt_axis_vel*dt) % self.encoder_cpr
        last_time_update = curr_time
                                    
    def connect(self, port=None, tilt_axis=0, timeout=30):
        if self.connected:
            self.logger.info("Already connected. Simulating disc/reconnect.")
            
        self.encoder_cpr = 4096
        self.logger.info("Connected to simulated ODrive.")
        return True
        
    def disconnect(self):
        self.connected = False
        return True
                
    def calibrate(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        self.logger.info("Calibrated.")
        return True
        
    def preroll(self, wait=True, reverse=False):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        
        return True
                
    def ensure_prerolled(self):
        return True
    
    def engaged(self):
        return self.engaged
        
    def engage(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        self.engaged = True
        return True
        
    def release(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        self.engaged = False
        return True
    
    def drive(self, yaw_motor_val, tilt_motor_val):
        if not self.connnected:
            self.logger.error("Not connected.")
            return
        self.yaw_axis.controller.pos_setpoint = yaw_motor_val
        self.tilt_axis.controller.pos_setpoint = -tilt_motor_val
        
        return True
        
    def get_errors(self, clear=True):
        if not self.driver:
            return None
        return "Simulated ODrive, no errors."
        
    def yaw_vel_estimate(self):  return self.yaw_axis_vel
    def tilt_vel_estimate(self): return self.tilt_axis_vel
    def yaw_pos(self):           return self.yaw_axis_pos
    def tilt_pos(self):          return self.tilt_axis_pos
    
    def yaw_current(self):       return 0
    def tilt_current(self):      return 0
    