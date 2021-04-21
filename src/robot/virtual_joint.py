import time
import numpy as np
import math


class VirtualJoint:
    def __init__(self, gear_ratio, torque_constant):
        self.pos = float(0)
        self.gear_ratio = gear_ratio
        self.torque_constant = torque_constant
        self.state = 'halt'
        self.pos_command = 0.0
        self.torque_limit = 10.0
        self.max_current = 20.0
        self.curr_current = 0.0
        self.curr_pos = 0.0
        self.curr_vel = 0.0
        self.last_error_status = {'axis':0, 'motor':0, 'controller':0, 'encoder':0}
        self.invert = False

    def calibrate_joint(self):
        cal_time = 2
        step = .01
        for i in np.linspace(self.pos, cal_time, int((cal_time - self.pos / step))):
            self.set_setpoint(i)
            time.sleep(step)

    def enable_joint(self):
        return True

    def set_setpoint(self, angle):
        self.pos = float(angle)
        if self.invert:
            self.pos = -self.pos

    def set_torque(self, torque):  # set torque in N*M
        pass

    def fuck(self):
        self.set_setpoint(0)

    def get_setpoint(self):
        return self.pos

    def get_pos(self):
        """
            [float] pos of arm in rad
        """
        return self.pos
    
    def get_curr_torque(self):
        return 0
    
    def get_curr_position(self):
        return self.pos / self.gear_ratio * 2 * math.pi
    
    def get_curr_velocity(self):
        return 0

    def get_vel(self):
        """
            [float] velocity of arm in rad/s
        """
        return 0

    def get_torque(self):
        """
            [float] torquw of arm in Nm
        """
        return 0

    def run_manual_homing_routine(self):
        self.set_setpoint(0)

    def get_error(self):
        return {
            'axis': 'haha axis go brr'
        }
