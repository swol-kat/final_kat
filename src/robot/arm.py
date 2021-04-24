import math
import numpy as np

import time
from .util import htm
from copy import copy, deepcopy

class Arm:
    def __init__(self, lower_axis, upper_axis, shoulder_axis, arm_vars, corner=1):
        self.lower_axis = lower_axis
        self.upper_axis = upper_axis
        self.shoulder_axis = shoulder_axis
        self.arm_vars = arm_vars
        self.target_pos = np.array([[0], [0], [0]])
        self.thetas = np.array([[0], [0], [0]])
        self.joint_vel = np.array([[0], [0], [0]])
        self.joint_torque = np.array([[0], [0], [0]])
        self.tip_force_limit = np.array([[3],[3],[3]])
        self.pos = np.array([[0.], [0.], [0.]])
        self.vel = np.zeros((3,1))
        self.force = np.zeros((3,1))
        self.last_update_time = time.time()
        self.state = 'idle'
        self.last_error_update = time.time()
        self.corner = corner

    def ikin(self, pos_vect):
        x, y, z = pos_vect.reshape(3)
        d1 = self.arm_vars['D1'] 
        d2 = self.arm_vars['D2']
        a2 = self.arm_vars['A2']
        a3 = self.arm_vars['A3']

        r = math.hypot(x, y)
        alpha = math.atan2(y, x)
        u = math.sqrt(r ** 2 - d2 ** 2)
        beta = math.atan2(d2, u)
        # t1
        if self.corner in (2,4):
            t1 = alpha - beta
        else:
            t1 = alpha + beta
        
        
        s = z - d1
        D = (r ** 2 + s ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)

        # making sure D doesnt excced -1 one cause floating points
        D = min(max(D, -1), 1)
        # t2
        if self.corner in (1, 2):
            t3 = - math.atan2(math.sqrt(1 - D ** 2), D)
        else:
            t3 = - math.atan2(- math.sqrt(1 - D ** 2), D)
        phi = math.atan2(s, r)
        gamma = math.atan2(a3 * math.sin(t3), a2 + a3 * math.cos(t3))


        if self.corner in (2,4):
            t2 = - (phi + gamma)
        else: 
            t2 = phi+gamma
            t3 = -t3
        

        t3 = t2 + t3
        return np.array([t1, t2, t3]).reshape((3, 1))

    def set_tip_force_limit(self, x, y, z):
        self.tip_force_limit = np.array([[x],[y],[z]])

    def jacobian(self, thetas=None):
        if not thetas:
            thetas = self.thetas

        pe = self.fwkin(thetas).reshape((1,3))

        J = np.zeros((6, 1))
        for i in range(3):
            tf = self.fwkin(thetas, joint=i, vector=False)
            z = tf[0:3, 2]
            pi = tf[0:3, 3]
            Jp = np.cross(z, pe - pi)
            Jo = z
            Ji = np.concatenate((Jp.reshape((3,1)), Jo.reshape((3,1))), axis=0)
            J = np.concatenate((J, Ji), axis=1)
        
        return np.delete(J, 0, 1)

    def fwkin(self, thetas=None, joint=3, vector=True):
        """
        converts joint angles stored in self.thetas to workspace returns:
        [float]  returns a either a 4x4 matrix of the transform from joint 1 to the input joint or a 3x1 vector
                 depending on the vector variable
        """
        # dh table
        if thetas is None:
            thetas = self.thetas

        flip = 1
        if self.corner in (2,4):
            flip = -1
        t1, t2, t3 = thetas.reshape(3)

        dh_table = [[t1, self.arm_vars['D1'], 0, flip* math.pi / 2],
                    [t2, self.arm_vars['D2'], self.arm_vars['A2'], 0],
                    [t3-t2, 0, self.arm_vars['A3'], 0]]
        # identity matrix
        t_final = np.identity(4)
        # calculate fwkin
        for i in range(joint):
            params = dh_table[i]
            t_final = t_final @ htm(*params)
        # if vector return just the pos var
        if vector:
            return t_final[0:3, 3].reshape(3, 1)

        return t_final

    def export_data(self):
        joint_pos = self.get_joint_pos()
        forces = deepcopy(self.force.reshape(3))
        joint_pos['x'].append(forces[0] + joint_pos['x'][-1])
        joint_pos['y'].append(forces[1] + joint_pos['y'][-1])
        joint_pos['z'].append(forces[2] + joint_pos['z'][-1])

        return {
            'joint_pos': joint_pos,
            'thetas': self.thetas.reshape(3).tolist(),
            'pos': self.pos.reshape(3).tolist(),
            'joint_vel': self.joint_vel.reshape(3).tolist(),
            'joint_torque': self.joint_torque.reshape(3).tolist(),
            'vel': self.vel.reshape(3).tolist(),
            'force': self.force.reshape(3).tolist(),
        }

    def get_joint_pos(self):
        xs = [0]
        ys = [0]
        zs = [0]
        for i in range(4):
            pos = self.fwkin(joint=i + 1, disp=True)
            x, y, z = pos.reshape(3)
            xs.append(x)
            ys.append(y)
            zs.append(z)
        return {
            'x': xs,
            'y': ys,
            'z': zs
        }
    
    def get_tip_vel(self):
        jacob = self.jacobian()[0:3,:]

        tip_vel = jacob @ self.joint_vel

        return tip_vel.reshape((3,1))

    def get_tip_force(self):
        jacob = self.jacobian()[0:3,:]

        tip_force = np.linalg.pinv(np.transpose(jacob)) @ self.joint_torque

        return tip_force.reshape((3,1))

    def go_to_thetas(self, thetas):
        t1, t2, t3 = thetas.reshape(3)
        #check if elbow over travel. don't let joint go too far
        # if t3-t2 > math.pi * 0.75 :
        #     t3 = t2 + math.pi * 0.75
        # if t3-t2 < math.pi * -0.5 :
        #     t3 = t2 - math.pi * 0.5
        self.shoulder_axis.set_setpoint(t1)
        self.upper_axis.set_setpoint(t2)
        self.lower_axis.set_setpoint(t3)

    def send_to_pos(self, target_pos = None):
        if target_pos:
            thetas = self.ikin(target_pos)
        else: 
            thetas = self.ikin(self.target_pos)
        self.go_to_thetas(thetas)

    def jog(self,thetas,pos):
        if thetas and np.sum(thetas) != 0:
            thetas = np.array(thetas).reshape((3,1))
            self.send_to_pos(self.thetas + thetas)
        if pos and np.sum(pos) != 0:
            pos = np.array(pos).reshape((3, 1))
            self.go_to_raw(self.pos+pos,False)

    def calibrate_arm_start(self):
        print("calibrating arm")
        self.shoulder_axis.start_calibration()
        self.upper_axis.start_calibration()
        self.lower_axis.start_calibration()
    
    def is_arm_calibrated(self):
        return self.lower_axis.is_calibration_complete() and self.upper_axis.is_calibration_complete() and self.shoulder_axis.is_calibration_complete()
    
    def stop(self):
        self.shoulder_axis.disable()
        self.upper_axis.disable()
        self.lower_axis.disable()

    def enable(self):
        self.shoulder_axis.enable()
        self.upper_axis.enable()
        self.lower_axis.enable()
    
    def poll_errors(self):
        #errors will be checked on odrives and get_error will be updated
        self.last_update_time = time.time()
        self.shoulder_axis.poll_errors()
        self.upper_axis.poll_errors()
        self.lower_axis.poll_errors()

    def get_error(self):
        #needs logic to send the error request to joint, then recieve error message
        return {
            'shoulder': self.shoulder_axis.get_errors(),
            'upper': self.upper_axis.get_errors(),
            'lower': self.lower_axis.get_errors(),
            'time': self.last_update_time
        }
    
    #TODO: implement Arm homing program in joint and arm code

    def update(self): 
        # gets angle from each of the three joints
        t1 = self.shoulder_axis.get_curr_position()
        t2 = self.upper_axis.get_curr_position()
        t3 = self.lower_axis.get_curr_position()

        self.thetas = np.array([[t1], [t2], [t3]])

        self.send_to_pos()
        
        
        self.pos = self.fwkin()

        self.joint_vel = np.array([[self.shoulder_axis.get_curr_velocity()], [self.upper_axis.get_curr_velocity()], [self.lower_axis.get_curr_velocity()]])

        self.joint_torque = np.array([[self.shoulder_axis.get_curr_torque()], [self.upper_axis.get_curr_torque()], [self.lower_axis.get_curr_torque()]])

        self.vel = self.get_tip_vel()


        self.last_update_time = time.time()