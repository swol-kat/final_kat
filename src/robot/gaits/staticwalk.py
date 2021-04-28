import math
import time

import numpy as np

from .gait import Gait
from ..util import get_body_pts, get_rot_leg_orig, euler_tm, swing_pos
import copy


class StaticWalk(Gait):

    def __init__(self):
        super().__init__()
        self.state = 1
        self.gait_state = 'init'
        self.last_loop_time = time.time()
        self.body_move_time = 2
        self.step_move_time = 1
        self.params = {}
        self.step_height = 2

    def loop(self, robot):
        width = robot.config["width"]
        length = robot.config["length"]
        print(self.gait_state)
        if self.gait_state == 'init': # run this once at the start
            # step 1 get points of triangle
            triangle_vertecies = []
            for leg in robot.arms:
                i = leg.corner -1
                if i != self.state:
                    # we want this legs current pos
                    triangle_vertecies.append(self.prev_foot_pos[i])

            # step 2 calculate center of mass
            x_center = sum([point[0] for point in triangle_vertecies])/3
            y_center = sum([point[1] for point in triangle_vertecies])/3

            # set target_base_state = to the center of the triangle mass
            self.start_body_pos = copy.deepcopy(robot.target_base_state)
            self.final_body_pos = copy.deepcopy(robot.target_base_state)
            self.final_body_pos.update_state(x=x_center, y = y_center)
            if self.state == 1:
                self.x_vel = robot.movement_vector['x'] 
                self.y_vel = robot.movement_vector['y'] 
            
            #calculate arm positions and set
            body_pts = get_body_pts(robot.target_base_state, width, length, False)
            rot = euler_tm(robot.target_base_state.alpha, robot.target_base_state.beta, robot.target_base_state.gamma)
            for leg in robot.arms:
                i = leg.corner -1
                target_foot_pos = self.prev_foot_pos[i]
                target_foot_pos = target_foot_pos - body_pts[i]
                target_foot_pos = get_rot_leg_orig(i).transpose() @ rot.transpose() @ target_foot_pos
                leg.target_pos = target_foot_pos
                
            # update stte
            self.gait_state = 'move_body'
            self.last_loop_time = time.time() 

        elif self.gait_state == 'move_body':
            # calculate where body should be 
            dt = time.time() - self.last_loop_time
            precent_through = min((dt/self.body_move_time,1))
            pos_x  = (self.final_body_pos.x - self.start_body_pos.x) * precent_through + self.start_body_pos.x
            pos_y = (self.final_body_pos.y - self.start_body_pos.y) * precent_through + self.start_body_pos.y

            # set robot target_body to pos 
            robot.target_base_state.update_state(x=pos_x, y = pos_y)
            
            #calculate arm positions and set
            body_pts = get_body_pts(robot.target_base_state, width, length, False)
            rot = euler_tm(robot.target_base_state.alpha, robot.target_base_state.beta, robot.target_base_state.gamma)
            for leg in robot.arms:
                i = leg.corner -1
                target_foot_pos = self.prev_foot_pos[i]
                target_foot_pos = target_foot_pos - body_pts[i]
                target_foot_pos = get_rot_leg_orig(i).transpose() @ rot.transpose() @ target_foot_pos
                leg.target_pos = target_foot_pos


            if precent_through >= 1:
                self.gait_state = 'step'
                self.last_loop_time = time.time()


        elif self.gait_state == 'step':
            dt = time.time() - self.last_loop_time
            precent_through = min((dt/self.body_move_time,1))

            #calculate arm positions and set
            body_pts = get_body_pts(robot.target_base_state, width, length, False)
            rot = euler_tm(robot.target_base_state.alpha, robot.target_base_state.beta, robot.target_base_state.gamma)
            
            l = math.hypot(self.x_vel,self.y_vel) * self.step_move_time
            p = math.atan2(self.y_vel, self.x_vel)
            d_swing = swing_pos(precent_through,step_height= self.step_height, step_length= l, phi = p)
            for leg in robot.arms:
                i = leg.corner -1
                target_foot_pos = copy.deepcopy(self.prev_foot_pos[i])
                if i == self.state:
                    target_foot_pos += d_swing
                    if precent_through >= 1:
                            self.prev_foot_pos[i] = target_foot_pos
                target_foot_pos = target_foot_pos - body_pts[i]
                target_foot_pos = get_rot_leg_orig(i).transpose() @ rot.transpose() @ target_foot_pos
                leg.target_pos = target_foot_pos


            if precent_through >= 1:
                self.gait_state = 'init'
                self.next_state()



    def next_state(self):
        self.state += 1
        self.state %= 4
