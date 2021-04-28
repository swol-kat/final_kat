import math
import time

import numpy as np

from .gait import Gait
from ..util import get_body_pts, get_rot_leg_orig, euler_tm, swing_pos, ground_pos


class StaticWalk(Gait):

    def __init__(self):
        super().__init__()
        self.params = {
            'step_time': .5,  # seconds per movment
            'step_height': 2,  # inches,
        }
        self.state = [1]
        self.init_state = [1]
        self.x_vel = 0
        self.y_vel = 0
        self.last_time = time.time()

    def loop(self, robot):
        width = robot.config["width"]
        length = robot.config["length"]
        step_time = robot.loop_time/4

        dt = time.time() - self.last_time
        body_pts = get_body_pts(robot.target_base_state, width, length, False)
        rot = euler_tm(robot.target_base_state.alpha, robot.target_base_state.beta, robot.target_base_state.gamma)
        l = math.hypot(self.x_vel, self.y_vel) * step_time
        phi = math.atan2(self.y_vel, self.x_vel) 
        d_swing = swing_pos(min(dt / step_time, 1), self.params['step_height'], l, phi)

        for leg in robot.arms:
            i = leg.corner - 1
            if i in self.state:
                # robot leg should be in air
                target_foot_pos = self.prev_foot_pos[i] + d_swing
            else: 
                # robot leg should be on ground 
                target_foot_pos = self.prev_foot_pos[i]

            if dt > step_time:
                self.prev_foot_pos[i] = target_foot_pos

            target_foot_pos = target_foot_pos - body_pts[i]
            target_foot_pos = get_rot_leg_orig(i).transpose() @ rot.transpose() @ target_foot_pos
            leg.target_pos = target_foot_pos


        if dt > step_time:
            self.next_state()
            self.last_time = time.time()
            self.x_vel = robot.executing_movement_vector['x'] *4
            self.y_vel = robot.executing_movement_vector['y'] *4



    def next_state(self):
        for i, leg in enumerate(self.state):
            self.state[i] += 1
            self.state[i] %= 4
