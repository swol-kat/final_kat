import numpy as np

from .gait import Gait
from ..util import get_body_pts, get_rot_leg_orig, euler_tm
from ..util import BodyState
import time


class Wiggle(Gait):

    def __init__(self):
        super().__init__()
        self.last_time = time.time()


    def loop(self, robot):
        """
        :param self.parms contains zcg and target orientation for this gait
        :param robot: takes in the robot object
        :return:
        """
        delta_t = (time.time()-self.last_time)

        pos_adjust = {k: v*delta_t for k,v in robot.movement_vector.items()}
        #print(robot.movement_vector)
        robot.target_base_state.move(**pos_adjust)
        #print(robot.target_base_state)
        width = robot.config["width"]
        length = robot.config["length"]

        body_pts = get_body_pts(robot.target_base_state, width, length, False)
        zero_body_pts = get_body_pts(BodyState(), width, length)

        rot = euler_tm(robot.target_base_state.alpha, robot.target_base_state.beta, robot.target_base_state.gamma)

        for i, leg in enumerate(robot.arms):
            # get_taget_foot_pos in world points

            target_foot_pos = np.array([zero_body_pts[i][0], zero_body_pts[i][1], 0])
            target_foot_pos = target_foot_pos - body_pts[i]
            target_foot_pos = get_rot_leg_orig(i).transpose() @ rot.transpose() @ target_foot_pos
            leg.target_pos = target_foot_pos

        self.last_time = time.time()
