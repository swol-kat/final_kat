import json

from .gaits import Gait, Wiggle, OpenWalk, StaticWalk
from .util import BodyState, Plot, get_body_pts
import time
import copy

class Robot:
    config: dict
    gait: Gait
    
    arms: list
    base_state: BodyState
    target_base_state: BodyState
    movement_vector: dict

    def __init__(self, arm_dict ):
        self.reload_config()
        self.arms = [
            arm_dict['front_right'],
            arm_dict['front_left'],
            arm_dict['back_left'],
            arm_dict['back_right']
        ]

    def boot(self):
        """
        boot up routine for robot
        :return:
        """
        print('Robot is booting')

        self.base_state = BodyState(z=5)
        self.target_base_state = BodyState(z=5)
        self.stance_width = 5
        self.stance_length = 5
        self.executing_movement_vector = dict(x=0, y=0, z=0, alpha=0, beta=0, gamma=0)
        self.movement_vector = dict(x=0, y=0, z=0, alpha=0, beta=0, gamma=0)

        self.gait = Wiggle()
        self.gait.prev_foot_pos = get_body_pts(BodyState(),self.config['width']+self.stance_width,self.config['length']+self.stance_length, False)
        self.last_time = time.time()

        self.loop_time = 3
        self.in_loop = False
        self.last_loop_time = time.time()
         

    def loop(self):
        """
        main control loop of robot run this in a while loop or something
        :return:
        """
        if self.in_loop:
            pos_adjust = {k: v*(time.time()-self.last_time) for k,v in self.executing_movement_vector.items()}
            self.last_time = time.time()
            self.target_base_state.move(**pos_adjust)
            print(self.target_base_state)
            self.base_state = self.target_base_state
            if time.time() - self.last_loop_time > self.loop_time:
                self.in_loop = False
        else:
            self.target_base_state = BodyState(z=13)
            self.gait.prev_foot_pos = get_body_pts(BodyState(),self.config['width']+self.stance_width,self.config['length']+self.stance_length, False)
            self.executing_movement_vector = copy.deepcopy(self.movement_vector)
            self.last_loop_time = time.time()
            self.in_loop = True              
                
        if self.gait:
            self.gait.loop(self)
        for arm in self.arms:
            arm.update()


    def reload_config(self):
        """
        reloads the robot_config file
        :return:
        """
        self.config = {
                        "width": 10.0038,
                        "length": 10.493,
                        "arm_data": {
                            "D1": 3.319,
                            "D2": 3.125,
                            "A2": 7.913,
                            "A3": 7.913
                        },
                        "joint_lim": {
                        }
                        }
        #self.config = json.load(open('robot/robot_config.json'))


if __name__ == "__main__":
    robot = Robot()
