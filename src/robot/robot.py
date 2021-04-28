import json

from .gaits import Gait, Wiggle, OpenWalk
from .util import BodyState
import time

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

        self.base_state = BodyState(z=13)
        self.target_base_state = BodyState(z=13)
        
        self.movement_vector = dict(x=0, y=0, z=0, alpha=0, beta=0, gamma=0)
        self.last_time = time.time()

        self.gait = OpenWalk()
        self.last_time = time.time()
        print('Robot is booted')
         

    def loop(self):
        """
        main control loop of robot run this in a while loop or something
        :return:
        """
        pos_adjust = {k: v*(time.time()-self.last_time) for k,v in self.movement_vector.items()}
        self.last_time=time.time()
        self.target_base_state.move(**pos_adjust)
        self.base_state = self.target_base_state

        if self.gait:
            self.gait.loop(self)
        for arm in self.arms:
            arm.update()
        self.last_time = time.time()
        
    def reset():
        self.base_state = BodyState(z=13)
        self.target_base_state = BodyState(z=13)
        self.movement_vector = dict(x=0, y=0, z=0, alpha=0, beta=0, gamma=0)
        self.state.gait=Wiggle()

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
