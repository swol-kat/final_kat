import numpy as np
class Gait:

    def __init__(self):
        self.params = {}
        self.prev_foot_pos = [np.zeros(3) for i in range(4)]

    def __bool__(self):
        return True

    def loop(self, robot):
        """
        :param robot: robot object so gait can get data
        :return:
        """
        pass


