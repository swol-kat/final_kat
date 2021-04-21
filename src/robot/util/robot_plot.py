import matplotlib.pyplot as plt
import numpy as np

from .kinematics import euler_tm
from .robot_util import get_body_pts, get_rot_leg_orig


class Plot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.fig.show()

    def config_plt(self):
        # self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-20, 20])
        self.ax.set_ylim([-20, 20])
        self.ax.set_zlim([0, 30])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.text(0, 10, 0, 'Front is here')

    def plot(self, robot):
        """
        :param robot:
        takes in base state and arms param and plots them
        :return:
        """
        xcg, ycg, zcg, psi, theta, phi = list(robot.base_state)

        width = robot.config["width"]
        length = robot.config["length"]

        # clearing and configing plot
        self.ax.clear()
        self.config_plt()
        # mark center point
        self.ax.plot(xcg, ycg, 0, marker='o', markersize=3, color="red")
        self.ax.plot(xcg, ycg, zcg, marker='o', markersize=3, color="blue")
        # calculate body points from body

        body_pts = get_body_pts(robot.base_state, width, length, False)

        rot = euler_tm(phi, theta, psi)

        plot_points = body_pts
        plot_points.append(body_pts[0])
        plot_points = np.array(plot_points)
        self.ax.plot(plot_points[:, 0], plot_points[:, 1], plot_points[:, 2], 'o-')

        for i, leg in enumerate(robot.arms):

            rotated_pts = rot @ get_rot_leg_orig(i) @ leg.pos.reshape(3, 1)
            point = body_pts[i]
            x, y, z = rotated_pts.reshape(3) + point

            color = 'red'
            if leg.contact:
                color = 'green'

            # calculate leg pos
            xs, ys, zs = [[p] for p in point.reshape(3)]
            for j in range(4):
                pos = leg.fwkin(joint=j + 1, disp=True)
                pos = rot @ get_rot_leg_orig(i) @ pos
                xy, yy, zy = pos.reshape(3) + point
                xs.append(xy)
                ys.append(yy)
                zs.append(zy)

            self.ax.plot(xs, ys, zs, 'o-')

            # convert from foot space to world space
            self.ax.plot(x, y, z, marker='o', markersize=5, color=color)

        self.fig.canvas.draw()


if __name__ == "__main__":
    p = Plot()
    p.plot()
