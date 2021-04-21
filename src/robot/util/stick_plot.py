import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Plot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.show()

    def config_plt(self):
        # self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-20, 20])
        self.ax.set_ylim([-20, 20])
        self.ax.set_zlim([0, 20])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

    def plot(self, arm):
        self.ax.clear()
        xs = [0]
        ys = [0]
        zs = [0]
        for i in range(4):
            pos = arm.fwkin(joint=i + 1)
            x, y, z = pos.reshape(3)
            xs.append(x)
            ys.append(y)
            zs.append(z)

        self.config_plt()
        self.ax.plot(xs, ys, zs, 'o-')
        self.fig.canvas.draw()

