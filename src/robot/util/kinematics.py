import numpy as np
import math


def s(angle):
    return np.sin(angle)


def c(angle):
    return np.cos(angle)


def htm(theta, d, a, alpha):
    """
    :param theta:
    :param d:
    :param a:
    :param alpha:
    converts dh parameters to homogenus transform matrix
    :return:
    """
    return np.array([[c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
                     [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta)],
                     [0, s(alpha), c(alpha), d],
                     [0, 0, 0, 1]])


def euler_tm(a, b, g):
    """
    :param a: alpha (rotz) yaw
    :param b: beta (roty) pitch
    :param g: gamma (rotx) roll
    converts euler angels to rotation matrix
    :return:
    """
    rz = np.array([[c(a), -s(a), 0],
                   [s(a), c(a), 0],
                   [0, 0, 1]])

    ry = np.array([[c(b), 0, s(b)],
                   [0, 1, 0],
                   [-s(b), 0, c(b)]])
    rx = np.array([[1, 0, 0],
                   [0, c(g), -s(g)],
                   [0, s(g), c(g)]])

    return rz @ ry @ rx
