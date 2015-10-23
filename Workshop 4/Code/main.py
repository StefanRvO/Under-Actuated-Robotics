#!/usr/bin/python3
from PendulumOnCart import PendulumOnCart
from DrawPendulumOnCart import DrawPendulumOnCart
from math import radians
import numpy as np


def __main__():
    time = 0.01
    P = PendulumOnCart(g = 9.82, m_c = 10, m = 1, l = 1, fric_c = 0, fric_p = 0, \
        init_angle = radians(20), init_angle_speed = 0.0001, init_x = 0, init_x_speed = 0, \
        noise = np.array([[0], [0], [0], [0]]))

    Drawer = DrawPendulumOnCart(P, frameskip = 1)
    Drawer.startAnimation()

__main__()
