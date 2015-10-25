#!/usr/bin/python3
from PendulumOnCart import PendulumOnCart
from DrawPendulumOnCart import DrawPendulumOnCart
from PendulumOnCartController import PendulumOnCartController
from StatePlotter import StatePlotter
from math import radians
import numpy as np


def __main__():
    time = 0.005
    P = PendulumOnCart(g = 9.82, m_c = 1, m = 1, l = 1, \
        init_angle = radians(0.1), init_angle_speed = 0, init_x = 0, init_x_speed = 0, \
        noise = np.array( [ [0], [0], [0], [0] ]), timestep = time)
    C = PendulumOnCartController(P, np.array([[0],[0],[0],[0]]))
    G = StatePlotter(timestep = time)
    Drawer = DrawPendulumOnCart(P, C, G, frameskip = 10000 )
    Drawer.startAnimation(speedup = 10)
    G.plotData("state.png", "phase.png")

__main__()
