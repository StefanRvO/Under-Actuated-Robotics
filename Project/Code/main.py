#!/usr/bin/python3
from DoublePendulumOnCart import DoublePendulumOnCart
from DrawDoublePendulumOnCart import DrawDoublePendulumOnCart
from DoublePendulumOnCartController import DoublePendulumOnCartController
from StatePlotter import StatePlotter
from math import radians
import numpy as np


def __main__():
    time = 0.005
    P = DoublePendulumOnCart(m= 0.5,   \
                             m1 = 0.8, \
                             m2 = 0.15, \
                             L1 = 1,   \
                             L2 = 1,   \
                             init_angle1 = -radians(181),  \
                             init_angle2 = radians(181), \
                             timestep = time,  \
                             init_angle1_speed = 0, \
                             init_angle2_speed = 0, \
            init_x = 0, init_x_speed = 0, g = 9.82)

    C = DoublePendulumOnCartController(P, setpoint = np.array([[0.],[0.],[0.],[0.], [0.], [0.]]), \
    K_LQR = np.array([ [2.23607, 10.07, -98.05, 11.97, 196.82, 53.34] ]), outmax = 20, outmin = -20)
    G = StatePlotter(timestep = time)
    Drawer = DrawDoublePendulumOnCart(P, C, G, frameskip = 1)
    Drawer.startAnimation(speedup = 1)
    G.plotData("state.png", "phase.png")

__main__()
