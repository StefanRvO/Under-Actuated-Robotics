#!/usr/bin/python3
from DoublePendulumOnCart import DoublePendulumOnCart
from DrawDoublePendulumOnCart import DrawDoublePendulumOnCart
from DoublePendulumOnCartController import DoublePendulumOnCartController
from StatePlotter import StatePlotter
from math import radians
import numpy as np


def __main__():
    time = 0.005
    P = DoublePendulumOnCart(timestep = time, init_angle1 = -radians(10), init_angle2 = -radians(0),  \
            init_angle1_speed = 0, init_angle2_speed = 0, init_x = 5, init_x_speed = 0, g = 9.82)

    C = DoublePendulumOnCartController(P, setpoint = np.array([[0.],[0.],[0.],[0.], [0.], [0.]]), \
    K_LQR = np.array([ [22, 36, -306, 30, 558, 111] ]), outmax = 200, outmin = -200)
    G = StatePlotter(timestep = time)
    Drawer = DrawDoublePendulumOnCart(P, C, G, frameskip = 2 )
    Drawer.startAnimation(speedup = 0.5)
    G.plotData("state.png", "phase.png")

__main__()
