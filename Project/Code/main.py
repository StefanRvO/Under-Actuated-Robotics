#!/usr/bin/python3
from DoublePendulumOnCart import DoublePendulumOnCart
from DrawDoublePendulumOnCart import DrawDoublePendulumOnCart
from DoublePendulumOnCartController import DoublePendulumOnCartController
from StatePlotter import StatePlotter
from math import radians
import numpy as np


def __main__():
    time = 0.01
    P = DoublePendulumOnCart(timestep = time, init_angle1 = -0.3, init_angle2 = 0.5, \
            init_angle2_speed = 0, init_angle1_speed = 0, init_x = 1, init_x_speed = 0, g = 9.82)
    C = DoublePendulumOnCartController(P, np.array([[0.],[0.],[0.],[0.]]), outmax = 20, outmin = -20)
    G = StatePlotter(timestep = time)
    Drawer = DrawDoublePendulumOnCart(P, C, G, frameskip = 1 )
    Drawer.startAnimation(speedup = 1)
    G.plotData("state.png", "phase.png")

__main__()
