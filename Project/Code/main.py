#!/usr/bin/python3
from DoublePendulumOnCart import DoublePendulumOnCart
from DrawDoublePendulumOnCart import DrawDoublePendulumOnCart
from DoublePendulumOnCartController import DoublePendulumOnCartController
from StatePlotter import StatePlotter
from math import radians
import numpy as np


def __main__():
    time = 0.01
    P = DoublePendulumOnCart(timestep = time, init_angle1 = -3 , init_angle2 = -5,  \
            init_angle1_speed = -2, init_angle2_speed = -7, init_x = 8, init_x_speed = 5, g = 9.82)
    C = DoublePendulumOnCartController(P, np.array([[0.],[0.],[0.],[0.]]), outmax = 20, outmin = -20)
    G = StatePlotter(timestep = time)
    Drawer = DrawDoublePendulumOnCart(P, C, G, frameskip = 1 )
    Drawer.startAnimation(speedup = 5)
    G.plotData("state.png", "phase.png")

__main__()