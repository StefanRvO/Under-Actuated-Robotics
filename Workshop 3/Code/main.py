#!/usr/bin/python3
from SimplePendulum import SimplePendulum
from DrawSimplePendulum import DrawPendulum
from SimplePendulumController import SimplePendulumController
from StatePlotter import StatePlotter
from math import radians
import numpy as np


def __main__():
    time = 0.01
    P = SimplePendulum(g = 9.82, b = 0.0, m = 1, l = 1, timestep = time, \
        init_angle = radians(20), init_angle_speed = 0.001, noise = np.array([[0], [0]]))

    C = SimplePendulumController(P, K_LQR = np.array([ [200, 30] ]),K_SU = 3000, \
         setpoint = np.array([[np.radians(180)], [0]]), \
	 outmax = 0, outmin = -0, SwingupLimit = radians(1))

    G = StatePlotter(timestep = time)
    Plotter = DrawPendulum(P, C, G, frameskip = 4)
    Plotter.startAnimation(speedup = 0.5)
    G.plotData("state.png", "phase.png")
    
__main__()
