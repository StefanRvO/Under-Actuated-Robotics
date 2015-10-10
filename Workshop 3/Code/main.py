#!/usr/bin/python3

from SimplePendulum import SimplePendulum
from DrawSimplePendulum import DrawPendulum
from SimplePendulumController import SimplePendulumController
from StatePlotter import StatePlotter
from math import radians
import numpy as np
def __main__():
	P = SimplePendulum(g = 9.82, b = 0, m = 1, l = 1, timestep = .01, \
		init_angle = radians(0), init_angle_speed = 0.00000001, noise = np.array([[0], [0]]))

	C = SimplePendulumController(P, K_LQR = np.array([ [120, 30] ]),K_SU = 3000, \
		 setpoint = np.array([[np.radians(180)], [0]]), outmax = .5, outmin = -.5)
	G = StatePlotter(timestep = 0.01)
	Plotter = DrawPendulum(P, C, G)
	Plotter.startAnimation(speedup = 5)
	G.plotData("state.png", "phase.png")
	
__main__()
