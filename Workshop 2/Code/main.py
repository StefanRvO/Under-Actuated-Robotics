#!/usr/bin/python3

from SimplePendulum import SimplePendulum
from DrawSimplePendulum import DrawPendulum
from SimplePendulumController import SimplePendulumController
from math import radians
import numpy as np
def __main__():
	P = SimplePendulum(g = -9.82, b = 1, m = 1, l = 1, timestep = .01, \
		init_angle = radians(5), init_angle_speed = 0)
	C = SimplePendulumController(P, outmax = None, outmin = None)
	Plotter = DrawPendulum(P, C)
	Plotter.startAnimation()
	
__main__()
