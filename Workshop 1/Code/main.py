from PendulumWithMovingMass import PendulumWithMovingMass
from DrawPendulumWithMovingMass import DrawPendulum
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import time

def __main__():
	P = PendulumWithMovingMass(K = 20, start_angle = 1, start_x1 = 0.2, timestep = 0.01, M_2 = 1, M_1 = 1)
	Plotter = DrawPendulum(P)
	Plotter.startAnimation()

__main__()
