from PendulumWithMovingMass import PendulumWithMovingMass
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import time

class DrawPendulum:
	def __init__(self,Pendulum):
		self.P = Pendulum

		#setup matplotlib stuff
		self.fig = plt.figure(num=None, figsize=(8, 7), dpi=80, facecolor='w', edgecolor='k')
		self.ax = plt.axes(xlim=(-self.P.L_1 * 1.2, self.P.L_1 * 1.2), ylim =(-self.P.L_1 * 1.2, self.P.L_1 * 1.2))
		self.rod, = self.ax.plot([], [], 'o-', lw=2)
		self.sleeve = plt.Circle((self.P.L_1 * 10000, self.P.L_1 * 10000), 0.05, fc='g') #create Off-screen, as we need to initialize for blitting
		self.ax.add_patch(self.sleeve)
		
	#init function, setup background of each frame
	def init(self):
		self.rod.set_data([], [])
		self.ax.add_patch(self.sleeve)
		return self.rod, self.sleeve,

	#animation function
	def animate(self, i):
		self.P.doStep()
		self.rod.set_data([0, self.P.L_1 * np.cos(self.P.angle - np.pi / 2)], [0, self.P.L_1 * np.sin(self.P.angle - np.pi / 2)])
		self.sleeve.center = (self.P.x1 * np.cos(self.P.angle - np.pi / 2), self.P.x1 * np.sin(self.P.angle - np.pi / 2))
		return self.rod, self.sleeve,

	#begin the animation
	def startAnimation(self):
		self.anim = animation.FuncAnimation(self.fig, self.animate, init_func = self.init, frames = 60, interval = 1000 * self.P.timestep, blit=True)
		plt.show()
