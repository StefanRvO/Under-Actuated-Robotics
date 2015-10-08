import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

class DrawPendulum:
	def __init__(self,Pendulum, Controller):
		self.P = Pendulum
		self.C = Controller
		self.lastout = np.array([ [self.P.angle], [self.P.angle_speed] ])

		#setup matplotlib stuff
		self.fig = plt.figure(num=None, figsize=(8, 7), dpi=80, facecolor='w', edgecolor='k')
		self.ax = plt.axes(xlim=(-self.P.l * 1.2, self.P.l * 1.2), ylim =(-self.P.l * 1.2, self.P.l * 1.2))
		self.rod, = self.ax.plot([], [], 'o-', lw=2)
		self.end_dot = plt.Circle((self.P.l * 10000, self.P.l * 10000), 0.05, fc='g') #create Off-screen, as we need to initialize for blitting
		self.ax.add_patch(self.end_dot)
		
	#init function, setup background of each frame
	def init(self):
		self.rod.set_data([], [])
		self.ax.add_patch(self.end_dot)
		return self.rod, self.end_dot,

	#animation function
	def animate(self, i):
		control = self.C.calcControlSignal(self.lastout)
		self.lastout = self.P.doStep(control)
		#print(self.P.angle_speed)
		self.rod.set_data([0, self.P.l * np.cos(self.P.angle + np.pi / 2)], [0, self.P.l * np.sin(self.P.angle + np.pi / 2)])
		self.end_dot.center = (self.P.l * np.cos(self.P.angle + np.pi / 2), self.P.l * np.sin(self.P.angle + np.pi / 2))
		return self.rod, self.end_dot,

	#begin the animation
	def startAnimation(self):
		self.anim = animation.FuncAnimation(self.fig, self.animate, init_func = self.init, frames = 60, interval = 1000 * self.P.timestep, blit=True)
		plt.show()
