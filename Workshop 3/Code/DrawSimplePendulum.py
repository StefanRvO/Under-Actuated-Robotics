import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

class DrawPendulum:
	def __init__(self,Pendulum, Controller, Graph, frameskip = 1):
		self.P = Pendulum
		self.C = Controller
		self.G = Graph
		self.lastout = np.array([ [self.P.angle], [self.P.angle_speed] ])
		self.frameskip = frameskip

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
		for i in range(self.frameskip):
			self.SimStep()

		self.rod.set_data([0, self.P.l * np.cos(self.P.angle - np.pi / 2)], [0, self.P.l * np.sin(self.P.angle - np.pi / 2)])
		self.end_dot.center = (self.P.l * np.cos(self.P.angle - np.pi / 2), self.P.l * np.sin(self.P.angle - np.pi / 2))
		return self.rod, self.end_dot,

	def SimStep(self):
		self.G.recordDataPoint(self.lastout)
		control = self.C.calcControlSignal(self.lastout)
		self.lastout = self.P.doStep(control)
		self.G.recordControlSignal(control)

	#begin the animation
	def startAnimation(self, speedup = 1, runtime = None, filename = None):
		
		if not runtime is None: #If runtime restriction is given, only run for this number of seconds
			number_of_steps = int(runtime / self.P.timestep)
			number_of_frames = int(number_of_steps / self.frameskip)
			anim = animation.FuncAnimation(self.fig, self.animate, init_func = self.init, frames = number_of_frames, interval = (1000. * self.P.timestep) / speedup, blit=True, repeat = False)
			if not filename is None: #If filename is given, save animation as file
				anim.save(filename, fps = 60)
			else:
				plt.show()
		else:	#If no runtime restriction, run until window is closed
			anim = animation.FuncAnimation(self.fig, self.animate, init_func = self.init, interval = (1000. * self.P.timestep) / speedup, blit=True, repeat = False)
			plt.show()

	def startSimulation(self, runtime):
		number_of_steps = int(runtime / self.P.timestep)
		for i in range(number_of_steps):
			self.SimStep()

