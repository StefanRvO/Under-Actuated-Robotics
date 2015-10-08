#outputs the controll signal given the state-estimate, x, and the desired state, x_d
#calculates the control kooeficients using python-controlsystem
import numpy as np
import control #Python control systems

class SimplePendulumController:

	def __init__(self, Plant, K, setpoint, outmax = None, outmin = None): #K is a np matrix
		self.outmax = outmax
		self.outmin = outmin
		self.P = Plant
		self.K = K
		self.x_d = setpoint

	def LTIController(self, x):
		return -np.dot(self.K, (x - self.x_d))

	def OutputLimiter(self, u):
		if not (self.outmax == None):
			if u > self.outmax:
				u = self.outmax
		if not (self.outmin == None):
			if u < self.outmin:
				u = self.outmin
		return u

	def calcControlSignal(self, x):
		output = self.LTIController(x)
		output = self.FeedBackCompensator(x, output)
		output = output[0][0]
		return self.OutputLimiter(output)

	def FeedBackCompensator(self, x, v):
		output = self.P.g * self.P.l * self.P.m * np.sin(x[0])
		output = output + (self.P.l ** 2) * v + self.P.b * x[1]
		return output

	def SetNewDesired(self, desired):
		self.x_d = desired


