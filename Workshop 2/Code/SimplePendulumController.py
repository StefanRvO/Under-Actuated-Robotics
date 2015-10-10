#outputs the controll signal given the state-estimate, x, and the desired state, x_d
#calculates the control kooeficients using python-controlsystem
import numpy as np
import control #Python control systems

class SimplePendulumController:

	def __init__(self, Plant, outmax = None, outmin = None): #K is a np matrix
		self.K = self.calcK(Plant)
		self.outmax = outmax
		self.outmin = outmin
	def calcControlSignal(self, x, x_d):
		output = -np.dot(self.K, (x - x_d))
		output = output[0][0]
		if not (self.outmax == None):
			if output > self.outmax:
				output = self.outmax
		if not (self.outmin == None):
			if output < self.outmin:
				output = self.outmin
		return output

	def calcK(self, Plant):
		#Calculates the K matrix using python control systems and lqr pole placement
		#Define A matrix
		A = [ [0, 1], [-Plant.g / Plant.l, -Plant.b / (Plant.m * (Plant.l ** 2))] ]
		#Define B matrix
		B = [ [0], [1 / (Plant.m * (Plant.l ** 2))] ]
		#define C matrix. It's just the identity
		C = np.identity(2)
		#The D matrix is just 0
		D = [ [0], [0] ]

		#Create the statespace representation
		sys = control.ss(A, B, C, D)

		#Create the state weight matrix, weight position 20000, and speed 100
		Q = [ [20000, 0], [0, 100] ]
		#Create input weight matrix
		R = [ [1] ]
		#Compute LQR
		LQR_vals = control.lqr(sys, Q, R)
		return LQR_vals[0] #return K values of LQR return


