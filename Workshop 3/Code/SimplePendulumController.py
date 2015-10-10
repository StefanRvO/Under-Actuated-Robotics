#outputs the controll signal given the state-estimate, x, and the desired state, x_d
#calculates the control kooeficients using python-controlsystem
import numpy as np
import control #Python control systems

def fixify_angle(angle, fixpoint):
	times = (angle - fixpoint / 2*np.pi)
	while angle > fixpoint + np.pi:
		angle -= 2*np.pi
	while angle < fixpoint - np.pi:
		angle += 2*np.pi
	return angle

class SimplePendulumController:

	def __init__(self, Plant, K_LQR, K_SU, setpoint, outmax = None, outmin = None, SwingupLimit = np.radians(5)): #K is a np matrix
		self.outmax = outmax
		self.outmin = outmin
		self.P = Plant
		self.K_LQR = K_LQR
		self.K_SU = K_SU
		self.x_d = setpoint
		self.P.angle_setpoint = setpoint[0][0]
		self.SwingupLimit = SwingupLimit

	def PendulumEnergy(self, state):
		energy = self.P.m * self.P.l * self.P.g * (1 + np.cos(state[0][0] + np.pi))#positional energy
		energy += 0.5 * self.P.m * (self.P.l ** 2) * (state[1][0] ** 2)#Rotational energy
		return energy

	def SwingUpController(self, x, setpoint):
		desiredEnergy = self.PendulumEnergy(setpoint)
		currentEnergy = self.PendulumEnergy(x)
		print(currentEnergy)
		delta_energy = currentEnergy - desiredEnergy
		out = [[-self.K_SU * x[1][0] * delta_energy]]
		return out

	def TopController(self, x, setpoint):
		return -np.dot(self.K_LQR, (x - setpoint))

	def LTIController(self, x, setpoint):
		if( abs(x[0][0] - setpoint[0][0]) < self.SwingupLimit):
			return self.TopController(x, setpoint)
		else:
			return self.SwingUpController(x, setpoint)

	def OutputLimiter(self, u):
		if not (self.outmax == None):
			u[0][0] = min(u[0][0], self.outmax)
		if not (self.outmin == None):
			u[0][0] = max(u[0][0], self.outmin)
		return u

	def calcControlSignal(self, x):
		x[0][0] = fixify_angle(x[0][0], self.x_d[0][0])
		#print(x[0][0])
		output = self.LTIController(x, self.x_d)
		#output = self.FeedBackCompensator(x, output)
		output = self.OutputLimiter(output)
		return self.OutputLimiter(output)

	def FeedBackCompensator(self, x, v):
		output = self.P.g * self.P.l * self.P.m * np.sin(x[0][0])
		output = output + (self.P.l ** 2) * v + self.P.b * x[1][0]
		return output

	def SetNewDesired(self, desired):
		self.x_d = desired
		self.P.angle_setpoint = setpoint[0][0]


