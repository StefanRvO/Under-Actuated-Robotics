#outputs the controll signal given the state-estimate, x, and the desired state, x_d

class SimplePendulumController:
	def __init__(self, K, outmax = None, outmin = None): #K is a np matrix
		self.K = K
		self.outmax = outmax
		self.outmin = outmin

	def calcControlSignal(self, x, x_d):
		output = (x_d - x) * self.K
		output = output[0][0]
		if not (self.outmax == None):
			if output > self.outmax:
				output = self.outmax
		if not (self.outmin == None):
			if output < self.outmin:
				output = self.outmin
		return output
