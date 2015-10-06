import numpy as np
class SimplePendulum:
	#state variables
	angle = 0
	angle_speed = 0
	def __init__(self, b = 0, m = 1, g = -9.82, l = 1, timestep = 0.01, \
			init_angle = 0, init_angle_speed = 0 ):
		self.b = b #damping kooeficient
		self.m = m #mass of pendulum (sphere at the end)
		self.g = g #gravity
		self.l = l #lenght of pendulum
		self.timestep = timestep
		self.angle = init_angle
		self. angle_speed = init_angle_speed
	def doStep(self, control_input = np.array([[0]])): #control input is a scalar
		#calculate numerator
		num = self.m * self.g * self.l * np.sin(self.angle) \
			 + self.b * self.angle_speed - control_input
		#calculate demominator
		denom = self.m * (self.l ** 2)
		#calculate acceleration
		angle_acc = - (num / denom)
		#calculate new angle speed
		self.angle_speed = self.angle_speed + angle_acc * self.timestep
		#calculate new angle
		self.angle = self.angle + self.angle_speed * self.timestep
		
		#returns a np matrix of size 2 x 1
		return np.array([ [self.angle], [self.angle_speed] ])
			

		
	
