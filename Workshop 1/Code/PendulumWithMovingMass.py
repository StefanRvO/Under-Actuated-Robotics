import numpy as np
class PendulumWithMovingMass:
	K = 0 #Spring constant
	L_0 = 0 #Lenght of unstretched spring
	L_1 = 0 #Lenght of Rod
	L_2 = 0 #Lenght of Sleeve
	M_1 = 0	#Mass of Rod
	M_2 = 0 #Mass of Sleeve
	g   = 0 #Gravitational acceleration
	I_zz1 = 0 #Moment of inertia of rod arround the point it swings arround
	I_zz2 = 0 #Moment of inertia of sleeve around it's center of mass
	angle = 0 #Angle of rod
	angle_speed = 0 #speed of angle change
	angle_acceleration = 0 #acceleration af angle change
	x1 = 0	#displacement of sleeve down the rod
	x1_speed = 0 #speed of displacement change
	x1_acceleration = 0 #acceleration af displacement change.
	timestep = 0


	def __init__(self, K = 10, L_0 = 0., L_1 = 1., L_2 = 0.1, 	\
				 M_1 = 1., M_2 = 0.5, g = 9.82,   	\
				 I_zz2 = 0.1, 		\
				 start_angle = 0., start_x1 = 0.,	\
				 timestep = 0.01 ):
		self.K = K
		self.L_0 = L_0
		self.L_1 = L_1
		self.L_2 = L_2
		self.M_1 = M_1
		self.M_2 = M_2
		self.g = g
		self.I_zz1 = M_1 * L_1 * L_1 / 3. #Rod rotating around the end
		self.I_zz2 = M_2 / 2 * L_2/2 * L_2/2  #pointmass around point
		self.angle = start_angle
		self.x1 = start_x1
		self.timestep = timestep
	def doStep(self):
		#calculate the accelerations
		self.angle_acceleration = self.M_2 * self.x1 * (-2. * self.x1_speed*self.angle_speed - self.g * np.sin(self.angle))
		self.angle_acceleration = self.angle_acceleration - self.M_1 * self.g * self.L_1 / 2. * np.sin(self.angle)
		self.angle_acceleration = self.angle_acceleration / (self.I_zz1 + self.I_zz2 + self.M_2 * self.x1 * self.x1)
		self.x1_acceleration = self.x1 * self.angle_speed * self.angle_speed
		self.x1_acceleration = self.x1_acceleration + self.K * self.L_0 / self.M_2 + 1. / 2. * self.K * self.L_2 / self.M_2 - self.K * self.x1 / self.M_2
		self.x1_acceleration = self.x1_acceleration + self.g * np.cos(self.angle)
		#calculate speeds
		self.x1_speed = self.x1_speed + self.x1_acceleration * self.timestep
		self.angle_speed = self.angle_speed + self.angle_acceleration * self.timestep
		
		#calculate the positions
		self.x1 = self.x1 + self.x1_speed * self.timestep
		self.angle = self.angle + self.angle_speed * self.timestep
		
		#the sleeve should not be able to move outside the rod
		self.x1 = min(self.x1, self.L_1)
		self.x1 = max(self.x1, self.L_2)
			

		
	
