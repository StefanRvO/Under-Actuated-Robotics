import numpy as np
import random

class SimplePendulum:
    #state variables
    angle = 0
    angle_speed = 0
    def __init__(self, b = 0.1, m = 1, g = -9.82, l = 1, timestep = 0.01, \
            init_angle = 0, init_angle_speed = 0, noise = None ):
        self.b = b #damping kooeficient
        self.m = m #mass of pendulum (sphere at the end)
        self.g = g #gravity
        self.l = l #lenght of pendulum
        self.timestep = timestep
        self.angle = init_angle
        self.angle_speed = init_angle_speed
        self.noise = noise
        
    def angle_deriv(self, angle_speed):
        return angle_speed
        
    def angle_speed_deriv(self, angle, angle_speed, control):
        num = self.m * self.g * self.l * np.sin(angle) \
             + self.b * angle_speed - control[0][0]
        denom = self.m * (self.l ** 2)
        return - (num / denom)

    def Runge_Kutta(self, control_input):
	#progress one step using Runge-Kutta method
        ku1 = self.angle_speed_deriv(self.angle, self.angle_speed, control_input)
        kt1 = self.angle_deriv(self.angle_speed)
        ku2 = self.angle_speed_deriv(self.angle + kt1 * self.timestep * 0.5, self.angle_speed + ku1 * self.timestep * 0.5, control_input)
        kt2 = self.angle_deriv(self.angle_speed + ku1 * self.timestep * 0.5)
        ku3 = self.angle_speed_deriv(self.angle + kt2 * self.timestep * 0.5, self.angle_speed + ku2 * self.timestep * 0.5, control_input)
        kt3 = self.angle_deriv(self.angle_speed + ku2 * self.timestep * 0.5)
        ku4 = self.angle_speed_deriv(self.angle + kt3 * self.timestep, self.angle_speed + ku3 * self.timestep, control_input)
        kt4 = self.angle_deriv(self.angle_speed + ku3 * self.timestep)
        
        #calculate new angle speed
        self.angle_speed = self.angle_speed + (self.timestep / 6) * (ku1 + ku2 * 2 + ku3 * 2 + ku4)
        #calculate new angle
        self.angle = self.angle + (self.timestep / 6) * (kt1 + kt2 * 2 + kt3 * 2 + kt4)

    def Euler(self, control_input):
	#progress one step using Euler method
        ku1 = self.angle_speed_deriv(self.angle, self.angle_speed, control_input)
        #calculate new angle speed
        self.angle_speed = self.angle_speed + self.timestep * ku1
        kt1 = self.angle_deriv(self.angle_speed)
        #calculate new angle
        self.angle = self.angle + self.timestep * kt1

    def doStep(self, control_input = np.array([[0]])): #control input is a scalar
    	#Do progression
        self.Euler(control_input)

        #returns a np matrix of size 2 x 1
        output = np.array([ [self.angle], [self.angle_speed] ])
        #apply noise to the output
        output = self.outputNoisyfier(output)
        return output

    def outputNoisyfier(self, clean_out):
        if(self.noise is None):
            return clean_out
        
        dirty = np.dot(self.noise, random.uniform(-0.5, 0.5))    
        return clean_out + dirty
