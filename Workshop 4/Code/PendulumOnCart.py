import numpy as np
import random

class PendulumOnCart:
    #state variables
    angle = 0
    angle_speed = 0
    x = 0
    x_speed = 0

    def __init__(self,g = -9.82, m_c = 1, m = 1, l = 1, fric_c = 0, fric_p = 0, \
        init_angle = 0, init_angle_speed = 0, init_x = 0, timestep = 0.01, \
        init_x_speed = 0,  noise = None ):
        self.timestep = timestep
        self.g = g #acceleration due to gravity
        self.m_c = m_c #mass of cart
        self.m = m #mass of pole
        self.l = l #Lenght of pole
        self.fric_c = fric_c # cooeficient of friction on cart track
        self.fric_p = fric_p #cooeficient of friction on pole
        self.angle = init_angle
        self.angle_speed = init_angle_speed
        self.x = init_x
        self.x_speed = init_x_speed
        self.noise = noise

    def angle_deriv(self, angle_speed):
        return angle_speed

    def angle_speed_deriv(self, angle, angle_speed, x, x_speed, control):
        #use the equation in (https://webdocs.cs.ualberta.ca/~sutton/papers/barto-sutton-anderson-83.pdf)
        #In the equation the use half pole lenght, we use full lenght
        #Factor multiplied to the cos part in the numerator

        cos_factors_num = - control[0][0] - self.m * (self.l / 2) * (angle_speed ** 2) \
            * np.sin(angle) + self.fric_c * np.sign(x_speed)
        cos_factors = cos_factors_num / (self.m_c + self.m)

        #numerator of equation
        num = self.g * np.sin(angle) + np.cos(angle) * cos_factors - \
            (self.fric_p * angle_speed) / (self.m * self.l / 2)
        #denomintor of equation
        denom = (self.l / 2) * ((4./3.) - (self.m * (np.cos(angle) ** 2) / (self.m_c + self.m)))

        return num / denom

    def x_deriv(self, x_speed):
        return x_speed

    def x_speed_deriv(self, angle, angle_speed, angle_speed_deriv, x, x_speed, control):
        #use the equation in (https://webdocs.cs.ualberta.ca/~sutton/papers/barto-sutton-anderson-83.pdf)
        #In the equation the use half pole lenght, we use full lenght
        num = control[0][0] + self.m * (self.l / 2) * \
            ( (angle_speed ** 2) * np.sin(angle) - angle_speed_deriv * np.cos(angle) ) - \
            self.fric_c * np.sign(x_speed)
        denom = self.m_c + self.m

        return num / denom

    def Euler(self, control_input):
        #progress one step using Euler method

        angle_speed_deriv = self.angle_speed_deriv(self.angle, self.angle_speed, self.x, self.x_speed, control_input)
        x_speed_deriv = self.x_speed_deriv(self.angle, self.angle_speed, angle_speed_deriv, self.x, self.x_speed, control_input)
        #update anglespeed and xspeed
        self.angle_speed += angle_speed_deriv * self.timestep
        self.x_speed += x_speed_deriv * self.timestep

        angle_deriv = self.angle_deriv(self.angle_speed)
        x_deriv = self.x_deriv(self.x_speed)

        self.angle += angle_deriv * self.timestep
        self.x += x_deriv * self.timestep




    def doStep(self, control_input = np.array([[0]])): #control input is a scalar
        #Do progression
        self.Euler(control_input)

        #returns a np matrix of size 2 x 1
        output = np.array([ [self.angle], [self.angle_speed], [self.x], [self.x_speed] ])
        #apply noise to the output
        output = self.outputNoisyfier(output)
        return output

    def outputNoisyfier(self, clean_out):
        if(self.noise is None):
            return clean_out

        dirty = np.dot(self.noise, random.uniform(-0.5, 0.5))
        return clean_out + dirty
