import numpy as np
import random

class PendulumOnCart:
    #state variables
    angle = 0
    angle_speed = 0
    x = 0
    x_speed = 0

    def __init__(self,g = 9.82, m_c = 1, m = 1, l = 1,  \
        init_angle = 0, init_angle_speed = 0, init_x = 0, timestep = 0.01, \
        init_x_speed = 0,  noise = None ):
        self.timestep = timestep
        self.g = g #acceleration due to gravity
        self.m_c = m_c #mass of cart
        self.m = m #mass of pole
        self.l = l #Lenght of pole
        self.angle = init_angle
        self.angle_speed = init_angle_speed
        self.x = init_x
        self.x_speed = init_x_speed
        self.noise = noise

    def angle_deriv(self, angle_speed):
        return angle_speed

    def angle_speed_deriv(self, angle, angle_speed, control):
        num = control[0][0] * np.cos(angle) + (self.m + self.m_c) * self.g * np.sin(angle) - \
            self.l * self.m * np.cos(angle) * np.sin(angle) * (angle_speed ** 2)

        denom = self.l * (self.m + self.m_c - self.m * (np.cos(angle) ** 2))
        return num / denom

    def x_deriv(self, x_speed):
        return x_speed

    def x_speed_deriv(self, angle, angle_speed, control):
        num = control[0][0] + self.g * self.m * np.cos(angle) * np.sin(angle) - \
            self.l * self.m * np.sin(angle) * (angle_speed ** 2)
        denom = self.m + self.m_c - self.m * (np.cos(angle) ** 2)

        return num / denom

    def Runge_Kutta(self, control_input): #Computation of runge kutta. Need to implement some kind of general method when it become to troublesome this way
        angle_ku1 = self.angle_speed_deriv(self.angle, self.angle_speed, control_input)
        x_ku1     = self.x_speed_deriv(self.angle, self.angle_speed, control_input)
        angle_kt1 = self.angle_deriv(self.angle_speed)
        x_kt1     = self.x_deriv(self.x_speed)
        angle_ku2 = self.angle_speed_deriv(self.angle + angle_kt1 * self.timestep * 0.5, self.angle_speed + angle_ku1 * self.timestep * 0.5, control_input)
        x_ku2     = self.x_speed_deriv(self.angle + angle_kt1 * self.timestep * 0.5, self.angle_speed + angle_ku1 * self.timestep * 0.5, control_input)
        angle_kt2 = self.angle_deriv(self.angle_speed + angle_ku1 * self.timestep * 0.5)
        x_kt2     = self.x_deriv(self.x_speed + x_ku1 * self.timestep * 0.5)
        angle_ku3 = self.angle_speed_deriv(self.angle + angle_kt2 * self.timestep * 0.5, self.angle_speed + angle_ku2 * self.timestep * 0.5, control_input)
        x_ku3     = self.x_speed_deriv(self.angle + angle_kt2 * self.timestep * 0.5, self.angle_speed + angle_ku2 * self.timestep * 0.5, control_input)
        angle_kt3 = self.angle_deriv(self.angle_speed + angle_ku2 * self.timestep * 0.5)
        x_kt3     = self.x_deriv(self.x_speed + x_ku2 * self.timestep * 0.5)
        angle_ku4 = self.angle_speed_deriv(self.angle + angle_kt3 * self.timestep, self.angle_speed + angle_ku3 * self.timestep, control_input)
        x_ku4     = self.x_speed_deriv(self.angle + angle_kt3 * self.timestep, self.angle_speed + angle_ku3 * self.timestep, control_input)
        angle_kt4 = self.angle_deriv(self.angle_speed + angle_ku3 * self.timestep)
        x_kt4     = self.x_deriv(self.x_speed + x_ku3 * self.timestep)

        #calculate new angle speed
        self.angle_speed = self.angle_speed + (self.timestep / 6) * (angle_ku1 + angle_ku2 * 2 + angle_ku3 * 2 + angle_ku4)
        #calculate new x speed
        self.x_speed = self.x_speed + (self.timestep / 6) * (x_ku1 + x_ku2 * 2 + x_ku3 * 2 + x_ku4)
        #calculate new angle
        self.angle = self.angle + (self.timestep / 6) * (angle_kt1 + angle_kt2 * 2 + angle_kt3 * 2 + angle_kt4)
        #calculate new x
        self.x = self.x + (self.timestep / 6) * (x_kt1 + x_kt2 * 2 + x_kt3 * 2 + x_kt4)

    def Euler(self, control_input):
        #progress one step using Euler method

        angle_speed_deriv = self.angle_speed_deriv(self.angle, self.angle_speed, control_input)
        x_speed_deriv = self.x_speed_deriv(self.angle, self.angle_speed, control_input)
        #update anglespeed and xspeed
        self.angle_speed += angle_speed_deriv * self.timestep
        self.x_speed += x_speed_deriv * self.timestep

        angle_deriv = self.angle_deriv(self.angle_speed)
        x_deriv = self.x_deriv(self.x_speed)

        self.angle += angle_deriv * self.timestep
        self.x += x_deriv * self.timestep




    def doStep(self, control_input = np.array([[0]])): #control input is a scalar
        #Do progression
        self.Runge_Kutta(control_input)

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
