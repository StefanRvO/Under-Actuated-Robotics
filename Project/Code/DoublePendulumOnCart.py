import numpy as np
from numpy import cos, sin
import random

class DoublePendulumOnCart:
    #state variables
    angle1 = 0
    angle1_speed = 0
    angle2 = 0
    angle2_speed = 0
    x = 0
    x_speed = 0

    def __init__(self,g = 9.82, m = 1, m1 = 1, m2 = 1, L1 = 1, L2 = 1,  \
        init_angle1 = 0, init_angle1_speed = 0, init_angle2 = 0, init_angle2_speed = 0, init_x = 0, timestep = 0.01, \
        init_x_speed = 0, noise = None ):
        self.timestep = timestep
        self.g = g #acceleration due to gravity
        self.m = m #mass of cart
        self.m1 = m1 #mass of pole
        self.m2 = m2
        self.L1 = L1 #Lenght of pole1
        self.L2 = L2
        self.angle1 = init_angle1
        self.angle1_speed = init_angle1_speed
        self.angle2 = init_angle2
        self.angle2_speed = init_angle2_speed
        self.x = init_x
        self.x_speed = init_x_speed
        self.noise = noise
        self.l1 = L1/2.
        self.l2 = L2/2.
        ####Calculate Moment of inertia####
        self.J1 = self.m1 * self.L1 ** 2 / 12.
        self.J2 = self.m2 * self.L2 ** 2 / 12.
        ####Define some constants to make calculations easier####
        self.h1 = self.m + self.m1 + self.m2
        self.h2 = self.m1 * self.l1 + self.m2 * self.L1
        self.h3 = self.m2 * self.l2
        self.h4 = self.m1 * (self.l1 ** 2) + m2 * (self.L1 ** 2) + self.J1
        self.h5 = self.m2 * self.l2 * self.L1
        self.h6 = self.m2 * (self.l2 ** 2) + self.J2
        self.h7 = self.m1 * self.l1 * self.g + self.m2 * self.L1 * self.g
        self.h8 = self.m2 * self.l2 * self.g
    def angle1_deriv(self, angle1_speed):
        return angle1_speed
    def angle2_deriv(self, angle2_speed):
        return angle2_speed

    def x_deriv(self, x_speed):
        return x_speed

    def angle1_speed_deriv(self, angle1, angle1_speed, angle2, angle2_speed, x, x_speed, control):
        num1 = (-self.h2 * self.h6 * cos(angle1) + self.h3 * self.h5 * cos(angle1 - angle2) * cos(angle2)) * control[0][0]
        num2 = self.h1* self.h6 * self.h7 * sin(angle1)
        num3 = -(self.h3 ** 2) * self.h7 * (cos(angle2) ** 2) * sin(angle1)
        num4 =  -self.h1 * self.h5 * self.h8 * cos(angle1 - angle2) * sin(angle2)
        num5 = self.h2 * self.h3 * self.h8 * cos(angle1) * cos(angle2) * sin(angle2)
        num6 = -1/2. * (self.h2 * (-self.h3 * self.h5 + self.h2 * self.h6) * sin(2 * angle1) + \
            + self.h5 * (-self.h2 * self.h3 + self.h1 * self.h5) * sin(2 * (angle1 - angle2)))  * (angle1_speed ** 2)
        num7 =  - self.h1 * self.h5 * self.h6 * sin(angle1 - angle2) * (angle2_speed ** 2)
        num8 =  (self.h3 ** 2) * self.h5 * (cos(angle2) ** 2) * sin(angle1 - angle2) * (angle2_speed ** 2)
        num9 =  -self.h2 * self.h3 * self.h6 * cos(angle1) * sin(angle2) * (angle2_speed ** 2)
        num10 = (self.h3 ** 2) * self.h5 * cos(angle1 - angle2) * cos(angle2) * sin(angle2) * (angle2_speed ** 2)
        num = num1 + num2 + num3 + num4 + num5 + num6 + num7 + num8 +num9 + num10
        #print(num)
        denom1 = self.h1 * self.h4 * self.h6
        denom2 = -(self.h2 ** 2) * self.h6 * (cos(angle1) ** 2)
        denom3 =  - self.h1 * (self.h5 ** 2) * (cos(angle1 - angle2) ** 2)
        denom4 = 2 * self.h2 * self.h3 * self.h5 * cos(angle1) * cos(angle1 - angle2)* cos(angle2)
        denom5 = -(self.h3 ** 2) * self.h4 * (cos(angle2) ** 2)
        denom = denom1 + denom2 + denom3 + denom4 + denom5
        #print(denom)
        return num / denom

    def angle2_speed_deriv(self, angle1, angle1_speed, angle2, angle2_speed, x, x_speed, control):
        num1 = (self.h2 * self.h5 * cos(angle1) * cos(angle1 - angle2) - self.h3 * self.h4 * cos(angle2)) * control[0][0]
        num2 = - self.h1 * self.h5 * self.h7 * cos(angle1 - angle2) * sin(angle1)
        num3 = self.h2 * self.h3 * self.h7 * cos(angle1) * cos(angle2) * sin(angle1)
        num4 = self.h1 * self.h4 * self.h8 * sin(angle2)
        num5 = - (self.h2 ** 2) * self.h8 * (cos(angle1) ** 2) * sin(angle2)
        num6 =(-self.h4 * (self.h2 * self.h3 - self.h1 * self.h5) * cos(angle2) * sin(angle1) + ( (self.h2 ** 2) - self.h1 * self.h4) * self.h5 * cos(angle1) * sin(angle2)) * (angle1_speed ** 2)
        num7 = - self.h2 * self.h3 * self.h5 * cos(angle1) * cos(angle2) * sin(angle1 - angle2) * (angle2_speed ** 2)
        num8 = 1/2. * self.h1 * (self.h5 ** 2) * sin(2*(angle1 - angle2)) * (angle2_speed ** 2)
        num9 = self.h2 * self.h3 * self.h5 * cos(angle1) * cos(angle1 - angle2) *sin(angle2) * (angle2_speed ** 2)
        num10 = -(self.h3 ** 2) * self.h4 * cos(angle2) * sin(angle2) * (angle2_speed ** 2)
        num = num1 + num2 + num3 + num4 + num5 +num6 + num7 + num8 + num9 + num10
        #print(num)
        denom1 =  self.h1 * self.h4 * self.h6
        denom2 = -  (self.h2 ** 2) * self.h6 * (cos(angle1) ** 2)
        denom3 = -  self.h1 * (self.h5 ** 2) * (cos(angle1 - angle2) ** 2)
        denom4 = 2 * self.h2 * self.h3 * self.h5 * cos(angle1) * cos(angle1 - angle2) * cos(angle2)
        denom5 = -  (self.h3 ** 2) * self.h4 * (cos(angle2) ** 2)
        denom = denom1 + denom2 + denom3 + denom4 + denom5
        #print(denom)
        return num / denom

    def x_speed_deriv(self, angle1, angle1_speed, angle2, angle2_speed, x, x_speed, control):
        #This is a long expression, derived in the mathematica notebook
        num1_1 = (-self.h2 * self.h6 * cos(angle1) + self.h3 * self.h5 * cos(angle1 - angle2) * cos(angle2))

        num1_2 = (-self.h6 * self.h7 * sin(angle1)) + \
            (self.h5 * cos(angle1 - angle2)) * (self.h8 * sin(angle2) + self.h5 * sin(angle1 - angle2) * (angle1_speed ** 2)) + \
            (self.h5 * self.h6 * sin(angle1 - angle2) * (angle2_speed ** 2))

        num1 = num1_1 * num1_2
        #print(num1)
        num2_1 = (self.h4 * self.h6 - (self.h5 ** 2) * (cos(angle1 - angle2) ** 2))
        num2_2 = self.h6 * control[0][0] + \
            (self.h2 * self.h6 * sin(angle1) - self.h3 * self.h5 * cos(angle2) * sin(angle1 - angle2)) * (angle1_speed ** 2) + \
            self.h3 * sin(angle2) * (-self.h8 * cos(angle2) + self.h6 * (angle2_speed ** 2) )

        num2 = num2_1 * num2_2
        #print(num2)
        num = num1 - num2
        denom = self.h6 * \
        (self.h1 * self.h4 * self.h6 - (self.h2 ** 2) * self.h6 * (cos(angle1) ** 2 ) - self.h1 * (self.h5 ** 2) * (cos(angle1 - angle2) ** 2) + \
        2 * self.h2 * self.h3 * self.h5 * cos(angle1) * cos(angle1 - angle2) * cos(angle2) - (self.h3 ** 2) * self.h4 * (cos(angle2) ** 2))

        return -num / denom

    def Runge_Kutta(self, control_input): #Computation of runge kutta. Need to implement some kind of general method when it become to troublesome this way
        angle1_ku1 = self.angle1_speed_deriv(self.angle1, self.angle1_speed, self.angle2, self.angle2_speed, self.x, self.x_speed, control_input)
        angle2_ku1 = self.angle2_speed_deriv(self.angle1, self.angle1_speed, self.angle2, self.angle2_speed, self.x, self.x_speed, control_input)
        x_ku1      = self.x_speed_deriv(self.angle1, self.angle1_speed, self.angle2, self.angle2_speed, self.x, self.x_speed, control_input)
        angle1_kt1 = self.angle1_deriv(self.angle1_speed)
        angle2_kt1 = self.angle2_deriv(self.angle2_speed)
        x_kt1     = self.x_deriv(self.x_speed)

        angle1_ku2 = self.angle1_speed_deriv(self.angle1 + angle1_kt1 * self.timestep * 0.5, self.angle1_speed + angle1_ku1 * self.timestep * 0.5 \
                    , self.angle2 + angle2_kt1 * self.timestep * 0.5, self.angle2_speed + angle2_ku1 * self.timestep * 0.5 \
                    ,self.x + x_kt1 * self.timestep * 0.5, self.x_speed + x_ku1 * self.timestep * 0.5, control_input)
        angle2_ku2 = self.angle2_speed_deriv(self.angle1 + angle1_kt1 * self.timestep * 0.5, self.angle1_speed + angle1_ku1 * self.timestep * 0.5 \
                    , self.angle2 + angle2_kt1 * self.timestep * 0.5, self.angle2_speed + angle2_ku1 * self.timestep * 0.5 \
                    ,self.x + x_kt1 * self.timestep * 0.5, self.x_speed + x_ku1 * self.timestep * 0.5, control_input)
        x_ku2      = self.x_speed_deriv(self.angle1 + angle1_kt1 * self.timestep * 0.5, self.angle1_speed + angle1_ku1 * self.timestep * 0.5 \
                    , self.angle2 + angle2_kt1 * self.timestep * 0.5, self.angle2_speed + angle2_ku1 * self.timestep * 0.5 \
                    ,self.x + x_kt1 * self.timestep * 0.5, self.x_speed + x_ku1 * self.timestep * 0.5, control_input)
        angle1_kt2 = self.angle1_deriv(self.angle1_speed + angle1_ku1 * self.timestep * 0.5)
        angle2_kt2 = self.angle2_deriv(self.angle2_speed + angle2_ku1 * self.timestep * 0.5)
        x_kt2     = self.x_deriv(self.x_speed + x_ku1 * self.timestep * 0.5)

        angle1_ku3 = self.angle1_speed_deriv(self.angle1 + angle1_kt2 * self.timestep * 0.5, self.angle1_speed + angle1_ku2 * self.timestep * 0.5 \
                    , self.angle2 + angle2_kt2 * self.timestep * 0.5, self.angle2_speed + angle2_ku2 * self.timestep * 0.5 \
                    ,self.x + x_kt2 * self.timestep * 0.5, self.x_speed + x_ku2 * self.timestep * 0.5, control_input)
        angle2_ku3 = self.angle2_speed_deriv(self.angle1 + angle1_kt2 * self.timestep * 0.5, self.angle1_speed + angle1_ku2 * self.timestep * 0.5 \
                    , self.angle2 + angle2_kt2 * self.timestep * 0.5, self.angle2_speed + angle2_ku2 * self.timestep * 0.5 \
                    ,self.x + x_kt2 * self.timestep * 0.5, self.x_speed + x_ku2 * self.timestep * 0.5, control_input)
        x_ku3      = self.x_speed_deriv(self.angle1 + angle1_kt2 * self.timestep * 0.5, self.angle1_speed + angle1_ku2 * self.timestep * 0.5 \
                    , self.angle2 + angle2_kt2 * self.timestep * 0.5, self.angle2_speed + angle2_ku2 * self.timestep * 0.5 \
                    ,self.x + x_kt2 * self.timestep * 0.5, self.x_speed + x_ku2 * self.timestep * 0.5, control_input)
        angle1_kt3 = self.angle1_deriv(self.angle1_speed + angle1_ku2 * self.timestep * 0.5)
        angle2_kt3 = self.angle2_deriv(self.angle2_speed + angle2_ku2 * self.timestep * 0.5)
        x_kt3      = self.x_deriv(self.x_speed + x_ku2 * self.timestep * 0.5)

        angle1_ku4 = self.angle1_speed_deriv(self.angle1 + angle1_kt3 * self.timestep, self.angle1_speed + angle1_ku3 * self.timestep \
                    , self.angle2 + angle2_kt3 * self.timestep, self.angle2_speed + angle2_ku3 * self.timestep \
                    ,self.x + x_kt3 * self.timestep, self.x_speed + x_ku3 * self.timestep, control_input)
        angle2_ku4 = self.angle2_speed_deriv(self.angle1 + angle1_kt3 * self.timestep, self.angle1_speed + angle1_ku3 * self.timestep \
                    , self.angle2 + angle2_kt3 * self.timestep, self.angle2_speed + angle2_ku3 * self.timestep \
                    ,self.x + x_kt3 * self.timestep, self.x_speed + x_ku3 * self.timestep, control_input)
        x_ku4      = self.x_speed_deriv(self.angle1 + angle1_kt3 * self.timestep, self.angle1_speed + angle1_ku3 * self.timestep \
                    , self.angle2 + angle2_kt3 * self.timestep, self.angle2_speed + angle2_ku3 * self.timestep \
                    ,self.x + x_kt3 * self.timestep, self.x_speed + x_ku3 * self.timestep, control_input)
        angle1_kt4 = self.angle1_deriv(self.angle1_speed + angle1_ku3 * self.timestep)
        angle2_kt4 = self.angle2_deriv(self.angle2_speed + angle2_ku3 * self.timestep)
        x_kt4      = self.x_deriv(self.x_speed + x_ku3 * self.timestep)

        #calculate new angle speed
        self.angle1_speed = self.angle1_speed + (self.timestep / 6) * (angle1_ku1 + angle1_ku2 * 2 + angle1_ku3 * 2 + angle1_ku4)
        self.angle2_speed = self.angle2_speed + (self.timestep / 6) * (angle2_ku1 + angle2_ku2 * 2 + angle2_ku3 * 2 + angle2_ku4)
        #calculate new x speed
        self.x_speed = self.x_speed + (self.timestep / 6) * (x_ku1 + x_ku2 * 2 + x_ku3 * 2 + x_ku4)
        #calculate new angle
        self.angle1 = self.angle1 + (self.timestep / 6) * (angle1_kt1 + angle1_kt2 * 2 + angle1_kt3 * 2 + angle1_kt4)
        self.angle2 = self.angle2 + (self.timestep / 6) * (angle2_kt1 + angle2_kt2 * 2 + angle2_kt3 * 2 + angle2_kt4)
        #calculate new x
        self.x = self.x + (self.timestep / 6) * (x_kt1 + x_kt2 * 2 + x_kt3 * 2 + x_kt4)

    def Euler(self, control_input):
        #progress one step using Euler method
        angle1_speed_deriv = self.angle1_speed_deriv(self.angle1, self.angle1_speed, self.angle2, self.angle2_speed, self.x, self.x_speed, control_input)
        angle2_speed_deriv = self.angle2_speed_deriv(self.angle1, self.angle1_speed, self.angle2, self.angle2_speed, self.x, self.x_speed, control_input)
        x_speed_deriv = self.x_speed_deriv(self.angle1, self.angle1_speed, self.angle2, self.angle2_speed, self.x, self.x_speed, control_input)
        #update anglespeed and xspeed
        stracc = str(x_speed_deriv) + "\n" + str(angle1_speed_deriv) + "\n" + str(angle2_speed_deriv) + "\n"
        #print(stracc)
        self.angle1_speed += angle1_speed_deriv * self.timestep
        self.angle2_speed += angle2_speed_deriv * self.timestep
        self.x_speed += x_speed_deriv * self.timestep
        angle1_deriv = self.angle1_deriv(self.angle1_speed)
        angle2_deriv = self.angle2_deriv(self.angle2_speed)
        x_deriv = self.x_deriv(self.x_speed)
        self.angle1 += angle1_deriv * self.timestep
        self.angle2 += angle2_deriv * self.timestep
        self.x += x_deriv * self.timestep


    def doStep(self, control_input = np.array([[0]])): #control input is a scalar
        #Do progression
        #print(control_input)
        self.Runge_Kutta(control_input)

        #returns a np matrix of size 2 x 1
        output = np.array([[self.x], [self.x_speed], [self.angle1], [self.angle1_speed], [self.angle2], [self.angle2_speed], ])
        #apply noise to the output
        output = self.outputNoisyfier(output)
        return output

    def outputNoisyfier(self, clean_out):
        if(self.noise is None):
            return clean_out

        dirty = np.dot(self.noise, random.uniform(-0.5, 0.5))
        return clean_out + dirty
