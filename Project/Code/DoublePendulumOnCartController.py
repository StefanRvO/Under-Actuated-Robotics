#outputs the controll signal given the state-estimate, x, and the desired state, x_d
import numpy as np
from numpy import cos, sin
import random
import scipy.linalg

def fixify_angle(angle, fixpoint):
    while(angle > fixpoint + np.pi):
         angle -= np.pi * 2
    while(angle < fixpoint - np.pi):
        angle += np.pi * 2
    return angle


def zigmoid(z):
    return np.tanh(z)

class DoublePendulumOnCartController:

    def __init__(self, Plant, setpoint, K_LQR, outmax = None, outmin = None):
        self.outmax = outmax
        self.outmin = outmin
        self.P = Plant
        self.x_d = setpoint
        self.times = 0
        self.K_LQR = K_LQR

        self.M2 = self.P.m1 * self.P.l1 + self.P.m2 * self.P.L1
        self.M1 = self.P.m2 * self.P.l2 * self.P.L1
        self.M3 = self.P.m2 * self.P.l2
        self.I1 = self.P.J1 + self.P.m2 * self.P.L1 ** 2
        self.I2 = self.P.J2
        self.Itheta = self.P.J1 + self.P.m2 * self.P.L1 ** 2
        self.A2  = np.array([ [0., 1., 0., 0.], [self.M2 * self.P.g / self.Itheta, 0.,0.,0.], [0.,0.,0.,1.], [0.,0.,0.,0.]])
        self.B2 = np.array([[0.],[-self.M2/self.Itheta], [0.], [1.]])
        self.Q2 = np.zeros((4,4))
        self.Q2[0][0] = 1.
        self.Q2[1][1] = 1.
        self.Q2[2][2] = 1.
        self.Q2[3][3] = 1.
        self.R2 = np.array([[1.]])
        self.S2 = self.getS2()

    def getG(self, x):
        P1 = self.P.m1 * self.P.l1 * self.P1Energy(x) * x[3][0] * cos(x[2][0])
        P2 = self.P.m2 * self.P.l2 * self.P2Energy(x) * x[5][0] * cos(x[3][0])
        return P1 + P2
    def getS2(self): #Returns the solution to the riccati equation for the step 2 stabilising controller
        P2 = scipy.linalg.solve_continuous_are(self.A2 + np.identity(4), self.B2, self.Q2, np.array([[3.]]))
        return np.dot(np.transpose(self.B2), P2)

    def P1Energy(self, state):
        TP1 = 1/2. * self.P.m1 * ( (state[1][0] + self.P.l1 * state[3][0] * cos(state[2][0])) ** 2 + \
        (self.P.l1 *  state[3][0] * sin(state[2][0])) ** 2 ) + 1/2. * self.P.J1 * (state[3][0] ** 2)
        VP1 = self.P.m1 * self.P.g * self.P.l1 * cos(state[2][0])
        return TP1 + VP1

    def P1SimpleEnergy(self, angle, speed):
        E =1/2. * self.P.J1 * (speed ** 2) + self.P.m1 * self.P.g * self.P.l1 * (cos(angle) - 1)
        return E

    def P2SimpleEnergy(self, angle, speed):
        E =1/2. * self.P.J2 * (speed ** 2) + self.P.m2 * self.P.g * self.P.l2 * (cos(angle) - 1)
        return E

    def P2Energy(self, state):
        TP2 =  1/2. * self.P.m2 * ((state[1][0] + self.P.L1 * state[3][0] * cos(state[2][0]) + \
                self.P.l2 * state[5][0] * cos(state[4][0])) ** 2 + \
        (self.P.L1 *  state[3][0] * np.sin(state[2][0]) + self.P.l2 * state[5][0] * sin(state[4][0])  ) ** 2 ) \
         + 1/2. * self.P.J2 * (state[5][0] ** 2)
        VP2 = self.P.m2 * self.P.g * (self.P.L1 * cos(state[2][0]) + self.P.l2 * cos(state[4][0]) )
        return TP2 + VP2

    def CartEnergy(self, state):
        T_cart = 1/2. * self.P.m * (state[1][0] ** 2)
        V_cart = 0
        return T_cart + V_cart

    def Energy(self, state):

        return self.P1Energy(state) + self.P2Energy(state) + self.CartEnergy(state)

    def getU2(self, x, z2): #Returns u2 for use in the swingup controller for P2
        P=self.P
        num1_1_1 = - P.h3 * P.h4 * P.h5
        num1_1_2 = (P.h3 ** 2) *P.h4 *P.L1
        num1_1_3 = -(P.h5 ** 2 - 2 * P.h4 * P.h6) * (P.h2 - P.h1 * P.L1)
        num1_1 = 2 * (num1_1_1 + num1_1_2 + num1_1_3) * sin(x[2][0])

        num1_2 = (P.h3 ** 2 * P.h4 * P.L1 + 2 * P.h5 ** 2 * (P.h2 - P.h1 * P.L1) + \
                P.h3 * (-2 * P.h4 * P.h5 + P.h2 * P.h5 * P.L1)) * sin(x[2][0] - 2* x[4][0])

        num1_3 = P.h3 * (P.h3 * P.h4 - P.h2 * P.h5) * P.L1 * sin(x[2][0] + 2* x[4][0])
        num1 = (num1_1 + num1_2 + num1_3) * (x[3][0] ** 2)

        num2_1 = 2 * P.h3 * P.h5 * P.h7 * cos(x[2][0] - x[4][0]) *cos(x[4][0]) * sin(x[2][0])
        num2_2 = - P.h2 * P.h6 * P.h7 * sin(2 * x[2][0])
        num2_3 = P.h1 * P.h6 * P.h7 * P.L1 * sin(2 * x[2][0])
        num2_4 = - (P.h3 ** 2) * P.h7 * P.L1 * (cos(x[4][0]) ** 2) * sin(2 * x[2][0])
        num2_5 = 2 * P.h2 * P.h5 * P.h8 * cos(x[2][0]) * cos(x[2][0] - x[4][0]) * sin(x[4][0])
        num2_6 = - 2 * P.h1 * P.h5 * P.h8 * P.L1 * cos(x[2][0]) * cos(x[2][0] - x[4][0]) * sin(x[4][0])
        num2_7 = - P.h3 * P.h4 * P.h8 * sin(2 * x[4][0])
        num2_8 = P.h2 * P.h3 * P.h8 * P.L1 * (cos(x[2][0]) ** 2) * sin(2 * x[4][0])

        num2_9_1 =  -2 * P.h2 * P.h3 * P.h6 * P.L1 * (cos(x[2][0]) ** 2) * sin(x[4][0])
        num2_9_2 = - P.h3 * (P.h5 **2 * sin(2* x[2][0] - x[4][0]) + (P.h5 **2 - 2 * P.h4 * P.h6) * sin(x[4][0]))
        num2_9_3 = P.h5 * cos(x[2][0]) * \
            ((2* P.h2 * P.h6 + P.h3 ** 2 * P.L1 - 2 * P.h1 * P.h6 * P.L1) * sin(x[2][0] - x[4][0]) + \
             P.h3 ** 2 * P.L1 * sin(x[2][0] + x[4][0]))
        num2_9 = ( num2_9_1 + num2_9_2 + num2_9_3) * x[5][0] ** 2

        num2_10_1 = -P.h1 * P.h4 * P.h6
        num2_10_2 = P.h2 ** 2 * P.h6 * (cos(x[2][0]) ** 2)
        num2_10_3 = P.h1 * P.h5 ** 2 * (cos(x[2][0] - x[4][0]) ** 2)
        num2_10_4 = - 2 * P.h2 * P.h3 * P.h5 * cos(x[2][0]) * cos(x[2][0] - x[4][0]) * cos(x[4][0])
        num2_10_5 = P.h3 ** 2 * P.h4 * cos(x[4][0]) ** 2
        num2_10 = 2 * (num2_10_1 + num2_10_2 + num2_10_3 + num2_10_4 + num2_10_5) * z2
        num2 = 2 * (num2_1 + num2_2 + num2_3 + num2_4 + num2_5 + num2_6 + num2_7 + num2_8 + num2_9 + num2_10)
        num = num1 + num2

        denom1 = P.h4 * P.h6
        denom2= - P.h2 * P.h6 * P.L1 * (cos(x[2][0]) ** 2)
        denom3 = -(P.h5 ** 2) * (cos(x[2][0] - x[4][0]) ** 2)
        denom4 = P.h3 * P.h5 * P.L1 * cos(x[2][0]) * cos(x[2][0] - x[4][0]) * cos(x[4][0])
        denom = 4 * (denom1 + denom2 + denom3 + denom4)
        #print(denom)
        return -num / denom

    def TopController(self, x, setpoint):
        print("P1P2 TOP")
        U =  -np.dot(self.K_LQR, (x - setpoint))[0][0]
        return U * 10

    def driveForce(self, x):
        K1 = 6.5
        K2 = 6.5
        return -x[0][0] * K1 -x[1][0] * K2

    def P1SwingUp(self, x, setpoint):
        K = 100.0
        EP1 = self.P1SimpleEnergy(x[2][0], x[3][0])
        Energy_part = K * np.sign(EP1  * x[3][0] * cos(x[2][0]))
        Drive = self.driveForce(x)
        if(np.sign(Energy_part) == np.sign(Drive) or abs(Energy_part) > abs(Drive)):
            return Energy_part + Drive
        else:
            return Energy_part

    def EP(self, x):
        EP = 0.5 * self.P.h4 * x[3][0] ** 2
        EP += 0.5 * self.P.h6 * x[5][0] ** 2
        EP += self.P.h5 * x[3][0] * x[5][0] * cos(x[2][0] - x[4][0])
        EP += self.P.h7 * cos(x[2][0])
        EP += self.P.h8 * cos(x[4][0])
        return EP
    def P1Top(self, x, setpoint):
        K2 = 1.
        #print("P1top")
        first = 1 / (np.dot(self.S2, self.B2))
        x2 = np.array([x[2], x[3], x[0], x[1]])
        sigma2 = np.dot(self.S2, x2)
        second = np.dot(self.S2, self.A2)
        second = np.dot(second, x2) + self.R2 * np.sign(sigma2) + K2 * sigma2
        output = -first * second * 10
        #print(output)

        return output[0][0]

    def SwingUpController(self, x, setpoint):
        K1 = 2
        K2 = 3
        K3 = 200
        output = -K1 * x[0][0]
        output += -K2 * x[1][0]
        E = self.EP(x)
        EWanted = self.EP(setpoint)
        U_BAR = self.P.h2 * x[3][0] * cos(x[2][0]) + self.P.h3 * x[5][0] * cos(x[4][0])
        output += K3 * (E - EWanted) * U_BAR
        print((E - EWanted))
        #if(cos(x[2][0]) < 0.8):
        #    output = self.P1SwingUp(x, setpoint)
        #else:
        #    output = self.P2Swingup(x, setpoint)
        if(cos(x[2][0]) > 0.8 and E - EWanted > -0.5):
            print("Test")
            output = self.P1Top(x, setpoint)
        #if(cos(x[2][0]) > 0.9 and cos(x[4][0]) > 0.9 and abs(E - EWanted) < 0.1):
        #    print(x)
        #    print(abs(E - EWanted))
        #    print("!!")
        #    output = self.TopController(x, setpoint)
        #print(output)
        return output

    def getU3(self, z2, x):
        A11 = self.I1
        A22 = self.I2
        A12 = self.M1 * cos(x[2][0] - x[4][0])
        A21 = A12
        f11 = self.M1 * x[5][0] ** 2 * sin(x[2][0] - x[4][0]) - self.M2 * self.P.g * sin(x[2][0])
        f12 = self.M2 * cos(x[2][0])
        f21 = -self.M1 * x[3][0] ** 2 * sin(x[2][0] - x[4][0]) - self.M3 * self.P.g * sin(x[4][0])
        f22 = self.M3 * cos(x[4][0])

        num1 = (z2 + self.P.L1 * x[3][0] ** 2 * sin(x[2][0])) * (A11 * A22 - A12 * A21)
        num2 = - (A12 * f21 - A22 * f11) * self.P.L1 * cos(x[2][0])
        num = num1 + num2
        denom1 = A11 * A22
        denom2 = -A12 * A21
        denom3 = (A12 * f22 - A22 * f12) * self.P.L1 * cos(x[2][0])
        denom = denom1 + denom2 + denom3

        return num/denom
    def P2Swingup(self, x, setpoint):
        output = self.P1Top(x, setpoint)
        K = 200
        #EP2 = self.P2SimpleEnergy(x[4][0], x[5][0])
        #print(EP2)
        #E = self.EP(x)
        #EWanted = self.EP(setpoint)
        #EP2 = (E - EWanted)
        #Energy_part = K * np.sign(EP2  * x[5][0] * cos(x[4][0]))
        #UZ2 = 0
        #if(sin(x[4][0]) * x[5][0] < 0 and cos(x[4][0]) < np.pi / 6):
        #    UZ2 = Energy_part
        #U2 = self.getU3(UZ2, x)

        #print(U2)
        #output += U2
        return output

    def Controller(self, x, setpoint):
        output = 0.
        if abs(x[2][0]) < np.radians(10) and abs(x[4][0]) < np.radians(5) and abs(x[5][0]) < 1:
            output = self.TopController(x, setpoint)
        else:
            output = self.SwingUpController(x, setpoint)
        #print(self.Energy(x))
        #print(output)
        #print(x)
        return np.array([[output]])

    def OutputLimiter(self, u):
        if not (self.outmax is None):
            u[0][0] = min(u[0][0], self.outmax)
        if not (self.outmin is None):
            u[0][0] = max(u[0][0], self.outmin)
        return u

    def calcControlSignal(self, x):

        x[2][0] = fixify_angle(x[2][0], 0)
        x[4][0] = fixify_angle(x[4][0], 0)
        #print(x[2][0])
        #print(x[4][0])
        output = self.Controller(x, self.x_d)
        output = self.OutputLimiter(output)
        #print(output)
        #print(x)
        return output

    def SetNewDesired(self, desired):
        self.x_d = desired
        self.P.angle_setpoint = setpoint[0][0]
