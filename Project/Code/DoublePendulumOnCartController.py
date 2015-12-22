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
        self.Itheta = self.P.J1 + self.P.m2 * self.P.L1 ** 2
        self.A2  = np.array([ [0., 1., 0., 0.], [self.M2 * self.P.g / self.Itheta, 0.,0.,0.], [0.,0.,0.,1.], [0.,0.,0.,0.]])
        self.B2 = np.array([[0.],[-self.M2/self.Itheta], [0.], [1.]])
        self.Q2 = np.zeros((4,4))
        self.Q2[0][0] = 700.
        self.Q2[1][1] = 700.
        self.Q2[2][2] = 5.
        self.Q2[3][3] = 50.
        self.R2 = np.array([[1.]])
        self.S2 = self.getS2()
    def getS2(self): #Returns the solution to the riccati equation for the step 2 stabilising controller
        P2 = scipy.linalg.solve_continuous_are(self.A2 + np.identity(4), self.B2, self.Q2, np.array([[1.]]))
        return np.dot(np.transpose(self.B2), P2)

    def P1Energy(self, state):
        TP1 = 1/2. * self.P.m1 * ( (state[1][0] + self.P.l1 * state[3][0] * cos(state[2][0])) ** 2 + \
        (self.P.l1 *  state[3][0] * sin(state[2][0])) ** 2 ) + 1/2. * self.P.J1 * (state[3][0] ** 2)
        VP1 = self.P.m1 * self.P.g * self.P.l1 * cos(state[2][0])
        return TP1 + VP1

    def P1SimpleEnergy(self, angle, speed):
        E =1/2. * self.P.J1 * (speed ** 2) + self.P.m1 * self.P.g * self.P.l1 * (cos(angle) - 1)
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


    def TopController(self, x, setpoint):
        print("P1P2 TOP")
        return -np.dot(self.K_LQR, (x - setpoint))[0][0]

    def driveForce(self, x):
        K1 = 6.5
        K2 = 6.5
        return -x[0][0] * K1 -x[1][0] * K2

    def P1SwingUp(self, x, setpoint):
        if(cos(x[2][0]) > 0.8):
            return self.P1Top(x, setpoint)
        K = 100.0
        EP1 = self.P1SimpleEnergy(x[2][0], x[3][0])
        EPWanted = self.P1SimpleEnergy(0, 0)
        E_Diff = EP1-EPWanted
        Energy_part = K * np.sign(E_Diff * x[3][0] * cos(x[2][0]))
        Drive = self.driveForce(x)
        if(np.sign(Energy_part) == np.sign(Drive) or abs(Energy_part) > abs(Drive)):
            return Energy_part + Drive
        else:
            return Energy_part


    def P1Top(self, x, setpoint):
        K2 = 1.
        print("1000")
        first = 1 / (np.dot(self.S2, self.B2))
        x2 = np.array([x[2], x[3], x[0], x[1]])
        sigma2 = np.dot(self.S2, x2)
        second = np.dot(self.S2, self.A2)
        second = np.dot(second, x2) + self.R2 * np.sign(sigma2) + K2 * sigma2
        output = -first * second * 30
        print(output)

        return output[0][0]

    def SwingUpController(self, x, setpoint):
        output = self.P1SwingUp(x, setpoint)
        return output
    def Controller(self, x, setpoint):
        output = 0.
        if(abs(x[2][0]) < np.radians(10) and abs(x[4][0]) < np.radians(10)
            and abs(x[3][0] < 0.5 and abs(x[5][0] < 0.5))):
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
