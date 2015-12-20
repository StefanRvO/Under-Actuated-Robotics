#outputs the controll signal given the state-estimate, x, and the desired state, x_d
import numpy as np
from numpy import cos, sin
import random
def fixify_angle(angle, fixpoint):
    angle -= (int(angle / (2 * np.pi)) ) * 2 * np.pi
    return angle


def zigmoid(z):
    return np.tanh(z)

class DoublePendulumOnCartController:

    def __init__(self, Plant, setpoint, outmax = None, outmin = None, outkooeficient = 10):
        self.outmax = outmax
        self.outmin = outmin
        self.P = Plant
        self.x_d = setpoint
        self.last_reward = 0
        self.pos_rew_times = 0
        self.weights = np.array([[0.],[.0], [.0], [.0]])
        self.times = 0
        self.outkooeficient = outkooeficient

    def Energy(self, state):
        T_cart = 1/2. * self.P.m * (state[1][0] ** 2)

        TP1 = 1/2. * self.P.m1 * ( (state[1][0] + self.P.l1 * state[3][0] * cos(state[2][0])) ** 2 + \
        (self.P.l1 *  state[3][0] * sin(state[2][0])) ** 2 ) + 1/2. * self.P.J1 * (state[3][0] ** 2)

        TP2 =  1/2. * self.P.m2 * ((state[1][0] + self.P.L1 * state[3][0] * cos(state[2][0]) + \
                self.P.l2 * state[5][0] * cos(state[4][0])) ** 2 + \
        (self.P.L1 *  state[3][0] * np.sin(state[2][0]) + self.P.l2 * state[5][0] * sin(state[3][0])  ) ** 2 ) \
         + 1/2. * self.P.J2 * (state[5][0] ** 2)

        T = T_cart + TP1 + TP2

        V_cart = 0
        VP1 = self.P.m1 * self.P.g * self.P.l1 * cos(state[2][0])
        VP2 = self.P.m2 * self.P.g * (self.P.L1 * cos(state[2][0]) + self.P.l2 * cos(state[4][0]) )
        V = V_cart + VP1 + VP2
        E = T + V
        return E
    def getControl(self, x):
        pass

    def Controller(self, x, setpoint):
        output = [[7.3]]
        #print(self.Energy(x))
        #print(x)
        return output

    def OutputLimiter(self, u):
        if not (self.outmax is None):
            u[0][0] = min(u[0][0], self.outmax)
        if not (self.outmin is None):
            u[0][0] = max(u[0][0], self.outmin)
        return u

    def calcControlSignal(self, x):
        #x[0][0] = fixify_angle(x[0][0], self.x_d[0][0]) #Fix angle so the controller don't swing around multiple times to get to the position
        #print(str(self.Energy(x)) + "\t" + str(x[0][0]) + "\t" + str(x[1][0]))
        output = self.Controller(x, self.x_d)
        output = self.OutputLimiter(output)
        #print(output)
        #print(x)
        return output

    def SetNewDesired(self, desired):
        self.x_d = desired
        self.P.angle_setpoint = setpoint[0][0]
