#outputs the controll signal given the state-estimate, x, and the desired state, x_d
import numpy as np
from NeuralNetwork import LearningController

def fixify_angle(angle, fixpoint):
    angle -= (int(angle / (2 * np.pi)) ) * 2 * np.pi
    return angle

class PendulumOnCartController:

    def __init__(self, Plant, setpoint, outmax = None, outmin = None): #K is a np matrix
        self.outmax = outmax
        self.outmin = outmin
        self.P = Plant
        self.x_d = setpoint
        self.Learner = LearningController([4, 1])
        self.last_reward = 0
        self.pos_rew_times = 0

    def Energy(self, state):
        energy = 0.5 * (self.P.m_c + self.P.m) * (state[3][0] ** 2) - \
            self.P.m * self.P.l * state[3][0] * state[1][0] * np.cos(state[0][0]) + \
            0.5 * self.P.m * (self.P.l ** 2) * (state[1][0] ** 2) + \
            self.P.m * self.P.g * self.P.l * np.cos(state[0][0])
        return energy

    def Learn(self, x):
        reward = 0
        if abs(x[0][0]) < np.radians(2) and abs(x[1][0]) < 0.01 and abs(x[2][0]) < 0.1 and abs(x[3][0]) < 0.01:
            reward = 1
        elif self.reflex(x) is not 0:
            reward = -1
        else:
            reward = 0
        u = self.Learner.getControl(x, reward - self.last_reward)  -0.5
        if(reward == 1):
            self.pos_rew_times += 1
            if(self.pos_rew_times == 1000):
                self.pos_rew_times = 0
                self.last_reward = 0
                print("success")
                self.P.reset()
                return np.array([[0]])
        elif reward == -1:
            pos_rew_times = 0
            self.last_reward = 0
            print("fail")
            self.P.reset()
            return np.array([[0]])
        self.last_reward = reward
        #print(u)
        return u

    def reflex(self, x):
        if x[2][0] < self.P.x_limits[0] or x[0][0] > self.P.angle_limits[1]: return 1
        if x[2][0] > self.P.x_limits[1] or x[0][0] < self.P.angle_limits[0]: return -1
        else: return 0

    def Controller(self, x, setpoint):
        return 10 * self.Learn(x) + self.reflex(x)

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
        return output

    def SetNewDesired(self, desired):
        self.x_d = desired
        self.P.angle_setpoint = setpoint[0][0]
