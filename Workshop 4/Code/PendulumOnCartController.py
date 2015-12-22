#outputs the controll signal given the state-estimate, x, and the desired state, x_d
import numpy as np
import random
def fixify_angle(angle, fixpoint):
    angle -= (int(angle / (2 * np.pi)) ) * 2 * np.pi
    return angle


def zigmoid(z):
    return np.tanh(z)

class PendulumOnCartController:

    def __init__(self, Plant, setpoint, outmax = None, outmin = None, outkooeficient = 10):
        self.outmax = outmax
        self.outmin = outmin
        self.P = Plant
        self.x_d = setpoint
        self.last_reward = 0
        self.pos_rew_times = 0
        self.weights = np.array([[0.85770962],[0.50614361], [0.36952404], [0.31212509]])
        self.times = 0
        self.outkooeficient = outkooeficient

    def Energy(self, state):
        energy = 0.5 * (self.P.m_c + self.P.m) * (state[3][0] ** 2) - \
            self.P.m * self.P.l * state[3][0] * state[1][0] * np.cos(state[0][0]) + \
            0.5 * self.P.m * (self.P.l ** 2) * (state[1][0] ** 2) + \
            self.P.m * self.P.g * self.P.l * np.cos(state[0][0])
        return energy

    def getControl(self, x):
        reward = self.reward(x)
        d_reward = reward - self.last_reward
        self.last_reward = reward
        x_scaled = zigmoid(x)
        x_scaled[0][0] = -x[0][0] / self.P.angle_limits[1]
        x_scaled[2][0] = x[2][0] / self.P.x_limits[1]
        x_scaled[1][0] = zigmoid(-x[1][0] / 1)
        x_scaled[3][0] = zigmoid(x[3][0] / 5)

        if(x_scaled[1][0] > 1): x_scaled[1][0] = 1
        if(x_scaled[1][0] < -1): x_scaled[1][0] = -1
        if(x_scaled[3][0] > 1): x_scaled[3][0] = 1
        if(x_scaled[3][0] < -1): x_scaled[3][0] = -1

        #print(x_scaled)
        output = np.array([[0.]])
        weightsum = sum(self.weights)
        if(weightsum == 0): weightsum += 0.000001
        for i in range(4):
                output[0][0] += self.weights[i][0] * x_scaled[i][0] #/ weightsum

        #print(output)
        q = abs(min(0, d_reward))
        #print(q)
        #print(reward_change)
        #print()

        #print(weightsum)
        if(q > 0):
                for i in range(4):
                        self.weights[i][0] += 0.03 * abs(x_scaled[i][0]) * q
                #print(self.weights, end ="\n\n")
                #print(x, end = "\n\n\n\n")
        return zigmoid(output)


    def Learn(self, x):
        self.times += 1
        if(self.times > 20 / self.P.timestep):
            #print("success " + str(self.times))
            #print(x)
            self.P.reset()
            self.times = 0
            return np.array([[0]])

        u = self.getControl(x)
        if self.reset(x):
            print("fail " + str(self.times))
            self.pos_rew_times = 0
            self.times = 0
            self.P.reset()
            print(self.weights)
            return np.array([[0]])

        return u

    def reflex(self, x):
        if(x[0][0] < self.P.angle_limits[0] + np.radians(4)): return 1
        if(x[0][0] > self.P.angle_limits[1] - np.radians(4)): return -1
        if x[2][0] <  (self.P.x_limits[0] + 2) : return 1
        if x[2][0] > (self.P.x_limits[1] - 2) : return -1
        else: return 0
    def reset(self, x):
        if x[2][0] < self.P.x_limits[0] or  (x[0][0] > self.P.angle_limits[1]): return 1
        if x[2][0] > self.P.x_limits[1] or  (x[0][0] < self.P.angle_limits[0]): return 1
        else: return 0
    def reward(self, x):
        if x[2][0] < (self.P.x_limits[0] + 2) or  (x[0][0] < self.P.angle_limits[0] + np.radians(4)):
                return -1
        if x[2][0] > (self.P.x_limits[1] - 2) or  ( x[0][0] > self.P.angle_limits[1] - np.radians(4)):
                return -1
        return 0

    def Controller(self, x, setpoint):
        output = 0
        if not self.reset(x): output+= self.reflex(x) * 10
        output += self.Learn(x) * self.outkooeficient

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
