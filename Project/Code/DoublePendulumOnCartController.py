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
        self.state = ""
        self.M2 = self.P.m1 * self.P.l1 + self.P.m2 * self.P.L1
        self.M1 = self.P.m2 * self.P.l2 * self.P.L1
        self.M3 = self.P.m2 * self.P.l2
        self.I1 = self.P.J1 + self.P.m2 * self.P.L1 ** 2
        self.I2 = self.P.J2
        self.Itheta = self.P.J1 + self.P.m2 * self.P.L1 ** 2
        self.A2  = np.array([ [0., 1., 0., 0.], [self.M2 * self.P.g / self.Itheta, 0.,0.,0.], [0.,0.,0.,1.], [0.,0.,0.,0.]])
        self.B2 = np.array([[0.],[-self.M2/self.Itheta], [0.], [1.]])
        self.Q2 = np.zeros((4,4))
        self.Q2[0][0] = 700.
        self.Q2[1][1] = 700.
        self.Q2[2][2] = 50
        self.Q2[3][3] = 50
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


    def TopController(self, x, setpoint):
        print("P1P2 TOP")
        self.state = "TOP"
        U =  -np.dot(self.K_LQR, (x - setpoint))[0][0]
        return U

    def driveForce(self, x):
        K1 = 1
        K2 = 0.5
        return -x[0][0] * K1 -x[1][0] * K2

    def P1SwingUp(self, x, setpoint):
        K = 10
        E = self.EP(x)
        EWanted = self.EP(setpoint)

        EP1 = E-EWanted * 1.2
        print(EP1)
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
        print("P1TOP")

        print("A1:" + str(np.degrees(x[2][0])))
        print("A1':" + str(x[3][0]))
        print("A2:" + str(np.degrees(x[4][0])))
        print("A2'':" + str(x[5][0]))
        print()
        K2 = 1.
        self.state = "P1TOP"
        #print("P1top")
        first = 1 / (np.dot(self.S2, self.B2))
        x2 = np.array([x[2], x[3], x[0], x[1]])
        sigma2 = np.dot(self.S2, x2)
        second = np.dot(self.S2, self.A2)
        second = np.dot(second, x2) + self.R2 * np.sign(sigma2) + K2 * sigma2
        output = -first * second
        #print(output)

        return output[0][0]

    def SwingUpController(self, x, setpoint):
        E = self.EP(x)
        EWanted = self.EP(setpoint)

        if(cos(x[2][0]) > 0.8 and E - EWanted > 0 and E - EWanted < EWanted * 0.1  or self.state == "P1TOP"):
            output = self.P2Swingup(x, setpoint)
            if(E < 0.95 * EWanted): self.state = ""
            return output
        #return self.P1SwingUp(x, setpoint)
        K1 = 1
        K2 = 1
        K3 = 20
        #self.P.x = 0
        output = -K1 * x[0][0]
        output += -K2 * x[1][0]
        U_BAR = self.P.h2 * x[3][0] * cos(x[2][0]) + self.P.h3 * x[5][0] * cos(x[4][0])
        output += K3 * (E - (EWanted * 1.02)) * U_BAR
        #output += K3 * 1000 * U_BAR
        #print("EnergyControl")
        print(E-EWanted)
        #output = 0
        output = self.FeedbackLineariser(output, x)
        #print(output)
        return output

    def FeedbackLineariser(self, u, x):
        M22 = np.array([[self.P.h1]])
        M12 = np.array([[self.P.h2 * cos(x[2][0])], \
                        [self.P.h3 * cos(x[4][0])]])
        M21 = M12.T
        M11 = np.array([[self.P.h4, self.P.h5 * cos(x[2][0] - x[4][0])], \
                        [self.P.h5 * cos(x[2][0] - x[4][0]), self.P.h6]])
        N2 = np.array([ [-self.P.h2 * (x[3][0] ** 2) * sin(x[2][0]) - self.P.h3 * (x[5][0] ** 2) * sin(x[4][0]) ]])
        N1 = np.array([ [self.P.h5 * (x[5][0] ** 2) * sin(x[2][0] - x[4][0]) - self.P.h7 * sin(x[2][0]) ], \
                        [-self.P.h5 * (x[3][0] ** 2) * sin(x[2][0] - x[4][0]) - self.P.h8 * sin(x[4][0]) ]])
        M22_BAR = M22 - np.dot(np.dot(M21, scipy.linalg.inv(M11)),M12)
        N2_BAR = N2 - np.dot(np.dot(M21, scipy.linalg.inv(M11)),N1)
        #print(M22_BAR)


        return M22_BAR[0][0] * u + N2_BAR[0][0]
    def P2Swingup(self, x, setpoint):
        output = self.P1Top(x, setpoint) * 20
        #K = 0.1
        #EP2 = self.P2SimpleEnergy(x[4][0], x[5][0])
        #print(EP2)
        #E = self.EP(x)
        #EWanted = self.EP(setpoint)
        #EP2 = (E - EWanted)
        #U_BAR = self.P.h2 * x[3][0] * cos(x[2][0]) + self.P.h3 * x[5][0] * cos(x[4][0])

        #UZ2 = 0
        #if(sin(x[4][0]) * x[5][0] < 0 and cos(x[4][0]) < np.pi / 6):
        #    UZ2 = K * (E - EWanted) * U_BAR
        #U2 = self.getU2(UZ2, x)

        #print(U2)
        #output += -U2
        return output

    def Controller(self, x, setpoint):
        output = 0.
        if abs(x[2][0]) < np.radians(5) and abs(x[4][0]) < np.radians(5)  and abs(x[5][0]) < 1.5 or self.state == "TOP":
            output = self.TopController(x, setpoint) * 10
            if(x[2][0] > np.radians(15) or abs(x[4][0]) > np.radians(15)): self.state = ""
        else:
            output = self.SwingUpController(x, setpoint)
        #print(self.Energy(x))
        #print(output)
        #print(x)
        #output = self.TopController(x, setpoint)
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
