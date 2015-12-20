import numpy as np
import sys
from matplotlib import pyplot as plt
from matplotlib import animation
import matplotlib

class DrawDoublePendulumOnCart:
    def __init__(self,P, C, G , frameskip = 1):
        self.P = P
        self.C = C
        self.G = G
        self.lastout = np.array([[self.P.x], [self.P.x_speed], [self.P.angle1], [self.P.angle1_speed], [self.P.angle2], [self.P.angle2_speed], ])
        self.frameskip = frameskip

        #setup matplotlib stuff
        self.fig = plt.figure(num=None, figsize=(8, 7), dpi=80, facecolor='w', edgecolor='k')
        self.ax = plt.axes( xlim =( -(self.P.L1 + self.P.L2) * 3, (self.P.L1 + self.P.L2) * 3), \
                            ylim =(-(self.P.L1 + self.P.L2) * 3, (self.P.L1 + self.P.L2) * 3))
        self.rod1, = self.ax.plot([], [], 'o-', lw=2)
        self.rod2, = self.ax.plot([], [], 'o-', lw=2)
        self.rod1_dot = plt.Circle((self.P.L1 * 10000, self.P.L1 * 10000), 0.075, fc='g') #create Off-screen, as we need to initialize for blitting
        self.rod2_dot = plt.Circle((self.P.L2 * 10000, self.P.L2 * 10000), 0.075, fc='g') #create Off-screen, as we need to initialize for blitting

        self.end_dot = plt.Circle((self.P.L2 * 10000, self.P.L2 * 10000), 0.1, fc='g') #create Off-screen, as we need to initialize for blitting
        self.cart = matplotlib.patches.Rectangle((10000 * self.P.L1, 10000 * self.P.L1), width = 1, height = 0.5, alpha = 0.5) #create the cart offscreen
        self.tracks = matplotlib.patches.Rectangle((-9999999, -0.35), 9999999 * 2, 0.1) #Top of Tracks are at -0.25
        self.ax.add_patch(self.rod1_dot)
        self.ax.add_patch(self.rod2_dot)
        self.ax.add_patch(self.end_dot)
        self.ax.add_patch(self.cart)
        self.ax.add_patch(self.tracks)
        self.ax.add_patch(self.rod1)
        self.ax.add_patch(self.rod2)


    #init function, setup background of each frame
    def init(self):
        self.rod1.set_data([], [])
        self.rod2.set_data([], [])
        self.ax.add_patch(self.rod1_dot)
        self.ax.add_patch(self.rod2_dot)
        self.ax.add_patch(self.end_dot)
        self.ax.add_patch(self.cart)
        self.ax.add_patch(self.tracks)
        self.ax.add_patch(self.rod1)
        self.ax.add_patch(self.rod2)
        return  self.cart, self.rod1, self.rod2, self.rod1_dot, self.rod2_dot, self.tracks, self.end_dot

    #animation function
    def animate(self, i):
        self.SimStep()
        self.cart.set_xy((self.P.x - 0.5,-0.25))
        self.rod1.set_data([self.P.x, self.P.x + np.sin(self.P.angle1) * self.P.L1], [0, np.cos(self.P.angle1) * self.P.L1])
        self.rod1_dot.center = (self.P.x, 0)
        self.rod2.set_data([self.P.x + np.sin(self.P.angle1) * self.P.L1, self.P.x + np.sin(self.P.angle1) * self.P.L1 + np.sin(self.P.angle2) * self.P.L2], \
                            [np.cos(self.P.angle1) * self.P.L1, np.cos(self.P.angle1) * self.P.L1 + np.cos(self.P.angle2) * self.P.L2])
        self.rod2_dot.center = (self.P.x + np.sin(self.P.angle1) * self.P.L1, np.cos(self.P.angle1) * self.P.L1)

        self.end_dot.center = (self.P.x + np.sin(self.P.angle1) * self.P.L1 + np.sin(self.P.angle2) * self.P.L2,
                                np.cos(self.P.angle1) * self.P.L1 + np.cos(self.P.angle2) * self.P.L2)
        return  self.cart, self.rod1, self.rod2, self.rod1_dot, self.rod2_dot, self.tracks, self.end_dot

    def SimStep(self):
        self.G.recordDataPoint(self.lastout)
        control = [[0.]] #self.C.calcControlSignal(self.lastout)
        self.lastout = self.P.doStep(control)
        #self.G.recordControlSignal(control)

    #begin the animation
    def startAnimation(self, speedup = 1, runtime = None, filename = None):

        if not runtime is None: #If runtime restriction is given, only run for this number of seconds
            number_of_steps = int(runtime / self.P.timestep)
            number_of_frames = int(number_of_steps / self.frameskip)
            anim = animation.FuncAnimation(self.fig, self.animate, init_func = self.init, frames = number_of_frames, interval = (1000. * self.P.timestep) / speedup, blit=True, repeat = False)
            if not filename is None: #If filename is given, save animation as file
                anim.save(filename, fps = 60)
            else:
                plt.show()
        else:    #If no runtime restriction, run until window is closed
            anim = animation.FuncAnimation(self.fig, self.animate, init_func = self.init, interval = (1000. * self.P.timestep) / speedup, blit=True, repeat = False)
            plt.show()

    def startSimulation(self, runtime):
        number_of_steps = int(runtime / self.P.timestep)
        for i in range(number_of_steps):
            self.SimStep()
