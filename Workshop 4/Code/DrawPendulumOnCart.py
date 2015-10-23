import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import matplotlib

class DrawPendulumOnCart:
    def __init__(self,P, C = None, G = None, frameskip = 1):
        self.P = P
        self.C = C
        self.G = G
        self.lastout = np.array([ [self.P.angle], [self.P.angle_speed], [self.P.x], [self.P.x_speed] ])
        self.frameskip = frameskip

        #setup matplotlib stuff
        self.fig = plt.figure(num=None, figsize=(8, 7), dpi=80, facecolor='w', edgecolor='k')
        self.ax = plt.axes(xlim=(-self.P.l * 2.5, self.P.l * 2.5), ylim =(-self.P.l * 2.5, self.P.l * 2.5))
        self.rod, = self.ax.plot([], [], 'o-', lw=2)
        self.rod_dot = plt.Circle((self.P.l * 10000, self.P.l * 10000), 0.075, fc='g') #create Off-screen, as we need to initialize for blitting
        self.cart = matplotlib.patches.Rectangle((10000 * self.P.l, 10000 * self.P.l), width = 1, height = 0.5, alpha = 0.5) #create the cart offscreen
        self.tracks = matplotlib.patches.Rectangle((-9999999, -0.35), 9999999 * 2, 0.1) #Top of Tracks are at -0.25
        self.ax.add_patch(self.rod_dot)
        self.ax.add_patch(self.cart)
        self.ax.add_patch(self.tracks)
        self.ax.add_patch(self.rod)

    #init function, setup background of each frame
    def init(self):
        self.rod.set_data([], [])
        self.ax.add_patch(self.rod_dot)
        self.ax.add_patch(self.cart)
        self.ax.add_patch(self.tracks)
        self.ax.add_patch(self.rod)
        return  self.cart, self.rod, self.rod_dot, self.tracks,

    #animation function
    def animate(self, i):
        for i in range(self.frameskip):
            self.SimStep()


        self.cart.set_xy((self.P.x - 0.5,-0.25))
        self.rod.set_data([self.P.x, self.P.x + np.sin(self.P.angle) * self.P.l], [0, np.cos(self.P.angle) * self.P.l])
        self.rod_dot.center = (self.P.x, 0)
        return self.cart, self.rod, self.rod_dot, self.tracks,

    def SimStep(self):
        self.P.doStep()

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
