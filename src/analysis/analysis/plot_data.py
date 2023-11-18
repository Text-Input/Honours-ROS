import matplotlib.pyplot as plt
import matplotlib.animation as anim

import random

class Visualize():

    def __init__(self):
        self.dirty = True

        plt.ion()

        self.x = [0]
        self.y = [1]

    def timer_callback(self, state):
        #self.x.append(self.x[-1] + 1)
        #self.y.append(self.y[-1] + random.randrange(-1,3))

        #plt.plot(self.x, self.y)
        plt.plot(state)
        plt.draw()
        plt.pause(0.01)