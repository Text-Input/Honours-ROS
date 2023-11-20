import matplotlib.pyplot as plt
import matplotlib.animation as anim

class Visualize:

    def __init__(self):

        self.fig, self.ax = plt.subplots()

        # Enable animations
        plt.ion()

    def timer_callback(self, state):
        # Reset plot
        self.ax.clear()
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")

        # Draw targets
        x = []
        y = []
        for k, v in state['targets'].items():
            x.append(v['x'])
            y.append(v['y'])
        self.ax.scatter(x,y)

        # Draw agents
        x = []
        y = []
        for k, v in state['agents'].items():
            x.append(v['x'])
            y.append(v['y'])
        self.ax.scatter(x,y)

        plt.draw()
        plt.pause(0.01)