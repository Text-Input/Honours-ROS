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
        col = []
        for i, v in enumerate(state['targets']):
            if 'x' in v and 'y' in v:
                x.append(v['x'])
                y.append(v['y'])
                col.append(self.target_colour(v))
        self.ax.scatter(x,y, c=col)

        # Draw agents
        x = []
        y = []
        col = []
        for i, v in enumerate(state['agents']):
            if 'x' in v and 'y' in v:
                x.append(v['x'])
                y.append(v['y'])
                col.append(self.agent_colours[i])
        self.ax.scatter(x,y,c=col, marker="s")

        #draw paths
        for i, v in enumerate(state['agents']):
            if 'remaining' not in v:
                continue
            if 'x' not in v:
                continue

            x = [v['x']]
            y = [v['y']]
            for j in v['remaining']:
                x.append(state['targets'][j]['x'])
                y.append(state['targets'][j]['y'])
            self.ax.plot(x, y, '-', c=self.agent_colours[i])

        plt.draw()
        plt.pause(0.01)

    def target_colour(self, tgt):
        if not tgt['enabled']:
            return "#acacac"

        # pick from target type array
        return self.target_colours[tgt['type']]

    agent_colours = [
            "#000000",
            "#b00000", "#00b000", "#0000b0",
            "#b0b000", "#b000b0", "#00b0b0",
        ]

    target_colours = [
        "#000000",
        "#f00000", "#00f000", "#0000f0",
        "#f0f000", "#f000f0", "#00f0f0",
    ]
