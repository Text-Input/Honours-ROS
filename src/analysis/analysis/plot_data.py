import matplotlib.pyplot as plt
import matplotlib.animation as anim

class Visualize:

    def __init__(self):

        self.fig, self.axs = plt.subplots(1, 2)

        # Enable animations
        plt.ion()

    def timer_callback(self, state):
        # Reset plot
        self.draw_map(self.axs[0], state)
        self.draw_path_lens(self.axs[1], state)

        plt.draw()
        plt.pause(0.01)

    def draw_map(self, ax, state):
        ax.clear()
        ax.set_xlabel("x")
        ax.set_ylabel("y")

        completed_targets = set()
        for v in state['agents']:
            if 'completed' not in v:
                continue
            completed_targets.update(v['completed'])

        # Draw targets
        x = []
        y = []
        col = []
        cmp_x = []
        cmp_y = []
        cmp_col = []
        for i, v in enumerate(state['targets']):
            if 'x' in v and 'y' in v:
                if i in completed_targets:
                    cmp_x.append(v['x'])
                    cmp_y.append(v['y'])
                    cmp_col.append(self.target_colour(v))
                else:
                    x.append(v['x'])
                    y.append(v['y'])
                    col.append(self.target_colour(v))
        ax.scatter(x,y, c=col)
        ax.scatter(cmp_x,cmp_y, c=cmp_col, marker='D')

        # Draw agents
        x = []
        y = []
        col = []
        for i, v in enumerate(state['agents']):
            if 'x' in v and 'y' in v:
                x.append(v['x'])
                y.append(v['y'])
                col.append(self.agent_colours[i])
        ax.scatter(x,y,c=col, marker="s")

        #draw future paths
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
            ax.plot(x, y, '--', c=self.agent_colours[i])

        #draw past paths
        for i, v in enumerate(state['agents']):
            if 'completed' not in v:
                continue
            if 'x' not in v:
                continue

            x = []
            y = []
            for j in v['completed']:
                x.append(state['targets'][j]['x'])
                y.append(state['targets'][j]['y'])
            x.append(v['x'])
            y.append(v['y'])
            ax.plot(x, y, '-', c=self.agent_colours[i])

    def draw_path_lens(self, ax, state):
        ax.clear()
        ax.set_xlabel("Agent")
        ax.set_ylabel("# of Tasks Remaining")

        names = []
        bars = []
        for i, a in enumerate(state['agents']):
            if 'remaining' not in a:
                continue
            names.append(i)
            bars.append(len(a['remaining']))

        ax.bar(names, bars)



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
