import json
import math

class Analysis:

    def __init__(self, state_file):
        f = open(state_file, 'r')

        # deserialize statefile into memory
        self.states = []
        for line in f:
            self.states.append(json.loads(line))

        self.agents = [{} for x in range(6)]
        self.last_completion = (-1,-1)

    def longest_path(self):
        lens = self.path_lengths()

        max_value = max(lens)
        max_agent = lens.index(max_value)
        return max_agent, max_value


    def path_lengths(self):
        if 'final_path_length' not in self.agents[0]:
            targets = self.completed_targets()

            target_positions = self.states[-1]['targets']

            for i, list in enumerate(targets):

                # Sum over the distance between each pair of targets this agent passed through,
                # starting from its start position

                # Count from starting position
                prev_x = self.states[1]['agents'][i]['x']
                prev_y = self.states[1]['agents'][i]['y']

                length = 0
                for target in list:
                    x = target_positions[target]['x']
                    y = target_positions[target]['y']

                    dx = prev_x - x
                    dy = prev_y - y
                    length += math.hypot(dx, dy)

                self.agents[i]['final_path_length'] = length

        return [x['final_path_length'] for x in self.agents]


    # percent of time spent moving per agent
    def percent_moving_agents(self):
        if 'percent_moving' not in self.agents[0]:

            is_moving = [0 for x in range(6)]
            samples = [0 for x in range(6)]

            for i in range(1,len(self.states)):
                for j in range(6):
                    if 'x' in self.states[i]['agents'][j] and 'x' in self.states[i-1]['agents'][j]:
                        new = self.states[i]['agents'][j]
                        old = self.states[i-1]['agents'][j]
                        samples[j] += 1

                        if new['x'] != old['x'] or new['y'] != old['y']:
                           is_moving[j] += 1
            return [x/y for (x,y) in zip(is_moving, samples)]

        return [x['percent_moving'] for x in self.agents]

    # percent of time spent moving averaged over all agents
    def percent_moving_overall(self):
        per_agent = self.percent_moving_agents()
        return sum(per_agent) / len(per_agent)


    # Number of targets completed by each agent
    def targets_per_agent(self, agent=None):
        completed = self.completed_targets(agent)

        if agent is not None:
            return len(completed)
        else:
            return [len(x) for x in completed]

    def time_to_complete(self):

        last_completion_at = 0
        last_completion_size = 0
        for timestep, state in enumerate(self.states):
            completed = set()
            for i in range(6):
                if 'completed' not in state['agents'][i]:
                    continue
                completed.update(state['agents'][i]['completed'])

            if len(completed) > last_completion_size:
                last_completion_at = timestep
                last_completion_size = len(completed)

        self.last_completion = (last_completion_at,last_completion_size)
        return self.last_completion


    # Helpers
    def completed_targets(self, agent=None):
        if 'completed' not in self.agents[0]:

            state = self.states[-1]

            for i, a in enumerate(state['agents']):
                self.agents[i]['completed'] = a['completed']

        if agent is not None:
            return self.agents[agent]['completed']
        else:
            return [x['completed'] for x in self.agents]



def main(args=None):
    file = ""
    if args is not None:
        file = args[0]
    else:
        file = "statefile"

    a = Analysis(file)
    print("targets completed per agent:", a.targets_per_agent())
    print("agent path lengths: ", a.path_lengths())
    print("longest agent / path: ", a.longest_path())
    print("percent of time moving per agent: ", a.percent_moving_agents())
    print("percent of time moving overall: ", a.percent_moving_overall())
    print("timesteps to final completion: ", a.time_to_complete())


if __name__ == '__main__':
    main()
