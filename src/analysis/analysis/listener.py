import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from functools import partial
import json
import time

from .plot_data import Visualize

from dynamic_interfaces.msg import WorldInfo, AgentTargetState, AllocationTimeInfo

# List of parameters to get the values of and write to disk
parameters = [
    ("dalg", rclpy.Parameter.Type.STRING),
    ("salg", rclpy.Parameter.Type.STRING),
    ("known_target_percentage", rclpy.Parameter.Type.DOUBLE)
]

class Subscriber(Node):

    def __init__(self):
        super().__init__('analysis')

        self.state = {}
        self.state['targets'] = [{'enabled': False} for x in range(50)]
        self.state['agents'] = [{} for x in range(6)]
        self.state['has_worldstate'] = False

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.target_sub = self.create_multiple_subscription(50, Pose, '/model/target%s/pose', self.target_callback)
        self.agent_sub = self.create_multiple_subscription(6, Pose, '/model/agent%s/pose', self.agent_callback)
        self.target_state_sub = self.create_multiple_subscription(7, AgentTargetState, '/agent%s/target_state', self.agent_state_callback)

        self.allocation_time_info = self.create_subscription(
            AllocationTimeInfo,
            '/allocation_time_info',
            self.allocation_time_info_callback,
            10)
        self.subscription = self.create_subscription(
            WorldInfo,
            '/world_info',
            self.world_info_callback,
            10)
        self.subscription  # prevent unused variable warning

        timestr = time.strftime("%Y%m%d-%H%M%S")
        folder_name = f"output/{timestr}"
        os.makedirs(folder_name, exist_ok=True)
        self.out_file = open(f'{folder_name}/statefile', 'w')
        self.out_file_allocation_info = open(f'{folder_name}/allocation_info', 'w')
        self.write_parameters(folder_name)

        self.vis = Visualize()

    def write_parameters(self, folder_name):
        self.declare_parameters("", [(x[0], x[1]) for x in parameters])

        params = self.get_parameters([x[0] for x in parameters])

        params_dict = {x.name: x.value for x in params}

        with open(f"{folder_name}/info", 'w') as file:
            file.write(json.dumps(params_dict))

    def allocation_time_info_callback(self, msg):
        out = dict()
        out["elapsed_time_us"] = msg.elapsed_time_us
        out["is_first_static"] = msg.is_first_static
        serialized = json.dumps(out)
        self.out_file_allocation_info.write(serialized + "\n")

    def timer_callback(self):
        self.vis.timer_callback(self.state)

    def target_callback(self, i, msg):
        pos = msg.position
        self.state['targets'][i]['x'] = pos.x
        self.state['targets'][i]['y'] = pos.y

        # Assume target position will not change
        self.destroy_subscription(self.target_sub[i])

    def agent_callback(self, i, msg):
        pos = msg.position
        # self.state['agents'][i] = {'x': pos.x, 'y':pos.y}
        self.state['agents'][i]['x'] = pos.x
        self.state['agents'][i]['y'] = pos.y

        if i == 0:
            # We assume that this gets called each simulation tick, so append the current state to the log.
            serialized = json.dumps(self.state)
            # Write out a new line as a JSON object
            self.out_file.write(serialized + "\n")


    def agent_state_callback(self, i, msg):
        # msg.completed_targets
        # msg.remaining_targets
        completed = [int(x[6:]) for x in msg.completed_targets]
        remaining = [int(x[6:]) for x in msg.remaining_targets]
        self.state['agents'][i]['completed'] = completed
        self.state['agents'][i]['remaining'] = remaining

    def world_info_callback(self, msg):
        if self.state['has_worldstate'] == True and msg.is_update == False:
            return

        self.state['has_worldstate'] = True
        for tgt in msg.targets:
            idx = int(tgt.name[6:])
            self.state['targets'][idx]['enabled'] = True
            self.state['targets'][idx]['type'] = int(tgt.type)

    def create_multiple_subscription(self, count, ty, topic, fn):
        subs = dict()
        for i in range(count):
            subs[i] = self.create_subscription(
                ty,
                topic % i,
                partial(fn, i),
                10
            )
        return subs

    def close(self):
        self.out_file.close()
        self.out_file_allocation_info.close()

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.close()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
