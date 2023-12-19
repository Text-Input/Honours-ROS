import sys
import json

import rclpy
from rclpy.node import Node
import os

from .plot_data import Visualize

class Subscriber(Node):

    def __init__(self, data_folder):
        super().__init__('analysis')

        f = open(f"{data_folder}/statefile", 'r')

        # deserialize statefile into memory
        self.states = []
        for line in f:
            self.states.append(json.loads(line))

        self.timestamp = 0

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vis = Visualize()

    def timer_callback(self):

        self.vis.timer_callback(self.states[self.timestamp])
        # Controls simulation playback speed
        # Number of timesteps to skip over per timer_period seconds
        # (currently 0.5 s, so 100 steps/sec playback)
        self.timestamp += 50
        if self.timestamp > len(self.states):
            self.timestamp = len(self.states)-1



def main(args=None):

    args = sys.argv[1:]
    print(args)
    if len(args) != 0:
        folder = f"./output/{args[0]}"
    else:
        folders = os.listdir("./output")
        folders.sort(reverse=True)
        folder = f"./output/{folders[0]}"

    rclpy.init(args=args)

    subscriber = Subscriber(folder)

    rclpy.spin(subscriber)

    subscriber.close()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
