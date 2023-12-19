import rclpy
from rclpy.node import Node

from dynamic_interfaces.msg import WorldInfo, AgentTargetState, AllocationTimeInfo

from functools import partial

import subprocess
import signal


class Run:
    def __init__(self, target_count, specialized, dynamic_alg, static_alg, targets_known):
        self.targets_known = targets_known
        self.static_alg = static_alg
        self.dynamic_alg = dynamic_alg
        self.specialized = specialized
        self.target_count = target_count

    def get_args(self):
        return [f"target_count:={self.target_count}",
                f"specialized:={self.specialized}",
                f"dalg:={self.dynamic_alg}",
                f"salg:={self.static_alg}",
                f"known_target_percentage:={self.targets_known}"]


def get_all_runs():
    target_counts = [100]
    specialized = ["true", "false"]
    dynamic_algs = ["minimize_time", "minimize_time_v2", "static_greedy"]
    static_algs = ["greedy"]
    targets_knowns = ["0.0", "0.1", "0.5", "0.8", "1.0"]

    for target_count in target_counts:
        for specialization in specialized:
            for dynamic_alg in dynamic_algs:
                for static_alg in static_algs:
                    for targets_known in targets_knowns:
                        yield Run(target_count, specialization, dynamic_alg, static_alg, targets_known)


class RunManager(Node):
    def __init__(self):
        super().__init__('run_manager')

        self.target_state_sub = self.create_multiple_subscription(6, AgentTargetState, '/agent%s/target_state',
                                                                  self.agent_state_callback)

        self.agent_completed_count = {x: 0 for x in range(6)}

        self.timer = self.create_timer(1, self.run)

        self.runs = get_all_runs()
        self.process = None
        self.target_count = 0

    def run(self):
        completed_targets = sum([val for key, val in self.agent_completed_count.items()])

        if completed_targets == self.target_count or self.process is None:
            if self.process is not None:
                self.process.send_signal(signal.SIGINT)
                self.process.wait()

            run = next(self.runs)
            self.target_count = run.target_count
            args = run.get_args()
            self.process = subprocess.Popen(["ros2", "launch", "dynamic_launch.py", *args])

            # Reset completed count just in case
            self.agent_completed_count = {x: 0 for x in range(6)}

    def agent_state_callback(self, i, msg):
        self.agent_completed_count[i] = len(msg.completed_targets)

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


def main(args=None):
    rclpy.init(args=args)

    subscriber = RunManager()

    rclpy.spin(subscriber)

    subscriber.close()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()