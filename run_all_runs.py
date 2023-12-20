import rclpy
from rclpy.node import Node

from dynamic_interfaces.msg import WorldInfo, AgentTargetState, AllocationTimeInfo

from functools import partial

import subprocess
import signal
import itertools


class Run:
    def __init__(self, target_count, specialized, dynamic_alg, static_alg, targets_known, chunk_size=1, seed=0):
        self.seed = seed
        self.chunk_size = chunk_size
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
                f"target_discovered_chunk_size:={self.chunk_size}",
                f"world_seed:={self.seed}",
                f"known_target_percentage:={self.targets_known}"]


def get_all_runs_vary_targets_known():
    target_counts = [100]
    specialized = ["true", "false"]
    dynamic_algs = ["minimize_time", "minimize_time_v2", "static_greedy"]
    static_algs = ["none"]
    targets_knowns = ["0.0", "0.1", "0.5", "0.8", "1.0"]

    for target_count in target_counts:
        for specialization in specialized:
            for dynamic_alg in dynamic_algs:
                for static_alg in static_algs:
                    for targets_known in targets_knowns:
                        yield Run(target_count, specialization, dynamic_alg, static_alg, targets_known)


def get_all_runs_vary_target_count():
    target_counts = [10, 50, 100, 200]
    specialized = ["true", "false"]
    dynamic_algs = ["minimize_time", "minimize_time_v2", "static_greedy"]
    static_algs = ["greedy", "none"]
    targets_knowns = ["0.0"]

    for target_count in target_counts:
        for specialization in specialized:
            for dynamic_alg in dynamic_algs:
                for static_alg in static_algs:
                    for targets_known in targets_knowns:
                        yield Run(target_count, specialization, dynamic_alg, static_alg, targets_known)


def get_all_runs_vary_chunk_size():
    target_counts = [200]
    specialized = ["false"]
    dynamic_algs = ["minimize_time", "minimize_time_v2", "static_greedy"]
    static_algs = ["none"]
    targets_knowns = ["0.0"]
    chunk_sizes = [1, 2, 5, 10, 20, 40, 80, 120, 150, 180, 200]

    for target_count in target_counts:
        for specialization in specialized:
            for dynamic_alg in dynamic_algs:
                for static_alg in static_algs:
                    for targets_known in targets_knowns:
                        for chunk_size in chunk_sizes:
                            yield Run(target_count, specialization, dynamic_alg, static_alg, targets_known, chunk_size)

def get_all_runs_vary_world_seeds():
    target_counts = [100]
    specialized = ["true", "false"]
    dynamic_algs = ["minimize_time_v2", "static_greedy"]
    static_algs = ["greedy"]
    targets_knowns = ["0.5"]
    seeds = [128371293798, 1283781293721937, 18293810238091328, 891203801238123, 8192038012938, 819203801283, 18290381203]

    for target_count in target_counts:
        for specialization in specialized:
            for dynamic_alg in dynamic_algs:
                for static_alg in static_algs:
                    for targets_known in targets_knowns:
                        for seed in seeds:
                            yield Run(target_count, specialization, dynamic_alg, static_alg, targets_known, seed=seed)

class RunManager(Node):
    def __init__(self):
        super().__init__('run_manager')

        self.target_state_sub = self.create_multiple_subscription(6, AgentTargetState, '/agent%s/target_state',
                                                                  self.agent_state_callback)

        self.agent_completed_count = {x: 0 for x in range(6)}
        self.agent_remaining_count = {x: 0 for x in range(6)}

        self.timer = self.create_timer(5, self.run)

        self.runs = get_all_runs_vary_chunk_size()
        self.process = None
        self.target_count = 0
        self.ran_at_least_once = True

    def run(self):
        completed_targets = sum([val for key, val in self.agent_completed_count.items()])

        # If we don't need to actually complete everything, just compute stuff
        # completed_targets = sum([val for key, val in self.agent_completed_count.items()]) \
        #                     + sum([val for key, val in self.agent_remaining_count.items()])

        if (completed_targets == self.target_count or self.process is None) and self.ran_at_least_once:
            if self.process is not None:
                self.process.send_signal(signal.SIGINT)
                self.process.wait()

            run = next(self.runs)
            self.target_count = run.target_count
            args = run.get_args()
            self.process = subprocess.Popen(["ros2", "launch", "dynamic_launch.py", *args])

            # Reset completed count just in case
            self.agent_completed_count = {x: 0 for x in range(6)}

            # Reset this flag. This just a sanity check to make sure the runs take a somewhat reasonable amount of time
            # Without this, it may be possible to have leftover messages update the completed count, killing prematurely
            # the next process.
            self.ran_at_least_once = False
        else:
            self.ran_at_least_once = True

    def agent_state_callback(self, i, msg):
        self.agent_completed_count[i] = len(msg.completed_targets)
        self.agent_remaining_count[i] = len(msg.remaining_targets)

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
