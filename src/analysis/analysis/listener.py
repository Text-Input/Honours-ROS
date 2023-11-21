import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from functools import partial

from .plot_data import Visualize

from dynamic_interfaces.msg import WorldInfo, AgentTargetState


class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')

        self.state = {}
        self.state['targets'] = [{'enabled': False} for x in range(50)]
        self.state['agents'] = [{} for x in range(7)]
        self.state['has_worldstate'] = False

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.target_sub = self.create_multiple_subscription(50, Pose, '/model/target%s/pose', self.target_callback)
        self.agent_sub = self.create_multiple_subscription(6, Pose, '/model/agent%s/pose', self.agent_callback)
        self.target_state_sub = self.create_multiple_subscription(7, AgentTargetState, '/agent%s/target_state', self.agent_state_callback)

        self.subscription = self.create_subscription(
            WorldInfo,
            '/world_info',
            self.world_info_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.vis = Visualize()

    def timer_callback(self):
        self.vis.timer_callback(self.state)

    def target_callback(self, i, msg):
        pos = msg.position
        self.state['targets'][i]['x'] = pos.x
        self.state['targets'][i]['y'] = pos.y

    def agent_callback(self, i, msg):
        pos = msg.position
        # self.state['agents'][i] = {'x': pos.x, 'y':pos.y}
        self.state['agents'][i]['x'] = pos.x
        self.state['agents'][i]['y'] = pos.y

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
        subs=[]
        for i in range(count):
            subs.append(self.create_subscription(
                ty,
                topic % i,
                partial(fn, i),
                10
            ))
        return subs

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
