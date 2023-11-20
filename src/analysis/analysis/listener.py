import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from functools import partial

from .plot_data import Visualize

from dynamic_interfaces.msg import WorldInfo


class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')

        self.state = {}
        self.state['targets'] = {}
        self.state['agents'] = {}

        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.world_info = self.create_subscription(
        #     String,
        #     '/world_info',
        #     self.world_info_callback,
        #     10)

        self.target_sub=[]
        for i in range(50):
            self.target_sub.append(self.create_subscription(
                Pose,
                '/model/target%s/pose' % i,
                partial(self.target_callback, i),
                10
            ))
        self.agent_sub=[]
        for i in range(6):
            self.agent_sub.append(self.create_subscription(
                Pose,
                '/model/agent%s/pose' % i,
                partial(self.agent_callback, i),
                10
            ))

        # self.subscription = self.create_subscription(
        #     WorldInfo,
        #     'topic',
        #     self.,
        #     10)
        # self.subscription  # prevent unused variable warning

        self.vis = Visualize()

    def timer_callback(self):

        self.vis.timer_callback(self.state)

    def target_callback(self, i, msg):
        pos = msg.position
        self.state['targets'][i] = {'x': pos.x, 'y':pos.y}

    def agent_callback(self, i, msg):
        pos = msg.position
        self.state['agents'][i] = {'x': pos.x, 'y':pos.y}

    def world_info_callback(self, msg):
        self.state['world_state'] = msg
        self.get_logger().info('I heard: "%s"' % msg.targets)

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
