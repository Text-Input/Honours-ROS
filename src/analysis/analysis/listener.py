import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import random

from .plot_data import Visualize


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.state = []

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.vis = Visualize()

    def timer_callback(self):
        msg = String()

        msg.data = str(random.randrange(-1,3))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

        self.vis.timer_callback(self.state)

    def listener_callback(self, msg):
        self.state.append(int(msg.data))
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
