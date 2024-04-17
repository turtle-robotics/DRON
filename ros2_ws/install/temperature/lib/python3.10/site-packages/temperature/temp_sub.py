#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperatureSubscriber(Node):

    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(Float64,'temperature',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    temperature_subscriber = TemperatureSubscriber()

    rclpy.spin(temperature_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    temperature_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()