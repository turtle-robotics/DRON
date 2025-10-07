#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Latlong

class GPSSubscriber(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(Latlong,'latlong',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.latitude)


def main(args=None):
    rclpy.init(args=args)

    gps_subscriber = GPSSubscriber()

    rclpy.spin(gps_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()