#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class TemperaturePublisher(Node):

    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float64, 'temperature', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64()

        msg.data = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing temperature data')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    temperature_publisher = TemperaturePublisher()

    rclpy.spin(temperature_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    temperature_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()