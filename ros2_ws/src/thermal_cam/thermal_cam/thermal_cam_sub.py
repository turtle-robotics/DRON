#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ThermalSubscriber(Node):

    def __init__(self):
        super().__init__("thermal_cam_sub")
        self.thermal_sub = self.create_subscription(Twist, "custom_topic", self.sub_callback, 10)

    def sub_callback(self, msg: Twist):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = ThermalSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()