#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class PublisherNode(Node):
    def __init__(self):
        super().__init__("thermal_cam_pub")
        self.random_pub = self.create_publisher(Twist, "custom_topic", 10)
        self.create_timer(1, self.publish_info)
        self.get_logger().info("Started...")

    def publish_info(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.random_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    # node = rclpy.create_node('minimal_publisher')

    # publisher = node.create_publisher(String, 'topic', 10)

    # msg = String()

    # i = 0
    # while rclpy.ok():
    #     msg.data = 'Hello World: %d' % i
    #     i += 1
    #     node.get_logger().info('Publishing: "%s"' % msg.data)
    #     publisher.publish(msg)
    #     sleep(0.5)  # seconds

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # node.destroy_node()
    rclpy.shutdown()

# if __name__ == "__main__":
#     main()