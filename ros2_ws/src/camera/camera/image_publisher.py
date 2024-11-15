#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv
#from cv_bridge import CvBridge
#bridge = CvBridge()

class ImagePublisher(Node):

	def __init__(self):
	        super().__init__('image_publisher')
	        self.publisher_left = self.create_publisher(Image, 'camera_left', 10)
	        self.publisher_right = self.create_publisher(Image, 'camera_right', 10)
	        timer_period = 1  # seconds
	        self.timer = self.create_timer(timer_period, self.timer_callback)
	        self.i = 0

	def timer_callback(self):
		imgLeft = cv.imread('../SV_In/stLeft.jpg', cv.IMREAD_COLOR)
		imgRight = cv.imread('../SV_In/stRight.jpg', cv.IMREAD_COLOR)
		#imageLeft_message = bridge.cv2_to_imgmsg(imgLeft, encoding="passthrough")
		#imageRight_message = bridge.cv2_to_imgmsg(imgRight, encoding="passthrough")
		imgLeftGray = cv.cvtColor(imgLeft, cv.COLOR_BGR2GRAY)
		imageLeft_msg = Image()
		imageLeft_msg.height = len(imgLeftGray)
		imageLeft_msg.width = len(imgLeftGray[0])
		imageLeft_msg.data = imgLeftGray.flatten().tolist()
		
		imgRightGray = cv.cvtColor(imgRight, cv.COLOR_BGR2GRAY)
		imageRight_msg = Image()
		imageRight_msg.height = len(imgRightGray)
		imageRight_msg.width = len(imgRightGray[0])
		imageRight_msg.data = imgRightGray.flatten().tolist()
		
		try:
			self.publisher_left.publish(imageLeft_msg)
			self.get_logger().info('Publishing Left Image Data')
		except CvBridgeError as e:
			self.get_logger().info(e)

		try:
			self.publisher_right.publish(imageRight_msg)
			self.get_logger().info('Publishing Right Image Data')
		except CvBridgeError as e:
			self.get_logger().info(e)
		self.i += 1


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
