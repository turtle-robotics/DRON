#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import board
import busio
from custom_interfaces.msg import Latlong
import adafruit_gps

class GPSPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(Latlong, 'latlong', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Latlong()
        global gps
        msg.latitude = gps.latitude
        msg.longitude = gps.longitude

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing GPS Data')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    RX = board.RX
    TX = board.TX

    uart = busio.UART(TX, RX, baudrate=9600, timeout=30)

    gps = adafruit_gps.GPS(uart, debug=False)

    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

    gps.send_command(b'PMTK220,1000')

    gps_publisher = GPSPublisher()

    rclpy.spin(gps_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()