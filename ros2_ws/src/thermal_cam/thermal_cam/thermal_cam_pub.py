#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import cv2 as cv
import logging
import sys
import signal
import serial
import os
import numpy as np

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame
from senxor.interfaces import get_serial, USB_Interface



# This will enable mi48 logging debug messages
logger = logging.getLogger(__name__)
logging.basicConfig(level=os.environ.get("LOGLEVEL", "DEBUG"))


# define a signal handler to ensure clean closure upon CTRL+C
# or kill from terminal
def signal_handler(sig, frame):
    global mi48
    """Ensure clean exit in case of SIGINT or SIGTERM"""
    logger.info("Exiting due to SIGINT or SIGTERM")
    mi48.stop()
    cv.destroyAllWindows()
    logger.info("Done.")
    sys.exit(0)


# Define the signals that should be handled to ensure clean exit
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class PublisherNode(Node):
    def __init__(self):
        super().__init__("thermal_cam_pub")
        ######################################################
        # ==============================     
        # create an USB interface object
        # ==============================
        try:
            ser = get_serial()
        except:
            # if on WSL (hack); apply similarly to other cases where
            # device may not be readily found by get_serial
            try:
                ser = serial.Serial('/dev/ttyACM0')  # hard coded and we need a better system
            except OSError:
                ser = serial.Serial('/dev/ttyS3')
        usb = USB_Interface(ser)
        logger.debug('Connected USB interface:')
        logger.debug(usb)

        # Make an instance of the MI48, attaching USB for
        # both control and data interface.
        self.mi48 = MI48([usb, usb])

        # print out camera info
        camera_info = self.mi48.get_camera_info()
        logger.info('Camera info:')
        logger.info(camera_info)

        # set desired FPS
        if len(sys.argv) == 2:
            STREAM_FPS = int(sys.argv[1])
        else:
            STREAM_FPS = 9
        self.mi48.set_fps(STREAM_FPS)

        # see if filtering is available in MI48 and set it up
        if int(self.mi48.fw_version[0]) >= 2:
            # Enable filtering with default strengths
            self.mi48.enable_filter(f1=True, f2=True)

            # If needed, set a temperature offset across entire frame
            # e.g. if overall accuracy (at product level) seems to be
            # 0.7 above the blackbody, then we need to subtract 0.7
            # from the readout of the MI48:
            # mi48.set_offset_corr(-5.55)
            #
            # However, for most applications the factory level, per pixel
            # calibration is sufficient, so keep offset 0
            self.mi48.set_offset_corr(0.0)

        # initiate continuous frame acquisition
        self.mi48.disable_low_netd()
        with_header = True
        self.mi48.start(stream=True, with_header=with_header)

        self.random_pub = self.create_publisher(Float32MultiArray, "thermal_topic", 10)
        self.create_timer(.1, self.publish_info)
        self.get_logger().info("Started...")

    def publish_info(self):
        data, header = self.mi48.read()
        if data is None:
            logger.critical('NONE data received instead of GFRA')
            self.mi48.stop()
            sys.exit(1)

        if header is not None:
            logger.debug('  '.join([format_header(header),
                                    format_framestats(data)]))

        frame = data_to_frame(data, self.mi48.fpa_shape)

        # Compress frame into 1-d array with first two floats being height and width
        height = frame.shape[0]
        width = frame.shape[1]

        frame_1d = frame.ravel()
        frame_1d = np.insert(frame_1d, 0, width)
        frame_1d = np.insert(frame_1d, 0, height)

        # Publish msg
        msg = Float32MultiArray()
                # Create layout
        layout = MultiArrayLayout()
        data = np.array(frame, dtype=np.float32)
        layout.dim.append(MultiArrayDimension(label="rows", size=data.shape[0], stride=data.shape[0]*data.shape[1]))
        layout.dim.append(MultiArrayDimension(label="cols", size=data.shape[1], stride=data.shape[0]))
        msg.layout = layout

        # Populate the 2D array
        msg.data = data.flatten().tolist()
        self.random_pub.publish(msg)

    def to_angle(cx, cy, frame, camera_fov):
        """convert frame coordinate to angle"""
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2
        ax = (cx - center_x) * camera_fov / center_x
        ay = (cy - center_y) * camera_fov / center_y

        return (ax, ay)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
