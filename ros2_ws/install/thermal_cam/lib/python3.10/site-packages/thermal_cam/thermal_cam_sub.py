#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2 as cv
import numpy as np

def to_angle(cx, cy, frame, camera_fov):
    """convert frame coordinate to angle"""
    center_x = frame.shape[1] // 2
    center_y = frame.shape[0] // 2
    ax = (cx - center_x) * camera_fov / center_x
    ay = (cy - center_y) * camera_fov / center_y

    return (ax, ay)

class ThermalSubscriber(Node):

    def __init__(self):
        super().__init__("thermal_cam_sub")
        self.thermal_sub = self.create_subscription(Float32MultiArray, "thermal_topic", self.sub_callback, 10)

    def sub_callback(self, msg: Float32MultiArray):
        rows_dim = next(dim for dim in msg.layout.dim if dim.label == "rows").size
        cols_dim = next(dim for dim in msg.layout.dim if dim.label == "cols").size

        # Reshape the flattened array
        array2d = []
        for i in range(rows_dim):
            row = []
            for j in range(cols_dim):
                index = i * cols_dim + j
                row.append(msg.data[index])
            array2d.append(row)

        # Process the 2D array
        # self.get_logger().info(f"Received Array: {array2d} of type: {np.array(array2d).shape}")
        #################################################
        # Display image
        hasFrame = True
        frame = np.array(array2d)


        img8 = cv.normalize(frame.astype('uint8'), None, 255, 0,
                            norm_type=cv.NORM_MINMAX,
                            dtype=cv.CV_8U)
        # convert frame to grayscale
        # gray = cv.cvtColor(img8, cv.COLOR_BGR2GRAY)
        # gaussian blur to reduce the number of contours
        blur = cv.medianBlur(img8, 25)
        # binary mask
        ret, mask = cv.threshold(blur, 100, 255, cv.THRESH_BINARY)

        # drawing vector from centerline to centroid of whitespace
        M = cv.moments(mask)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv.circle(frame, (cx, cy), 10, (0, 255, 0), thickness=3)
        print(f'centroid- x:{cx} y:{cy}', end=' \ ')

        center = (frame.shape[1] // 2, frame.shape[0] // 2)
        centroid = (cx, cy)
        # cv.line(frame, center, centroid, (0, 0, 255), 3)

        # drawing vectors from centerline to the centroid of each contour
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(frame, contours, -1, (0, 255, 0), 3)

        for i in range(len(contours)):
            M2 = cv.moments(contours[i])
            if M2["m00"] != 0:
                cx2 = int(M2["m10"] / M2["m00"])
                cy2 = int(M2["m01"] / M2["m00"])
                cv.circle(frame, (cx2, cy2), 10, (0, 255, 0), thickness=3)
                cnt_centroid = (cx2, cy2)
                cv.line(frame, center, cnt_centroid, (0, 0, 255), 3)
                # print(f'cnt{i+1}- x:{cx} y:{cy}', end=' \ ')
        # print()#add a nextline char

        camera_fov = 45  # fov in degrees
        direction_vector_angles = to_angle(centroid[0], centroid[1], frame, camera_fov)
        print(f'{direction_vector_angles[0]:.4}, {direction_vector_angles[1]:.4}')
        cv.imshow("feed", frame)
        cv.imshow("img", img8)
        # cv.imshow("gray", gray)
        cv.imshow("blur", blur)
        cv.imshow("thresh", mask)
        cv.waitKey(1)
        ############################################
        

def main(args=None):
    rclpy.init(args=args)
    node = ThermalSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()