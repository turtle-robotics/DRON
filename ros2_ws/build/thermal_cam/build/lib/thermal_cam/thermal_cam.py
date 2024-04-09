import cv2 as cv
import argparse


def to_angle(cx, cy, frame, camera_fov):
    """convert frame coordinate to angle"""
    center_x = frame.shape[1] // 2
    center_y = frame.shape[0] // 2
    ax = (cx - center_x) * camera_fov / center_x
    ay = (cy - center_y) * camera_fov / center_y

    return (ax, ay)


parser = argparse.ArgumentParser()
parser.add_argument('--image')

# finds working camera
import sys
import os
import signal
import time
import logging
import serial

from senxor.mi48 import MI48, format_header, format_framestats
from senxor.utils import data_to_frame
from senxor.interfaces import get_serial, USB_Interface

# This will enable mi48 logging debug messages
logger = logging.getLogger(__name__)
logging.basicConfig(level=os.environ.get("LOGLEVEL", "DEBUG"))

# Make the a global variable and use it as an instance of the mi48.
# This allows it to be used directly in a signal_handler.
global mi48


# define a signal handler to ensure clean closure upon CTRL+C
# or kill from terminal
def signal_handler(sig, frame):
    """Ensure clean exit in case of SIGINT or SIGTERM"""
    logger.info("Exiting due to SIGINT or SIGTERM")
    mi48.stop()
    cv.destroyAllWindows()
    logger.info("Done.")
    sys.exit(0)


# Define the signals that should be handled to ensure clean exit
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

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
        print("OSERROR")
        ser = serial.Serial('/dev/ttyS3')
usb = USB_Interface(ser)
logger.debug('Connected USB interface:')
logger.debug(usb)

# Make an instance of the MI48, attaching USB for
# both control and data interface.
mi48 = MI48([usb, usb])

# print out camera info
camera_info = mi48.get_camera_info()
logger.info('Camera info:')
logger.info(camera_info)

# set desired FPS
if len(sys.argv) == 2:
    STREAM_FPS = int(sys.argv[1])
else:
    STREAM_FPS = 9
mi48.set_fps(STREAM_FPS)

# see if filtering is available in MI48 and set it up
if int(mi48.fw_version[0]) >= 2:
    # Enable filtering with default strengths
    mi48.enable_filter(f1=True, f2=True)

    # If needed, set a temperature offset across entire frame
    # e.g. if overall accuracy (at product level) seems to be
    # 0.7 above the blackbody, then we need to subtract 0.7
    # from the readout of the MI48:
    # mi48.set_offset_corr(-5.55)
    #
    # However, for most applications the factory level, per pixel
    # calibration is sufficient, so keep offset 0
    mi48.set_offset_corr(0.0)

# initiate continuous frame acquisition
mi48.disable_low_netd()
with_header = True
mi48.start(stream=True, with_header=with_header)


while True:
    data, header = mi48.read()
    if data is None:
        logger.critical('NONE data received instead of GFRA')
        mi48.stop()
        sys.exit(1)

    if header is not None:
        logger.debug('  '.join([format_header(header),
                                format_framestats(data)]))

    frame = data_to_frame(data, mi48.fpa_shape)
    print(frame)
    hasFrame = True
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

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
