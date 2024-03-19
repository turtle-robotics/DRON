import cv2 as cv
import argparse
from vidgear.gears import VideoGear

def to_angle(cx, cy, frame, camera_fov):
    """convert frame coordinate to angle"""
    center_x = frame.shape[1]//2
    center_y = frame.shape[0]//2
    ax = (cx - center_x)*camera_fov/center[0]
    ay = (cy - center_y)*camera_fov/center[0]
    
    return (ax, ay)

parser=argparse.ArgumentParser()
parser.add_argument('--image')

args=parser.parse_args()
video=cv.VideoCapture(0) # "0" is default camera, "1" for secondary camera -- if using usb on laptop change to "1"

while True:
    hasFrame,frame=video.read()
    if not hasFrame:
        break
    
    #convert frame to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) 
    #gaussian blur to reduce the number of contours
    blur = cv.medianBlur(gray, 25)
    #binary mask
    ret, mask = cv.threshold(blur, 100, 255, cv.THRESH_BINARY)
    
    #drawing vector from centerline to centroid of whitespace 
    M = cv.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv.circle(frame, (cx, cy), 10, (0, 255, 0), thickness=3)
    print(f'centroid- x:{cx} y:{cy}', end=' \ ')
    
    center = (frame.shape[1]//2, frame.shape[0]//2)
    centroid = (cx, cy)
    #cv.line(frame, center, centroid, (0, 0, 255), 3)
    
    #drawing vectors from centerline to the centroid of each contour
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(frame, contours, -1, (0,255,0), 3)
    
    for i in range(len(contours)):
        M2 = cv.moments(contours[i])
        if M2["m00"] != 0:
            cx2 = int(M2["m10"] / M2["m00"])
            cy2 = int(M2["m01"] / M2["m00"])
            cv.circle(frame, (cx2, cy2), 10, (0, 255, 0), thickness=3)
            cnt_centroid = (cx2, cy2)
            cv.line(frame, center, cnt_centroid, (0, 0, 255), 3)
            #print(f'cnt{i+1}- x:{cx} y:{cy}', end=' \ ')
    #print()#add a nextline char
    
    
    camera_fov = 45 #fov in degrees
    direction_vector_angles = to_angle(centroid[0], centroid[1], frame, camera_fov)
    print(f'{direction_vector_angles[0]:.4}, {direction_vector_angles[1]:.4}')
    cv.imshow("feed", frame)
    #cv.imshow("gray", gray)
    cv.imshow("blur", blur)
    cv.imshow("thresh", mask)

    if cv.waitKey(1) & 0xFF == ord('q'): 
        break
