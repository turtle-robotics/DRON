import cv2 as cv
import argparse

parser=argparse.ArgumentParser()
parser.add_argument('--image')

args=parser.parse_args()
video=cv.VideoCapture(0)

while True:
    hasFrame,frame=video.read()
    if not hasFrame:
        break
    
    #convert frame to grayscale
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) 
    
    #gaussian blur to reduce the number of contours
    blur = cv.blur(gray, (25, 25))
    
    #binary mask
    ret, mask = cv.threshold(blur, 100, 255, cv.THRESH_BINARY)
    
    #drawing vector from centerline to centroid of all color 
    M = cv.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv.circle(frame, (cx, cy), 10, (0, 255, 0), thickness=3)
    
    #print(f'centroid - x:{cx} y:{cy}')
    
    center = (frame.shape[1]//2, frame.shape[0]//2)
    centroid = (cx, cy)
    #cv.line(frame, center, centroid, (0, 0, 255), 3)
    
    #drawing vectors from centerline to the centroid of each contour
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(frame, contours, -1, (0,255,0), 3)
    print(len(contours))
    for cnt in contours:
        M2 = cv.moments(cnt)
        if M2["m00"] != 0:
            cx2 = int(M2["m10"] / M2["m00"])
            cy2 = int(M2["m01"] / M2["m00"])
            cv.circle(frame, (cx2, cy2), 10, (0, 255, 0), thickness=3)
            cnt_centroid = (cx2, cy2)
            cv.line(frame, center, cnt_centroid, (0, 0, 255), 3)
            cv.line(frame, center, cnt_centroid, (0, 0, 255), 3)
            
    
    
    direction_vector = (centroid[0] - center[0], centroid[1] - center[1])
    
    cv.imshow("feed", frame)
    #cv.imshow("gray", gray)
    cv.imshow("blur", blur)
    cv.imshow("thresh", mask)

    if cv.waitKey(1) & 0xFF == ord('q'): 
        break
