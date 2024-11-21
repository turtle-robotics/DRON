#!/usr/bin/env python3
import cv2 as cv

img = cv.imread('../SV_In/ambush_5_left.jpg', cv.IMREAD_COLOR)
imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
print(np.shape(img))
print(np.shape(imgGray))
cv.imshow("Original", img)
cv.imshow("Grayscale", imgGray)
cv.waitKey(0)
cv.destroyAllWindows()
