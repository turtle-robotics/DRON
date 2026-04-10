#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import glob
#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import glob
import os

############### FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################
chessboardSize = (8,6) # Other chessboard sizes used - (5,3) OR (9,6)

LEFT_DIR  = 'images_vision/left'
RIGHT_DIR = 'images_vision/right'

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 25
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.

############################################################
# Build dictionaries of images keyed by filename
############################################################

left_images = {os.path.basename(p): p for p in glob.glob(f"{LEFT_DIR}/*.jpg")}
right_images = {os.path.basename(p): p for p in glob.glob(f"{RIGHT_DIR}/*.jpg")}

# Find filenames that exist in both folders
common_names = sorted(set(left_images.keys()) & set(right_images.keys()))

print(f"Found {len(common_names)} matching stereo pairs")

############################################################
# Process matching pairs
############################################################

for name in common_names:

    imgL = cv.imread(left_images[name])
    imgR = cv.imread(right_images[name])

    imgL = cv.resize(imgL, (int(imgL.shape[1] / 4), int(imgL.shape[0] / 4)))
    imgR = cv.resize(imgR, (int(imgR.shape[1] / 4), int(imgR.shape[0] / 4)))

    grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
    grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

    # Get the corners of the chess board
    retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
    retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)

    # Add object points and image points if chess board corners are found
    if retL and retR:

        objpoints.append(objp)

        cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv.imshow('img left', imgL)

        cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
        cv.imshow('img right', imgR)

        cv.waitKey(100)

cv.destroyAllWindows()

############# CALIBRATION #######################################################

heightL, widthL, channelsL = imgL.shape
print(heightL)
print(widthL)
retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(objpoints, imgpointsL, (int(widthL), int(heightL)), None, None)
newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (int(widthL), int(heightL)), 1, (int(widthL), int(heightL)))

heightR, widthR, channelsR = imgR.shape
retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints, imgpointsR, (int(widthR), int(heightR)), None, None)
newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (int(widthR), int(heightR)), 1, (int(widthR), int(heightR)))

ud_img = cv.undistort(imgL, cameraMatrixL, distL, None, newCameraMatrixL)
cv.imshow('undistorted_image', ud_img)
cv.waitKey()
cv.destroyAllWindows()

######### Stereo Vision Calibration #############################################
## stereoCalibrate Output: retStereo is RSME, newCameraMatrixL and newCameraMatrixR are the intrinsic matrices for both
                ## cameras, distL and distR are the distortion coeffecients for both cameras, rot is the rotation matrix,
                ## trans is the translation matrix, and essentialMatrix and fundamentalMatrix are self descriptive

# R and T are taken from stereoCalibrate to use in triangulation
header = ['Rotation','Translation', 'ProjectionLeft', 'ProjectionRight'] # for the csv file

flags = 0
#flags = cv.CALIB_FIX_INTRINSIC

criteria_stereo= (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 1000, 0.0001)

retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, cameraMatrixL, distL, cameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

fs = cv.FileStorage("stereocalib_params.xml", cv.FILE_STORAGE_WRITE)

# Write the parameters
fs.write("cameraMatrix1", newCameraMatrixL)
fs.write("cameraMatrix2", newCameraMatrixR)
fs.write("distCoeffs1", distL)
fs.write("distCoeffs2", distR)
fs.write("rotationVectors", rot)
fs.write("translationVectors", trans)

# Close the file
fs.release()

print(retStereo)
print('stereo vision done')
