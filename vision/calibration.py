import numpy as np
import cv2 as cv
import glob

# Termination criteria for corner sub-pixel accuracy
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.00001)

# Define the size of a chessboard square
square_size = 0.028 # meters

# Prepare object points based on the size of the chessboard squares
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2) * square_size

# Arrays to store object points and image points from all the images
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

images = glob.glob('vision/calibration/*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
    
    # If found, refine and add object points and image points
    if ret:
        objpoints.append(objp)
        
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print out the camera matrix and distortion coefficients
print("Camera Matrix (in m):")
print(mtx)
print("Distortion Coefficients:")
print(dist)
