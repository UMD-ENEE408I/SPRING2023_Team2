import numpy as np
import cv2 as cv
import glob
import os

# Outputs camera calibration parameters from photos.

chessboardSize = (9,6)
 
# Get corners from pictures of checkerboard

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,6,0)
objp = np.zeros((chessboardSize[0]*chessboardSize[1],3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

input1 = "/home/luke/Desktop/autonomous Robots/camera calibration/images/"
images = glob.glob(input1 + '*.png')

_img_shape = None

for fname in images:
    img = cv.imread(fname)
    
    if _img_shape == None:
    	_img_shape = img.shape[:2]
    	
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)
    
    # If found, add object points, image points (after refining them)

    if ret == True:

        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()

# Calibrate camera

calibration_flags = cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv.fisheye.CALIB_FIX_SKEW

N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
rms, _, _, _, _ = \
    cv.fisheye.calibrate(
    	np.expand_dims(np.asarray(objpoints), -2),
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
DIM = _img_shape[::-1]
print("K=np.array(" + str(K.tolist()) + ")")
K = K.tolist()
print("D=np.array(" + str(D.tolist()) + ")")
D = D.tolist()
	
#save outputs

output_DIM = "/home/luke/Desktop/autonomous Robots/camera calibration/output_DIM"
output_K = "/home/luke/Desktop/autonomous Robots/camera calibration/output_K"
output_D = "/home/luke/Desktop/autonomous Robots/camera calibration/output_D"

np.save(output_DIM, DIM)
np.save(output_K,K)
np.save(output_D,D)

