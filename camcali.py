import cv2
import numpy as np


output_DIM = "/home/luke/Desktop/autonomous Robots/camera calibration/output_DIM.npy"
output_K="/home/luke/Desktop/autonomous Robots/camera calibration/output_K.npy"
output_D="/home/luke/Desktop/autonomous Robots/camera calibration/output_D.npy"
inputimage ="/home/luke/Desktop/autonomous Robots/camera calibration/images/3.jpg"

DIM = np.load(output_DIM)
K = np.load(output_K)
D = np.load(output_D)

cap = cv2.VideoCapture(-1)

while True:
    flag, img = cap.read()
    try:
        balance = 0
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)
        calibrated = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imshow('result', calibrated)
    except:
        cap.release()
        raise

    k = cv2.waitKey(30)
cap.release()
cv2.destroyAllWindows()
