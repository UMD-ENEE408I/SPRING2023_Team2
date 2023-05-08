from pupil_apriltags import Detector
import cv2
import numpy as np
import time
import apriltag
import argparse
# construct the argument parser and parse the arguments

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)
output_DIM = "/home/luke/Desktop/autonomous Robots/camera calibration/output_DIM.npy"
output_K="/home/luke/Desktop/autonomous Robots/camera calibration/output_K.npy"
output_D="/home/luke/Desktop/autonomous Robots/camera calibration/output_D.npy"


DIM = np.load(output_DIM)
K = np.load(output_K)
D = np.load(output_D)

balance = 0
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1] #rotation vector
    p = pnp_ret[2] #translation vector

    return p.reshape((3,)), r.reshape((3,))


if __name__ == '__main__':
    vid = cv2.VideoCapture(0)

    tag_size=0.16 # tag size in meters

    while True:
        try:
            ret, img = vid.read()
            undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT) 
            gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)
            

            results = at_detector.detect(gray, estimate_tag_pose=False)

            for res in results:
                pose = find_pose_from_tag(K, res)
                distance = np.linalg.norm(pose[0])
                print(res.tag_id)
                print("Distance:", distance)
                
                rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                p_cam_c = np.array([[0],[0],[0],[1]])
                
                Tca = np.vstack((np.hstack((rot, pose[0].reshape((3,1)))), [0, 0, 0, 1]))
                
                #Twa = np.array([pose[0], pose[1]], [0 , 1])
                Tac = np.linalg.inv(Tca)
                p_cam_a = Tac @ p_cam_c

                print("position:", p_cam_a.T )
                distance = "/home/luke/Desktop/autonomous Robots/apirltag/distance"
                p_cam_a = "/home/luke/Desktop/autonomous Robots/apirltag/p_cam_a"
                np.save(distance, distance)
                np.save(p_cam_a, p_cam_a)



            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            vid.release()
            cv2.destroyAllWindows()
            print ('Exiting')
            exit(1)

