
import socket
import struct
import time
import math
from pupil_apriltags import Detector
import cv2
import numpy as np
#import apriltag
import argparse
import threading
import audiodatafunctionized
#import module

localIP    = ""   # Bind to all network interfaces
localPort  = 3333 # port on computer, shark
localPort2 = 2525 # port on computer, shark
localPort3 = 2222 # port on computer, minnow

max_buffer_size = 1024
distance1 = 0 # distance from shark1 to minnow
distance2 = 0 # distance from shark2 to minnow
#untested, and idk if it gets the right point

cameraR = []
camerap = []
#tag_size=0.16
global xcord
global ycord

class Streaming(object):
    def __init__(self):
        #print("Initializing Camera")
        self.at_detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        #output_DIM = "/home/luke/Desktop/autonomous Robots/camera calibration/output_DIM.npy"
        #output_K="/home/luke/Desktop/autonomous Robots/camera calibration/output_K.npy"
        #output_D="/home/luke/Desktop/autonomous Robots/camera calibration/output_D.npy"
        output_DIM = "output_DIM.npy"
        output_K="output_K.npy" 
        output_D="output_D.npy"
        #  C:\Users\gavin\Documents\Spring 2023\enee408I\ENEE408I_Notes_Examples-main\mouse_heltec_examples_platformio
        # output_DIM = "C:\Users\gavin\Documents\Spring 2023\enee408I\ENEE408I_Notes_Examples-main\mouse_heltec_examples_platformio\SkeletonCode\output_DIM.npy"
        # output_K="C:\Users\gavin\Documents\Spring 2023\enee408I\ENEE408I_Notes_Examples-main\mouse_heltec_examples_platformio\SkeletonCode\output_K.npy"
        # output_D="C:\Users\gavin\Documents\Spring 2023\enee408I\ENEE408I_Notes_Examples-main\mouse_heltec_examples_platformio\SkeletonCode\output_D.npy"
        
        DIM = np.load(output_DIM)
        K = np.load(output_K)
        D = np.load(output_D)

        balance = 0
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, np.eye(3), balance=balance)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, DIM, cv2.CV_16SC2)


    # def tag_map_generator(): # got the 4 poses based on what i set my workdXY frame as
    #     A1 = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])  #East
    #     B2 = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])#West
    #     C3 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) #North
    #     D0 = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]]) #South
    #     listofRot = [D0,A1,B2,C3]  #S,E,N,W
    #     # from tag n to e to s to w starting at index 0. make tag ids in order,next to each other
    #     max = 100 #side length of square arena/frame
    #     xValues = [max/2, max/2, max/2, 0]# W to e is +x      # 0,0 was the top left of grid
    #     yValues = [0, max/2, max, max/2]# n to s is +y axis,
    #     zValue = 0 # so the z heiht is always the same.
    #     # aBCD to 1234, for ids 30 to 46
    #     matrixValue = [2,1,0,3]# have nesw be the order of increasing id numbers  #   [1,2,1,2,3,3,3,0,1,2,1,2,1,2,1,2,3]
    #     # twaMatrix = []
    #     twaArrays = []
    #     # rwa = []
    #     for i in range(len(xValues)):  # gets a 3x1 matrix for
    #         # twaMatrix.append([[xValues[i]],[yValues[i]],[zValue]])
    #         twaArrays.append([xValues[i],yValues[i],zValue])


    #     tag_map = []     # Twa = [] # id 30 starts at 0,
    #     for i in range(len(xValues)):
    #         forbyfor = []
    #         rot = listofRot[matrixValue[i]] # gets the rotation 3x3 matrix
    #         forbyfor.append([rot[0][0], rot[0][1],  rot[0][2], twaArrays[i][0]])
    #         forbyfor.append([rot[1][0], rot[1][1],  rot[1][2], twaArrays[i][1]])
    #         forbyfor.append([rot[2][0], rot[2][1],  rot[2][2], twaArrays[i][2]])
    #         forbyfor.append([0,0,0,1])
    #         tag_map.append(forbyfor) # was Twa.append

    #     return tag_map



    # def find_pose_from_tag(K, detection):
    #     tag_size=0.16
    #     m_half_size = tag_size / 2

    #     marker_center = np.array((0, 0, 0))
    #     marker_points = []
    #     marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    #     marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    #     marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    #     marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    #     _marker_points = np.array(marker_points)

    #     object_points = _marker_points
    #     image_points = detection.corners

    #     pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
        
    #     if pnp_ret[0] == False:
    #         raise Exception('Error solving PnP')

    #     r = pnp_ret[1] #rotation vector
    #     p = pnp_ret[2] #translation vector

    #     return p.reshape((3,)), r.reshape((3,))


    def cammain(self,vidnum,cordlist,tagid, stop):
        def tag_map_generator(): # got the 4 poses based on what i set my workdXY frame as
            A1 = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])  #East
            B2 = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])#West
            C3 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) #North
            D0 = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]]) #South
            listofRot = [D0,A1,B2,C3]  #S,E,N,W
            # from tag n to e to s to w starting at index 0. make tag ids in order,next to each other
            max = 100 #side length of square arena/frame
            xValues = [max/2, max/2, max/2, 0]# W to e is +x      # 0,0 was the top left of grid
            yValues = [0, max/2, max, max/2]# n to s is +y axis,
            zValue = 0 # so the z heiht is always the same.
            # aBCD to 1234, for ids 30 to 46
            matrixValue = [2,1,0,3]# have nesw be the order of increasing id numbers  #   [1,2,1,2,3,3,3,0,1,2,1,2,1,2,1,2,3]
            # twaMatrix = []
            twaArrays = []
            # rwa = []
            for i in range(len(xValues)):  # gets a 3x1 matrix for
                # twaMatrix.append([[xValues[i]],[yValues[i]],[zValue]])
                twaArrays.append([xValues[i],yValues[i],zValue])


            tag_map = []     # Twa = [] # id 30 starts at 0,
            for i in range(len(xValues)):
                forbyfor = []
                rot = listofRot[matrixValue[i]] # gets the rotation 3x3 matrix
                forbyfor.append([rot[0][0], rot[0][1],  rot[0][2], twaArrays[i][0]])
                forbyfor.append([rot[1][0], rot[1][1],  rot[1][2], twaArrays[i][1]])
                forbyfor.append([rot[2][0], rot[2][1],  rot[2][2], twaArrays[i][2]])
                forbyfor.append([0,0,0,1])
                tag_map.append(forbyfor) # was Twa.append

            return tag_map
    
        def find_pose_from_tag( K, detection):
            tag_size=0.16
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
    
        vid = cv2.VideoCapture(vidnum)
        tag_size=0.16 # tag size in meters
        tag_map = tag_map_generator()

        while True:
            try:
                ret, img = vid.read()
                undistorted_img = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT) 
                gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)
                gray.astype(np.uint8)

                results = self.at_detector.detect(gray, estimate_tag_pose=False)

                for res in results:
                    pose = find_pose_from_tag(self.K, res) # should work with self.K,
                    distance = np.linalg.norm(pose[0])
                    print(res.tag_id)
                    print("Distance:", distance)

                    if res.tag_id == 0 or res.tag_id == 1 or res.tag_id == 2 or res.tag_id == 3 or res.tag_id == 8 or res.tag_id == 5 or res.tag_id == 6 or res.tag_id == 7 and distance < 0.1:
                        stop = 1 #stops and make 90-180 turn
                    if res.tag_id == 4:
                        stop = 2 #shark tagged minnow

                    
                    rot, jaco = cv2.Rodrigues(pose[1], pose[1])

                    pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                    img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                    cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                    p_cam_c = np.array([[0],[0],[0],[1]])
                    
                    Tca = np.vstack((np.hstack((rot, pose[0].reshape((3,1)))), [0, 0, 0, 1]))
                    
                    #TODO need to calculate TWA to get twc
                    Twa = np.array(tag_map[res.tag_id]) # %0 the first id in list so start index is 0
                    Twc = np.matmul(Twa,np.linalg.inv(Tca))  # positons in world frame, 4x4 matrix and the 4th 
                    # column is the pos in the world, with 1 at bottom. Twc[0:3,3] gets x, y, z

                    position = Twc[0:2:1, 3] # Last column, neglect z coord
                    print("position , hopefully x and y:", position )
                    print("position in world frame y might be z:", Twc )
                    xcord= position[0]
                    ycord= position[1]
                    #cordlist is a pointer/reference. 
                    cordlist[0] = xcord
                    cordlist[1] = ycord
                    tagid = res.tag_id

                    #Twa = np.array([pose[0], pose[1]], [0 , 1])
                    Tac = np.linalg.inv(Tca)
                    p_cam_a = Tac @ p_cam_c


                    cameraR = distance
                    camerap = p_cam_a
                    print("position:", p_cam_a.T )
                    distance = "distance"
                    p_cam_a = "p_cam_a"
                    #return xcord, ycord
                    #distance = "/home/luke/Desktop/autonomous Robots/apirltag/distance"
                    #p_cam_a = "/home/luke/Desktop/autonomous Robots/apirltag/p_cam_a"
                    np.save(distance, distance)
                    np.save(p_cam_a, p_cam_a)



                cv2.imshow("img", img)
                cv2.waitKey(10)
            except KeyboardInterrupt:
                vid.release()
                cv2.destroyAllWindows()
                print ('Exiting')
                exit(1)





# def getRadii(dt1, dt2):
#     # delta distance = delta t * speed of sound,
#     v = 343 # m/s

#     distance1 = v * dt1
#     distance2 = v * dt2
#     #return


def circleIntersect(c, r0, r1): # given the disatnce betwwen 2 mice, c, the 2 distances from the time differences, r0 and r1
    x0 = y0 = 0
    x1 = c # might need to be 0 and the y as c, idk yet
    y1 = 0  
    d = math.sqrt((x1-x0)^2 + (y1-y0)^2)
    a = (r0^2-r1^2+d^2)/(2*d)
    h = math.sqrt(r0^2-a^2)   # if value is <=0 then there isnt overlap
    x2 = x0 + a*(x1-x0)/d   
    y2 = y0 + a*(y1-y0)/d   
    x3 = x2 + h*(y1-y0)/d       # also x3=x2-h*(y1-y0)/d
    y3 = y2 - h*(x1-x0)/d 
    return (x3, y3) # get distplacement values


def getAngles(c, a, b): # a and b are r1 and r0, the 2 radii distances.
    # got a and b from localization of frequency.
    # c is the minnow and A is the left shark, B is right shark
    angleC = math.acos((a^2 + b^2 - c^2)/(2*a*b))
    angleA = math.acos((b^2 + c^2 - a^2)/(2*b*c))
    angleB = math.acos((c^2 + a^2 - b^2)/(2*c*a))
    # what the triangl looks like
    #       C
    #
    #   A       B
    return angleA, angleB, angleC





# alternative way, by doing the version from slides
def localize(c,r0,r1): # given the disatnce betwwen 2 mice, c, the 2 distances from the time differences, r0 and r1
    x0 = y0 = 0
    x1 = c # might need to be 0 and the y as c, idk yet
    y1 = 0  

    if(r1 - r0 >= 0): # then r1 is larger radius
        d = r1 - r0
    else:
        d = r0 - r1

    d = math.sqrt((x1-x0)^2 + (y1-y0)^2)
    a = (r0^2-r1^2+d^2)/(2*d)
    h = math.sqrt(r0^2-a^2)   # if value is <=0 then there isnt overlap
    x2 = x0 + a*(x1-x0)/d   
    y2 = y0 + a*(y1-y0)/d   
    x3 = x2 + h*(y1-y0)/d       # also x3=x2-h*(y1-y0)/d
    y3 = y2 - h*(x1-x0)/d 
    return (x3, y3) # get distplacement values




# main loop
if __name__ == '__main__':

    turn = 0
    tagged = 0
    cam1 = Streaming()
    #cam1.__init__(cam1)
    xcord = 0
    ycord = 0
    x2cord = 0
    y2cord = 0
    max = 100 #side length of square arena/frame
    xValues = [max/2, max/2, max/2, 0]# W to e is +x      # 0,0 was the top left of grid
    yValues = [0, max/2, max, max/2]
    
    #init 3 cameras and the variables to get last side length of triangle
    cordlist = np.zeros((2,)) # vector of length 2
    cordlist1 = np.zeros((2,)) # vector of length 2
    tagid0 = 0
    tagid1 = 0
    stop1 = 0
    stop2 = 0

    #sharks
    thread0 = threading.Thread(target=lambda: cam1.cammain(0, cordlist, tagid0,stop1))
    thread1 = threading.Thread(target=lambda: cam1.cammain(2, cordlist1, tagid1,stop2))

    # minnow thread, not using rn
    ###thread2 = threading.Thread(target=lambda: cam1.cammain(2))

    # when exiting ctrl C multiple timmes
    thread0.start()
    thread1.start()
    #   not using thread2.start()


    # i want to get x and y cord from cammain and compare them here.
    #print('test ',cameraR)
    print('after camera threads start')

    audiomodule = audiodatafunctionized.triangulation()


    # threads for audio streams, shoould return anything but update 2 global variables
    #threadAudio1 = threading.Thread(target=)
    #threadAudio2 = threading.Thread(target=)



    # we know where april tags are in world so, based on distance from those tags we can locate the robots in the world frame, 
    # then we can get the distance between the 2 sharks, length c.

    # create a module/instance of audio python file ???
    


    # Creating initial default packets to send to the mouse
    packet_shark1 = struct.pack("ffff", 0, 0,turn,tagged)
    packet_shark2 = struct.pack("ffff", 0, 0,turn,tagged)
    packet_minnow = struct.pack("ffff", 0, 0,turn,tagged)

    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((localIP, localPort))
    UDPServerSocket2 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)    # Bind to address and ip
    UDPServerSocket2.bind((localIP, localPort2))
    UDPServerSocket3 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)    # Bind to address and ip
    UDPServerSocket3.bind((localIP, localPort3))




    while True: # real main loop
        (message, ip_address) = UDPServerSocket.recvfrom(max_buffer_size)
        print('Message received: {}'.format(message))
        (message2, ip_address2) = UDPServerSocket2.recvfrom(max_buffer_size) # 2nd socket case
        print('Message received: {}'.format(message2))

        # (message3, ip_address3) = UDPServerSocket3.recvfrom(max_buffer_size) # 3rd socket
        # print('Message received: {}'.format(message3))

        c = 10  #placeholder value,  get distance between the 2 sharks
        
        # every loopc create audio object adn call the main loop, to do 1 loop/1 date set
        #audiomodule = audiodatafunctionized.triangulation()

        # should call its "main" method. loops until it is ready to finish
        dist1, dist2, timediff = audiomodule.startaudio() 


        # get last side length, and get values for if we need to turn or tagged
        if(tagid0 == tagid1):
            side_length = cordlist[0] - cordlist1[0]
        else:
            print('NOT LOOKING AT THE SAME TAG') # wouldnt work at all

        if(stop1 == 1 or stop2 == 1): # need to get abck to frame/arean case
            print("Turn!")
            turn = 1
        else:
            turn = 0
        if(stop1 == 2 or stop2 == 2): # end case
            print("Tagged!")
            tagged = 1
        else:
            tagged = 0


        # should get the source x and y of the minnnow. dont use them though
        (source_x, source_y) = circleIntersect(side_length, dist1, dist2)  # dists from audio, sideelength from camera
        # get 3 angles for the triangle
        angleA, angleB, angleC = getAngles(side_length, dist1, dist2)



        # angle A and B are the theta values we want to pass to the 2 sharks
        # the distances for the sharks are the radii, distyance 1 and 2.


        # TEMP VALUES...find the distances the sharks and minnows need to go, and the heading
        distance = 10.5
        theta = 90.0

        # call sound lcoalization method, returns 2 sets of distances and thetas, or just 1 set.

        # set up the packets again
        packet_shark1 = struct.pack("ffff", distance1, angleA, turn, tagged)
        packet_shark2 = struct.pack("ffff", distance2, angleB, turn, tagged)
        packet_minnow = struct.pack("ffff", distance, angleC, turn, tagged)

        # send packets
        UDPServerSocket.sendto(packet_shark1, ip_address) # 1st robot
        UDPServerSocket2.sendto(packet_shark2, ip_address2) #2nd robot


        # either need delay/condition to send movement to minnow. Or make it constant or move and stop.
        #send packet to minnow
       # UDPServerSocket3.sendto(packet_minnow, ip_address3)# 3rd robot




        # what we do: use signals and speed of sound to find distance/radii and find the angles and disances.
        
        # what we need to produce/output: distance we want each mice to move, the heading/theta they need to rotate.




# for localization you need the x and y distances/difference from each other. Also need distance to the source, which is from signal/mic.
# the x and y is probably from camera/world frame.

# so 2 mice get their location from apriltag. Then you get the x and y parts so you compare those to each other.

#use threading to listen/read in the microphone.
#import threading
#thread1 = threading.Thread(target= read function here)




