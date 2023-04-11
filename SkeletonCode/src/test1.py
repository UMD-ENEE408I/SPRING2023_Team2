
import socket
import struct
import time
import math
 
localIP     = "" # Bind to all network interfaces
localPort   = 3333 # port on computer,shark
localPort2  = 2525 # port on computer,shark
localPort3  = 2222 # port on computer,minnow

max_buffer_size  = 1024
distance1 = 0
distance2 = 0
#untested, and idk if it gets the right point
def circleIntersect(c,r0,r1): # given the disatnce betwwen 2 mice, c, the 2 distances from the time differences, r0 and r1
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
    return (x3,y3) # get distplacement values


def getAngles(c, a, b): # a and b are r1 and r0
    # c is the minnow and A is the left shark, B is right shark
    angleC = math.acos((a^2 + b^2 - c^2)/(2*a*b))
    angleA = math.acos((b^2 + c^2 - a^2)/(2*b*c))
    angleB = math.acos((c^2 + a^2 - b^2)/(2*c*a))
    # what the triangl looks like
    #       C
    #
    #   A       B


def getRadii(dt1, dt2):
    # delta distance = delta t * speed of sound,
    v = 343 # m/s

    distance1 = v * dt1
    distance2 = v * dt2




# doing the version from slides
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
    return (x3,y3) # get distplacement values














# main loop
if __name__ == '__main__':

    # Creating initial default packets to send to the mouse
    packet_shark1 = struct.pack("ff", 0, 0)
    packet_shark2 = struct.pack("ff", 0, 0)
    packet_minnow = struct.pack("ff", 0, 0)



    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    UDPServerSocket.bind((localIP, localPort))
    UDPServerSocket2 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)    # Bind to address and ip
    UDPServerSocket2.bind((localIP, localPort2))
    UDPServerSocket3 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)    # Bind to address and ip
    UDPServerSocket3.bind((localIP, localPort3))

    while True:
        (message, ip_address) = UDPServerSocket.recvfrom(max_buffer_size)
        print('Message received: {}'.format(message))
        (message2, ip_address2) = UDPServerSocket2.recvfrom(max_buffer_size) # 2nd socket case
        print('Message received: {}'.format(message))

        (message3, ip_address3) = UDPServerSocket3.recvfrom(max_buffer_size) # 3rd socket
        print('Message received: {}'.format(message))


       
        # find the distances the sharks and minnows need to go, and the heading
        distance = 10.5
        theta = 90.0

        # call sound lcoalization method, returns 2 sets of distances and thetas, or just 1 set.

        # set up the packets again
        packet_shark1 = struct.pack("ff", distance, theta)
        packet_shark2 = struct.pack("ff", distance, theta)
        packet_minnow = struct.pack("ff", distance, theta)

        # send packets
        UDPServerSocket.sendto(packet_shark1, ip_address) # 1st robot
        UDPServerSocket2.sendto(packet_shark2, ip_address) #2nd robot


        # either need delay/condition to send movement to minnow. Or make it constant or move and stop.
        #send packet to minnow
        UDPServerSocket3.sendto(packet_minnow, ip_address)# 3rd robot




        # what we do: use signals and speed of sound to find distance and use lcoations to figure oout correct distances.
        
        # what we need to produce/output: distance we want each mice to move, the direction/theta they need to rotate
        # to move to desired endpoint.  
        # return a float for distance, float for theta, 



# do math in the .py file to get distances and theta to move
# the main.c file will do all the work on mice. so it will continuously loop



# for localization you need the x and y distances/difference from each other. Also need distance to the source, which is from signal/mic.
# the x and y is probably from camera/world frame.

# so 2 mice get their location from apriltag. Then you get the x and y parts so you compare those to each other.










