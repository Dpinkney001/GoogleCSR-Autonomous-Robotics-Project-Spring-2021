#!/usr/bin/env python
# license removed for brevity
#
# Example program that
# wanders avoid obstacles
# T3 version
# dml 2020
#
#

import math
import random
import matplotlib.pyplot as plt #python plotting library
import numpy as np              #numpy library

import rospy # needed for ROS
from sensor_msgs.msg import LaserScan    # ROS laser msg
from geometry_msgs.msg import Twist      # ROS Twist message
from nav_msgs.msg import Odometry        

motionTopic='/cmd_vel' # turtlebot vel topic
laserTopic = '/scan'   # laser scan topic
poseTopic = '/odom'    # odom topic

from tf.transformations import euler_from_quaternion

# comment dump
# width = 30     # bumper zone cone half angle

# global variable to hold the pose of the robot
gLoc = [0,0,0]

gScale = 10
gWidth = 20 * gScale
gHeight = 20 * gScale
gWidthOffset = int(gWidth/2)
gHeightOffset = int(gHeight/2)

gMap = np.zeros((gWidth,gHeight)) # zero out map initially
print(gMap)
# print(gWidth)
# print(gHeight)

def callback_laser(msg):
    '''Call back function for laser range data'''
    global gBumpLeft, gBumpRight
    global gMap, gLoc
    tooClose = 0.1 #meters
    center = 0     # laser reading directly in front of robot

    gBumpLeft,gBumpRight=False,False
    # print(msg.ranges)

    # 0-259 degree range from lazer
    for i in range(360):
        # determine if readings values are valid
        if not math.isnan( msg.ranges[i] ) and not math.isinf( msg.ranges[i] ):
            
            # x-cord
            x = (msg.ranges[i]) * math.cos(convertToRadians(i) + gLoc[2])
            # y-cord
            y = (msg.ranges[i]) * math.sin(convertToRadians(i) + gLoc[2])
            # print(str(i) + ': (' + str(x) + ',' + str(y)+ ')')
            ix = int(((x + gLoc[0]) * gScale) + gWidthOffset)
            iy = int(((y + gLoc[1]) * gScale) + gHeightOffset)

            # print(str(i) + ': (' + str(ix) + ',' + str(iy)+ ')')
            if(ix in range(gHeight) and iy in range(gWidth)): 
                gMap[ix][iy] += 1

            if msg.ranges[i]<tooClose:
                if i<center:
                    gBumpLeft=True 
                else:
                    gBumpRight=True
    return

def convertToRadians(degrees):
    return degrees*(math.pi/180.0)

# callback for the laser range data
# poseCallback
# this procedure is called to accept ROS pose topic info
def poseCallback(data):
    global gLoc
    gLoc[0] = data.pose.pose.position.x # -2
    gLoc[1] = data.pose.pose.position.y # 1
    orient = data.pose.pose.orientation

    quat = [orient.x, orient.y, orient.z, orient.w]
    (roll,pitch,yaw)=euler_from_quaternion(quat)
    if yaw<0: # make the yaw angle positive only
        yaw+=2*math.pi    
    gLoc[2]=yaw
    # gLoc 0,1,2 setup
    return

# Euclidean distance
def dist(x1,y1,x2,y2):
    delx = x2-x1
    dely = y2-y1
    d = math.sqrt( delx*delx+dely*dely)
    return d

# def isValidLazerReading()

# wander_node - version 1
# Moves forward until it detects a close surface
def wander_node():
    '''continually move forward until a close surface is detected'''
    global gBumpLeft, gBumpRight

    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('Wander_Node', anonymous=True)

    #register as a ROS publisher for the velocity topic 
    vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    # register as a subscribe for the laser scan topic
    scan_sub = rospy.Subscriber(laserTopic, LaserScan, callback_laser)
    # register as a subscriber for the pose topic
    rospy.Subscriber(poseTopic, Odometry, poseCallback)

    # this is how frequently the loop below iterates
    rospy.sleep(1)
    
    rate = rospy.Rate(10) # Hz

    msg = Twist() # new velocity message
    msg.linear.x,msg.angular.z=0,0
    vel_pub.publish(msg) # stop all motors up front

    turnVel = 1.5 # amoun to turn away from an obstacle
    while not rospy.is_shutdown():
        # print('is it going in here')

        flip = random.randint(0,10) # 10% prob of random direction change
        if gBumpLeft or flip==0:
                msg.linear.x,msg.angular.z=0.0,turnVel
        elif gBumpRight:
            msg.linear.x,msg.angular.z=0.0,-turnVel
        else:
                # random forward velocity 0.1 .. 0.5
                msg.linear.x,msg.angular.z= float(random.randint(1,6))/10.0,0

        vel_pub.publish(msg)
        rate.sleep()

    return

#
# This function is called by ROS when you stop ROS
# Here we use it to send a zero velocity to robot
# in case it was moving when you stopped ROS
#

def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    rospy.sleep(5)
    print(gMap)

    # map plotting commands
    plt.pcolor(gMap)
    plt.show()
    return

#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
        rospy.on_shutdown(callback_shutdown)
        wander_node()
    except rospy.ROSInterruptException:
        pass

