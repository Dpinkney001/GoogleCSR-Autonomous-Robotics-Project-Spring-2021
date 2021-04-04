# -*- coding: utf-8 -*-
"""
Created on Sun Apr  4 08:37:41 2021

@author: thees
Duvall Pinkney
"""
#!/usr/bin/env python
# license removed for brevity
#
#1. You will need to import numpy and matplotlib



#2. you need to make a numpy array to use as a 2D map array

#gMap = np.zeros((gWidth,gHeight)) # zero out map initially

#gWidth and gHeight are dimensions of the array

#3. You can access the numpy array as you would expect, e.g.

#gMap[ix][iy] = gMap[ix][iy] +1

#where ix and iy are the two integer indices

#4. In the shutdown callback, you can plot the map as follows

#    plt.pcolor(gMap)
#    plt.show()
 
#This will bring up a dialog box with a false color image of the map. There is a button on the dialog box to save the image
#if you want. You need to kill the dialog box before the program will end.
#
#

import math
import random
import rospy # needed for ROS
import matplotlib.pyplot as plt #python plotting library
import numpy as np              #numpy library


# global variable

gLoc = [0,0,0]    # pose of the robot



#global variable to hold the closest front distance

gBumpLeft,gBumpRight = False,False # virtual bumper set if laser shows obstacle close


from geometry_msgs.msg import Twist      # ROS Twist message
from sensor_msgs.msg import LaserScan    # ROS laser msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


#topics 

motionTopic='/cmd_vel' # turtlebot vel topic
laserTopic = '/scan'   # laser scan topic
poseTopic = '/odom' 




# callback for the laser range data
# calculates only if something too close
#
def callback_laser(msg):
    '''Call back function for laser range data'''
    global gBumpLeft, gBumpRight
    
    tooClose = 0.5 #meters
    center = 0     # laser reading directly in front of robot
    width = 30     # bumper zone cone half angle
    gBumpLeft,gBumpRight=False,False
    for i in range(center-width,center+width):
        if not math.isnan( msg.ranges[i] ) and not math.isinf( msg.ranges[i] ):
           if msg.ranges[i]<tooClose:
               if i<center:
                   gBumpLeft=True 
               else:
                   gBumpRight=True
    return
# wander_node_mod - version 1
# Moves forward until it detects a close surface
#

def wander_node_mod():
    
    
    
    return

# wander_node - version 1
# Moves forward until it detects a close surface
#

def wander_node():
    '''continually move forward until a close surface is detected'''
    global gBumpLeft, gBumpRight

    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('Wander_Node', anonymous=True)

    #register as a ROS publisher for the velocity topic 
    vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    # register as a subscribe for the laser scan topic
    scan_sub = rospy.Subscriber(laserTopic, LaserScan, callback_laser)

    # this is how frequently the loop below iterates
    rospy.sleep(1)
    
    rate = rospy.Rate(10) # Hz

    msg = Twist() # new velocity message
    msg.linear.x,msg.angular.z=0,0
    vel_pub.publish(msg) # stop all motors up front

    turnVel = 1.5 # amoun to turn away from an obstacle
    while not rospy.is_shutdown():

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

# square_node_mod - version 1
# to do create square node moditfication to incremently increase the
# size to map the area
def square_node_mod():
    
    #move in an expanding square per revolution
    rospy.init_node('Square_Node_Mod', anonymous=True)
    
    #
    vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    
    
    # register as a subscribe for the laser scan topic
    scan_sub = rospy.Subscriber(laserTopic, LaserScan, callback_laser)
    
    rospy.sleep(1)
    
    rate = rospy.Rate(0.5) # Hz

    msg = Twist() # new velocity message
    msg.linear.x,msg.angular.z=0,0
    vel_pub.publish(msg) # stop all motors up front

    while not rospy.is_shutdown():
    
       msg.linear.x = 0
       
       # note build a for loop increase the square
       for index in range(center-width):
           
       msg.angular.z = math.pi / 2.0
       vel_pub.publish(msg)
       rospy.sleep(1)
       msg.linear.x = 0.2
       msg.angular.z = 0
       vel_pub.publish(msg)
       rospy.sleep(3)
    
    
    
    return




def square_node():
    '''move roughly in a square'''
    
    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('Square_Node', anonymous=True)

    #register as a ROS publisher for the velocity topic 
    vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=1)

    # this is how frequently the loop below iterates
    rospy.sleep(1)
    
    rate = rospy.Rate(0.5) # Hz

    msg = Twist() # new velocity message
    msg.linear.x,msg.angular.z=0,0
    vel_pub.publish(msg) # stop all motors up front

    while not rospy.is_shutdown():
    
       msg.linear.x = 0
       msg.angular.z = math.pi / 2.0 
       vel_pub.publish(msg)
       rospy.sleep(1)
       msg.linear.x = 0.2
       msg.angular.z = 0
       vel_pub.publish(msg)
       rospy.sleep(3)

    return




#
# poseCallback
# this procedure is called to accept ROS pose topic info
#
def poseCallback(data):
    global gLoc
    gLoc[0] = data.pose.pose.position.x
    gLoc[1] = data.pose.pose.position.y
    orient = data.pose.pose.orientation
    quat = [orient.x, orient.y, orient.z, orient.w]
    (roll,pitch,yaw)=euler_from_quaternion(quat)
    if yaw<0: # make the yaw angle positive only
        yaw+=2*math.pi    
    gLoc[2]=yaw
    return

# Euclidean distance
def dist(x1,y1,x2,y2):
    delx = x2-x1
    dely = y2-y1
    d = math.sqrt( delx*delx+dely*dely)
    return d

# goto_node
# this procedure generates the velocity commands
# to move the turtlebot (TG) to a goal x and y 
#

def gotoTG_node(goalx, goaly):
    '''bring turtlenot to location (goalx,goaly)'''

    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('GotoTG_Node', anonymous=True)

    # register as a ROS publisher for the velocity topic 
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    # register as a subscriber for the pose topic
    rospy.Subscriber(poseTopic, Odometry, poseCallback)
    rospy.sleep(1)

    # this is how frequently the message is published
    rate = rospy.Rate(20) # Hz
    msg = Twist() # empty new velocity message
    ctr = 0       # counter used to produce messages
    while not rospy.is_shutdown() and dist(goalx,goaly,gLoc[0], gLoc[1])>0.5:

        delTheta = math.atan2(goaly-gLoc[1], goalx-gLoc[0]) - gLoc[2] #how much the angle needs ti change by

        # check for angle 'wrapping around'
        if delTheta<-math.pi and gLoc[2]>math.pi:
            delTheta = (2*math.pi+delTheta)

        if ctr>20:
            print (round(gLoc[0],2), round(gLoc[1],2) )
            ctr=0
        else:
            ctr = ctr+1
            
        msg.angular.z = 1.0 * delTheta # should saturate angular velocity

        velocity = dist(goalx,goaly,gLoc[0], gLoc[1])
        msg.linear.x = 0.1 * velocity # should saturate linear velocity
    
        pub.publish(msg)
        rate.sleep()
    print  ("Done ","d=",round(dist(goalx,goaly,gLoc[0], gLoc[1]),2))
    print  (" Loc= ",round(gLoc[0],2), round(gLoc[1],2)) 
    
    #reset the velocities to 0
    msg.angular.z = 0
    msg.linear.x = 0
    pub.publish(msg)
    
#using the algorithmsgiven like wander.py movesquareT3 and goto.py
#function to create a custom algorithm thatscans the area in 
#an expanding square pattern to map the room
# then after the first room is mapped then the goto function is called
# to send the bot to the empty space.
#then the expanding square is called to map the  next room. When that 
#room is mapped
#
# order of battle:
#   FUNCTION CALL: fusion_spider_node()
# ---> square_node_mod(): maps the room to find the empty space aka exits
# ----> call goto_mod(): to exit the room
# ----> calls square_node_mod(): to map the room to find the empty space
# ----> call goto_mod() to return to the exit.
# ----> if no more areas to map that are unknown call shudown function()
#-----
    
def fusion_spider_node():
    
    # do this while mapping is not completed
    square_node_mod()
    
    #if exits detected are > 1
    #then there are two entry points
    # proceed to the one not traveled yet
    # call goto to proceed to that point
    
    gotoTG_node(goalx, goaly)
    
    #when goto completes call square_node_mod() to map the area
    
    square_node_mod()

        

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
    plt.pcolor(gMap)
    plt.show()
    return



#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
        rospy.on_shutdown(callback_shutdown)
        fusion_spider_node()
    except rospy.ROSInterruptException:
        pass