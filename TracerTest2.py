#EXP_1
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

scale = 10
gWidth = 200
gHeight = 200
offset = int(gWidth/2)

gMapPose = np.zeros((gWidth, gHeight)) # initiate map with zeros
gMapLaser = np.zeros((gWidth,gHeight)) # zero out map initially


# callback for the laser range data
# poseCallback
# this procedure is called to accept ROS pose topic info
def poseCallback(data):
    global gLoc, gMapPose
    gLoc[0] = data.pose.pose.position.x # -2
    gLoc[1] = data.pose.pose.position.y # 1
    orient = data.pose.pose.orientation

    quat = [orient.x, orient.y, orient.z, orient.w]
    (roll,pitch,yaw)=euler_from_quaternion(quat)
    if yaw<0: # make the yaw angle positive only
        yaw+=2*math.pi    
    gLoc[2]=yaw
    #indexes need to be integer values
    ix =  (int)(scale * gLoc[0] + offset) #for the x index of the array 
    iy =  (int)(scale * gLoc[1] + offset) #for the y index of the array
    #making sure the indexes are in range and not picking up negative values
    if (ix > 0 and ix <= gWidth and iy > 0 and iy <= gHeight) :
   	gMapPose[ix][iy] = gMapPose[ix][iy] + 1 #increments 1 everytime we get a callback from odometry
    return

def callback_laser(msg):
    '''Call back function for laser range data'''
    global gBumpLeft, gBumpRight
    global gMapLaser, gLoc
    global scale, gWidth, gLength, offset
    tooClose = 0.5 #meters
    center = 0     # laser reading directly in front of robot
    width = 180
    gBumpLeft,gBumpRight=False,False
    # print(msg.ranges)

    # 0-259 degree range from lazer
    for i in range(center-width, center+width): #from -180 to 180
        # determine if readings values are valid
        if not math.isnan( msg.ranges[i] ) and not math.isinf( msg.ranges[i] ):
	    #adding the sensor's angle to the robot's angle
            alpha = gLoc[2] + (i*(math.pi/180.0))
            
	    #polar to cartesian conversion below
            ix = (msg.ranges[i]) * math.cos(alpha) #x-coordinate
            iy = (msg.ranges[i]) * math.sin(alpha) #y-coordinate

	    #doing linear transformation            
            transf_ix = int(((ix + gLoc[0]) * scale) + offset)
            transf_iy = int(((iy + gLoc[1]) * scale) + offset)

            #checking to see if the indexes are in range
            if(transf_ix in range(gHeight) and transf_iy in range(gWidth)): 
                gMapLaser[transf_ix][transf_iy] = gMapLaser[transf_ix][transf_iy] + 1

            if msg.ranges[i]<tooClose:
                if i<center:
                    gBumpLeft=True 
                else:
                    gBumpRight=True
    return



# tracer_node - test 1
# Moves forward until it detects a close surface
def tracer_node():
    '''continually move forward until a close surface is detected'''
    global gBumpLeft, gBumpRight

    # all ROS 'nodes' (ie your program) have to do the following
    rospy.init_node('Tracer_Node', anonymous=True)

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
        #square node goes here to map room

        msg.angular.z = math.pi / 2.0 
        vel_pub.publish(msg)
        rospy.sleep(1)
        msg.linear.x = 0.5 
        msg.angular.z = 0
        vel_pub.publish(msg)
        rospy.sleep(3)
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
    print(gMapLaser)

    # map plotting commands
    plt.pcolor(gMapLaser)
    plt.show()
    return

#-------------------------------MAIN  program----------------------
if __name__ == '__main__':
    try:
        rospy.on_shutdown(callback_shutdown)
        tracer_node()
    except rospy.ROSInterruptException:
        pass

