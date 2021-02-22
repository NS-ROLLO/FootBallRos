#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import cv2
import os
import numpy as np
import sys, select, termios, tty
import time
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import glob
import roslib
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

lowerBound=np.array([170,70,50])    # 0 70 50     

upperBound=np.array([179,255,255])   #10 255 255

kernelOpen=np.ones((5,5))
kernelClose=np.ones((20,20))

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

flag1=0
flag2=1
flag3=1
flag4=0
flag5=0
global destinatie

class image_feature:
    def __init__(self):
        self.bridge = CvBridge()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback2)
        print "subscribed to odometry"
        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.callback)
        print "subscribed to /camera/image/compressed"
      
        #rospy.init_node('turtlebot3_teleop')
        
        #turtlebot3_model = rospy.get_param("model", "burger")

    def callback2(self, ros_data):
        global destinatie
        global flag1
        global flag2
        global flag3
        global flag4
        global flag5
        '''
        if flag5==0:
            action=actionlib.SimpleActionClient('move_base',MoveBaseAction)
            action.wait_for_server()
            dest=MoveBaseGoal()
            dest.target_pose.header.frame_id="map"
            dest.target_pose.header.stamp=rospy.Time.now()
            dest.target_pose.pose.position.x=-0.1
            #dest.target_pose.pose.position.y=1.3
            dest.target_pose.pose.orientation.w=1.0

            action.send_goal(dest)
            wait=action.wait_for_result()
            flag5=1
        #E : -0.14,1.3
        #V:  -0.1, 2.2
        '''

        #-16

        if flag2==0: #0 pentru N sau S
            pos=ros_data.pose.pose.position.x
        else : #1 pentru V sau E
            pos=ros_data.pose.pose.position.y

        if flag3==0: # 0 pentru N sau V
            destinatie=-27
        else:   #1 pentru S sau E
            destinatie=-27

        if pos>=destinatie and flag3==0:
            flag4=0
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)
            
            if(flag1==0):
                rospy.loginfo("A ajuns la destinatie!")
                flag1=1   
        elif pos<=destinatie and flag3==1:
            flag4=0
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            pub.publish(twist)
            if(flag1==0):
                rospy.loginfo("A ajuns la destinatie!")
                flag1=1    
        else :
            flag4=1

    def callback(self, ros_data):
        
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        

        imgHSV= cv2.cvtColor(image_np,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(imgHSV,lowerBound,upperBound)

        maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
        maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)

        hist = cv2.calcHist([maskOpen],[0],None,[256],[0,256])

        pixel_alb=np.sum(maskOpen==255)
        pixeli=np.sum(maskOpen>=0)

        rezultat=(pixel_alb*100)/pixeli

        target_linear_vel   = 0.0
        target_angular_vel  = 0.0

        global flag4

        if flag4==1:
            if(rezultat>0.4): #pixel_alb>30000
                print 'Minge detectata'
                
                target_linear_vel=target_linear_vel+0.10
                twist = Twist()
                twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel
                pub.publish(twist)
                twist=Twist()

            else:
                print "not detected"
                target_angular_vel=target_angular_vel+0.20
                twist = Twist()
                twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel
                pub.publish(twist)
        
 
        cv2.imshow("Image window", image_np)
        cv2.waitKey(3)              
    
def main():
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()