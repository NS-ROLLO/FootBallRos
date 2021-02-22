#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):


    if msg.ranges[0] < 0.5:
        
        move.linear.x = 0.5
        move.angular.z = 0.0
        time.sleep(1)

    
    if msg.ranges[0] > 0.5:
        move.linear.x = 0.0
        move.angular.z = 0.5


    pub.publish(move)

rospy.init_node('rotw5_node')
sub = rospy.Subscriber('/scan', LaserScan, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
move = Twist()

rospy.spin()