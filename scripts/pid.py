#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist




rospy.init_node('pid_controller', anonymous=True)
pub = rospy.Publisher('/nature/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    twist = Twist()
    twist.linear.x = 1
    twist.angular.z = 0

    pub.publish(twist)
    rate.sleep() #sleep until the next time to publish