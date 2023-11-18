#!/usr/bin/env python3
import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry

datafile = "test1.csv"

def odomCallback(data):
    global location
    location = [data.pose.pose.position.x, data.pose.pose.position.y]
    
    f.write(f"{data.header.stamp.secs}.{data.header.stamp.nsecs}, {location[0]}, {location[1]}\n")
    


rospy.init_node('pid_controller', anonymous=True)
locationSub = rospy.Subscriber('/mavs_ros/odometry_true',Odometry, odomCallback)
f = open(datafile, "a")
while not rospy.is_shutdown():
    pass
f.close()