#!/usr/bin/env python3
import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

prev_error = 0
Dintegral = 0
Hprev_error = 0
Hintegral = 0
Hderivative_error = 0
location = [0,0]
rotation = [0,0,0,0]
yaw = 0.0

p = [[2.1,2.1],[12,14],[-4,-8],[5,6],[23,11],[-28,-8],[8,12],[12,9]]

def odomCallback(data):
    global location, rotation, yaw
    location = [data.pose.pose.position.x, data.pose.pose.position.y]
    q = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,
               data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    rotation = q
    yaw = math.atan2(2*(q[3]*q[2] + q[0]*q[3]), q[3]**2 + q[0]**2 - q[1]**2 -q[2])
    if (yaw < 0):
        yaw += 2*math.pi


def distance(p1,p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def angle(p):
    
    pVec = np.array([p[0] - location[0], p[1] - location[1]])
    pVecMag = np.sqrt(pVec.dot(pVec))
    pVec = pVec/pVecMag
    hVec = np.array([math.cos(yaw),math.sin(yaw)])
    hVecMag = np.sqrt(hVec.dot(hVec))
    hVec = hVec/hVecMag
    angle = np.math.atan2(np.linalg.det([pVec,hVec]),np.dot(pVec,hVec))
    if (angle < 0):
        angle += 2*math.pi
    angle = math.degrees(angle)
    if (angle > 180):
        angle = angle - 360
    return angle
    



rospy.init_node('pid_controller', anonymous=True)
locationSub = rospy.Subscriber('/nature/odometry',Odometry, odomCallback)
pub = rospy.Publisher('/nature/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)
action = np.array([])
Kp, Kd, Ki = 0.01, 0, 0.001
HKp, HKd, HKi = 0.1, 10, 0
i = 0
DintMax = 10
DintMin = -10
print(f"heading to point {i}: [{p[i][0]},{p[i][1]}]")
while not rospy.is_shutdown():
    twist = Twist()

    
    
   
    if i < len(p):
        print(f"currently at {location}")
        print(f"heading to point {i}: [{p[i][0]},{p[i][1]}]")  
        d = abs(distance(location, p[i])) 
        
        print(f"distance to point: {d}")
           
        
        Derror = distance(location, p[i])
        Dintegral += Derror
        if (Dintegral > DintMax):
            Dintegral = DintMax
        if (Dintegral < DintMin):
            Dintegral = DintMin
        print(Dintegral)
        Dderivative_error = Derror - prev_error
        Dprev_error = Derror 
        pid_control = Kp * Derror + Ki * Dintegral + Kd * Dderivative_error
        print(f"\tdistance err term {Kp * Derror}")
        print(f"\tdistance d err term {Kd * Dderivative_error}")
        print(f"\tdistance int err term {Ki * Dintegral}")
        

        Herror = -angle((p[i][0],p[i][1]))
        print(f"heading error {Herror}")
        
        if(abs(Herror) > 0):
            Hderivative_error = Herror - Hprev_error
            Hintegral += Herror
            action =  Herror * HKp + Hderivative_error * HKd + Hintegral * HKi
            
            print(f"\theading err term {Herror * HKp}")
            print(f"\theading d err term {Hderivative_error * HKd}")
            print(f"\theading int err term {Hintegral * HKi}")
        
            if(abs(action) > 0):
                twist.angular.z = action
            twist.linear.x = pid_control
            print(twist.linear.x)
            print()
        Hprev_error = Herror
    else:
        twist.linear.x = 0
        twist.angular.z = 0

    if d < 2:
        Dintegral = 0
        Hintegral = 0 #resets error when new control point
        i += 1
        
    pub.publish(twist)
    rate.sleep()
