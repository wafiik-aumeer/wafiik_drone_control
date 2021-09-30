#!/usr/bin/env python

import rospy
import rosservice
import rospy
import math 
import sys
import time
from numpy import sign
import numpy as np
import random
import roscpp
from std_msgs.msg       import Float64
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from udm_group2_drone_control.srv import *

def pi2pi(angl):

    global PI;
    
    while(abs(angl)-PI > 0.001):
        if(angl>PI):
            angl = angl-2*PI;
        
        if (angl<-PI):
            angl = angl+2*PI;

    return angl


#get drone's real position
def get_rotation(data1):
    global x_current, y_current, z_current, quat_x, quat_y, quat_z, quat_w, roll, pitch, yaw
    x_current  = data1.pose.position.x
    y_current  = data1.pose.position.y
    z_current  = data1.pose.position.z
    quat_x   = data1.pose.orientation.x 
    quat_y   = data1.pose.orientation.y
    quat_z   = data1.pose.orientation.z
    quat_w   = data1.pose.orientation.w
    orientation_list = [quat_x, quat_y, quat_z, quat_w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

def handle_req(req):
    global roll, pitch, yaw, x_current,y_current,z_current,PI
    altitude_data=float(req.altitude.data)
    x_current = 0.0
    y_current = 0.0
    z_current = 0.0

    quat_x  = 0.0
    quat_y  = 0.0
    quat_z  = 0.0
    quat_w  = 0.0
    goal_x  = float(req.horizontal_x.data)
    goal_y  = float(req.horizontal_y.data)
    roll  = 0.0
    pitch = 0.0
    yaw   = 0.0

    PI = 3.14159265359
    rate = rospy.Rate(10)
    
    flag = 0
    goal_altitude = altitude_data
    kp_z   = 0.2
    error = goal_altitude
    Kp_linear = 0.3
    Kd_linear = 0.05
    Ki_linear = 0.001
    Kp_angular = 3
    Kd_angular = 0.1
    Ki_angular = 0.01

    sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, get_rotation)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 15)

    cmd =Twist()

    while not rospy.is_shutdown():

        rate.sleep()
        rate.sleep()    
            
        distanceC = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)))
        distanceP = distanceC
        
        angular_errorP = 0.0

        total_error         = 0.0
        total_angular_error = 0.0

        while abs(distanceC)>1:

            distanceC     = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)))
            linear_speed  = (distanceC * Kp_linear) + ((distanceC - distanceP)* Kd_linear) + (total_error*Ki_linear)
            
            error = goal_altitude - z_current


            desired_angle_goal = math.atan2(goal_y-y_current, goal_x-x_current)
            angular_errorC     = pi2pi(desired_angle_goal-yaw)
            angular_speed      = (angular_errorC*Kp_angular) + ((angular_errorC-angular_errorP)*Kd_angular) + (total_angular_error*Ki_angular)

            print('distance = {} angular error = {} '.format(distanceC, angular_errorC*180/PI))

            cmd.linear.x  = linear_speed
            cmd.angular.z = angular_speed
            cmd.linear.z  = kp_z * error

            distanceP      = distanceC
            angular_errorP = angular_errorC

            total_error         = total_error + distanceC
            total_angular_error = total_angular_error + angular_errorC

            


            pub_cmd_vel.publish(cmd)
        
            print('Waypoint = {}--[{},{}] '.format(1,goal_x,goal_y))
            flag=1
            rate.sleep()
    
	rep=udm_drone_serviceResponse()
	rep.res.data=True
	return rep

    
def simple_server():
    global rate
    rospy.init_node('udm_drone_move_service')
    rate = rospy.Rate(10) # 5hz
    
    s = rospy.Service('udm_drone_move_service', udm_drone_service, handle_req)
    rospy.spin()

if __name__ == "__main__":
    simple_server() 
