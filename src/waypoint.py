#!/usr/bin/env python

from drone_control.srv import *
import rospy
import rosservice
from std_srvs.srv import Empty

import numpy as np
import random
import math
import roscpp
from   std_msgs.msg       import Float64
from   geometry_msgs.msg  import Twist
from   geometry_msgs.msg  import PoseStamped
from   drone_control.msg  import quad_pose
from   tf.transformations import euler_from_quaternion, quaternion_from_euler

x_current = 0.0
y_current = 0.0
z_current = 0.0

quat_x  = 0.0
quat_y  = 0.0
quat_z  = 0.0
quat_w  = 0.0

roll  = 0.0
pitch = 0.0
yaw   = 0.0

PI = 3.14159265359;

def pi2pi(angl):

    global PI;
    
    while(abs(angl)-PI > 0.001):
        if(angl>PI):
            angl = angl-2*PI;
        
        if (angl<-PI):
            angl = angl+2*PI;

    return angl

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


def main(sx,sy,sz):

    global roll, pitch, yaw, x_current,y_current,z_current,PI

    waypoint_x = sx 
    waypoint_y = sy 
    
    goal_z = sz
    
    kp_z   = 0.2
    
    Kp_linear = 0.3
    Kd_linear = 0.05
    Ki_linear = 0.001

    Kp_angular = 3
    Kd_angular = 0.1
    Ki_angular = 0.01

    flag = 0

    sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, get_rotation)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 15)
    pub_pose    = rospy.Publisher('/traj/pose', quad_pose, queue_size = 15)
    cmd =Twist()
    pose=quad_pose()


    while not rospy.is_shutdown():
        
        if(flag==0):
            rate.sleep()
            rate.sleep()

            goal_x    = waypoint_x 
            goal_y    = waypoint_y 
                
            distanceC = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)+ ((goal_z - z_current) ** 2)))
            distanceP = distanceC
                
            angular_errorP = 0.0

            total_error         = 0.0
            total_angular_error = 0.0

            while abs(distanceC)>0.1:

                distanceC     = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)))
                linear_speed  = (distanceC * Kp_linear) + ((distanceC - distanceP)* Kd_linear) + (total_error*Ki_linear)
                    
                error = goal_z - z_current


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

                pose.x = x_current
                pose.y = y_current
                pose.z = z_current

                pose.roll  = roll
                pose.pitch = pitch
                pose.yaw   = yaw


                pub_cmd_vel.publish(cmd)
                pub_pose.publish(pose)

                #print('Waypoint = {}--[{},{}] '.format(i+1,goal_x,goal_y))
                flag=1
                rate.sleep()

            error = z_current - 0.0
            #print('Height error = {} '.format(error))
            while error>0.2:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                error         = z_current - sz
                cmd.linear.z  = kp_z * error

                pose.x = x_current
                pose.y = y_current
                pose.z = z_current

                pose.roll  = roll
                pose.pitch = pitch
                pose.yaw   = yaw

                pub_cmd_vel.publish(cmd)
                pub_pose.publish(pose)
                #print('Height error = {} '.format(error))
                #print('desired altitude: ',sz)
                #print('current altitude: ',z_current)
                rate.sleep()
        else:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            cmd.linear.z  = 0.0
            pub_cmd_vel.publish(cmd)
            #print('Quad Landed Sucessfully Covering all the Waypoints')
            rate.sleep()


def handle_req(req):
    sx = int(req.x.data)
    sy = int(req.y.data)
    sz = int(req.z.data)

    try:
	main(sx,sy,sz)		
	rep=waypointResponse()
    	rep.res.data=True
    	return rep      
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    global rate
    rospy.init_node('waypoint_service')
    rate = rospy.Rate(10) # 5hz
    s = rospy.Service('waypoint_service', waypoint, handle_req)
    #rate.sleep()
    rospy.spin() 

