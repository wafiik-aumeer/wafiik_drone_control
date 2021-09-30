#!/usr/bin/env python

import numpy as np
from udm_group2_drone_control.srv import *
import rospy
import rosservice
from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg  import Twist    
import random
import math
import roscpp
from   std_msgs.msg       import Float64
from   geometry_msgs.msg  import PoseStamped
from   tf.transformations import euler_from_quaternion, quaternion_from_euler


x_current = 0.0
y_current = 0.0
z_current = 0.0
flag = 0
quat_x  = 0.0
quat_y  = 0.0
quat_z  = 0.0
quat_w  = 0.0

roll  = 0.0
pitch = 0.0
yaw   = 0.0

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
    
if __name__ == "__main__":
    rospy.init_node('waypoint_service')
    waypoint_x = np.array([4.0, 4.0, 0.0, 0.0, 4.0, 4.0, 0.0, 0.0, 4.0])
    waypoint_y = np.array([0.0 , 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0])
    altitude = "5"
    kp_z   = 0.2
    rate = rospy.Rate(10) # 5hz
    sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, get_rotation)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 15)
    cmd =Twist()
	

    rospy.wait_for_service('udm_drone_move_service') 
    while not rospy.is_shutdown():
        if(flag==0):
            rate.sleep()
            rate.sleep()
            for i in range(len(waypoint_x)):
                goal_x    = str(waypoint_x[i])
                goal_y    = str(waypoint_y[i])
                drone_move_service = rospy.ServiceProxy('udm_drone_move_service', udm_drone_service) 
                resp = drone_move_service(String(altitude), String(goal_x), String(goal_y))
                flag = 1
            
                print(resp)		

            error = z_current-0.0
            print('Height error = {} '.format(error))
            while error>0.2:
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0
                error         = z_current-0.0
                cmd.linear.z  = -kp_z * error

                pub_cmd_vel.publish(cmd)
                print('Height error = {} '.format(error))
                rate.sleep()
        else:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            cmd.linear.z  = 0.0
            pub_cmd_vel.publish(cmd)
            print('Quad Landed Sucessfully Covering all the Waypoints')
            rate.sleep()
