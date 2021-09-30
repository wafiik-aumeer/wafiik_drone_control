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

    roll  = 0.0
    pitch = 0.0
    yaw   = 0.0

    PI = 3.14159265359
    r = rospy.Rate(0.5)
    goal_altitude = altitude_data
    print("goal altitude" + str(altitude_data))
    kp_z   = 0.2
    error = goal_altitude

    while not rospy.is_shutdown():
        #print("x" + str(x_current) + "y" + str(y_current) + "z" + str(z_current))
        try:	 
            while error > 0.2:
		sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, get_rotation)  
		print("z" + str(z_current))         
                velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = kp_z * error
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                print('Height error = {} '.format(error))
                velocity_publisher.publish(vel_msg)
                print("z" + str(z_current))
                error = goal_altitude - z_current               
                r.sleep()

            rep=udm_serviceResponse()
            rep.res.data=True
            return rep
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    
def simple_server():
    global rate
    rospy.init_node('udm_altitude_service')
    rate = rospy.Rate(10) # 5hz
    s = rospy.Service('udm_altitude_service', udm_service, handle_req)
    rospy.spin()

if __name__ == "__main__":
    
    simple_server() 
