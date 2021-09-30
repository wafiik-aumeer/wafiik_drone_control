#!/usr/bin/python
import rospy
import math 
import sys
import time
from numpy import sign
import numpy as np
import random
import roscpp
from   std_msgs.msg       import Float64
from   geometry_msgs.msg  import Twist
from   geometry_msgs.msg  import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped 
from   tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist

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

if __name__ == "__main__":
    global roll, pitch, yaw, x_current,y_current,z_current,PI
    rospy.init_node('drone_move')
    r = rospy.Rate(0.5)
    sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, get_rotation)
    flag = 0
    goal_altitude = 3.0
    kp_z   = 0.2
    error = goal_altitude
    goal_x    = 2.0
    goal_y    = 2.0
    rate = rospy.Rate(10)
    Kp_linear = 0.3
    Kd_linear = 0.05
    Ki_linear = 0.001
    Kp_angular = 3
    Kd_angular = 0.1
    Ki_angular = 0.01

    while not rospy.is_shutdown():

         
	 if flag == 0:
		 rate.sleep()
            	 rate.sleep()
		 distanceC = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)))
		 distanceP = distanceC
		        
		 angular_errorP = 0.0

		 total_error         = 0.0
		 total_angular_error = 0.0
			
		 while error > 0.2:
	    
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
		
		    if error > 0.4:
		     	 while abs(distanceC)>1:
			    distanceC     = abs(math.sqrt(((goal_y-y_current) ** 2) + ((goal_x - x_current) ** 2)))
			    linear_speed  = (distanceC * Kp_linear) + ((distanceC - distanceP)* Kd_linear) + (total_error*Ki_linear)
			    error = goal_altitude - z_current

			    desired_angle_goal = math.atan2(goal_y-y_current, goal_x-x_current)
			    angular_errorC = pi2pi(desired_angle_goal-yaw)
			    angular_speed = (angular_errorC*Kp_angular) + ((angular_errorC-angular_errorP)*Kd_angular) +(total_angular_error*Ki_angular)
			    print('distance = {} angular error = {} '.format(distanceC, angular_errorC*180/PI))
			    
			    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
			    vel_msg = Twist()
			    vel_msg.linear.x = linear_speed
			    vel_msg.linear.y = 0
			    vel_msg.linear.z = kp_z * error
			    vel_msg.angular.x = 0
			    vel_msg.angular.y = 0
			    vel_msg.angular.z = angular_speed
			    distanceP      = distanceC
			    angular_errorP = angular_errorC

			    total_error         = total_error + distanceC
			    total_angular_error = total_angular_error + angular_errorC

		 	    print('Height error = {} '.format(error))
			    velocity_publisher.publish(vel_msg)
			    print("z" + str(z_current))
			    flag = 1
			    
			    r.sleep()
	    	
	  




             
