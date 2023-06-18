#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math
import time
# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray


hola_x = 0
hola_y = 0
hola_theta = 0
x_goals = [0,0,0,0,0]
y_goals = [0,0,0,0,0]
theta_goals = [0,0,0,0,0]

def odometryCb(msg):
    global hola_x, hola_y, hola_theta, roll, pitch
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y
    
    rot_q= msg.pose.pose.orientation
    (roll,pitch,hola_theta) = euler_from_quaternion([rot_q.x ,rot_q.y ,rot_q.z, rot_q.w]) 
    
def task1_goals_Cb(msg):
    global x_goals, y_goals, theta_goals

    x_goals.clear()
    y_goals.clear()
    theta_goals.clear()

    for waypoint_pose in msg.poses:
        x_goals.append(waypoint_pose.position.x)
        y_goals.append(waypoint_pose.position.y)

        orientation_q = waypoint_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        theta_goal = euler_from_quaternion (orientation_list)[2]
        theta_goals.append(theta_goal)
        
def main():
	# Initialze Node
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
    rospy.init_node('controller',anonymous=True)
    pub_obj = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    sub_obj = rospy.Subscriber('/odom',Odometry,odometryCb)
    
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively

	# Declare a Twist message
    vel = Twist()
	# Initialise the required variables to 0
    vel.linear.x = 0
    vel.linear.y = 0
    vel.angular.z = 0
	# <This is explained below>
	
	# For maintaining control loop rate.
    rate = rospy.Rate(100)

	# Initialise variables that may be needed for the control loop
    global hola_x, hola_y, hola_theta, x_goals, y_goals, theta_goals
    rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
    time.sleep(5)
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
    #x_d = 1
    #x_goals = [1,-1,-1,1,0]
    #y_d = 1
    #y_goals = [1,1,-1,-1,0]
    #theta_d = (math.pi)/4
    #p = math.pi
    #theta_goals = [0.785, 2.335, -2.335, -0.785, 0]
    Kp_x = 2
    Kp_y = 2
    Kp_theta = 3
	# and also Kp values for the P Controller
	#
	# 
	# Control Loop goes here
	#
	#
    
    for i in range(len(x_goals)):
        x_d = x_goals[i]
        y_d = y_goals[i]
        theta_d = theta_goals[i]
        
        while not rospy.is_shutdown():
        
		# Find error (in x, y and theta) in global frame
            Errorx = x_d - hola_x
            Errory = y_d - hola_y
            Errortheta = theta_d - hola_theta
		# the /odom topic is giving pose of the robot in global frame
		# the desired pose is declared above and defined by you in global frame
		# therefore calculate error in global frame
	
		# (Calculate error in body frame)
		# But for Controller outputs robot velocity in robot_body frame, 
		# i.e. velocity are define is in x, y of the robot frame, 
		# Notice: the direction of z axis says the same in global and body frame
		# therefore the errors will have have to be calculated in body frame.
		# 
            final_Errorx = Errorx * math.cos(hola_theta) + Errory * math.sin(hola_theta)
            final_Errory =  - Errorx * math.sin(hola_theta) + Errory * math.cos(hola_theta)
            final_Errortheta = Errortheta
		# This is probably the crux of Task 1, figure this out and rest should be fine.

		# Finally implement a P controller
		# to react to the error with velocities in x, y and theta.
            vel_z = final_Errortheta*Kp_theta
            vel_y = final_Errory*Kp_y
            vel_x = final_Errorx*Kp_x
		# Safety Check
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run
            
            vel.linear.x = vel_x
            vel.linear.y = vel_y
            vel.angular.z = vel_z 
            
            pub_obj.publish(vel)
            if (math.fabs(final_Errorx) < 0.02) and (math.fabs(final_Errory) < 0.02) and (math.fabs(final_Errortheta) < 0.015) :
                #breaks if the goal is reached
                vel.linear.x = 0
                vel.linear.y = 0
                vel.linear.z = 0
                pub_obj.publish(vel)
                time.sleep(5)
                break
                
            rate.sleep()
    rospy.spin()
        



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
