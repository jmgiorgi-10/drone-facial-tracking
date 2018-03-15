#!/usr/bin/env python

## Copyright 2018 Joaquin Giorgi jmgiorgi@bu.edu
import cv2
import dlib
import numpy as np
import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Twist

def takeoff():
	rate = rospy.Rate(10);  # 10 Hertz.
	takeoff_pub.publish(Empty())
	rate.sleep()

def land():
	land_pub.publish(Empty())

## Function for sending geometry twist commands.
def setCommand(x, y, z, ang_x, ang_y, ang_z):
	rate = rospy.Rate(10);
	vel_msg = Twist();
	vel_msg.linear.x = x;
	vel_msg.linear.y = y;
	vel_msg.linear.z = z;
	vel_msg.angular.x = ang_x;
	vel_msg.angular.y = ang_y;
	vel_msg.angular.z = ang_z;
	move_pub.publish(vel_msg);
	rate.sleep();

def callback(ydata):
	global times;
	if (times > 300):
		ycomm = ydata.data * -1;
		setCommand(ydata.data,0,0,0,0,0);  ## Just adjust drone y-position for now.

if __name__ == '__main__':
	takeoff_pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size = 10)  ## Declare all publishers.
	land_pub = rospy.Publisher("ardrone/land", Empty, queue_size = 10)
	move_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
	rospy.init_node('takeoff');
	##mag_sub = rospy.Subscriber("dir_mag", numpy_msg(Floats), callback, queue_size = 10)
    
	## Subscribe to control_effort topic from pid node, to give right values to setCommand.
	control_sub = rospy.Subscriber("control_effort", Float64, callback, queue_size=10)	

	start = time.time();
	times = 0;
	while (not rospy.is_shutdown()):
		if (times < 1200):
			takeoff();
			times += 1;
		else:
			land();
		#if (times < 30):
		 	##setCommand(0,0,0,0,0,0);
		
		#print(times);
