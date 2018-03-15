#!/usr/bin/env python

## Constantly publishes the desired setpoint(s) for the pid loop.
from std_msgs.msg import Float64
from std_msg.msg import Bool

if __name__ == '__main__':
	## Publish to pid setpoint topic (constant value in this case).
	setpoint_pub = rospy.Publisher("setpoint", Float64, queue_size=1)
	while (not(rospy.is_shutdown())):
		setpoint_pub.publish(0.0);