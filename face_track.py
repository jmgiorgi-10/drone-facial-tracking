#!/usr/bin/env python

# Python rospy test
import rospy
import numpy as np
import cv2
import dlib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

def callback(data):
	global tracking
	global maxArea
	global xx, yy, ww, hh
	global tx, ty, tw, th
	## Convert image topic to cv format.
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		height, width, channels = cv_image.shape;
	except CvBridgeError as e:
		print(e)
	## Apply frontalface cascade detection.
	if tracking == False:
		#small_image = cv2.resize(cv_image, (320,240))  ## Resize original image.
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		faces = lbp_face_cascade.detectMultiScale(gray, 1.3, 5)
		for (x,y,w,h) in faces:
			if (w*h > maxArea):
				xx = int(x);
				yy = int(y);
				ww = int(w);
				hh = int(h);
				maxArea = w*h;
		    #cv2.rectangle(small_image,(xx,yy),(xx+ww,yy+hh),(255,0,0),2) 
		if (maxArea > 0):
			tracker.start_track(cv_image, dlib.rectangle(xx-10,yy-20,xx+ww+10,yy+hh+20))
			tracking = True
		#else:
			#image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	if (tracking == True):
		tracking_quality = tracker.update(cv_image)
		print(tracking_quality)
		print("\n")

		if (tracking_quality >= 15):
			track_pos = tracker.get_position()
			tx = int(track_pos.left())
			ty = int(track_pos.top())
			tw = int(track_pos.width())
			th = int(track_pos.height())
			cv2.rectangle(cv_image, (tx,ty),(tx + tw,ty + th),(255,0,0),2)
			#image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		else:
			tracking = False;
			maxArea = 0;
	## Publish result as image topic.                                                                                                        
	try:
		image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	except CvBridgeError as e:
		print(e)
	## Publish y & z positions of detected face (y --> x, z -- > y)
	pos_array = np.array([np.float32(tx + tw/2), np.float32(ty - th/2)]);
	pos_pub.publish(pos_array);  ## Publish coordinates of current face center.
	if (tracking == True):
		ystate_pub.publish(-(tx + tw/2));
	else:
		ystate_pub.publish(320);
	hsetpoint_pub.publish(320);
if __name__ == '__main__':
	maxArea = 0
	xx = 0;
	yy = 0;
	ww = 0
	hh = 0;
	tx = 0;
	ty = 0;
	tw = 0;
	th = 0;
	tracking = False  ## Can we start tracking yet?
	tracker = dlib.correlation_tracker()
	
	face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
	lbp_face_cascade = cv2.CascadeClassifier('lbpcascade_frontalface.xml')
	bridge = CvBridge()
	rospy.init_node('face_tester')
	image_sub = rospy.Subscriber("/ardrone/image_raw", Image, callback, queue_size=1, buff_size = 2**24)
	image_pub = rospy.Publisher("face_image", Image, queue_size=1)
	pos_pub = rospy.Publisher("face_pos", numpy_msg(Floats), queue_size=1)

	
	## Publish to pid state topic (center of face position, found through detection/tracking).
	ystate_pub = rospy.Publisher("state", Float64, queue_size=1)
    ## Current horizontal setpoint for drone
	hsetpoint_pub = rospy.Publisher("setpoint", Float64, queue_size=1)
	enable = rospy.Publisher("pid_enable", Bool, queue_size=10)
	enable.publish(True);
	rospy.spin()
