#!/usr/bin/env python

# import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# Initialize the ROS node named limo_pov_node
rospy.init_node('limo_pov_node', anonymous=True)

# Print "hello limo_pov" to terminal 
rospy.loginfo("Hello Limo_pov")

# Initialize the CvBridge class
bridge = CvBridge()

# function to show the image in OpenCV window
def show_image(img):
    cv2.imshow("LIMO POV", img)
    cv2.waitKey(3)

def show_changed_image(img):
    converted = img[:, :, [2, 0, 1]]
    cv2.imshow("LIMO's WACKY POV", converted)
    cv2.waitKey(3)

# callback for the Image messsage
def image_callback(img_msg):
    #convert ROS image message to CV2 image
    try: 
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e: 
        rospy.logerr("CvBridge Error: {0}".format(e))
    #show image
    show_image(cv_image)
    show_changed_image(cv_image)

# Initialize a subscriber to the camera/rgb/image_raw topic with the 
# function image callback as a callback
sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# initialize an OpenCV window named "LIMO POV"
#cv2.namedWindow("LIMO POV", 1)

# loop to keep the program from shutting down unless ROS is shut down 
# or ctrl c is pressed
while not rospy.is_shutdown():
	rospy.spin()
