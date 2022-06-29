#!/usr/bin/env python

""" 
    Your package should be called “limo_pov”
    Your node should be called “limo_pov_node”
    Your node should follow the behaviour described below
        Subscribe to “/camera/rgb/image_raw” topic
        For each image in the stream, display it in a window called “LIMO POV” using OpenCV's “imshow” function
"""

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

rospy.loginfo("limo_pov script initiated")

# init node
rospy.init_node('limo_pov_node', anonymous=True)
rospy.loginfo("limo_pov_node initialised")

# init CVBridge class obj
bridge = CvBridge()

# func for show img in OpenCV window
def show_img(img):
    cv2.imshow("LIMO POV", img)
    cv2.waitKey(3)

def show_cvt_img(img):
    # swap the order of channels into GBR
    cvt = img[:, :, [2, 0, 1]]
    cv2.imshow("LIMO's WACKY POV", cvt)
    cv2.waitKey(3)

# callback for Image message
def img_callback(img_msg):
    # log info about the image topic
    rospy.loginfo(img_msg.header)

    # try: cvt ROS Image to CV2 Image
    try:
        cv_img = bridge.imgmsg_to_cv2(img_msg, "conversion")
    except CvBridgeError:
        rospy.logerr("CVBridge Err")

    show_img(cv_img)
    show_cvt_img(cv_img)

# subscriber
img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, img_callback)
cv2.namedWindow("LIMO POV", 1)

# loop unless intervened
while not rospy.is_shutdown():
    rospy.spin()
