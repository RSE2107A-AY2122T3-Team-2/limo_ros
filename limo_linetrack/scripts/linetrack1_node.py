#!/usr/bin/env python

import cv2
import numpy as np
import matplotlib.pylab as plt
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def draw_the_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank_image, (x1,y1), (x2,y2), (0, 0, 255), thickness=5)

    img = cv2.addWeighted(img, 0.6, blank_image, 1, 0.0)
    return img

# Initialize the CvBridge class
bridge = CvBridge()

# Initialize a subscriber to the camera/rgb/image_raw topic with the 
# function image callback as a callback

# callback for the Image messsage
def image_callback(img_msg):
    #convert ROS image message to CV2 image
    try: 
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError, e: 
        rospy.logerr("CvBridge Error: {0}".format(e))
    #show image
    img_pro(rgb)

sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

print(image.shape)
height = image.shape[0]
width = image.shape[1]
region_of_interest_vertices = [
    (0, height),
    (0, 900),
    (750, height/2),
    #(width/2, height/2),
    #(1150, height/2),
    #(width, 1000),
    #(width,height)

]
gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
gauss_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
canny_image = cv2.Canny(gauss_image, 100, 200)
cropped_image = region_of_interest(canny_image,
                np.array([region_of_interest_vertices], np.int32),)
lines = cv2.HoughLinesP(cropped_image,
                        rho=6,
                        theta=np.pi/180,
                        threshold=160,
                        lines=np.array([]),
                        minLineLength=40,
                        maxLineGap=25)
image_with_lines = draw_the_lines(img, lines)

# Initialize the CvBridge class
bridge = CvBridge()

# Initialize a subscriber to the camera/rgb/image_raw topic with the 
# function image callback as a callback

# callback for the Image messsage
def image_callback(img_msg):
    #convert ROS image message to CV2 image
    try: 
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError, e: 
        rospy.logerr("CvBridge Error: {0}".format(e))
    #show image
    img_pro(rgb)

sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

cv2.imshow("Detected Lane Lines", image_with_lines)
cv2.waitKey(0)
