#!/usr/bin/env python

# import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# Initialize the ROS node named limo_pov_node
rospy.init_node('linetrack_node', anonymous=True)

# Print "hello limo_pov" to terminal 
rospy.loginfo("Hello linetrack")

def show_image(img):
    cv2.imshow("LIMO POV", img)
    cv2.waitKey(3)

def img_pro(img):
    # find white area
    lower = np.uint8([200, 200, 180])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(img, lower, upper)
    cv2.imshow("LIMO POV", white_mask)
    cv2.waitKey(3)
 
    # apply morphology close
    kernel = np.ones((5,5), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

    # get contours and filter on area
    result = img.copy()
    contours = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    result = img.copy()
    #result1 = cv2.cvtColor(result, cv2.COLOR_HSV2RGB)
    for c in contours:
        area = cv2.contourArea(c)
        #if area > 1000 and area < 15000:
        cv2.drawContours(result, [c], -1, (0, 0, 255), 2)
    
    cv2.imshow("Detected Lanes Lines", result)
    cv2.waitKey(3)



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

# loop to keep the program from shutting down unless ROS is shut down 
# or ctrl c is pressed
while not rospy.is_shutdown():
	rospy.spin()
