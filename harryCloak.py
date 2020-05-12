#! /usr/bin/env python

# Author Pierre-Vincent Gouel (pg58)

import roslib
import sys
import rospy
import cv2
import numpy as np
import time

# Import the necessary libraries 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialisation of the ROS node and creation of a openCV bridge instance and a time instance
rospy.init_node('kinect_observer',anonymous=True)
bridge = CvBridge()
ref_image = None
start_time = time.time()

# The callback called every time the camera send a new image
def callback(img):
    global bridge, ref_image
# Get the image and convert it into bridge format  
    cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
# Save the background, gives the time to the person to leave to then capture the background
    current_time = time.time()-start_time
    if current_time>3 and current_time <4:
        ref_image =  cv_image
        print "geting image"
# If the program run a long enough time (the backround is captured)
    elif current_time > 4:
# Convert the image into HSV format which allows easily to detect the color range of the cloakss 
        hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
# Creation of the mask of the red color
        lower_cloak1 = np.array([170,150,70])
        upper_cloak1 = np.array([180,255,255])
        lower_cloak2 = np.array([0,150,70])
        upper_cloak2 = np.array([15,255,255])
        mask1 = cv2.inRange(hsv_img,lower_cloak1,upper_cloak1)
        mask2 = cv2.inRange(hsv_img,lower_cloak2,upper_cloak2)
        mask = mask1 + mask2
# Creation of the form used for the filtering and applies erosion and dilation to have a smoother image
        kernel = np.ones((3,3),np.uint8)
        mask = cv2.erode(mask,kernel,iterations = 1)
        mask = cv2.dilate(mask,kernel,iterations =1)
# Create a copy of the background with only the pixels at cloak position visible
        trans_img = cv2.bitwise_and(ref_image,ref_image,mask=mask)
# Put the image at the cloak position in black in the eal-time captured image
        front_img = cv_image
        front_img[mask>0] = [0,0,0]
# Superpose the 2 images
        final_img = cv2.bitwise_or(front_img,trans_img)
# Show the final image
        cv2.imshow("raw image",final_img)
    else:
        print "moove"
    cv2.waitKey(3)
# Subscribe to the rostopic where the image is shown
image_sub = rospy.Subscriber("/image_raw",Image,callback)
rospy.spin()
