#! /usr/bin/env python

# Author Pierre-Vincent Gouel (pg58)

import roslib
import sys
import rospy
import cv2
import numpy as np
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('kinect_observer',anonymous=True)
bridge = CvBridge()
ref_image = None
start_time = time.time()

def callback(img):
    global bridge, ref_image
    cv_image = bridge.imgmsg_to_cv2(img, "bgr8")# Converts the image to be manipulated through Opencv
    current_time = time.time()-start_time
    if current_time>3 and current_time <4:
        ref_image =  cv_image
        print "geting image"
    elif current_time > 4:
        hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        lower_cloak1 = np.array([170,150,70])
        upper_cloak1 = np.array([180,255,255])
        lower_cloak2 = np.array([0,150,70])
        upper_cloak2 = np.array([15,255,255])
        mask1 = cv2.inRange(hsv_img,lower_cloak1,upper_cloak1)
        mask2 = cv2.inRange(hsv_img,lower_cloak2,upper_cloak2)
        mask = mask1 + mask2
        kernel = np.ones((3,3),np.uint8)
        mask = cv2.erode(mask,kernel,iterations = 1)
        mask = cv2.dilate(mask,kernel,iterations =1)
        trans_img = cv2.bitwise_and(ref_image,ref_image,mask=mask)
        front_img = cv_image
        front_img[mask>0] = [0,0,0]
        final_img = cv2.bitwise_or(front_img,trans_img)
        cv2.imshow("raw image",final_img)
    else:
        print "moove"
    cv2.waitKey(3)

image_sub = rospy.Subscriber("/image_raw",Image,callback)
rospy.spin()
