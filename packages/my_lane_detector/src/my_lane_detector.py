#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib
import math

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:     			
        		
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### REMEMBER TO CHANGE THE TOPIC NAME! #####        
        self.image_sub = rospy.Subscriber('/rjbot/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        #############################################

        rospy.init_node("my_lane_detector")
    
    def image_callback(self,msg):
        rospy.loginfo("image_callback")


        # Convert to opencv image 
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        #### YOUR CODE GOES HERE ####
        
        #Crop Image
        img_crop = img [140:200, 130:450]
        
        #Apply HSV 
        img_HSV = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
        img_GS = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
        
        
        #bgr
        #White Mask
        lower_white = np.array([0,0,200])
        upper_white = np.array([180,30,255])
        mask_white = cv2.inRange(img_HSV, lower_white, upper_white)
        
        #Yellow Mask
        lower_yel = np.array([0,140,140])
        upper_yel = np.array([180,255,255])
        mask_yel = cv2.inRange(img_HSV, lower_yel, upper_yel)
        
        #Apply Masks to img
        img_yel = cv2.bitwise_and(img_HSV, img_HSV, mask = mask_yel)
        img_white = cv2.bitwise_and(img_HSV, img_HSV, mask = mask_white)
        
        #Combine Masks
        img_mask = cv2.bitwise_or(img_yel, img_white)
        
        #Apply Canny
        t_lower = 50
        t_upper = 150
        img_out = cv2.Canny(img_mask, t_lower, t_upper)
        
        dst = cv2.Canny(img_GS, 50, 200, None, 3)
        
        # Hough Transform
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)
        
        
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
        
        if linesP is not None:
        	for i in range(0, len(linesP)):
        		l = linesP[i][0]
        		cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        
        

        print("Shape ", img_out.shape)
        

        #############################

        # Show image in a window
        cv2.imshow('img_out',img_out)
        cv2.imshow('hough',cdstP)
        cv2.waitKey(1)
   

    def run(self):
    	rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
    
    
