#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2 as cv
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
        img_GS = cv.cvtColor(img_crop, cv.COLOR_BGR2GRAY)
        
        dst = cv.Canny(img_GS, 50, 200, None, 3)
        
        # Copy edges to the images that will display the results in BGR
        cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)
        
        lines = cv.HoughLines(dst, 1, np.pi / 180, 150, None, 0, 0)
        if lines is not None:
        	for i in range(0, len(lines)):
        		rho = lines[i][0][0]
        		theta = lines[i][0][1]
        		a = math.cos(theta)
        		b = math.sin(theta)
        		x0 = a * rho
        		y0 = b * rho
        		pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        		pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        		cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)
        
        linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
        
        if linesP is not None:
        	for i in range(0, len(linesP)):
        		l = linesP[i][0]
        		cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)
        cv.imshow("Source", img_crop)
        cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        
        cv.waitKey(1)
 
    def run(self):
    	rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
    
    
