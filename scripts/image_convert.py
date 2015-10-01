#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from roslib import message

import numpy as np
import cv2, ast
import time, redis, os, sys

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
cv2.namedWindow("frame", 1)
#cv2.namedWindow("processed_1",1)
#cv2.namedWindow("processed_2",1)
#cv2.namedWindow("processed_3",1)
#detector = cv2.SimpleBlobDetector()

CAM_WIDTH = 640
CAM_HEIGHT = 480
_ts = time.strftime("%Y-%m-%d-%H%M%S")

## Define the codec and create VideoWriter object
fourcc = cv2.cv.CV_FOURCC(*'MJPG')#(*'H264')#(*'MJPG')
out = cv2.VideoWriter('balaji_output'+_ts+'.avi',fourcc, 25.0, (CAM_WIDTH,CAM_HEIGHT))

def filter_points(image_message):
    print 'got image...'
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8") #passthrough
    out.write(cv_image)
    #cv_processed = bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8") #passthrough
    #output = cv_image.copy()
    cv2.imshow('frame',cv_image)
    #print type(cv_image)
    #b,g,r = cv2.split(cv_image)
    #r = cv_image[:,:,2]
    #b = cv_image[:,:,0]
    #g = cv_image[:,:,1]
    #hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #lower_blue = np.array([110,50,50])
    #upper_blue = np.array([130,255,255])
    #lower_ = np.array([  0,  0,  0])
    #upper_ = np.array([180,255,230])
    #lower_blue = np.array([  0,230,  0])
    #upper_blue = np.array([150,150,255])
    
    #mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    #mask = cv2.inRange(hsv, lower_, upper_)
    #res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
    #res_blue = cv2.bitwise_and(cv_image,cv_image, mask= mask_blue)
    
    #mask_filter = 255 in mask
    #print mask
    #print mask_filter
    
    #print GetValue('Blob_Detection.blob_color') 
    
    #xy =  np.where(mask == 255)
    #print xy[0][0], xy[1][0]
    #keypoints = detector.detect(mask)
    
    #im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #circles = cv2.HoughCircles(mask, cv2.cv.CV_HOUGH_GRADIENT, 1.2, 100)
    
    #if circles is not None:
    #    circles = np.round(circles[0, :]).astype("int")
    #    for (x, y, r) in circles:
    #        cv2.circle(output, (x, y), r, (0, 255, 0), 4)
    #        cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

    #el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    #mask = cv2.dilate(mask, el, iterations=3)
    
    #cv2.imshow('processed_1',mask)
    #cv2.imshow('processed_2',res)
    #cv2.imshow('processed_3',im_with_keypoints)
    #cv2.imshow('processed_3',output)
    cv2.waitKey(3)

def talker():
    
    #rate = rospy.Rate(100)
    #while not rospy.is_shutdown():
    rospy.spin()
    
rospy.init_node('image_process', anonymous=True)
rospy.Subscriber('/usb_cam/image_raw', Image, filter_points)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
cv2.destroyAllWindows()
#out.release()
