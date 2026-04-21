#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from ros_opencv import ROS2OPENCV
from std_msgs.msg import String

class FireDetector(ROS2OPENCV):
    def __init__(self, node_name):
        super(FireDetector, self).__init__(node_name)
        self.tts_topic = rospy.get_param("~tts_topic", "/robot_voice/tts_topic")
        self.fire_pub = rospy.Publisher(self.tts_topic,String,queue_size=10)
        self.detect_box = None
        self.initRange()
    
    def process_image(self, frame):
        src = frame.copy()
        ###convert rgb to hsv###
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        
        ###create inrange mask(yellow and red)###
        res = src.copy()
        mask_red1 = cv2.inRange(hsv, self.red_min, self.red_max)
        mask_red2 = cv2.inRange(hsv, self.red2_min, self.red2_max)
        mask_yellow = cv2.inRange(hsv, self.yellow_min, self.yellow_max)
        mask = cv2.bitwise_or(mask_red1, mask_red2)
        mask = cv2.bitwise_or(mask, mask_yellow)
        #cv2.imshow("mask",mask)
        '''
        for(color_min, color_max, name) in self.COLOR_ARRAY:
            mask = cv2.inRange(hsv, color_min, color_max)
            res = cv2.bitwise_or(res, res, mask=mask)
        '''
        res = cv2.bitwise_and(src, src, mask=mask)
        h,w = res.shape[:2]
        blured = cv2.blur(res,(5,5))
        ret, bright = cv2.threshold(blured,10,255,cv2.THRESH_BINARY)
        ###open and close calculate###
        gray = cv2.cvtColor(bright,cv2.COLOR_BGR2GRAY)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        #cv2.imshow("gray", closed)
        contours, hierarchy = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (255,0,0), 2)
        total_area = 0
        for i in range(0, len(contours)):
            x, y, w, h = cv2.boundingRect(contours[i])
            ###calculate total fire area###
            total_area += w*h
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        
        #####publish fire message#####
        if (len(contours)>10) and (total_area > 40):
            fire_str = "fire"
            self.fire_pub.publish(fire_str)
            cv2.circle(frame, (30,30), 10, (0, 0, 255), -1)
        #####return result##### 
        processed_image = frame.copy() 
        return processed_image
    
    def initRange(self):
        self.red_min = np.array([0, 128, 46])
        self.red_max = np.array([5, 255,  255])
        self.red2_min = np.array([156, 128,  46])
        self.red2_max = np.array([180, 255,  255])
        self.yellow_min = np.array([15, 128, 46])
        self.yellow_max = np.array([50, 255, 255])
        '''
        self.COLOR_ARRAY = [
            [self.red_min, self.red_max, 'red'],
            [self.red2_min, self.red2_max, 'red'],
            [self.yellow_min, self.yellow_max, 'yellow']
        ]
        '''

if __name__ == '__main__':
    try:
        node_name = "fire_detector"
        FireDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
cv2.destroyAllWindows()
