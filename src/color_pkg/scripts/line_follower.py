#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from ros_opencv import ROS2OPENCV
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class LinDetector(ROS2OPENCV):
    def __init__(self, node_name):
        super(LinDetector, self).__init__(node_name)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.move_direction = 0 #0-mid 1-left 2-right 3-stop
        self.speed_linear = rospy.get_param("~speed_linear", 0.2)
        self.speed_angular = rospy.get_param("~speed_angular", 0.2)

        self.joySubscriber = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.robot_enable_button = rospy.get_param('~enable_button', 4)
        self.axes = []
        self.buttons = []

    def process_image(self, frame):
        src = frame.copy()
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
        dst = cv2.dilate(dst, None, iterations=2)
        dst = cv2.erode(dst, None, iterations=2)
        
        cv2.imshow("dst", dst)

        color = dst[100]
        white_count = np.sum(color == 255)
        white_index = np.where(color == 255)
        if white_count == 0:
            white_count = 1
        center = 0
        #center = (white_index[0][white_count - 1] + white_index[0][0]) / 2
        offset = center - self.frame_width / 2

        if(offset > 5):
            self.move_direction = 2
        elif(offset < -5):
            self.move_direction = 1
        else:
            self.move_direction = 0
        
        cmd = Twist()
        ###generate cmd###
        if self.move_direction == 0:           
            cmd.linear.x = self.speed_linear
        elif self.move_direction == 1:
            cmd.linear.x = self.speed_linear
            cmd.angular.z = self.speed_angular
        elif self.move_direction == 2:
            cmd.linear.x = self.speed_linear
            cmd.angular.z = 0 - self.speed_angular
            
        if (self.buttons[self.robot_enable_button] == 1):
            self.cmd_pub.publish(cmd)
        else:
            self.cmd_pub.publish(Twist())
        #####return result##### 
        processed_image = frame.copy() 
        return processed_image

    def joy_cb(self, data):
        self.axes = data.axes
        self.buttons = data.buttons

if __name__ == '__main__':
    try:
        node_name = "line_follower"
        LinDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down line follower node"
cv2.destroyAllWindows()
    

