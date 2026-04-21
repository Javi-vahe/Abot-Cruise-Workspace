#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import time

class ROS2OPENCV(object):
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.on_shutdown(self.shutdown)
        self.window_name = self.node_name
        self.rgb_topic = rospy.get_param("~rgb_image_topic", "/usb_cam/image_raw")
        self.depth_topic = rospy.get_param("~depth_image_topic", "/depth/image_raw")
        self.rgb_image_sub = rospy.Subscriber(self.rgb_topic,Image,self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber(self.depth_topic,Image,self.depth_image_callback)
        self.bridge = CvBridge()
        
        self.frame = None
        self.frame_width = None
        self.frame_height = None
        self.frame_size = None
        self.drag_start = None
        self.selection = None
        self.track_box = None
        self.detect_box = None
        self.display_box = None
        self.marker_image = None
        self.processed_image = None
        self.display_image = None
        self.target_center_x = None
        self.cps = 0
        self.cps_values = list()
        self.cps_n_values = 20
        self.angle_Horizontal = 100.0
        

    #####rgb-image callback function#####
    def rgb_image_callback(self, data):
        start = time.time()
        frame = self.convert_image(data)
        if self.frame is None:
            self.frame = frame.copy()
            self.marker_image = np.zeros_like(frame)
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size 
            cv2.imshow(self.window_name, self.frame)
            cv2.setMouseCallback(self.window_name,self.onMouse)
            cv2.waitKey(3)
        else:
            self.frame = frame.copy()
            self.marker_image = np.zeros_like(frame)
            processed_image = self.process_image(frame)
            self.processed_image = processed_image.copy()
            self.display_selection()
            self.display_image = cv2.bitwise_or(self.processed_image, self.marker_image)
            ###show track-box###
            if self.track_box is not None and self.is_rect_nonzero(self.track_box):
                tx, ty, tw, th = self.track_box
                cv2.rectangle(self.display_image, (tx, ty), (tx+tw, ty+th), (0, 0, 0), 2)
            ###show detect-box###
            elif self.detect_box is not None and self.is_rect_nonzero(self.detect_box):
                dx, dy, dw, dh = self.detect_box
                cv2.rectangle(self.display_image, (dx, dy), (dx+dw, dy+dh), (255, 50, 50), 2)
            ###calcate time and fps###
            end = time.time()
            duration = end - start
            fps = int(1.0/duration)
            self.cps_values.append(fps)
            if len(self.cps_values)>self.cps_n_values:
                self.cps_values.pop(0)
            self.cps = int(sum(self.cps_values)/len(self.cps_values))
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            if self.frame_size[0] >= 640:
                vstart = 25
                voffset = int(50+self.frame_size[1]/120.)
            elif self.frame_size[0] == 320:
                vstart = 15
                voffset = int(35+self.frame_size[1]/120.)
            else:
                vstart = 10
                voffset = int(20 + self.frame_size[1] / 120.)
            cv2.putText(self.display_image, "CPS: " + str(self.cps), (10, vstart), font_face, font_scale,(255, 255, 0))
            cv2.putText(self.display_image, "RES: " + str(self.frame_size[0]) + "X" + str(self.frame_size[1]), (10, voffset), font_face, font_scale, (255, 255, 0))            
            ###show result###
            cv2.imshow(self.window_name, self.display_image)
            cv2.waitKey(3)
              
    #####depth-image callback function#####
    def depth_image_callback(self, data):
        dept_frame = self.convert_depth_image(data)
    
    #####convert ros-image to cv2-image#####
    def convert_image(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            cv_image = np.array(cv_image, dtype=np.uint8)
            return cv_image
        except CvBridgeError, e:
            print e

    #####convert ros-depth-image to cv2-gray-image#####
    def convert_depth_image(self, ros_image):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
            depth_image = np.array(depth_image, dtype=np.float32)
            return depth_image
        except CvBridgeError, e:
            print e 

    #####process image##### 
    def process_image(self, frame):
        return frame
    
    #####process depth image#####
    def process_depth_image(self, frame):
        return frame


    #####mouse event#####
    def onMouse(self, event, x, y, flags, params):
        if self.frame is None:
            return
        if event == cv2.EVENT_LBUTTONDOWN and not self.drag_start:
            self.track_box = None
            self.detect_box = None
            self.drag_start = (x, y)
        if event == cv2.EVENT_LBUTTONUP:
            self.drag_start = None
            self.detect_box = self.selection
        if self.drag_start:
            xmin = max(0, min(x, self.drag_start[0]))
            ymin = max(0, min(y, self.drag_start[1]))
            xmax = min(self.frame_width, max(x, self.drag_start[0]))
            ymax = min(self.frame_height, max(y, self.drag_start[1]))
            self.selection = (xmin, ymin, xmax-xmin, ymax-ymin)

    #####display selection box#####    
    def display_selection(self):
        if self.drag_start and self.is_rect_nonzero(self.selection):
            x, y, w, h = self.selection
            cv2.rectangle(self.marker_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
    
    #####calculate if rect is zero#####
    def is_rect_nonzero(self, rect):
        try:
            (_,_,w,h) = rect
            return ((w>0)and(h>0))
        except:
            try:
                ((_,_),(w,h),a) = rect
                return (w > 0) and (h > 0)
            except:
                return False        

    #####shut down#####
    def shutdown(self):
        rospy.loginfo("Shutting down node")
        cv2.destroyAllWindows() 


if __name__ == '__main__':
    try:
        ROS2OPENCV("ros_opencv")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node"
        cv2.destroyAllWindows()