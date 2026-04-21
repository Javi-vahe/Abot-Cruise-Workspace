#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from good_features import GoodFeatures
from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class LKTracker(GoodFeatures):
    def __init__(self, node_name):
        super(LKTracker, self).__init__(node_name)
        self.feature_size = rospy.get_param("~feature_size", 1)
        self.lk_winSize = rospy.get_param("~lk_winSize", (10, 10))
        self.lk_maxLevel = rospy.get_param("~lk_maxLevel", 2)
        self.lk_criteria = rospy.get_param("~lk_criteria", (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01))
        self.lk_params = dict( winSize  = self.lk_winSize, 
                  maxLevel = self.lk_maxLevel, 
                  criteria = self.lk_criteria) 
        self.detect_interval = 1
        self.keypoints = None
        self.angle_xPublisher = rospy.Publisher('/object_tracker/angle_X', Float64, queue_size=10 )

        self.detect_box = None
        self.track_box = None
        self.mask = None
        self.gray = None
        self.prev_gray = None
        self.last_position = None
    
    def process_image(self, frame):
        try:
            if self.detect_box is None:
                return frame            
            src = frame.copy()
            self.gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            self.gray = cv2.equalizeHist(self.gray)
            if self.track_box is None or not self.is_rect_nonzero(self.track_box):
                self.track_box = self.detect_box
                self.keypoints = self.get_keypoints(self.gray, self.track_box)
            else:
                if self.prev_gray is None:
                    self.prev_gray = self.gray
                self.track_box = self.track_keypoints(self.gray, self.prev_gray)
                x, y, w, h = self.track_box
                new_pose = Point()
                new_pose.x = float(x + w/2)
                new_pose.y = float(y + h/2)
                new_pose.z = 0           
                self.last_position = new_pose
                angle_x = self.calculateAngleX(new_pose)
                               
                self.angle_xPublisher.publish(angle_x)

            self.prev_gray = self.gray
        except:
            pass
        return frame
    
    def track_keypoints(self, gray, prev_gray):
        img0, img1 = prev_gray, gray
        # Reshape the current keypoints into a numpy array required by calcOpticalFlowPyrLK()
        p0 = np.float32([p for p in self.keypoints]).reshape(-1, 1, 2)
        # Calculate the optical flow from the previous frame to the current frame
        p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **self.lk_params)
        try:
            # Do the reverse calculation: from the current frame to the previous frame
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **self.lk_params)
            # Compute the distance between corresponding points in the two flows
            d = abs(p0-p0r).reshape(-1, 2).max(-1)
            # If the distance between pairs of points is < 1 pixel, set a value in the "good" array to True, otherwise False
            good = d<1
            new_keypoints = list()
            for(x, y), good_flag in zip(p1.reshape(-1, 2), good):
                if not good_flag:
                    continue
                new_keypoints.append((x, y))
                cv2.circle(self.marker_image, (x, y), self.feature_size, (255, 255, 0), -1)
            self.keypoints = new_keypoints
            # Convert the keypoints list to a numpy array
            keypoints_array = np.float32([p for p in self.keypoints]).reshape(-1, 1, 2)
            if len(self.keypoints)>6:
                track_box = cv2.boundingRect(keypoints_array)
            else:
                track_box = cv2.boundingRect(keypoints_array)
        except:
            track_box = None
        return track_box
    
    def calculateAngleX(self, pos):
        
        centerX = pos.x                 
        displacement = 2*centerX / self.frame_width - 1
        angle = -1 * (displacement * self.angle_Horizontal)
        return angle


if __name__ == '__main__':
    try:
        node_name = "lk_tracker"
        LKTracker(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down LK Tracking node."
        cv2.DestroyAllWindows()        


