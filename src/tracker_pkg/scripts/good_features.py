#!/usr/bin/env python
import rospy
import cv2
from ros_opencv import ROS2OPENCV
import numpy as np

class GoodFeatures(ROS2OPENCV):
    def __init__(self, node_name):
        super(GoodFeatures, self).__init__(node_name)
        self.feature_size = rospy.get_param("~feature_size", 1)

        # Good features parameters
        self.gf_maxCorners = rospy.get_param("~gf_maxCorners", 200)
        self.gf_qualityLevel = rospy.get_param("~gf_qualityLevel", 0.02)
        self.gf_minDistance = rospy.get_param("~gf_minDistance", 7)
        self.gf_blockSize = rospy.get_param("~gf_blockSize", 10)
        self.gf_useHarrisDetector = rospy.get_param("~gf_useHarrisDetector", True)
        self.gf_k = rospy.get_param("~gf_k", 0.04)

        # Store all parameters together for passing to the detector
        self.gf_params = dict(maxCorners = self.gf_maxCorners, 
                       qualityLevel = self.gf_qualityLevel,
                       minDistance = self.gf_minDistance,
                       blockSize = self.gf_blockSize,
                       useHarrisDetector = self.gf_useHarrisDetector,
                       k = self.gf_k) 
        self.keypoints = list()
        self.detect_box = None
        self.mask = None 

    def process_image(self, frame):
        try:
            if not self.detect_box:
                return frame
            src = frame.copy()
            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY) 
            gray = cv2.equalizeHist(gray)
            keypoints = self.get_keypoints(gray, self.detect_box)
            if keypoints is not None and len(keypoints) > 0:
                for x, y in keypoints:
                    cv2.circle(self.marker_image, (x, y), self.feature_size, (0, 255, 0), -1)
        except:
            pass
        return frame
    
    def get_keypoints(self, input_image, detect_box):
        self.mask = np.zeros_like(input_image)
        try:
            x, y, w, h = detect_box
        except:
            return None
        self.mask[y:y+h, x:x+w] = 255
        keypoints = list()
        kp = cv2.goodFeaturesToTrack(input_image, mask = self.mask, **self.gf_params)
        if kp is not None and len(kp) > 0:
            for x, y in np.float32(kp).reshape(-1, 2):
                keypoints.append((x, y))
        return keypoints


                    
if __name__ == '__main__':
    try:
        node_name = "good_features"
        GoodFeatures(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down the Good Features node."
        cv2.DestroyAllWindows()    