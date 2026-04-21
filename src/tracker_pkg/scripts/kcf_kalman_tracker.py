#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from ros_opencv import ROS2OPENCV
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class KcfKalmanTracker(ROS2OPENCV):
    def __init__(self, node_name):
        super(KcfKalmanTracker, self).__init__(node_name)
        self.tracker = cv2.TrackerKCF_create()
        self.detect_box = None
        self.track_box = None
        ####init kalman####
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
        self.kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32)*0.03
        self.measurement = np.array((2,1),np.float32)
        self.prediction = np.array((2,1),np.float32)

        self.last_position = Point()
        self.positionPublisher = rospy.Publisher('/object_tracker/target_position', Point, queue_size=10)
        self.angle_xPublisher = rospy.Publisher('/object_tracker/angle_X', Float64, queue_size=10 )

    
    def process_image(self, frame):
        new_pose = None
        try:
            if self.detect_box is None:
                return frame
            src = frame.copy()
            if self.track_box is None or not self.is_rect_nonzero(self.track_box):
                self.track_box = self.detect_box
                if self.tracker is None:
                    raise Exception("tracker not init")
                status = self.tracker.init(src, self.track_box)
                if not status:
                    raise Exception("tracker initial failed")
            else:
                self.track_box = self.track(frame)
                ###update pose###
                new_pose = Point()
                new_pose.x = self.prediction[0]
                new_pose.y = self.prediction[1]
                new_pose.z = 0
                self.positionPublisher.publish(new_pose)
                self.last_position = new_pose
                angle_x = self.calculateAngleX(new_pose)
                #print (angle_x)
                self.angle_xPublisher.publish(angle_x)
        except:
            pass
        return frame
    
    def track(self, frame):
        status, coord = self.tracker.update(frame)
        center = np.array([[np.float32(coord[0]+coord[2]/2)],[np.float32(coord[1]+coord[3]/2)]])
        self.kalman.correct(center)
        self.prediction = self.kalman.predict()
        cv2.circle(frame, (int(self.prediction[0]),int(self.prediction[1])),4,(255,60,100),2)
        round_coord = (int(coord[0]), int(coord[1]), int(coord[2]), int(coord[3]))
        return round_coord
    
    def calculateAngleX(self, pos):
        centerX = pos.x        
        displacement = 2*centerX / self.frame_width - 1
        angle = -1 * (displacement * self.angle_Horizontal)
        return angle

if __name__ == '__main__':
    try:
        node_name = "kcfkalmantracker"
        KcfKalmanTracker(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
cv2.destroyAllWindows()

                    
