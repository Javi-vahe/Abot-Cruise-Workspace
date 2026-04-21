#!/usr/bin/env python

import rospy
import cv2
from ros_opencv import ROS2OPENCV
from std_msgs.msg import String
import os
import sys

class FaceDetector(ROS2OPENCV):
    def __init__(self, node_name):
        super(FaceDetector, self).__init__(node_name)
        self.tts_topic = rospy.get_param("~tts_topic", "/robot_voice/tts_topic")
        self.hello_pub = rospy.Publisher(self.tts_topic,String,queue_size=10)
        self.detect_box = None
        self.result = None
        self.count = 0
        self.face_cascade = cv2.CascadeClassifier('/home/abot/abot_ws/src/face_pkg/scripts/cascades/haarcascade_frontalface_default.xml')
        self.dirname = "/home/abot/abot_ws/src/face_pkg/scripts/data/jackchen/"
        if (not os.path.isdir(self.dirname)):
            os.makedirs(self.dirname)
    
    def process_image(self, frame):
        src = frame.copy()
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        result = src.copy()
        self.result = result
        if (len(faces) >= 1):
            hello_str = "Hello, my friend."
            self.hello_pub.publish(hello_str)
        for (x, y, w, h) in faces:
            result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
            f = cv2.resize(gray[y:y+h, x:x+w], (200, 200))
            if self.count<20:
                cv2.imwrite(self.dirname + '%s.pgm' % str(self.count), f)
                self.count += 1
        return result
    
    def detect_face(self, gray):
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        if (len(faces)==1):
            (x, y, w, h) = faces[0]
            return (x, y, w, h)
        else:
            return None
        
if __name__ == '__main__':
    try:
        node_name = "face_detector"
        FaceDetector(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
cv2.destroyAllWindows()
