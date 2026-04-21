#!/usr/bin/env python

import rospy
import cv2
import numpy as np 
from ros_opencv import ROS2OPENCV

class PersonDetect(ROS2OPENCV):
    def __init__(self, node_name):
        super(PersonDetect, self).__init__(node_name)
        self.detect_box = None
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    
    def process_image(self, frame):
        src = frame.copy()
        self.detect_box = self.detect_people(src)

        return src

    def detect_people(self, frame):
        found, w = self.hog.detectMultiScale(frame)
        found_filtered = []
        for ri, r in enumerate(found):
            for qi, q in enumerate(found):
                if ri != qi and self.is_inside(q, r):
                    break
                else:
                    found_filtered.append(r)
        for person in found_filtered:
            self.draw_person(frame, person)
        if len(found_filtered) == 1:
            (x, y, w, h) = found_filtered[0] 
            return (x, y, w, h)
        else:
            return None
    
    def is_inside(self, o, i):
        ox, oy, ow, oh = o
        ix, iy, iw, ih = i
        return ox<ix and oy<iy and ox+ow>ix+iw and oy+oh>iy+ih

    def draw_person(self, frame, person):
        x, y, w, h = person
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
    
if __name__ == '__main__':
    try:
        node_name = "person_detector"
        PersonDetect(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
cv2.destroyAllWindows()


