#!/usr/bin/env python

import rospy
import os
import sys
import cv2
import numpy as np

def read_images(path, sz=None):
    c = 0
    X, y = [], []
    names = []
    for dirname, dirnames, filenames in os.walk(path):
        for subdirname in dirnames:
            subject_path = os.path.join(dirname, subdirname)
            for filename in os.listdir(subject_path):
                try:
                    if (filename == ".directory"):
                        continue
                    filepath = os.path.join(subject_path, filename)
                    im = cv2.imread(os.path.join(subject_path, filename), cv2.IMREAD_GRAYSCALE)
                    if (im is None):
                        print("image" + filepath + "is None")
                    if (sz is not None):
                        im = cv2.resize(im, sz)
                    X.append(np.asarray(im, dtype=np.uint8))
                    y.append(c)
                except:
                    print("unexpected error")
                    raise
            c = c+1
            names.append(subdirname)
    return [names, X, y]

def face_rec():
    read_dir = "/home/abot/abot_ws/src/face_pkg/scripts/data"
    [names, X, y] = read_images(read_dir)
    y = np.asarray(y, dtype=np.int32)
    model = cv2.face_EigenFaceRecognizer.create()
    model.train(np.asarray(X), np.asarray(y))
    
    face_cascade = cv2.CascadeClassifier('/home/abot/abot_ws/src/face_pkg/scripts/cascades/haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        x, y = frame.shape[0:2]
        small_frame = cv2.resize(frame, (int(y/2), int(x/2)))
        result = small_frame.copy()
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in faces:
            result = cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
            roi = gray[x:x+w, y:y+h]
            try:
                roi = cv2.resize(roi, (200,200), interpolation=cv2.INTER_LINEAR)
                [p_label, p_confidence] = model.predict(roi)
                print(names[p_label] + str(p_confidence))
                cv2.putText(result, names[p_label], (x, y-20), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            except:
                continue
        cv2.imshow("recognize_face", result)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    try:
        node_name = "face_detector"
        rospy.init_node(node_name)
        face_rec()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node"
        cv2.destroyAllWindows()










