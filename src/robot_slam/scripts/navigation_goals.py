#!/usr/bin/env python
#coding: utf-8

import rospy
import actionlib
import sys
reload(sys)
sys.setdefaultencoding('utf-8')
import os
import serial
import time
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from geometry_msgs.msg  import Point
from std_msgs.msg import String

serialPort = "/dev/shoot"
baudRate = 9600
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)

count=0
Yaw_th=0.0045

class object_posistion_1:
    def __init__(self):
        self.find_sub=rospy.Subscriber('/object_position',Point,self.find_cb)
    def find_cb(self,data):
        point_msg=data
        print('x:',point_msg.x)
        print('y:',point_msg.y)
        print('z:',point_msg.z)

class object_posistion_2:
    def __init__(self):
        self.find_sub=rospy.Subscriber('/object_position',Point,self.find_cb)
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    def find_cb(self,data):
        global flog0,flog1
        point_msg=data
        flog0=point_msg.x-320
        flog1=abs(flog0)
        if abs(flog1)>0.5
            print('no')
            msg=Twist()
            msg.angular.z=-0.5*flog0
        elif abs(flog1)<=0.5:
            print('ok')
            ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            print 0
            time.sleep(0.05)
	    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

class ARTracker_1:
    def __init__(self):
        self.find_sub=rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.ar_cb)
        self.ar_x_0 = 0.0
        self.ar_y_0 = 0.0
        self.id = None 
    def ar_cb(self,data):
        for marker in data.markers:
            if marker.id==0:
                self.ar_x_0=marker.pose.pose.position.x
                self.ar_y_0=marker.pose.pose.position.y
                self.id=marker.id
                print('Detected AR Marker ID:',self.id)
                print('AR Marker Position (x,y):',self.ar_x_0,self.ar_y_0)

class ARTracker_2:
    def __init__(self):
        self.find_sub=rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.ar_cb)
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    def ar_cb(self,data):
        global ar_x,ar_x_abs,Yaw_th
        ar_markers=data
        for marker in data.markers:
            if marker.id==0:
                ar_x=marker.pose.pose.position.x
                ar_x_abs=abs(ar_x)
                if ar_x_abs>=Yaw_th:
                    print('no')
                    msg=Twist()
                    msg.angular.z-1*ar_x
                    self.pub.publish(msg)
                elif ar_x_abs<Yaw_th:
                    print('ok')
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print 0
                    time.sleep(0.05)
	            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

class ARTracker_3:
    def __init__(self):
        self.find_sub=rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.ar_cb)
        self.ar_x_0 = 0.0
        self.ar_y_0 = 0.0
        self.id = None 
    def ar_cb(self,data):
        for marker in data.markers:
            if marker.id==0:
                self.ar_x_0=marker.pose.pose.position.x
                self.ar_y_0=marker.pose.pose.position.y
                self.id=marker.id
                print('Detected AR Marker ID:',self.id)
                print('AR Marker Position (x,y):',self.ar_x_0,self.ar_y_0)

class ARTracker_4:
    def __init__(self):
        self.find_sub=rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.ar_cb)
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    def ar_cb(self,data):
        global ar_x,ar_x_abs,Yaw_th
        ar_markers=data
        for marker in data.markers:
            if marker.id==0:
                ar_x=marker.pose.pose.position.x
                ar_x_abs=abs(ar_x)
                if ar_x_abs>=Yaw_th:
                    print('no')
                    msg=Twist()
                    msg.angular.z-1*ar_x
                    self.pub.publish(msg)
                elif ar_x_abs<Yaw_th:
                    print('ok')
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print 0
                    time.sleep(0.05)
	            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

class move_robot1:
    def __init__(self):
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    def move_cb(self):
        global time
        time=0
        msg=Twist()
        msg.linear.x=0.0
        msg.linear.y=-0.2
        msg.linear.z=0.0
        msg.angular.x=0.0
        msg.angular.y=0.0
        msg.angular.z=0.0

        while time<30:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time+=1

class move_robot2:
    def __init__(self):
        self.pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1000)
    def move_cb(self):
        global time
        time=0
        msg=Twist()
        msg.linear.x=-0.2
        msg.linear.y=0.0
        msg.linear.z=0.0
        msg.angular.x=0.0
        msg.angular.y=0.0
        msg.angular.z=0.0

        while time<20:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time+=1

class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords',String,queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
    
   
    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))
        arrive_str = "arrived to traget point"
        self.arrive_pub.publish(arrive_str)

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        msg = feedback
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s"%p)
        #arrive_str = "going to next point"
        #self.arrive_pub.publish(arrive_str)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)
    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    print ('Please 1 to continue: ')
    input = raw_input()
    print (goals)
    r = rospy.Rate(1)
    r.sleep()
    navi = navigation_demo()
    if (input == '1'):
        navi.goto(goals[0])
        if(count==0):
            obj_pos=object_posistion_1()
            obj_pos=object_posistion_2()
            count+=1
            if(count==1):
                navi.goto(goals[1])
                ar_tracker=ARTracker_1()
                ar_tracker=ARTracker_2()
                count+=1
                    if(count==2):
                        navi.goto(goals[2])
                        ar_tracker=ARTracker_3()
                        ar_tracker=ARTracker_4()
                        navi.goto(goals[3])
                        move=move_robot1()
                        move.move_cb()
                        move=move_robot2()
                        move.move_cb()

    while not rospy.is_shutdown():
          r.sleep()
