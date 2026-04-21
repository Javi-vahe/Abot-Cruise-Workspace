#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import os
robot_slam_path = "/home/abot/abot_ws_a/src/robot_slam/voices/"
sbd_1 = robot_slam_path + "number1.mp3"    # 识别点1-8
sbd_2 = robot_slam_path + "number2.mp3"
sbd_3 = robot_slam_path + "number3.mp3"
sbd_4 = robot_slam_path + "number4.mp3"
sbd_5 = robot_slam_path + "number5.mp3"
sbd_6 = robot_slam_path + "number6.mp3"
sbd_7 = robot_slam_path + "number7.mp3"
sbd_8 = robot_slam_path + "number8.mp3"
tcd_1 = robot_slam_path + "point1.mp3"    # 停车点1-8
tcd_2 = robot_slam_path + "point2.mp3"
tcd_3 = robot_slam_path + "point3.mp3"
tcd_4 = robot_slam_path + "point4.mp3"
tcd_5 = robot_slam_path + "point5.mp3"
tcd_6 = robot_slam_path + "point6.mp3"
tcd_7 = robot_slam_path + "point7.mp3"
tcd_8 = robot_slam_path + "point8.mp3"
zd = robot_slam_path + "ending.mp3" # 终点
print("zd mp3 path: ", zd)

os.system('mplayer %s' % sbd_1)
# os.system('mplayer %s' % sbd_1)
# os.system('mplayer %s' % sbd_1)
# os.system('mplayer %s' % sbd_1)