#!/bin/bash
. /opt/ros/kinetic/setup.bash
. /home/abot/vision_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch usb_cam usb_cam.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch tracker_pkg lk_tracker.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch tracker_pkg follower.launch; exec bash"' \
