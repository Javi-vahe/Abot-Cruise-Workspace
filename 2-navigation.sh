#--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch usb_cam usb_cam-test.launch; exec bash"' \
###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch find_object_2d find_object_2d.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch robot_slam multi_goal.launch; exec bash"' \

