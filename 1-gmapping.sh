###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch robot_slam gmapping.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/Abot_Cruise_ws/devel/setup.bash; roslaunch robot_slam view_mapping.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \ 

