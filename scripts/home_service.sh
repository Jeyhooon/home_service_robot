#!/bin/sh
xterm -e "roslaunch add_markers launch_myworld.launch" &
sleep 15
xterm -e "roslaunch add_markers launch_amcl.launch" &
sleep 5
xterm -e "roslaunch add_markers launch_rviz_config.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects" &
sleep 10
xterm -e "roslaunch add_markers launch_add_markers.launch"
