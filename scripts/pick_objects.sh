#!/bin/sh
xterm -e "roslaunch add_markers launch_myworld.launch" &
sleep 15
xterm -e "roslaunch add_markers launch_amcl.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects"
