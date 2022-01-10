#!/bin/sh
xterm -e "roslaunch add_markers launch_myworld.launch" &
sleep 15
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects move_robot"