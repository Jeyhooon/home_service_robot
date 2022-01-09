#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 15
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch add_markers launch_rviz_config.launch" &
sleep 5
xterm -e "rosrun pick_objects move_robot" &
sleep 10
xterm -e "roslaunch add_markers launch_add_markers.launch"
