#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "pick_objects/GoToPosition.h"
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
// (typedef keyword is used to assign a new name to any existing data-type.)
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


bool handle_go_to_position_request(pick_objects::GoToPosition::Request& req, pick_objects::GoToPosition::Response& res)
{
  ROS_INFO("GoToPositionRequest received: x:%1.2f, y:%1.2f", (float)req.position_x, (float)req.position_y);
           
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = req.position_x;
  goal.target_pose.pose.position.y = req.position_y;
  goal.target_pose.pose.position.z = req.position_z;
  goal.target_pose.pose.orientation.x = req.orientation_x;
  goal.target_pose.pose.orientation.y = req.orientation_y;
  goal.target_pose.pose.orientation.z = req.orientation_z;
  goal.target_pose.pose.orientation.w = req.orientation_w;
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Robot is travelling to the requested zone ...");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, Robot reached to the requested zone! :)");
  	res.reach_feedback = true;
  }
  else{
    ROS_INFO("The robot failed to reach the requested zone :(");
  	res.reach_feedback = false;
  }
    
  
  return res.reach_feedback;
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("/go_to_position", handle_go_to_position_request);
  ROS_INFO("Ready to move the robot");

  ros::spin();

  return 0;
}