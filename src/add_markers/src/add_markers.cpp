#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include "add_markers/GoToPosition.h"

ros::ServiceClient client;

bool sendMoveRequest(add_markers::GoToPosition srv)
{
	if (!client.call(srv))
    {
      ROS_ERROR("Failed to call service go_to_position");
      return false;
    }
  else
    {
      return true;
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  	client = n.serviceClient<add_markers::GoToPosition>("/go_to_position");
  
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker (pick up zone).  This is a full 6DOF pose relative to the frame/time specified in the header
    float value;
  	add_markers::GoToPosition goal_srv;
  
    n.getParam("/pickup_zone/px", value);
    marker.pose.position.x = value;
    goal_srv.request.position_x = value;
  
    n.getParam("/pickup_zone/py", value);
    marker.pose.position.y = value;
    goal_srv.request.position_y = value;
  
  	n.getParam("/pickup_zone/pz", value);
    marker.pose.position.z = value;
    goal_srv.request.position_z = value;
  	
  	n.getParam("/pickup_zone/ox", value);
    marker.pose.orientation.x = value;
    goal_srv.request.orientation_x = value;
  
  	n.getParam("/pickup_zone/oy", value);
    marker.pose.orientation.y = value;
    goal_srv.request.orientation_y = value;
  
  	n.getParam("/pickup_zone/oz", value);
    marker.pose.orientation.z = value;
    goal_srv.request.orientation_z = value;
  
  	n.getParam("/pickup_zone/ow", value);
    marker.pose.orientation.w = value;
    goal_srv.request.orientation_w = value;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    // publishing the marker at pick up zone
    marker_pub.publish(marker);
  	bool reached = sendMoveRequest(goal_srv);
    ros::spinOnce();
  
    while(!reached){
      ros::spinOnce();
      r.sleep();
    }

     // robot reached the goal; delete the marker
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    ROS_INFO("Robot is picking up the object ... ");
    ros::Duration(5).sleep(); // sleep for 5 seconds

    // add marker at drop off zone:
    marker.action = visualization_msgs::Marker::ADD;
  
    n.getParam("/dropoff_zone/px", value);
    marker.pose.position.x = value;
    goal_srv.request.position_x = value;
  
    n.getParam("/dropoff_zone/py", value);
    marker.pose.position.y = value;
    goal_srv.request.position_y = value;
  
  	n.getParam("/dropoff_zone/pz", value);
    marker.pose.position.z = value;
    goal_srv.request.position_z = value;
  	
  	n.getParam("/dropoff_zone/ox", value);
    marker.pose.orientation.x = value;
    goal_srv.request.orientation_x = value;
  
  	n.getParam("/dropoff_zone/oy", value);
    marker.pose.orientation.y = value;
    goal_srv.request.orientation_y = value;
  
  	n.getParam("/dropoff_zone/oz", value);
    marker.pose.orientation.z = value;
    goal_srv.request.orientation_z = value;
  
  	n.getParam("/dropoff_zone/ow", value);
    marker.pose.orientation.w = value;
    goal_srv.request.orientation_w = value;
  
    reached = sendMoveRequest(goal_srv);
    ros::spinOnce();
    while(!reached){
      ros::spinOnce();
      r.sleep();
    }
    marker_pub.publish(marker);
    ROS_INFO("Robot delivered the object to the dropoff zone!");

    ros::spin();


    return 0;
}
