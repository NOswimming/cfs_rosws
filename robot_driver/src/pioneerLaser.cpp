#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include "Robot.cpp"
using namespace std;

// Only contains the main function for logic see Robot.cpp

/**
 * Main function for starting the pioneer laser node used by ROS.
 * Creates the publishers and subscribers for the pioneer laser node.
 */
int main(int argc, char **argv)
{
	// Creates the ROS pioneer laser node
	ros::init(argc, argv, "pioneer_laser_node");
	ros::NodeHandle my_handle;
	
	
	if(debug.PrintStateChange == true)
	{
		ROS_INFO("Debugging works");
		ROS_INFO("Map size: %d %d", mapSize.x, mapSize.y);
		ROS_INFO("Map offset %f %f", mapOffset.x, mapOffset.y);
		
	}

	// Create ROS publishers for topics
	ros::Publisher vel_pub_object = my_handle.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);

	// Create subscribers with callback functions for topics
	ros::Subscriber laser_sub_object = my_handle.subscribe("/scan", 1, laserScanCallback);
	ros::Subscriber map_sub_object = my_handle.subscribe("/map", 1, mapCallback);
	ros::Subscriber odom_sub_object = my_handle.subscribe("/RosAria/pose", 1, odomCallback);

	// Publish the topics using the publishers at a set rate
	// Publish rate (10 Hz)
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

		// Publish the topics
		vel_pub_object.publish(velocityCommand);
	}

	return 0;
}
