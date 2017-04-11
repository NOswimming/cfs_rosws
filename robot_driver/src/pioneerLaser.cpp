#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include "Debug.cpp"
#include "Vector2Int.cpp"
#include "Vector2Float.cpp"
#include "Robot.cpp"

using namespace std;

// Only contains the main function and callback functions.
// For robot logic see Robot.cpp

////////// CALLBACK FUNCTIONS //////////

/**
 * Callback function for subscription to the gmapping occupancy grid.
 * @param map - gmapping occupancy grid.
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	// Populate the tiled map by reducing the occupancy grid to a lower resolution and summing the values
	populateTiledMap(map);
	// Update the naviagtion map based on the tiled map values
	populateNavigationMap();

	if (debug.WriteMapUpdateToFile)
	{
		ROS_INFO("\nMap Size: %d, %d\nMap Origin: %f, %f", map->info.width, map->info.height, map->info.origin.position.x, map->info.origin.position.y);
		DEBUG_printTiledMap(tiledMap, tiledMapSize);
		DEBUG_printNavigationMap(navigationMap, navigationMapSize);
	}
}

/**
 * Callback function for subscription to the robot odometry.
 * @param odom - robot odometry message
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
	// Update current pose
	currentPose = odom->pose.pose;

	checkState();


	if (debug.PrintCurrentPose)
	{
		ROS_INFO("CURRENT POSE\nPosition: %f, %f Orientation: %f", currentPose.position.x, currentPose.position.y, currentPose.orientation.z);
	}

}

/**
 * Callback function for subscription to the robot laser scanner.
 * @param laserScanData - robot laser scan message.
 */
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserScanData)
{
	// Array length
	float rangeDataNum = 1 + (laserScanData->angle_max - laserScanData->angle_min) / (laserScanData->angle_increment);

	float leftVal = averageRange(laserScanData, rangeDataNum - 65, rangeDataNum - 1);
	float rightVal = averageRange(laserScanData, 0, 63);
	float midVal = averageRange(laserScanData, rangeDataNum / 2 - 32, rangeDataNum / 2 + 32);

	//To avoid obstacles
	for (int j = 0; j < rangeDataNum; ++j) // Go through the laser data
	{
		if (laserScanData->ranges[j] < laserScanTolerance)
		{
			// If a laser scan is within the tolerance then move to object avoidance state
			setNextState(STATE_OBJECTAVOIDANCE, false);

			if (rightVal > leftVal)
			{
				// Turn right
				setRobotMovement(0, -0.6);
			}
			else
			{
				// Turn left
				setRobotMovement(0, 0.6);
			}
			return;

		}
	}

	if (currentState == STATE_OBJECTAVOIDANCE)
	{
		setNextState(previousState, false);
	}

}

////////// MAIN FUNCTION //////////

/**
 * Main function for starting the pioneer laser node used by ROS.
 * Creates the publishers and subscribers for the pioneer laser node.
 */
int main(int argc, char **argv)
{
	initializeRobot();
	
	// Creates the ROS pioneer laser node
	ros::init(argc, argv, "pioneer_laser_node");
	ros::NodeHandle my_handle;

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
