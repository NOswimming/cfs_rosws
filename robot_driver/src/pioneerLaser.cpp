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
	// do not populate the map on object avoidance or basic
	if (currentState == STATE_OBJECTAVOIDANCE || currentState == STATE_BASIC)
	{
		return;
	}
	
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
	
	bool leftObstacle = false;
	bool rightObstacle = false;
	bool midObstacle = false;
	
	int numberOfPoints = 5;

	// Find obstacles with laser scanners
	for (int j = 0; j < rangeDataNum - numberOfPoints; ++j) // Go through the laser data
	{
		// average to elimate noise
		float currentVal  = averageRange(laserScanData, j, j + numberOfPoints);
		
		//float lsTolerance = getLaserScanTolerance(j, rangeDataNum);
		
		if (currentVal < laserScanTolerance)
		{
			if(j < rangeDataNum / 3 - numberOfPoints)
			{
				rightObstacle = true;
			}
			else if(j < rangeDataNum * 2 / 3 - numberOfPoints)
			{
				midObstacle = true;
			}
			else
			{
				leftObstacle = true;
			}
		}
	}
	
	//ROS_INFO("Obstacles: L %d M %d R %d", leftObstacle, midObstacle, rightObstacle);
	
	if (rightObstacle || midObstacle || leftObstacle)
	{
		// If there is an obstacle then go to object avoidance state
		setNextState(STATE_OBJECTAVOIDANCE, false);
	
		// Turn based on obstacles	
		if (rightObstacle && midObstacle && leftObstacle)
		{
			// U turn
			// turn right
			setRobotMovement(0, -1.6);
		}
		else if (rightObstacle && leftObstacle)
		{
			// U turn
			// turn right
			setRobotMovement(0, -1.6);
		}
		else if (leftObstacle)
		{
			// turn right
			setRobotMovement(0, -0.3);
		}
		else if (rightObstacle)
		{
			// turn left
			setRobotMovement(0, 0.3);
		}
		else if (midObstacle)
		{
			// turn right
			setRobotMovement(0, -0.6);
		}
	}

	if (!rightObstacle && !midObstacle && !leftObstacle && currentState == STATE_OBJECTAVOIDANCE)
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
